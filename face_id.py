# face_id.py
import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import pickle
import time
from pathlib import Path

# ── config ───────────────────────────────────────────
DETECTOR_ENGINE_PATH  = "/home/nvidia/Desktop/face_id/face_det.engine"
EMBEDDER_ENGINE_PATH  = "/home/nvidia/Desktop/face_id/face_embed.engine"
DB_PATH               = "/home/nvidia/Desktop/face_id/face_db.pkl"

DET_INPUT_W,   DET_INPUT_H   = 640, 640   # face detector input size
EMBED_INPUT_W, EMBED_INPUT_H = 112, 112   # ArcFace standard input size

FACE_CONF_THRESH = 0.5    # minimum detector confidence to accept a face
SIM_THRESH       = 0.45   # cosine similarity threshold for a positive match
NO_FACE_TIMEOUT  = 2.0    # seconds without a detected face before flagging

UNKNOWN_LABEL = "unknown"
# ────────────────────────────────────────────────────


# ── Database ─────────────────────────────────────────────────────────────────

class FaceDatabase:
    """
    Stores known-person embeddings on disk.
    Each person maps to a list of 512-d L2-normalised embeddings
    collected during enrollment.
    """

    def __init__(self, db_path: str):
        self.db_path = Path(db_path)
        # {name: np.ndarray shape [N, 512]}
        self.records: dict[str, np.ndarray] = {}
        if self.db_path.exists():
            self._load()

    # ── public ──────────────────────────────────────

    def identify(self, embedding: np.ndarray) -> tuple[str, float]:
        """
        Compare embedding against all stored records.
        Returns (name, cosine_similarity) of the best match,
        or (UNKNOWN_LABEL, best_score) if below SIM_THRESH.
        """
        if not self.records:
            return UNKNOWN_LABEL, 0.0

        best_name  = UNKNOWN_LABEL
        best_score = -1.0

        emb = embedding / (np.linalg.norm(embedding) + 1e-9)

        for name, stored in self.records.items():
            # stored shape: [N, 512], already L2-normalised at enroll time
            sims  = stored @ emb          # [N]
            score = float(sims.max())
            if score > best_score:
                best_score = score
                best_name  = name

        if best_score < SIM_THRESH:
            return UNKNOWN_LABEL, best_score
        return best_name, best_score

    def add(self, name: str, embeddings: np.ndarray):
        """
        Add / append embeddings for a person.
        embeddings: shape [N, 512] or [512].
        Called by enroll.py — not needed at runtime.
        """
        if embeddings.ndim == 1:
            embeddings = embeddings[np.newaxis]
        # L2-normalise before storing
        norms = np.linalg.norm(embeddings, axis=1, keepdims=True) + 1e-9
        embeddings = embeddings / norms
        if name in self.records:
            self.records[name] = np.concatenate([self.records[name], embeddings])
        else:
            self.records[name] = embeddings

    def save(self):
        with open(self.db_path, "wb") as f:
            pickle.dump(self.records, f)
        print(f"[DB] Saved -> {self.db_path}  ({len(self.records)} people)")

    def __len__(self):
        return len(self.records)

    # ── private ─────────────────────────────────────

    def _load(self):
        with open(self.db_path, "rb") as f:
            self.records = pickle.load(f)
        print(f"[DB] Loaded <- {self.db_path}  ({len(self.records)} people)")


# ── TensorRT helpers ─────────────────────────────────────────────────────────

def load_engine(engine_path: str):
    logger = trt.Logger(trt.Logger.WARNING)
    with open(engine_path, "rb") as f, trt.Runtime(logger) as runtime:
        return runtime.deserialize_cuda_engine(f.read())


def allocate_buffers(engine):
    inputs, outputs, bindings = [], [], []
    stream = cuda.Stream()
    for binding in engine:
        size  = trt.volume(engine.get_binding_shape(binding))
        dtype = trt.nptype(engine.get_binding_dtype(binding))
        host_mem   = cuda.pagelocked_empty(size, dtype)
        device_mem = cuda.mem_alloc(host_mem.nbytes)
        bindings.append(int(device_mem))
        if engine.binding_is_input(binding):
            inputs.append({"host": host_mem, "device": device_mem})
        else:
            outputs.append({"host": host_mem, "device": device_mem})
    return inputs, outputs, bindings, stream


def infer(context, inputs, outputs, bindings, stream) -> np.ndarray:
    cuda.memcpy_htod_async(inputs[0]["device"], inputs[0]["host"], stream)
    context.execute_async_v2(bindings=bindings, stream_handle=stream.handle)
    cuda.memcpy_dtoh_async(outputs[0]["host"], outputs[0]["device"], stream)
    stream.synchronize()
    return outputs[0]["host"]


# ── Preprocessing ─────────────────────────────────────────────────────────────

def preprocess_for_detector(crop: np.ndarray) -> np.ndarray:
    """BGR crop → [1, 3, 640, 640] float32 in [0, 1]"""
    img = cv2.resize(crop, (DET_INPUT_W, DET_INPUT_H))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))
    return np.ascontiguousarray(img[np.newaxis])


def preprocess_for_embedder(face_crop: np.ndarray) -> np.ndarray:
    """BGR face crop → [1, 3, 112, 112] float32, ArcFace normalisation"""
    img = cv2.resize(face_crop, (EMBED_INPUT_W, EMBED_INPUT_H))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 127.5 - 1.0   # → [-1, 1]
    img = np.transpose(img, (2, 0, 1))
    return np.ascontiguousarray(img[np.newaxis])


# ── Postprocessing ────────────────────────────────────────────────────────────

def postprocess_detector(raw: np.ndarray, crop_w: int, crop_h: int
                          ) -> np.ndarray:
    """
    Parse face detector output.

    Expected TRT output shape: [max_det, 5]  →  [x1, y1, x2, y2, conf]
    Coordinates are in detector input space (DET_INPUT_W × DET_INPUT_H).
    Returns boxes in crop pixel coordinates, shape [N, 4] int32.

    ── Adjust this function if your model output differs ──
    """
    # reshape in case the buffer is flat
    preds = raw.reshape(-1, 5)

    mask  = preds[:, 4] >= FACE_CONF_THRESH
    preds = preds[mask]

    if len(preds) == 0:
        return np.empty((0, 4), dtype=np.int32)

    sx = crop_w / DET_INPUT_W
    sy = crop_h / DET_INPUT_H

    boxes = preds[:, :4].copy()
    boxes[:, [0, 2]] *= sx
    boxes[:, [1, 3]] *= sy
    return boxes.astype(np.int32)


def _largest_face(boxes: np.ndarray) -> np.ndarray:
    """Return the single largest box by area."""
    areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
    return boxes[np.argmax(areas)]


# ── FaceIdentifier ────────────────────────────────────────────────────────────

class FaceIdentifier:
    """
    Main interface for detect_people.py.

    Usage:
        fi = FaceIdentifier()

        # inside tracking loop:
        for tid, box in tracker.get_unverified_tracks():
            result = fi.identify(frame, box, track_id=tid)
            if result["status"] == "verified":
                tracker.mark_verified(tid)
            elif result["status"] == "alert":
                # TODO: trigger speaker + LED (2-second no-face timeout)
                pass

    result dict keys:
        status      : "verified" | "unknown" | "no_face" | "alert"
        name        : person name or UNKNOWN_LABEL
        confidence  : cosine similarity score (float)
        face_box    : [x1,y1,x2,y2] in original frame coords, or None
    """

    def __init__(self):
        print("[face_id] Loading engines...")
        det_engine   = load_engine(DETECTOR_ENGINE_PATH)
        embed_engine = load_engine(EMBEDDER_ENGINE_PATH)

        self._det_ctx    = det_engine.create_execution_context()
        self._embed_ctx  = embed_engine.create_execution_context()

        self._det_in,   self._det_out,   self._det_bind,   self._det_stream   = allocate_buffers(det_engine)
        self._embed_in, self._embed_out, self._embed_bind, self._embed_stream = allocate_buffers(embed_engine)

        self.db = FaceDatabase(DB_PATH)

        # track_id → timestamp of first frame without a detected face
        self._no_face_since: dict[int, float] = {}

        print("[face_id] Ready")

    # ── public ──────────────────────────────────────

    def identify(self, frame: np.ndarray, person_box: np.ndarray,
                 track_id: int = -1) -> dict:
        x1, y1, x2, y2 = person_box
        person_crop = frame[y1:y2, x1:x2]

        if person_crop.size == 0:
            return self._result("no_face", UNKNOWN_LABEL, 0.0, None)

        crop_h, crop_w = person_crop.shape[:2]

        # ── 1. detect face within person crop ───────
        face_box_in_crop = self._detect_face(person_crop, crop_w, crop_h)

        if face_box_in_crop is None:
            status = self._handle_no_face(track_id)
            return self._result(status, UNKNOWN_LABEL, 0.0, None)

        # face found — reset no-face timer
        self._no_face_since.pop(track_id, None)

        # ── 2. convert face box → original frame coords ─
        fx1, fy1, fx2, fy2 = face_box_in_crop
        face_box_global = np.array([x1 + fx1, y1 + fy1, x1 + fx2, y1 + fy2])

        # ── 3. extract embedding ─────────────────────
        face_crop = person_crop[fy1:fy2, fx1:fx2]
        if face_crop.size == 0:
            return self._result("no_face", UNKNOWN_LABEL, 0.0, None)

        embedding = self._extract_embedding(face_crop)

        # ── 4. compare against database ─────────────
        name, confidence = self.db.identify(embedding)

        if name == UNKNOWN_LABEL:
            return self._result("unknown", UNKNOWN_LABEL, confidence,
                                face_box_global)

        return self._result("verified", name, confidence, face_box_global)

    # ── private ─────────────────────────────────────

    def _detect_face(self, crop: np.ndarray, crop_w: int, crop_h: int
                     ) -> np.ndarray | None:
        inp = preprocess_for_detector(crop)
        np.copyto(self._det_in[0]["host"], inp.ravel())
        raw = infer(self._det_ctx,
                    self._det_in, self._det_out,
                    self._det_bind, self._det_stream)
        boxes = postprocess_detector(raw, crop_w, crop_h)
        if len(boxes) == 0:
            return None
        return _largest_face(boxes)

    def _extract_embedding(self, face_crop: np.ndarray) -> np.ndarray:
        inp = preprocess_for_embedder(face_crop)
        np.copyto(self._embed_in[0]["host"], inp.ravel())
        raw = infer(self._embed_ctx,
                    self._embed_in, self._embed_out,
                    self._embed_bind, self._embed_stream)
        emb = raw.copy().reshape(-1)
        emb /= (np.linalg.norm(emb) + 1e-9)
        return emb

    def _handle_no_face(self, track_id: int) -> str:
        """
        Track how long a person has had no detectable face.
        Returns "alert" after NO_FACE_TIMEOUT seconds, else "no_face".

        TODO: on "alert", trigger speaker + LED signal.
        """
        now = time.time()
        if track_id not in self._no_face_since:
            self._no_face_since[track_id] = now
            return "no_face"
        if now - self._no_face_since[track_id] >= NO_FACE_TIMEOUT:
            return "alert"
        return "no_face"

    @staticmethod
    def _result(status: str, name: str, confidence: float,
                face_box) -> dict:
        return {
            "status":     status,
            "name":       name,
            "confidence": confidence,
            "face_box":   face_box,
        }


# ── Draw helper (used by detect_people.py) ───────────────────────────────────

def draw_face_result(frame: np.ndarray, result: dict) -> np.ndarray:
    """
    Overlay face box and identity label on frame.
    Call this after draw() in detect_people.py.
    """
    if result["face_box"] is None:
        return frame

    x1, y1, x2, y2 = result["face_box"]
    status = result["status"]

    color = {
        "verified": (0, 200, 0),
        "unknown":  (0, 0, 220),
        "alert":    (0, 140, 255),
    }.get(status, (180, 180, 180))

    label = {
        "verified": f"{result['name']} ({result['confidence']:.2f})",
        "unknown":  f"UNKNOWN ({result['confidence']:.2f})",
        "alert":    "ALERT: approach camera",
    }.get(status, status)

    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    cv2.putText(frame, label,
                (x1, max(y1 - 8, 0)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    return frame


# ── Standalone test ───────────────────────────────────────────────────────────

if __name__ == "__main__":
    fi = FaceIdentifier()
    print(f"[test] Enrolled people in DB: {len(fi.db)}")
    for name in fi.db.records:
        print(f"  - {name}: {len(fi.db.records[name])} embeddings")
