# face_id.py
import cv2
import numpy as np
import pickle
import time
import os
from pathlib import Path
from typing import Dict, Optional, Tuple
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

# ── config ───────────────────────────────────────────
BASE_DIR   = "/home/nvidia/Desktop/yolo11_jetson"
DB_PATH    = os.path.join(BASE_DIR, "face_db.pkl")

DET_ENGINE_PATH   = os.path.join(BASE_DIR, "face_det.engine")    # SCRFD TRT
EMBED_ENGINE_PATH = os.path.join(BASE_DIR, "face_embed.engine")  # MobileFaceNet TRT

SCRFD_INPUT_W, SCRFD_INPUT_H = 640, 640  # engine was built at this fixed size
SCRFD_STRIDES    = [8, 16, 32]
SCRFD_NUM_ANCHORS = 2                    # anchors per spatial location

DET_CONF_THRESH = 0.5    # SCRFD face detection confidence
NMS_IOU_THRESH  = 0.45   # NMS IoU threshold
SIM_THRESH      = 0.45   # cosine similarity threshold for a positive match
NO_FACE_TIMEOUT = 2.0    # seconds before triggering alert

EMBED_INPUT_W, EMBED_INPUT_H = 112, 112  # MobileFaceNet input

UNKNOWN_LABEL = "unknown"

# ArcFace canonical 5-point landmark positions for 112×112 aligned crop
_REF_LANDMARKS = np.array(
    [[38.2946, 51.6963],   # left eye
     [73.5318, 51.5014],   # right eye
     [56.0252, 71.7366],   # nose tip
     [41.5493, 92.3655],   # left mouth corner
     [70.7299, 92.2041]],  # right mouth corner
    dtype=np.float32,
)
# ─────────────────────────────────────────────────────


# ── Engine check ─────────────────────────────────────────────────────────────

def ensure_models():
    missing = [p for p in (DET_ENGINE_PATH, EMBED_ENGINE_PATH)
               if not os.path.isfile(p)]
    if missing:
        raise RuntimeError(
            "TRT engine(s) not found:\n" +
            "\n".join(f"  {p}" for p in missing)
        )


# ── Database ──────────────────────────────────────────────────────────────────

class FaceDatabase:
    """
    Stores known-person L2-normalised SFace embeddings on disk.
    Each person maps to an array of embeddings shape [N, 128].
    """

    def __init__(self, db_path: str):
        self.db_path = Path(db_path)
        self.records = {}  # type: Dict[str, np.ndarray]
        if self.db_path.exists():
            self._load()

    def identify(self, embedding: np.ndarray) -> Tuple[str, float]:
        if not self.records:
            return UNKNOWN_LABEL, 0.0

        emb = _normalize(embedding)
        best_name, best_score = UNKNOWN_LABEL, -1.0

        for name, stored in self.records.items():
            score = float((stored @ emb).max())
            if score > best_score:
                best_score, best_name = score, name

        if best_score < SIM_THRESH:
            return UNKNOWN_LABEL, best_score
        return best_name, best_score

    def add(self, name: str, embeddings: np.ndarray):
        if embeddings.ndim == 1:
            embeddings = embeddings[np.newaxis]
        embeddings = _normalize_batch(embeddings)
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

    def _load(self):
        with open(self.db_path, "rb") as f:
            self.records = pickle.load(f)
        print(f"[DB] Loaded <- {self.db_path}  ({len(self.records)} people)")


# ── Face detector (SCRFD TRT — GPU) ──────────────────────────────────────────

class _SCRFDDetector:
    """
    SCRFD face detector via TensorRT.
    Input : 640×640 RGB, normalised to [-1, 1]  (same norm as InsightFace repo)
    Output: 9 bindings — score + bbox + kps for strides 8, 16, 32

    detect() returns array shape [N, 15]:
        [x, y, w, h,  lm0x, lm0y, ..., lm4x, lm4y,  score]
    This matches the format expected by _align_face() and the rest of the pipeline.
    """

    def __init__(self):
        logger = trt.Logger(trt.Logger.WARNING)
        with open(DET_ENGINE_PATH, "rb") as f, trt.Runtime(logger) as rt:
            engine = rt.deserialize_cuda_engine(f.read())

        self._ctx    = engine.create_execution_context()
        self._ctx.set_binding_shape(0, (1, 3, SCRFD_INPUT_H, SCRFD_INPUT_W))
        self._stream = cuda.Stream()
        self._inputs, self._outputs, self._bindings = [], [], []

        for i, binding in enumerate(engine):
            shape = self._ctx.get_binding_shape(i)
            size  = int(trt.volume(shape))
            dtype = trt.nptype(engine.get_binding_dtype(binding))
            h_mem = cuda.pagelocked_empty(size, dtype)
            d_mem = cuda.mem_alloc(h_mem.nbytes)
            self._bindings.append(int(d_mem))
            if engine.binding_is_input(binding):
                self._inputs.append({"host": h_mem, "device": d_mem})
            else:
                self._outputs.append({"host": h_mem, "device": d_mem})

        # pre-compute anchor centres for each stride (reused every frame)
        self._anchors = _build_scrfd_anchors(
            SCRFD_INPUT_H, SCRFD_INPUT_W, SCRFD_STRIDES, SCRFD_NUM_ANCHORS
        )
        print("[face_id] SCRFD detector: TRT GPU")

    def detect(self, image: np.ndarray) -> np.ndarray:
        """
        image : BGR crop (any size) — resized internally to 640×640.
        Returns [N, 15] float32: [x, y, w, h, 5×lm_xy, score]
        Coordinates are scaled back to the input image size.
        """
        orig_h, orig_w = image.shape[:2]
        sx = orig_w / SCRFD_INPUT_W
        sy = orig_h / SCRFD_INPUT_H

        # preprocess: resize → RGB → [-1,1] → NCHW
        resized = cv2.resize(image, (SCRFD_INPUT_W, SCRFD_INPUT_H))
        rgb     = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        inp     = (rgb.astype(np.float32) - 127.5) / 128.0
        inp     = np.ascontiguousarray(np.transpose(inp, (2, 0, 1))[np.newaxis])

        np.copyto(self._inputs[0]["host"], inp.ravel())
        cuda.memcpy_htod_async(
            self._inputs[0]["device"], self._inputs[0]["host"], self._stream)
        self._ctx.execute_async_v2(
            bindings=self._bindings, stream_handle=self._stream.handle)
        for out in self._outputs:
            cuda.memcpy_dtoh_async(out["host"], out["device"], self._stream)
        self._stream.synchronize()

        # decode outputs: 3 strides × (score, bbox, kps) = 9 output tensors
        # engine binding order: score8, bbox8, kps8, score16, bbox16, kps16, score32, bbox32, kps32
        all_faces = []
        for s_idx, stride in enumerate(SCRFD_STRIDES):
            scores = self._outputs[s_idx * 3 + 0]["host"].reshape(-1, 1)
            bboxes = self._outputs[s_idx * 3 + 1]["host"].reshape(-1, 4)
            kpss   = self._outputs[s_idx * 3 + 2]["host"].reshape(-1, 10)

            anchors = self._anchors[stride]  # [M, 2] cx,cy
            mask    = scores[:, 0] >= DET_CONF_THRESH
            if not np.any(mask):
                continue

            scores  = scores[mask, 0]
            bboxes  = bboxes[mask]
            kpss    = kpss[mask]
            anchors = anchors[mask]

            # decode bbox: ltrb distances → x1y1x2y2
            x1 = anchors[:, 0] - bboxes[:, 0] * stride
            y1 = anchors[:, 1] - bboxes[:, 1] * stride
            x2 = anchors[:, 0] + bboxes[:, 2] * stride
            y2 = anchors[:, 1] + bboxes[:, 3] * stride

            # decode landmarks: offset from anchor centre
            # anchors[:, 0:1] is (M,1) → broadcasts with kpss[:, 0::2] (M,5)
            lm = np.stack([
                anchors[:, 0:1] + kpss[:, 0::2] * stride,  # [M, 5] x
                anchors[:, 1:2] + kpss[:, 1::2] * stride,  # [M, 5] y
            ], axis=2).reshape(-1, 10)                      # [M, 10] interleaved x0y0x1y1...

            # scale back to original image size
            x1 *= sx;  x2 *= sx
            y1 *= sy;  y2 *= sy
            lm[:, 0::2] *= sx
            lm[:, 1::2] *= sy

            bw = x2 - x1
            bh = y2 - y1
            for i in range(len(scores)):
                if bw[i] > 0 and bh[i] > 0:
                    all_faces.append(
                        [x1[i], y1[i], bw[i], bh[i]]
                        + lm[i].tolist()
                        + [float(scores[i])]
                    )

        if not all_faces:
            return np.empty((0, 15), dtype=np.float32)

        faces = np.asarray(all_faces, dtype=np.float32)
        # NMS on [x1,y1,w,h] boxes
        boxes_xywh = faces[:, :4].tolist()
        scores_lst = faces[:, 14].tolist()
        keep = cv2.dnn.NMSBoxes(boxes_xywh, scores_lst, DET_CONF_THRESH, NMS_IOU_THRESH)
        if len(keep) == 0:
            return np.empty((0, 15), dtype=np.float32)
        return faces[keep.flatten()]

    def close(self):
        del self._ctx


# ── SCRFD anchor builder ──────────────────────────────────────────────────────

def _build_scrfd_anchors(input_h, input_w, strides, num_anchors):
    """
    Pre-compute anchor centre points for all strides.
    Returns dict: stride -> np.ndarray [M, 2] (cx, cy) in input-image pixels.
    """
    anchors = {}
    for stride in strides:
        fh = input_h // stride
        fw = input_w // stride
        cy, cx = np.mgrid[0:fh, 0:fw]
        # each location has num_anchors anchors with the same centre
        cx = np.stack([cx] * num_anchors, axis=2).reshape(-1).astype(np.float32)
        cy = np.stack([cy] * num_anchors, axis=2).reshape(-1).astype(np.float32)
        # convert grid index to pixel position (centre of each cell)
        anchors[stride] = np.stack(
            [(cx + 0.5) * stride, (cy + 0.5) * stride], axis=1
        )
    return anchors


# ── Face embedder (TRT MobileFaceNet — GPU) ───────────────────────────────────

class _TRTEmbedder:
    """
    TensorRT MobileFaceNet embedder running on the Jetson GPU.
    Input: 112×112 landmark-aligned face crop, ArcFace normalisation.
    Output: 512-d L2-normalised embedding.
    """

    def __init__(self):
        logger  = trt.Logger(trt.Logger.WARNING)
        with open(EMBED_ENGINE_PATH, "rb") as f, trt.Runtime(logger) as rt:
            engine = rt.deserialize_cuda_engine(f.read())

        self._ctx    = engine.create_execution_context()
        self._ctx.set_binding_shape(0, (1, 3, EMBED_INPUT_H, EMBED_INPUT_W))
        self._stream = cuda.Stream()
        self._inputs, self._outputs, self._bindings = [], [], []

        for i, binding in enumerate(engine):
            shape = self._ctx.get_binding_shape(i)
            size  = trt.volume(shape)
            dtype = trt.nptype(engine.get_binding_dtype(binding))
            h_mem = cuda.pagelocked_empty(size, dtype)
            d_mem = cuda.mem_alloc(h_mem.nbytes)
            self._bindings.append(int(d_mem))
            if engine.binding_is_input(binding):
                self._inputs.append({"host": h_mem, "device": d_mem})
            else:
                self._outputs.append({"host": h_mem, "device": d_mem})

    def encode(self, image: np.ndarray, face: np.ndarray) -> Optional[np.ndarray]:
        """face: 15-element YuNet row. Returns normalised embedding."""
        try:
            aligned = _align_face(image, np.asarray(face[:14], dtype=np.float32))
            inp = _preprocess_aligned(aligned)
            np.copyto(self._inputs[0]["host"], inp.ravel())
            cuda.memcpy_htod_async(self._inputs[0]["device"],
                                   self._inputs[0]["host"], self._stream)
            self._ctx.execute_async_v2(bindings=self._bindings,
                                       stream_handle=self._stream.handle)
            cuda.memcpy_dtoh_async(self._outputs[0]["host"],
                                   self._outputs[0]["device"], self._stream)
            self._stream.synchronize()
            emb = self._outputs[0]["host"].copy().reshape(-1)
            return _normalize(emb)
        except Exception as e:
            print(f"[face_id] embed error: {e}")
            return None

    def close(self):
        del self._ctx


# ── Helpers ───────────────────────────────────────────────────────────────────

def _normalize(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else v


def _normalize_batch(a: np.ndarray) -> np.ndarray:
    norms = np.linalg.norm(a, axis=1, keepdims=True)
    return a / np.maximum(norms, 1e-12)


def _largest_face(faces: np.ndarray) -> Optional[np.ndarray]:
    if len(faces) == 0:
        return None
    return max(faces, key=lambda f: float(f[2] * f[3]))


def _align_face(image: np.ndarray, face: np.ndarray) -> np.ndarray:
    """Warp face crop to 112×112 using 5 SCRFD landmarks."""
    landmarks = face[4:14].reshape(5, 2)
    mat, _ = cv2.estimateAffinePartial2D(landmarks, _REF_LANDMARKS,
                                          method=cv2.LMEDS)
    if mat is None:
        mat = cv2.getAffineTransform(landmarks[:3], _REF_LANDMARKS[:3])
    return cv2.warpAffine(image, mat, (112, 112), flags=cv2.INTER_LINEAR)


def _preprocess_aligned(aligned: np.ndarray) -> np.ndarray:
    """112×112 BGR aligned crop → [1,3,112,112] float32 ArcFace normalisation"""
    img = cv2.cvtColor(aligned, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 127.5 - 1.0
    img = np.transpose(img, (2, 0, 1))
    return np.ascontiguousarray(img[np.newaxis])


# ── FaceIdentifier ────────────────────────────────────────────────────────────

class FaceIdentifier:
    """
    Main interface for detect_people.py.

    Usage (inside tracking loop):
        for tid, box in tracker.get_unverified_tracks():
            result = fi.identify(frame, box, track_id=tid)
            frame  = draw_face_result(frame, result)
            if result["status"] == "verified":
                tracker.mark_verified(tid)
            elif result["status"] == "alert":
                pass  # TODO: trigger speaker + LED

    result dict keys:
        status      : "verified" | "unknown" | "no_face" | "alert"
        name        : matched name or UNKNOWN_LABEL
        confidence  : cosine similarity score
        face_box    : [x1,y1,x2,y2] in original frame coords, or None
    """

    def __init__(self):
        ensure_models()
        print("[face_id] Loading models...")
        self._detector = _SCRFDDetector()
        self._encoder  = _TRTEmbedder()
        self.db        = FaceDatabase(DB_PATH)
        self._no_face_since = {}  # type: Dict[int, float]
        print("[face_id] Ready (SCRFD TRT detection + MobileFaceNet TRT embedding)")

    def identify(self, frame: np.ndarray, person_box: np.ndarray,
                 track_id: int = -1) -> dict:
        x1, y1, x2, y2 = person_box
        crop = frame[y1:y2, x1:x2]
        if crop.size == 0:
            return self._result("no_face", UNKNOWN_LABEL, 0.0, None)

        face = _largest_face(self._detector.detect(crop))

        if face is None:
            return self._result(self._handle_no_face(track_id),
                                UNKNOWN_LABEL, 0.0, None)

        self._no_face_since.pop(track_id, None)

        # face box in original frame coords
        fx, fy, fw, fh = int(face[0]), int(face[1]), int(face[2]), int(face[3])
        face_box_global = np.array([x1+fx, y1+fy, x1+fx+fw, y1+fy+fh])

        embedding = self._encoder.encode(crop, face)
        if embedding is None:
            return self._result("no_face", UNKNOWN_LABEL, 0.0, face_box_global)

        name, confidence = self.db.identify(embedding)
        status = "verified" if name != UNKNOWN_LABEL else "unknown"
        return self._result(status, name, confidence, face_box_global)

    def detect_and_embed(self, image: np.ndarray,
                         search_box: Optional[np.ndarray] = None
                         ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Detect the largest face and return (embedding, face_box_global).
        Used by enroll.py. search_box = [x1,y1,x2,y2] crops the search area.
        Returns (None, None) if no face found.
        """
        if search_box is not None:
            x1, y1, x2, y2 = search_box
            crop   = image[y1:y2, x1:x2]
            offset = (x1, y1)
        else:
            crop   = image
            offset = (0, 0)

        face = _largest_face(self._detector.detect(crop))
        if face is None:
            return None, None

        embedding = self._encoder.encode(crop, face)
        if embedding is None:
            return None, None

        fx, fy, fw, fh = int(face[0]), int(face[1]), int(face[2]), int(face[3])
        ox, oy = offset
        face_box = np.array([ox+fx, oy+fy, ox+fx+fw, oy+fy+fh])
        return embedding, face_box

    def close(self):
        self._detector.close()
        self._encoder.close()

    # ── private ─────────────────────────────────────

    def _handle_no_face(self, track_id: int) -> str:
        now = time.time()
        if track_id not in self._no_face_since:
            self._no_face_since[track_id] = now
            return "no_face"
        if now - self._no_face_since[track_id] >= NO_FACE_TIMEOUT:
            return "alert"
        return "no_face"

    @staticmethod
    def _result(status, name, confidence, face_box):
        return {"status": status, "name": name,
                "confidence": confidence, "face_box": face_box}


# ── Draw helper ───────────────────────────────────────────────────────────────

def draw_face_result(frame: np.ndarray, result: dict) -> np.ndarray:
    if result["face_box"] is None:
        return frame

    x1, y1, x2, y2 = result["face_box"]
    status = result["status"]

    color = {"verified": (0, 200, 0),
             "unknown":  (0, 0, 220),
             "alert":    (0, 140, 255)}.get(status, (180, 180, 180))

    label = {"verified": f"{result['name']} ({result['confidence']:.2f})",
             "unknown":  f"UNKNOWN ({result['confidence']:.2f})",
             "alert":    "ALERT: approach camera"}.get(status, status)

    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    cv2.putText(frame, label, (x1, max(y1 - 8, 0)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    return frame


# ── Standalone test ───────────────────────────────────────────────────────────

if __name__ == "__main__":
    fi = FaceIdentifier()
    print(f"[test] Enrolled people in DB: {len(fi.db)}")
    for name in fi.db.records:
        print(f"  - {name}: {len(fi.db.records[name])} embeddings")
