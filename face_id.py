# face_id.py
import cv2
import numpy as np
import pickle
import time
import urllib.request
import os
from pathlib import Path
from typing import Dict, Optional, Tuple
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

# ── config ───────────────────────────────────────────
BASE_DIR   = "/home/nvidia/Desktop/yolo11_jetson"
MODEL_DIR  = os.path.join(BASE_DIR, "face_models")
DB_PATH    = os.path.join(BASE_DIR, "face_db.pkl")

SSD_PROTOTXT      = os.path.join(MODEL_DIR, "deploy.prototxt")
SSD_CAFFEMODEL    = os.path.join(MODEL_DIR, "res10_300x300_ssd_iter_140000_fp16.caffemodel")
EMBED_ENGINE_PATH = os.path.join(BASE_DIR, "face_embed.engine")

DET_CONF_THRESH = 0.5    # SSD face detection confidence
SIM_THRESH      = 0.45   # cosine similarity threshold for a positive match
NO_FACE_TIMEOUT = 2.0    # seconds before triggering alert

EMBED_INPUT_W, EMBED_INPUT_H = 112, 112  # ArcFace/MobileFaceNet input

UNKNOWN_LABEL = "unknown"

# SFace canonical landmark positions for 112×112 aligned crop
_SFACE_DST_LANDMARKS = np.array(
    [[38.2946, 51.6963],
     [73.5318, 51.5014],
     [56.0252, 71.7366],
     [41.5493, 92.3655],
     [70.7299, 92.2041]],
    dtype=np.float32,
)
# ─────────────────────────────────────────────────────


# ── Model download ────────────────────────────────────────────────────────────

def _download(url: str, path: str):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    print(f"[face_id] Downloading {os.path.basename(path)} ...")
    urllib.request.urlretrieve(url, tmp)
    os.rename(tmp, path)
    print(f"[face_id] Saved -> {path}")


def ensure_models():
    if not os.path.isfile(SSD_PROTOTXT) or not os.path.isfile(SSD_CAFFEMODEL):
        raise RuntimeError(
            "SSD face detection models not found in {}.\n"
            "Download them first:\n"
            "  wget https://raw.githubusercontent.com/opencv/opencv/master"
            "/samples/dnn/face_detector/deploy.prototxt\n"
            "  wget https://raw.githubusercontent.com/opencv/opencv_3rdparty"
            "/dnn_samples_face_detector_20180205_fp16"
            "/res10_300x300_ssd_iter_140000_fp16.caffemodel".format(MODEL_DIR)
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


# ── Face detector (OpenCV DNN SSD ResNet-10) ──────────────────────────────────

class _SSDDetector:
    """
    OpenCV DNN SSD face detector (res10_300x300).
    Works on any OpenCV 3.3+ with DNN support.
    Outputs [N, 15] format using synthetic landmarks so the
    rest of the pipeline (alignment + embedding) is unchanged.
    """

    def __init__(self):
        self._net = cv2.dnn.readNetFromCaffe(SSD_PROTOTXT, SSD_CAFFEMODEL)
        backend = getattr(cv2.dnn, "DNN_BACKEND_CUDA", None)
        target  = getattr(cv2.dnn, "DNN_TARGET_CUDA",  None)
        if backend is not None and target is not None:
            self._net.setPreferableBackend(backend)
            self._net.setPreferableTarget(target)
            print("[face_id] SSD detector: CUDA backend")
        else:
            print("[face_id] SSD detector: CPU backend")

    def detect(self, image: np.ndarray) -> np.ndarray:
        """Returns array shape [N, 15]: [x,y,w,h, 5×landmark xy, score]"""
        h, w = image.shape[:2]
        blob = cv2.dnn.blobFromImage(image, 1.0, (300, 300), (104, 177, 123))
        self._net.setInput(blob)
        dets = self._net.forward()   # [1, 1, N, 7]

        faces = []
        for i in range(dets.shape[2]):
            conf = float(dets[0, 0, i, 2])
            if conf < DET_CONF_THRESH:
                continue
            x1 = max(0, int(dets[0, 0, i, 3] * w))
            y1 = max(0, int(dets[0, 0, i, 4] * h))
            x2 = min(w, int(dets[0, 0, i, 5] * w))
            y2 = min(h, int(dets[0, 0, i, 6] * h))
            bw, bh = x2 - x1, y2 - y1
            if bw <= 0 or bh <= 0:
                continue
            lm = _synthetic_landmarks(float(x1), float(y1), float(bw), float(bh))
            faces.append([float(x1), float(y1), float(bw), float(bh)]
                         + lm.reshape(-1).tolist() + [conf])

        if not faces:
            return np.empty((0, 15), dtype=np.float32)
        return np.asarray(faces, dtype=np.float32)


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


def _synthetic_landmarks(x, y, w, h) -> np.ndarray:
    return np.array(
        [[x + 0.35*w, y + 0.40*h], [x + 0.65*w, y + 0.40*h],
         [x + 0.50*w, y + 0.58*h], [x + 0.38*w, y + 0.76*h],
         [x + 0.62*w, y + 0.76*h]], dtype=np.float32)


def _align_face(image: np.ndarray, face: np.ndarray) -> np.ndarray:
    landmarks = face[4:14].reshape(5, 2)
    mat, _ = cv2.estimateAffinePartial2D(landmarks, _SFACE_DST_LANDMARKS,
                                          method=cv2.LMEDS)
    if mat is None:
        mat = cv2.getAffineTransform(landmarks[:3], _SFACE_DST_LANDMARKS[:3])
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
        self._detector = _SSDDetector()
        self._encoder  = _TRTEmbedder()
        self.db        = FaceDatabase(DB_PATH)
        self._no_face_since = {}  # type: Dict[int, float]
        print("[face_id] Ready (YuNet detection + TRT GPU embedding)")

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
