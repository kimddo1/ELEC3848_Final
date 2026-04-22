# detect_people.py
import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import time
import signal
import os
from face_id import FaceIdentifier, draw_face_result
from arduino_link import ArduinoLink, default_command_handler
import snapshot_writer

# ── graceful shutdown flag ────────────────────────────────────────────────────
# SIGINT handler sets this; main loop checks it each frame instead of catching
# KeyboardInterrupt, which does not reliably stop mid-CUDA operations.
_shutdown = False

def _request_shutdown(sig, frame):
    global _shutdown
    print("\n[Main] Shutdown requested — finishing current frame...")
    _shutdown = True

signal.signal(signal.SIGINT,  _request_shutdown)
signal.signal(signal.SIGTERM, _request_shutdown)
# ─────────────────────────────────────────────────────────────────────────────

# ── config ───────────────────────────────────────────
ENGINE_PATH = "/home/nvidia/Desktop/yolo11_jetson/yolo11n.engine"
INPUT_W, INPUT_H = 320, 320
CONF_THRESH = 0.5
IOU_THRESH  = 0.45
FACE_ID_EVERY_N_FRAMES = 8   # run face ID once every N frames to save GPU
UNKNOWN_CONFIRM_COUNT  = 3   # consecutive "unknown" frames needed before FACE_UNKNOWN is sent
ANNOUNCE_CONFIRM_FRAMES = 24 # consecutive active frames before PERSON_DETECTED is sent
                              # = ~1 s at 24 FPS — ghost re-ID tracks die before reaching this
PERSON_CLS  = 0          # COCO class 0 = person
ARDUINO_PORT = "/dev/ttyUSB0"   # Arduino Mega (genuine): ttyACM0; CH340 clone: ttyUSB0

GST_PIPELINE = (
    "nvarguscamerasrc ! "
    "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
    "nvvidconv flip-method=2 ! "
    "video/x-raw,format=BGRx ! "
    "videoconvert ! "
    "video/x-raw,format=BGR ! "
    "appsink drop=1"
)
# ────────────────────────────────────────────────────


class PersonTracker:
    """
    IoU-based multi-person tracker.

    Assigns a persistent integer track_id to each person across frames.
    Tracks that go unmatched for more than `max_disappeared` frames are dropped,
    and their verified status is cleared — forcing re-identification if they
    re-enter the scene.

    Interface for face-recognition colleague:
        tracker.get_unverified_tracks() -> list of (track_id, box [x1,y1,x2,y2])
        tracker.mark_verified(track_id)
    """

    def __init__(self, max_disappeared=45, match_iou=0.3):
        self.next_id = 0
        self.tracks = {}       # track_id -> {"box": np.array, "gone": int}
        self.verified = set()  # track_ids confirmed by face recognition
        self.max_disappeared = max_disappeared
        self.match_iou = match_iou

    # ── public ──────────────────────────────────────

    def update(self, boxes):
        """
        Feed new detection boxes ([x1,y1,x2,y2] array) each frame.
        Returns dict: track_id -> box for all currently active tracks.
        """
        # age every existing track; prune stale ones
        for tid in list(self.tracks):
            self.tracks[tid]["gone"] += 1
            if self.tracks[tid]["gone"] > self.max_disappeared:
                del self.tracks[tid]
                self.verified.discard(tid)

        if len(boxes) == 0:
            return self._active()

        if not self.tracks:
            for box in boxes:
                self._register(box)
            return self._active()

        track_ids  = list(self.tracks.keys())
        track_boxes = np.array([self.tracks[t]["box"] for t in track_ids])

        iou_matrix = self._iou_matrix(track_boxes, boxes)

        # greedy match: best IoU first
        matched_tracks = set()
        matched_dets   = set()
        pairs = np.dstack(np.unravel_index(
            np.argsort(-iou_matrix, axis=None), iou_matrix.shape
        ))[0]
        for ti, di in pairs:
            if ti in matched_tracks or di in matched_dets:
                continue
            if iou_matrix[ti, di] < self.match_iou:
                break
            tid = track_ids[ti]
            self.tracks[tid]["box"]  = boxes[di]
            self.tracks[tid]["gone"] = 0
            matched_tracks.add(ti)
            matched_dets.add(di)

        for di, box in enumerate(boxes):
            if di not in matched_dets:
                self._register(box)

        return self._active()

    def mark_verified(self, track_id):
        """Call this from face-recognition code once a person is identified."""
        if track_id in self.tracks:
            self.verified.add(track_id)

    def get_unverified_tracks(self):
        """Returns list of (track_id, box) that still need face identification."""
        return [
            (tid, info["box"])
            for tid, info in self.tracks.items()
            if tid not in self.verified and info["gone"] == 0
        ]

    # ── private ─────────────────────────────────────

    def _register(self, box):
        self.tracks[self.next_id] = {"box": box, "gone": 0}
        self.next_id += 1

    def _active(self):
        return {
            tid: info["box"]
            for tid, info in self.tracks.items()
            if info["gone"] == 0
        }

    @staticmethod
    def _iou_matrix(a, b):
        """Vectorised IoU between every pair in arrays a [N,4] and b [M,4]."""
        ax1, ay1, ax2, ay2 = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
        bx1, by1, bx2, by2 = b[:, 0], b[:, 1], b[:, 2], b[:, 3]

        inter_x1 = np.maximum(ax1[:, None], bx1[None, :])
        inter_y1 = np.maximum(ay1[:, None], by1[None, :])
        inter_x2 = np.minimum(ax2[:, None], bx2[None, :])
        inter_y2 = np.minimum(ay2[:, None], by2[None, :])

        inter = np.maximum(0, inter_x2 - inter_x1) * np.maximum(0, inter_y2 - inter_y1)
        area_a = (ax2 - ax1) * (ay2 - ay1)
        area_b = (bx2 - bx1) * (by2 - by1)
        union  = area_a[:, None] + area_b[None, :] - inter
        return inter / (union + 1e-9)


# ── TensorRT helpers ─────────────────────────────────────────────────────────

def load_engine(engine_path):
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
            inputs.append( {"host": host_mem, "device": device_mem})
        else:
            outputs.append({"host": host_mem, "device": device_mem})
    return inputs, outputs, bindings, stream


def preprocess(frame):
    """BGR frame → 320×320 NCHW float32 [0,1]"""
    img = cv2.resize(frame, (INPUT_W, INPUT_H))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))
    img = np.ascontiguousarray(img[np.newaxis])
    return img


def postprocess(output, orig_w, orig_h):
    """
    YOLO11 TensorRT output: [1, 84, 8400] → filtered person boxes.
    Returns boxes [N,4] in x1y1x2y2 (orig resolution) and confidences [N].
    """
    preds = output.reshape(84, -1).T

    boxes_xywh  = preds[:, :4]
    scores_all  = preds[:, 4:]
    class_ids   = np.argmax(scores_all, axis=1)
    confidences = scores_all[np.arange(len(class_ids)), class_ids]

    mask = (class_ids == PERSON_CLS) & (confidences >= CONF_THRESH)
    boxes_xywh  = boxes_xywh[mask]
    confidences = confidences[mask]

    if len(boxes_xywh) == 0:
        return np.empty((0, 4), dtype=np.int32), np.array([])

    sx = orig_w / INPUT_W
    sy = orig_h / INPUT_H
    x1 = (boxes_xywh[:, 0] - boxes_xywh[:, 2] / 2) * sx
    y1 = (boxes_xywh[:, 1] - boxes_xywh[:, 3] / 2) * sy
    x2 = (boxes_xywh[:, 0] + boxes_xywh[:, 2] / 2) * sx
    y2 = (boxes_xywh[:, 1] + boxes_xywh[:, 3] / 2) * sy
    boxes_xyxy = np.stack([x1, y1, x2, y2], axis=1).astype(np.int32)

    keep = cv2.dnn.NMSBoxes(
        boxes_xyxy.tolist(), confidences.tolist(), CONF_THRESH, IOU_THRESH
    )
    if len(keep) == 0:
        return np.empty((0, 4), dtype=np.int32), np.array([])

    keep = keep.flatten()
    return boxes_xyxy[keep], confidences[keep]


def draw(frame, active_tracks, verified_ids, track_names):
    """
    Verified tracks  → green box, person name
    Unverified tracks → red box,  "Identifying..."
    """
    for tid, box in active_tracks.items():
        x1, y1, x2, y2 = box
        is_verified = tid in verified_ids
        color = (0, 200, 0) if is_verified else (0, 0, 220)
        label = track_names.get(tid, "Identifying...") if is_verified else "Identifying..."
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, label,
                    (x1, max(y1 - 8, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

    cv2.putText(frame, f"count: {len(active_tracks)}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    return frame


def _send(link, msg, note=""):
    """Send a Jetson event to Arduino and log it clearly."""
    link.send(msg)
    suffix = f"  ({note})" if note else ""
    print(f"  [Jetson→Arduino] {msg}{suffix}")


def main():
    print("Loading TRT engine...")
    engine  = load_engine(ENGINE_PATH)
    context = engine.create_execution_context()
    inputs, outputs, bindings, stream = allocate_buffers(engine)
    print("Engine ready.")

    tracker     = PersonTracker(max_disappeared=45, match_iou=0.3)
    face_id     = FaceIdentifier()
    track_names        = {}    # track_id -> verified person name
    announced          = set() # track_ids for which PERSON_DETECTED was sent
    announce_count     = {}    # tid -> consecutive active frames (pre-announcement gate)
    face_snapped       = set() # track_ids for which a "face" snapshot was saved
    face_unknown_sent  = set() # track_ids for which FACE_UNKNOWN was sent
    face_unknown_count = {}    # tid -> consecutive "unknown" frame count (debounce)
    last_face_status   = {}    # tid -> last printed face status (suppress repeats)

    link = ArduinoLink(port=ARDUINO_PORT)
    link.register_command_handler(default_command_handler)
    link.start()

    print("Opening camera...")
    cap = cv2.VideoCapture(GST_PIPELINE, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera. Check GStreamer pipeline.")

    orig_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    orig_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera open: {orig_w}x{orig_h}  |  Press Q to quit\n")

    prev_t   = time.time()
    frame_no = 0
    while not _shutdown:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame — camera disconnected?")
            break

        snapshot_writer.update_frame(frame)   # keep latest frame for SNAP: commands

        # ── inference ──────────────────────────────
        inp = preprocess(frame)
        np.copyto(inputs[0]["host"], inp.ravel())
        cuda.memcpy_htod_async(inputs[0]["device"], inputs[0]["host"], stream)
        context.execute_async_v2(bindings=bindings, stream_handle=stream.handle)
        cuda.memcpy_dtoh_async(outputs[0]["host"], outputs[0]["device"], stream)
        stream.synchronize()

        boxes, _ = postprocess(outputs[0]["host"], orig_w, orig_h)

        # ── tracking ───────────────────────────────
        active_tracks = tracker.update(boxes)

        # ── serial: detect disappeared unverified tracks ────
        # Use tracker.tracks (all IDs, including gone>0 ones still within max_disappeared)
        # so a single missed frame does NOT trigger FACE_TIMEOUT prematurely.
        all_track_ids = set(tracker.tracks.keys())

        # Tracks that were announced but have now fully disappeared
        vanished = announced - tracker.verified - all_track_ids
        for tid in vanished:
            _send(link, "FACE_TIMEOUT", f"track #{tid} left scene")
            snapshot_writer.log_event("FACE_TIMEOUT", tid, "", "")
            announced.discard(tid)
            face_snapped.discard(tid)
            face_unknown_sent.discard(tid)
            face_unknown_count.pop(tid, None)
            last_face_status.pop(tid, None)

        # Ghost re-ID tracks: pending announcement but already pruned by tracker
        for tid in list(announce_count.keys()):
            if tid not in all_track_ids:
                announce_count.pop(tid, None)  # silently discard — never reached threshold

        # ── serial: announce new unverified tracks ──────────
        # Require ANNOUNCE_CONFIRM_FRAMES consecutive active frames before sending
        # PERSON_DETECTED — silently drops ghost re-ID tracks (a verified person's
        # box briefly misses IoU match → new track ID, gone again next frame).
        for tid, box in active_tracks.items():
            if tid not in tracker.verified and tid not in announced:
                announce_count[tid] = announce_count.get(tid, 0) + 1
                if announce_count[tid] >= ANNOUNCE_CONFIRM_FRAMES:
                    _send(link, "PERSON_DETECTED", f"track #{tid}")
                    announced.add(tid)
                    announce_count.pop(tid, None)
                    snapshot_writer.save(frame, box, "person", tid)

        # ── face identification (throttled) ────────
        if frame_no % FACE_ID_EVERY_N_FRAMES == 0:
            for tid, box in tracker.get_unverified_tracks():
                result = face_id.identify(frame, box, track_id=tid)
                frame  = draw_face_result(frame, result)
                # ── face snapshot (first time a face is found in this track) ──
                if result["status"] in ("verified", "unknown") and tid not in face_snapped:
                    snapshot_writer.save(frame, box, "face", tid)
                    face_snapped.add(tid)

                status = result["status"]

                # ── log face status only when it changes ──
                if last_face_status.get(tid) != status:
                    last_face_status[tid] = status
                    if status == "verified":
                        print(f"  Face #{tid}: VERIFIED → {result['name']}")
                    elif status == "unknown":
                        print(f"  Face #{tid}: unknown (not in database)")
                    elif status == "no_face":
                        print(f"  Face #{tid}: scanning — no face visible yet")
                    elif status == "alert":
                        print(f"  Face #{tid}: no face detected for 2 s")

                if status == "verified":
                    tracker.mark_verified(tid)
                    track_names[tid] = result["name"]
                    _send(link, f"FACE_VERIFIED:{result['name']}", f"track #{tid}")
                    announced.discard(tid)
                    face_unknown_sent.discard(tid)
                    face_unknown_count.pop(tid, None)
                    snapshot_writer.save(frame, box, "verified", tid, result["name"])
                    snapshot_writer.log_event("FACE_VERIFIED", tid, result["name"], "")

                elif status == "unknown" and tid not in face_unknown_sent:
                    # Require UNKNOWN_CONFIRM_COUNT consecutive "unknown" frames before
                    # declaring intruder — prevents a transient ArcFace miss from
                    # triggering SECURITY_ALERT while the real owner is still in frame.
                    face_unknown_count[tid] = face_unknown_count.get(tid, 0) + 1
                    if face_unknown_count[tid] >= UNKNOWN_CONFIRM_COUNT:
                        _send(link, "FACE_UNKNOWN", f"track #{tid}, {UNKNOWN_CONFIRM_COUNT} consecutive frames")
                        face_unknown_sent.add(tid)
                        snapshot_writer.log_event("FACE_UNKNOWN", tid, "", "")

                else:
                    # "no_face" or "alert" — reset unknown counter
                    face_unknown_count.pop(tid, None)
                    if status == "alert":
                        snapshot_writer.log_event("FACE_ALERT_NO_FACE", tid, "", "")
        frame_no += 1

        # ── display ────────────────────────────────
        frame = draw(frame, active_tracks, tracker.verified, track_names)

        now = time.time()
        fps = 1.0 / (now - prev_t + 1e-9)
        prev_t = now
        cv2.putText(frame, f"FPS: {fps:.1f}",
                    (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 200, 0), 2)

        cv2.imshow("YOLO11n - Person Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # ── cleanup ────────────────────────────────────────────────────────────────
    print("[Main] Releasing camera...")
    cap.release()
    cv2.destroyAllWindows()
    face_id.close()
    link.stop()
    print("[Main] Done.")
    # os._exit bypasses pycuda.autoinit's CUDA context destructor which causes
    # a core dump on Jetson when the process exits after a SIGINT mid-CUDA-op.
    os._exit(0)


if __name__ == "__main__":
    main()
