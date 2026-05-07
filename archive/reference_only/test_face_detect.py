# test_face_detect.py
# Step 1 test: YOLO person detection + SSD face detection inside person box.
# No recognition, no embeddings. Just verify face boxes appear.
# Press Q to quit.

import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import time
import os

# ── config ───────────────────────────────────────────
ENGINE_PATH = "/home/nvidia/Desktop/yolo11_jetson/yolo11n.engine"
INPUT_W, INPUT_H = 320, 320
CONF_THRESH = 0.5
IOU_THRESH  = 0.45
PERSON_CLS  = 0

SSD_PROTOTXT    = "/home/nvidia/Desktop/yolo11_jetson/face_models/deploy.prototxt"
SSD_CAFFEMODEL  = "/home/nvidia/Desktop/yolo11_jetson/face_models/res10_300x300_ssd_iter_140000_fp16.caffemodel"
SSD_CONF_THRESH = 0.5

FACE_REGION_TOP      = 0.0   # fraction of person box height to start searching
FACE_REGION_BOTTOM   = 0.40  # fraction of person box height to stop searching
FACE_DETECT_INTERVAL = 1.0   # run face detection once per second

GST_PIPELINE = (
    "nvarguscamerasrc ! "
    "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
    "nvvidconv flip-method=2 ! "
    "video/x-raw,format=BGRx ! "
    "videoconvert ! "
    "video/x-raw,format=BGR ! "
    "appsink drop=1"
)
# ─────────────────────────────────────────────────────


def load_ssd():
    net = cv2.dnn.readNetFromCaffe(SSD_PROTOTXT, SSD_CAFFEMODEL)
    # use CUDA backend if available, otherwise CPU
    backend = getattr(cv2.dnn, "DNN_BACKEND_CUDA", None)
    target  = getattr(cv2.dnn, "DNN_TARGET_CUDA",  None)
    if backend is not None and target is not None:
        net.setPreferableBackend(backend)
        net.setPreferableTarget(target)
        print("[ssd] Using CUDA backend")
    else:
        print("[ssd] Using CPU backend")
    return net


def load_engine(path):
    logger = trt.Logger(trt.Logger.WARNING)
    with open(path, "rb") as f, trt.Runtime(logger) as rt:
        return rt.deserialize_cuda_engine(f.read())


def allocate_buffers(engine):
    inputs, outputs, bindings = [], [], []
    stream = cuda.Stream()
    for binding in engine:
        size  = trt.volume(engine.get_binding_shape(binding))
        dtype = trt.nptype(engine.get_binding_dtype(binding))
        h_mem = cuda.pagelocked_empty(size, dtype)
        d_mem = cuda.mem_alloc(h_mem.nbytes)
        bindings.append(int(d_mem))
        if engine.binding_is_input(binding):
            inputs.append({"host": h_mem, "device": d_mem})
        else:
            outputs.append({"host": h_mem, "device": d_mem})
    return inputs, outputs, bindings, stream


def yolo_infer(context, inputs, outputs, bindings, stream, frame):
    img = cv2.resize(frame, (INPUT_W, INPUT_H))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    img = np.ascontiguousarray(np.transpose(img, (2, 0, 1))[np.newaxis])
    np.copyto(inputs[0]["host"], img.ravel())
    cuda.memcpy_htod_async(inputs[0]["device"], inputs[0]["host"], stream)
    context.execute_async_v2(bindings=bindings, stream_handle=stream.handle)
    cuda.memcpy_dtoh_async(outputs[0]["host"], outputs[0]["device"], stream)
    stream.synchronize()
    return outputs[0]["host"]


def parse_persons(raw, orig_w, orig_h):
    preds       = raw.reshape(84, -1).T
    class_ids   = np.argmax(preds[:, 4:], axis=1)
    confidences = preds[np.arange(len(class_ids)), 4 + class_ids]
    mask        = (class_ids == PERSON_CLS) & (confidences >= CONF_THRESH)
    boxes = preds[mask, :4]
    confs = confidences[mask]
    if len(boxes) == 0:
        return np.empty((0, 4), dtype=np.int32)
    sx, sy = orig_w / INPUT_W, orig_h / INPUT_H
    x1 = (boxes[:, 0] - boxes[:, 2] / 2) * sx
    y1 = (boxes[:, 1] - boxes[:, 3] / 2) * sy
    x2 = (boxes[:, 0] + boxes[:, 2] / 2) * sx
    y2 = (boxes[:, 1] + boxes[:, 3] / 2) * sy
    xyxy = np.stack([x1, y1, x2, y2], axis=1).astype(np.int32)
    keep = cv2.dnn.NMSBoxes(xyxy.tolist(), confs.tolist(), CONF_THRESH, IOU_THRESH)
    if len(keep) == 0:
        return np.empty((0, 4), dtype=np.int32)
    return xyxy[keep.flatten()]


def detect_face_ssd(net, crop):
    """Returns list of (x, y, w, h) in crop coordinates."""
    h, w = crop.shape[:2]
    blob = cv2.dnn.blobFromImage(crop, 1.0, (300, 300), (104, 177, 123))
    net.setInput(blob)
    dets = net.forward()   # [1, 1, N, 7]
    faces = []
    for i in range(dets.shape[2]):
        conf = float(dets[0, 0, i, 2])
        if conf < SSD_CONF_THRESH:
            continue
        x1 = max(0, int(dets[0, 0, i, 3] * w))
        y1 = max(0, int(dets[0, 0, i, 4] * h))
        x2 = min(w, int(dets[0, 0, i, 5] * w))
        y2 = min(h, int(dets[0, 0, i, 6] * h))
        fw, fh = x2 - x1, y2 - y1
        if fw > 0 and fh > 0:
            faces.append((x1, y1, fw, fh))
    return faces


def main():
    print("Loading YOLO engine...")
    engine  = load_engine(ENGINE_PATH)
    context = engine.create_execution_context()
    inputs, outputs, bindings, stream = allocate_buffers(engine)

    net = load_ssd()

    print("Opening camera...")
    cap = cv2.VideoCapture(GST_PIPELINE, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera.")

    orig_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    orig_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Running ({orig_w}x{orig_h}) — press Q to quit")

    prev_t       = time.time()
    last_det_t   = 0.0
    last_faces   = {}  # person_idx -> [(fx, fy, fw, fh) in frame coords]

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        now     = time.time()
        raw     = yolo_infer(context, inputs, outputs, bindings, stream, frame)
        persons = parse_persons(raw, orig_w, orig_h)

        run_det = (now - last_det_t >= FACE_DETECT_INTERVAL)

        face_count = 0
        for i, (px1, py1, px2, py2) in enumerate(persons):
            cv2.rectangle(frame, (px1, py1), (px2, py2), (255, 100, 0), 2)

            if run_det:
                box_h   = py2 - py1
                head_y1 = py1 + int(box_h * FACE_REGION_TOP)
                head_y2 = py1 + int(box_h * FACE_REGION_BOTTOM)
                crop    = frame[head_y1:head_y2, px1:px2]
                if crop.size == 0:
                    last_faces[i] = []
                    continue
                faces = detect_face_ssd(net, crop)
                last_faces[i] = [
                    (px1 + fx, head_y1 + fy, fw, fh)
                    for (fx, fy, fw, fh) in faces
                ]

            for (fx, fy, fw, fh) in last_faces.get(i, []):
                cv2.rectangle(frame,
                              (fx, fy), (fx + fw, fy + fh),
                              (0, 220, 0), 2)
                face_count += 1

        if run_det:
            last_faces = {i: v for i, v in last_faces.items()
                          if i < len(persons)}
            last_det_t = now

        fps = 1.0 / (now - prev_t + 1e-9)
        prev_t = now
        cv2.putText(frame,
                    f"FPS:{fps:.1f}  persons:{len(persons)}  faces:{face_count}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        cv2.imshow("Face Detection Test", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
