# Patrol Pro — Session Notes

## System Environment

| Item | Detail |
|---|---|
| Device | Jetson Nano |
| OS | Ubuntu 18.04 (JetPack 4.x) |
| Python | 3.6 |
| OpenCV | 4.1.1 (no `cv2.data`, no `cv2.FaceDetectorYN`) |
| TensorRT | 8.0.0.1 (`trtexec` at `/usr/src/tensorrt/bin/trtexec`) |
| CUDA | Available via pycuda |

---

## File Structure

```
yolo11_jetson/
  detect_people.py       # main program
  face_id.py             # face detection + recognition module
  enroll.py              # offline enrollment script
  capture_enroll.py      # interactive photo capture for enrollment
  yolo11n.engine         # YOLO11n TRT engine (person detection)
  face_det.engine        # SCRFD TRT engine (active — face detector)
  face_embed.engine      # MobileFaceNet TRT engine (face embedding, GPU)
  face_db.pkl            # face embedding database
  face_models/
    deploy.prototxt                              # SSD face detector architecture
    res10_300x300_ssd_iter_140000_fp16.caffemodel  # SSD face detector weights
    face_detection_yunet_2023mar.onnx            # downloaded but unused (OpenCV too old)
  enrollments/
    <person_name>/
      01.jpg ... 05.jpg

archive/
  reference_only/
    test_face_detect.py  # archived standalone face detection test (no recognition)
```

---

## Pipeline

```
detect_people.py
  YOLO11n (TRT/GPU)  — person detection, every frame
  PersonTracker      — IoU-based, assigns persistent track_ids
  FaceIdentifier     — runs every 8 frames (FACE_ID_EVERY_N_FRAMES) for unverified tracks only
    _SCRFDDetector   — SCRFD TRT (GPU), 640×640 input, 9-output anchor decode, 5-pt landmarks
    _TRTEmbedder     — MobileFaceNet TRT (GPU), 512-d embedding, ArcFace normalisation
    FaceDatabase     — cosine similarity vs face_db.pkl (threshold 0.45)
```

---

## TensorRT Engine Building

```bash
# Add trtexec to PATH
echo 'export PATH=$PATH:/usr/src/tensorrt/bin' >> ~/.bashrc
source ~/.bashrc

# Must cd into the folder containing .onnx files first
cd /home/nvidia/Desktop/yolo11_jetson

# Dynamic shape models require --minShapes/--optShapes/--maxShapes
trtexec --onnx=det_500m.onnx \
  --saveEngine=face_det.engine --fp16 \
  --minShapes=input.1:1x3x640x640 \
  --optShapes=input.1:1x3x640x640 \
  --maxShapes=input.1:1x3x640x640

trtexec --onnx=w600k_mbf.onnx \
  --saveEngine=face_embed.engine --fp16 \
  --minShapes=input.1:1x3x112x112 \
  --optShapes=input.1:1x3x112x112 \
  --maxShapes=input.1:1x3x112x112
```

**To find input tensor names if unknown:**
```bash
python3 -c "
import tensorrt as trt
logger = trt.Logger(trt.Logger.WARNING)
with open('model.engine', 'rb') as f, trt.Runtime(logger) as rt:
    engine = rt.deserialize_cuda_engine(f.read())
    for i in range(engine.num_bindings):
        print('INPUT' if engine.binding_is_input(i) else 'OUTPUT',
              engine.get_binding_name(i), engine.get_binding_shape(i))
"
```

---

## Common Errors & Fixes

### `trtexec: command not found`
```bash
/usr/src/tensorrt/bin/trtexec ...
# or add to PATH permanently (see above)
```

### `Cannot open engine file` / `Saving engine to file failed`
Engine built fine but save failed — output directory does not exist.
```bash
mkdir -p /path/to/output/dir
```

### `ValueError: could not broadcast input array from shape (X) into shape (3)`
TRT engine has dynamic input shapes but `set_binding_shape()` was not called before `allocate_buffers()`. Fix:
```python
ctx.set_binding_shape(0, (1, 3, H, W))
inputs, outputs, bindings, stream = allocate_buffers(engine, context=ctx)
```

### `TypeError: 'type' object is not subscriptable`
Python 3.6 does not support `tuple[str, float]` or `dict[str, ...]` as type hints.
Fix: use `from typing import Tuple, Dict, Optional` and replace accordingly.

### `SyntaxError: future feature annotations is not defined`
`from __future__ import annotations` requires Python 3.7+. Not available on this Jetson.
Fix: use `typing` module instead (see above).

### `AttributeError: module 'cv2' has no attribute 'data'`
OpenCV 4.1.1 does not have `cv2.data`. Use hardcoded cascade paths:
```python
candidates = [
    "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
    "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
]
```

### `TRT ERROR: setBindingDimensions exceed min~max range`
Engine was built without shape profiles — TRT locked spatial dims to 1.
Fix: rebuild engine with `--minShapes/--optShapes/--maxShapes` flags.

### `Segmentation fault (core dumped)` at exit
TRT/pycuda cleanup order issue on Jetson. Two required fixes:
1. Explicitly delete TRT contexts in every `close()` method:
```python
def close(self):
    del self._ctx
```
2. Call `face_id.close()` at the end of `main()` in `detect_people.py` — and make sure `FaceIdentifier.close()` closes both detector and encoder:
```python
def close(self):
    self._detector.close()
    self._encoder.close()
```

### `Failed to create CaptureSession` (camera won't open)
Argus camera daemon stuck after a hard crash. Fix:
```bash
sudo systemctl restart nvargus-daemon
```
Then run the program again. If daemon shows `failed` status:
```bash
sudo systemctl stop nvargus-daemon
sleep 2
sudo systemctl start nvargus-daemon
```

### GStreamer warning: `Cannot query video position`
Benign — live camera sources don't support position queries. Safe to ignore.

### `Gtk-Message: Failed to load module "canberra-gtk-module"`
Missing GTK sound module. No functional impact. Fix if desired:
```bash
sudo apt install libcanberra-gtk-module
```

---

## Face Detection Methods Tried

| Method | Result |
|---|---|
| Haar cascade | Works but poor accuracy, misses angled/partial faces |
| SSD ResNet-10 (cv2.dnn) | Good accuracy, OpenCV 4.1.1 compatible, but CPU-only — caused frame drops |
| YuNet (cv2.FaceDetectorYN) | Not available — requires OpenCV 4.5.4+, Jetson has 4.1.1 |
| **SCRFD TRT (face_det.engine)** | **Current solution** — GPU, 5-pt landmarks, no frame drops, confirmed working |

### SCRFD Output Format (for reference)
Engine outputs 9 tensors: score, bbox, keypoints × 3 strides (8, 16, 32).
Not a simple [N,5] post-NMS output — requires anchor decoding.

### SSD Face Detector
- Input: `blobFromImage(img, 1.0, (300,300), (104,177,123))`
- Output: `[1, 1, N, 7]` → `[batch, class, conf, x1, y1, x2, y2]` (normalized)
- Download:
  ```bash
  wget https://raw.githubusercontent.com/opencv/opencv/master/samples/dnn/face_detector/deploy.prototxt
  wget https://raw.githubusercontent.com/opencv/opencv_3rdparty/dnn_samples_face_detector_20180205_fp16/res10_300x300_ssd_iter_140000_fp16.caffemodel
  ```

---

## OpenCV Upgrade Options (YuNet requires 4.5.4+)

| Option | Time | Notes |
|---|---|---|
| `pip3 install opencv-contrib-python` | Fails | No pre-built wheel for Python 3.6 aarch64 |
| Q-engineering build script | ~4 hours | Builds from source with CUDA |
| Pre-built .whl | Not available | Q-engineering only provides shell scripts |

Build script (run overnight):
```bash
wget https://github.com/Qengineering/Install-OpenCV-Jetson-Nano/raw/main/OpenCV-4-10-0.sh
chmod +x OpenCV-4-10-0.sh
./OpenCV-4-10-0.sh
```

---

## Key Config Values

```python
# detect_people.py
FACE_ID_EVERY_N_FRAMES = 8   # face ID runs ~4x/sec at 30fps

# face_id.py
SIM_THRESH      = 0.45       # cosine similarity threshold for match
NO_FACE_TIMEOUT = 2.0        # seconds before alert status

# archive/reference_only/test_face_detect.py
FACE_REGION_TOP      = 0.0   # search from top of person box
FACE_REGION_BOTTOM   = 0.40  # search upper 40% only (head region)
FACE_DETECT_INTERVAL = 1.0   # run face detection once per second
SSD_CONF_THRESH      = 0.5   # SSD detection confidence
```

---

## Enrollment Workflow

```bash
# 1. Capture photos (5 per person)
python3 capture_enroll.py
# SPACE = manual capture, A = auto every 2s, Q = quit

# 2. Build face database
python3 enroll.py
# Tries 4 transforms × 3 scales per photo for better detection recall

# 3. Run main program
python3 detect_people.py
```

---

## Where Things Were Left Off

### Current Status — All Working
- `detect_people.py` running smoothly — person detection, face recognition, name display all confirmed working
- SCRFD TRT pipeline implemented and verified — no notable frame drops even during face detection
- Full pipeline confirmed: YOLO (person) → SCRFD (face, GPU) → MobileFaceNet (embed, GPU) → name label
- Noah's enrollment only has 1 good embedding — needs retake with `capture_enroll.py`

### Resolved: Frame Drop Issue
Replaced SSD ResNet-10 (CPU) with SCRFD TRT (GPU) as the face detector.
Both detection and embedding now run fully on GPU — frame drops eliminated.

### Remaining Task

