# Patrol Pro — Code Notes

Quick reference for every source file. Read this instead of the full source.
See SESSION_NOTES.md for environment, errors, pipeline architecture, and enrollment workflow.

---

## File Index

| File | Purpose | Status |
|---|---|---|
| `detect_people.py` | Main program — YOLO + tracking + face ID loop | Active |
| `face_id.py` | Face detection + recognition module | Active |
| `enroll.py` | Offline face database builder | Active |
| `capture_enroll.py` | Interactive camera capture for enrollment photos | Active |
| `test_face_detect.py` | Standalone YOLO + SSD face detection test (no recognition) | Retired / reference only |
| `hazard_monitor/hazard_monitor.ino` | Arduino firmware — LED, servo, buzzer, ultrasonic, OLED display | Active |
| `Servo_test/Servo_test.ino` | Standalone servo test — smooth 90↔130° sweep via serial command | Reference |
| `wall_bounce/wall_bounce.ino` | 4-motor drive + wall-bounce navigation (front ultrasonic) | Active |
| `arduino_link.py` | Jetson-side USB serial bridge to Arduino Mega | Active |
| `audio_player.py` | Non-blocking aplay wrapper + TTS fallback for verified_<name> | Active |
| `snapshot_writer.py` | Saves JPEG frame crops + CSV event log at each track transition | Active |
| `generate_audio.py` | Generates all required WAV clips via espeak+sox (run once on Jetson) | Run once |
| `bt_receiver.py` | Remote laptop script — reads HC-05 BT serial, prints formatted alerts | Run on laptop |

---

## detect_people.py

**Role:** Entry point. Runs YOLO person detection every frame, tracks persons, throttles face ID.

**Key classes / functions:**

| Name | What it does |
|---|---|
| `PersonTracker` | IoU-based multi-person tracker. Assigns persistent `track_id` per person across frames. Tracks go stale after `max_disappeared=45` frames and are dropped + unverified. |
| `PersonTracker.update(boxes)` | Feed YOLO boxes each frame. Returns `{track_id: box}` for active tracks. |
| `PersonTracker.get_unverified_tracks()` | Returns `[(track_id, box)]` for tracks not yet face-identified. |
| `PersonTracker.mark_verified(tid)` | Called once face recognition confirms a person. Skips that track from then on. |
| `load_engine(path)` | Deserialises a TRT engine from file. |
| `allocate_buffers(engine)` | Allocates paged-locked host + device memory for all engine bindings. |
| `preprocess(frame)` | BGR frame → 320×320 NCHW float32 [0,1] for YOLO input. |
| `postprocess(output, w, h)` | Decodes YOLO `[1,84,8400]` output → person boxes + NMS. Returns `(boxes, confs)`. |
| `draw(frame, active_tracks, verified_ids, track_names)` | Draws green (verified + name) or red (identifying) boxes. Person count overlay. |
| `main()` | Full loop: load engine → open camera → infer → track → face ID (throttled) → display. |

**Key config at top of file:**
```python
ENGINE_PATH            = ".../yolo11n.engine"
INPUT_W, INPUT_H       = 320, 320
CONF_THRESH            = 0.5
IOU_THRESH             = 0.45
FACE_ID_EVERY_N_FRAMES = 8      # face ID runs every 8 frames (~4x/sec at 30fps)
PERSON_CLS             = 0      # COCO class 0 = person
```

**Important details:**
- `track_names = {}` dict maps `track_id → name`; populated on verification, used by `draw()`
- `face_id.close()` called after the main loop to prevent segfault on exit
- Camera opened via GStreamer pipeline (`nvarguscamerasrc`, flip-method=2)

---

## face_id.py

**Role:** All face detection and recognition logic. Imported by `detect_people.py` and `enroll.py`.

**Key classes:**

### `FaceDatabase`
- Pickle-based store at `face_db.pkl`
- Records: `{name: np.ndarray [N, 512]}` — L2-normalised embeddings per person
- `identify(embedding)` → `(name, cosine_score)` — compares against all stored embeddings, returns best match above `SIM_THRESH = 0.45`
- `add(name, embeddings)` — appends or creates entry; normalises before storing
- `save()` / `_load()` — pickle serialisation

### `_SCRFDDetector` *(replaces old `_SSDDetector`)*
- TRT engine: `face_det.engine` (SCRFD det_500m, built at 640×640)
- Input: BGR crop → resized to 640×640 → RGB → normalised `(pixel - 127.5) / 128.0` → NCHW
- Output: 9 bindings — score, bbox (ltrb), kps (10 values = 5 landmarks × xy) for strides 8/16/32
- Anchor layout: 2 centre-point anchors per spatial location per stride
- Anchors pre-computed at init in `_build_scrfd_anchors()` — not recomputed per frame
- Decoding:
  - `x1 = cx - bbox[:,0]*stride`, `x2 = cx + bbox[:,2]*stride` (ltrb)
  - `kps_x = cx + kps[:,0::2]*stride` — use `cx[:,0:1]` (shape `M×1`) to broadcast over 5 landmarks
- Output format `[N, 15]`: `[x, y, w, h, lm0x, lm0y, ..., lm4x, lm4y, score]`
- NMS via `cv2.dnn.NMSBoxes` after decoding
- `close()`: `del self._ctx`

### `_TRTEmbedder`
- TRT engine: `face_embed.engine` (MobileFaceNet w600k, 112×112)
- Input: aligned 112×112 BGR crop → ArcFace normalisation `pixel/127.5 - 1.0` → NCHW
- Output: 512-d embedding, L2-normalised before returning
- `encode(image, face)` → calls `_align_face()` then runs TRT inference
- `close()`: `del self._ctx`

### `FaceIdentifier`
- Main public interface used by `detect_people.py` and `enroll.py`
- `identify(frame, person_box, track_id)` → result dict:
  ```python
  {"status": "verified"|"unknown"|"no_face"|"alert",
   "name": str, "confidence": float, "face_box": [x1,y1,x2,y2] or None}
  ```
  - Crops person box from frame, runs `_SCRFDDetector`, picks largest face, embeds, matches DB
  - `alert` status triggers after `NO_FACE_TIMEOUT = 2.0s` with no face detected on that track
- `detect_and_embed(image, search_box)` → `(embedding, face_box)` — used by `enroll.py`
- `close()`: calls `self._detector.close()` then `self._encoder.close()`

**Key helpers:**

| Function | Does |
|---|---|
| `_build_scrfd_anchors(h, w, strides, n)` | Pre-builds anchor centre grids for all strides. Returns `{stride: [M,2]}`. |
| `_align_face(image, face)` | Warps 5 SCRFD landmarks to ArcFace 112×112 canonical positions via `estimateAffinePartial2D`. Falls back to `getAffineTransform` on 3 points if LMEDS fails. |
| `_preprocess_aligned(aligned)` | 112×112 BGR → `[1,3,112,112]` float32, ArcFace norm. |
| `_normalize(v)` / `_normalize_batch(a)` | L2 normalisation (safe, avoids div-by-zero). |
| `_largest_face(faces)` | Returns the row with largest `w×h` from `[N,15]` array. |
| `draw_face_result(frame, result)` | Draws face box + label. Green=verified, red=unknown, orange=alert. |

**Key config at top of file:**
```python
DET_ENGINE_PATH   = ".../face_det.engine"
EMBED_ENGINE_PATH = ".../face_embed.engine"
SCRFD_INPUT_W/H   = 640, 640
SCRFD_STRIDES     = [8, 16, 32]
SCRFD_NUM_ANCHORS = 2
DET_CONF_THRESH   = 0.5
NMS_IOU_THRESH    = 0.45
SIM_THRESH        = 0.45
NO_FACE_TIMEOUT   = 2.0
```

---

## enroll.py

**Role:** Offline script. Reads enrollment photos → detects face → extracts embedding → saves `face_db.pkl`.

**Flow:**
1. Iterates `enrollments/<name>/` subdirectories
2. For each image: tries `_detect_with_augment()` — 4 transforms × 3 scales to maximise detection recall
3. Calls `fi.detect_and_embed(candidate)` for each augmented version until one succeeds
4. Stacks all good embeddings per person → `fi.db.add(name, emb_array)` → `fi.db.save()`
5. Calls `fi.close()` at end (prevents segfault)

**`_detect_with_augment(fi, image)`:**
- Transforms: identity, horizontal flip, rotate 90 CW, rotate 90 CCW
- Scales: 1.0×, 1.5×, 2.0×
- Returns first embedding found; `None` if all fail

**Re-run required when:**
- Switching face detector (SSD → SCRFD) — embeddings shift due to different alignment
- Adding new people
- Re-enrolling existing person with better photos

---

## capture_enroll.py

**Role:** Interactive camera tool to capture enrollment photos before running `enroll.py`.

**Controls:**
- `SPACE` — manual capture
- `A` — toggle auto-capture every `AUTO_INTERVAL = 2.0s`
- `Q` / `ESC` — quit

**Flow:**
- Prompts for person name → creates `enrollments/<name>/` → saves `01.jpg … 05.jpg`
- Resumes from existing count if folder already has photos
- `TARGET_COUNT = 5` photos per person
- After capture: prompts to capture another person or quit
- Prints reminder to run `enroll.py` when done

**No ML inference runs here** — pure camera capture only.

---

## test_face_detect.py

**Role:** Standalone test script. YOLO person detection + SSD face detection inside person box. No recognition, no embeddings. Used to verify face detection before wiring up the full pipeline.

**Status:** Retired — SCRFD is now used in production. This file still uses SSD and is kept for reference only.

**Key config:**
```python
FACE_REGION_TOP      = 0.0    # search from top of person box
FACE_REGION_BOTTOM   = 0.40   # upper 40% only (head region)
FACE_DETECT_INTERVAL = 1.0    # time-based throttle: detect once per second
SSD_CONF_THRESH      = 0.5
```

**Logic:**
- Runs YOLO every frame; caches last SSD face results and redraws on intervening frames
- Crops upper 40% of each person box, feeds to SSD
- Draws blue boxes (person), green boxes (face), FPS/count overlay

---

---

## hazard_monitor/hazard_monitor.ino

**Role:** Arduino Mega firmware. Continuously reads flame IR sensors, gas sensor, and ultrasonics. Drives WS2812B LED panel, buzzer, SG90 servo, and SSD1306 OLED. Streams sensor data over serial at 115200 baud. Accepts interactive serial commands.

**Hardware:**

| Component | Pin |
|---|---|
| WS2812B LED panel (24 LEDs) | D24 |
| SG90 servo | D39 |
| Passive buzzer | D41 |
| OLED SSD1306 (I2C) | SDA=20, SCL=21 (Mega) |
| Flame IR sensors (side/back) | IR_1: D48/A2, IR_2: D47/A3, IR_BACK: D45/A6 |
| Flame IR array (front, 5-element) | IR_5_1–5: D11/A8, D10/A9, D7/A10, D6/A11, D4/A12 |
| Gas sensor (MQ-2) | D44/A0 |
| Ultrasonic sensors (HC-SR04, ×4) | U1: trig D33/echo D32, U2: D28/D25, U3: D46/D13, U4: D30/D29 |

**Key structs:**

| Struct | Fields |
|---|---|
| `DualSensor` | `name`, `digitalLabel`, `analogLabel`, `digitalPin`, `analogPin` |
| `UltrasonicSensor` | `name`, `trigLabel`, `echoLabel`, `trigPin`, `echoPin` |

**Key functions:**

| Function | Does |
|---|---|
| `readAnalogAverage(pin, samples=6)` | Averages 6 ADC readings per pin to reduce noise |
| `readDistanceCm(trig, echo)` | Single HC-SR04 reading; returns `-1` on timeout or out-of-range |
| `readDistanceMedian(trig, echo)` | 3 readings → median; more stable than single shot |
| `readAllHazardSensors()` | Populates `gAnalogValues[]` and `gDigitalValues[]` for all 9 dual sensors |
| `sideFlameTriggered(value)` | `value < SIDE_FLAME_THRESHOLD (300)` → flame detected |
| `frontActive(value)` | `value > FRONT_ACTIVE_TH (80)` → weak flame signal |
| `frontStrong(value)` | `value > FRONT_STRONG_TH (400)` → strong flame signal |
| `getFrontDirectionFromCurrentReadings()` | Returns `"FRONT_CENTER"`, `"FRONT_LEFT"`, `"FRONT_RIGHT"`, `"FRONT_WIDE"`, or `"NONE"` from 5-element IR array |
| `getOverallDirectionFromCurrentReadings()` | Combines side/back IR results; falls through to front direction if no side/back flame |
| `getHazardTypeFromState(hazard, gas, dir)` | Returns `"NONE"`, `"FLAME"`, `"SMOKE_GAS"`, or `"BOTH"` |
| `updateHazardState(forceRead)` | Rate-limited (100 ms); updates `gGasAlert`, `gDirection`, `gHazardDetected`, `gHazardType` |
| `setStatusLed(color)` | Sets all 24 WS2812B LEDs to one colour via FastLED |
| `runLedTest()` | Flashes Red → Green → Blue → White → Off for startup self-test |
| `centerServo()` | Writes 90° to servo |
| `runServoSweep()` | Sweeps 90° → 140° in 2° steps (55 ms/step), then returns to centre |
| `initOled()` | I2C scan at 0x3C then 0x3D; sets `oledReady` flag |
| `drawNormalOled()` | Shows "PATROL / SAFE" in normal state |
| `drawAlertOled(visible)` | Shows "!!! ALERT !!!" + hazard type + direction; called with `visible` toggled for blink effect |
| `updateAlertOutputs()` | Called every loop — blinks LED + OLED at 50 ms if hazard, else holds white/safe |
| `printSnapshot()` | Prints all sensor readings + hazard summary to serial |
| `printHelp()` | Lists serial commands |
| `handleSerial()` | Reads serial input and dispatches `h/o/p/b/l/v/c` commands |

**Serial commands:**

| Key | Action |
|---|---|
| `h` | Print help |
| `o` | Print one sensor snapshot now |
| `p` | Toggle auto-stream on/off (streams every 500 ms when on) |
| `b` | Short buzzer beep (2200 Hz, 150 ms) |
| `l` | FastLED colour test |
| `v` | Servo sweep 90° → 140° → 90° |
| `c` | Centre servo at 90° |

**Normal vs alert behaviour:**
- Normal: LED solid white, OLED shows "PATROL / SAFE"
- Hazard detected: LED red fast blink (50 ms), OLED alert blink with type and direction

**Key config:**
```cpp
SERIAL_BAUD           = 115200
STREAM_INTERVAL_MS    = 500       // auto sensor print interval
HAZARD_SAMPLE_INTERVAL_MS = 100   // max hazard poll rate
ALERT_BLINK_MS        = 50        // LED/OLED blink period during alert
SIDE_FLAME_THRESHOLD  = 300       // IR_1/2/BACK: analog < 300 = flame
FRONT_ACTIVE_TH       = 80        // IR_5 array: > 80 = weak flame
FRONT_STRONG_TH       = 400       // IR_5 array: > 400 = strong flame
GAS_THRESHOLD         = 150       // MQ-2 analog threshold
GAS_STABLE_COUNT      = 3         // consecutive readings needed to confirm gas
OBSTACLE_LED_CM       = 20.0      // ultrasonic warning distance
ULTRA_TIMEOUT_US      = 25000     // pulseIn timeout (~430 cm max range)
SERVO_CENTER_DEG      = 90
SERVO_CLOCKWISE_DEG   = 140
NUM_LEDS              = 24
LED_BRIGHTNESS        = 96
```

---

## Servo_test/Servo_test.ino

**Role:** Standalone Arduino test sketch. Smooth servo movement between 90° (down) and 130° (up) triggered by serial commands `u` / `d`. Used to verify SG90 wiring and movement range independently of the main hazard monitor firmware.

**Status:** Reference / test only — not part of the production pipeline.

**Key config:**
```cpp
SERVO_PIN    = 39
DOWN_ANGLE   = 90    // resting / down position
UP_ANGLE     = 130   // raised / up position
UP_DELAY_MS  = 80    // ms per degree going up (slow, deliberate)
DOWN_DELAY_MS = 40   // ms per degree going down (faster return)
```

**Functions:**

| Function | Does |
|---|---|
| `smoothMove(from, to, stepDelayMs)` | Steps servo 1° at a time with `delay(stepDelayMs)` between steps — direction auto-detected |
| `setup()` | Attaches servo, writes to `DOWN_ANGLE`, prints instructions |
| `loop()` | Reads one char from serial; `u/U` → smooth up, `d/D` → smooth down; flushes any extra bytes |

**Serial commands:**

| Key | Action |
|---|---|
| `u` / `U` | Smooth move 90° → 130° (80 ms/step) |
| `d` / `D` | Smooth move 130° → 90° (40 ms/step) |

---

---

## wall_bounce/wall_bounce.ino

**Role:** Standalone Arduino Mega sketch implementing basic motor control and wall-bounce navigation. Phase 1 (motors) and Phase 2 (patrol loop) reference implementation — logic will be merged into main firmware in Phase 4.

**Status:** Active / standalone. Not yet merged with `hazard_monitor.ino`.

**Motor pins (4-motor differential drive):**

| Motor | PWM | IN1 | IN2 |
|---|---|---|---|
| M1 (left front) | D12 | D34 | D35 |
| M2 (right front) | D8 | D37 | D36 |
| M3 (left rear) | D9 | D43 | D42 |
| M4 (right rear) | D5 | A4 | A5 |

`MOTOR_SIGN[4] = {-1, +1, -1, +1}` — corrects for opposite-facing motor orientations.

**Ultrasonic pins (same layout as `hazard_monitor.ino` except U1 echo — see note below):**

| Sensor | Role | Trig | Echo |
|---|---|---|---|
| U1 | Left | D33 | D40 ⚠️ |
| U2 | Right | D28 | D25 |
| U3 | Rear | D46 | D13 |
| U4 | Front | D30 | D29 |

U1 echo pin confirmed D32 — matches `hazard_monitor.ino`. Corrected in `wall_bounce.ino` Session 3.

**Key functions:**

| Function | Does |
|---|---|
| `writeMotorRaw(pwm, in1, in2, signedPwm)` | Low-level: sets H-bridge direction + PWM magnitude |
| `writeMotorByIndex(index, signedPwm)` | Applies `MOTOR_SIGN`, `MOTOR_TRIM`, `motorRuntimeTrim` then calls `writeMotorRaw` |
| `drive4(m1, m2, m3, m4)` | Sends signed PWM to all 4 motors simultaneously |
| `driveTank(leftPwm, rightPwm)` | Left-side and right-side differential drive (applies `SIDE_BALANCE`) |
| `driveForward(pwm)` | All motors forward with `FORWARD_RATIO` scaling |
| `driveBackward(pwm)` | All motors backward with `BACKWARD_RATIO` scaling |
| `rotateLeft(pwm)` / `rotateRight(pwm)` | Tank-style spin in place |
| `stopMotors()` | Zero all motors |
| `rotateNinetyDegreesLeft/Right()` | Timed 90° turn using `TURN_LEFT_90_MS=2400` / `TURN_RIGHT_90_MS=2550` |
| `readDistanceCm(trig, echo)` | Single HC-SR04 ping; returns 999.0 on timeout or out-of-range |
| `readDistanceMedianCm(trig, echo)` | 3-reading median for stability |
| `goForwardUntilWallThenTurn()` | Core loop: poll front US → if ≤ 25 cm: stop + optional backup + turn right; else drive forward |

**Key config:**
```cpp
PWM_FORWARD        = 42      // forward cruise PWM
PWM_BACKUP         = 34      // short backup PWM
PWM_TURN_90        = 28      // rotation PWM
TURN_LEFT_90_MS    = 2400    // timed 90° left
TURN_RIGHT_90_MS   = 2550    // timed 90° right
FRONT_WALL_STOP_CM = 25.0    // stop threshold
STOP_SETTLE_MS     = 150     // post-stop settle delay
LOOP_DELAY_MS      = 30      // main loop tick
FORWARD_RATIO[4]   = {0.9650, 0.8462, 0.9650, 0.8462}
BACKWARD_RATIO[4]  = {1.0000, 0.8462, 1.0000, 0.8462}
```

---

## snapshot_writer.py

**Role:** Saves JPEG crops of detected persons and logs events to CSV. Imported by `detect_people.py` and `arduino_link.py`.

**Public functions:**

| Function | Does |
|---|---|
| `update_frame(frame)` | Store the latest BGR camera frame. Call every main loop tick. |
| `save(frame, box, category, track_id, name="")` | Crop `box` from `frame`, write to `snapshots/<category>/`, log a `<CATEGORY>_SNAP` event. Returns saved path or `""` on failure. `box=None` saves full frame. |
| `save_current(category, track_id=-1, name="")` | Like `save()` but uses the last frame from `update_frame()`. Called by `arduino_link` on `SNAP:` commands from Arduino. |
| `log_event(event, track_id, name, mode)` | Append one row to `logs/events.csv`. Creates file + header if missing. |

**Output layout:**
```
snapshots/
  person/     — <category>_<tid>_<timestamp>.jpg  (new YOLO track)
  face/       — <category>_<tid>_<timestamp>.jpg  (first SCRFD face detected)
  verified/   — <category>_<tid>_<timestamp>.jpg  (face matched DB)
logs/
  events.csv  — timestamp, event, track_id, name, mode
```

**events.csv event types:**
| Event | Logged when |
|---|---|
| `PERSON_SNAP` | New person track snapshot saved |
| `FACE_SNAP` | Face-detected snapshot saved |
| `VERIFIED_SNAP` | Verified snapshot saved |
| `FACE_VERIFIED` | Face matched DB (log row only, no image) |
| `FACE_UNKNOWN` | No face for timeout period |
| `FACE_TIMEOUT` | Person left frame without verification |

**Key config:**
```python
SNAP_DIR     = "<script_dir>/snapshots"
LOG_DIR      = "<script_dir>/logs"
LOG_FILE     = "<script_dir>/logs/events.csv"
JPEG_QUALITY = 92
```

---

## arduino_link.py

**Role:** Jetson-side USB serial bridge to Arduino Mega. Runs a background thread for continuous serial I/O. Imported by `detect_people.py`.

**Key class: `ArduinoLink`**

| Method | Does |
|---|---|
| `__init__(port, baud)` | Sets up send queue, handler list, stop event |
| `register_command_handler(handler)` | Register callback `handler(cmd: str, arg: str)` for Arduino→Jetson commands |
| `send(event)` | Queue a Jetson event string (non-blocking, thread-safe) |
| `is_connected()` | Returns current connection state |
| `start()` | Launches background daemon thread |
| `stop()` | Sets stop event, joins thread (max 5 s) |
| `_run()` | Thread body: connect, send heartbeats, flush outbound queue, read inbound lines, auto-reconnect on error |
| `_dispatch(line)` | Parses inbound line; calls handlers for `PLAY`/`MODE`/`SNAP`, prints others as Arduino debug |

**`default_command_handler(cmd, arg)`** — used by `detect_people.py`:
- `PLAY:<name>` → stub (Phase 5: `audio_player.play(arg)`)
- `MODE:<state>` → prints current Arduino mode
- `SNAP:<category>` → stub (Phase 7: `snapshot_writer.save(frame, arg)`)

**Jetson → Arduino events:**

| Event | Sent when |
|---|---|
| `PERSON_DETECTED` | New unverified track appears (once per track via `announced` set) |
| `FACE_VERIFIED:<name>` | `mark_verified()` called after DB match |
| `FACE_UNKNOWN` | `face_id` alert status (no face for 2 s) |
| `FACE_TIMEOUT` | Track disappeared from frame before verification |
| `HEARTBEAT` | Automatically every 2 s by background thread |

**Arduino → Jetson commands (received):**

| Command | Meaning |
|---|---|
| `PLAY:<name>` | Play named audio clip |
| `MODE:<state>` | Log current Arduino mode |
| `SNAP:<category>` | Save snapshot frame |

**Key config:**
```python
SERIAL_PORT          = "/dev/ttyUSB0"   # or /dev/ttyACM0
SERIAL_BAUD          = 115200
RECONNECT_DELAY_S    = 3.0
HEARTBEAT_INTERVAL_S = 2.0
```

---

## Edit History

### face_id.py
| When | Change |
|---|---|
| Session 1 | Created — `FaceDatabase`, `_SSDDetector` (SSD ResNet-10, CPU), `_TRTEmbedder` (MobileFaceNet TRT), `FaceIdentifier`, `draw_face_result`. Synthetic landmarks for alignment. |
| Session 1 | Fixed Python 3.6 type hint errors (`tuple[...]` → `Tuple[...]` from `typing`) |
| Session 1 | Fixed `cv2.data` AttributeError — switched to hardcoded cascade paths |
| Session 1 | Replaced Haar cascade with SSD ResNet-10 as face detector |
| Session 2 | **Replaced `_SSDDetector` with `_SCRFDDetector`** — TRT GPU, full 9-output anchor decoding, real 5-pt landmarks. Removed synthetic landmarks. Removed SSD model paths and `urllib` import. Updated `ensure_models()` to check TRT engine files. Added `_build_scrfd_anchors()`. |
| Session 2 | Fixed landmark broadcast bug: `anchors[:,0]` → `anchors[:,0:1]` for `(M,1)` shape |
| Session 2 | Fixed `FaceIdentifier.close()` — added `self._detector.close()` (was only closing encoder) |

### detect_people.py
| When | Change |
|---|---|
| Session 1 | Created — YOLO TRT inference, `PersonTracker`, face ID loop, `draw()` |
| Session 1 | Added `track_names` dict; updated `draw()` to show person name instead of "verified" |
| Session 2 | Added `face_id.close()` after main loop to prevent segfault on exit |
| Session 3 | **Phase 3:** Imported `ArduinoLink` + `default_command_handler` from `arduino_link.py`. Added `ARDUINO_PORT` config. Added `announced` set for per-track `PERSON_DETECTED` dedup. Wired all 4 serial events: `PERSON_DETECTED` (new track), `FACE_VERIFIED:<name>` (on verify), `FACE_UNKNOWN` (alert status), `FACE_TIMEOUT` (track vanished). Added `link.stop()` at exit. |
| Session 3 | **Phase 7:** Imported `snapshot_writer`. Added `face_snapped` set. Added `snapshot_writer.update_frame(frame)` each tick. Save: `person` crop on new track, `face` crop on first face detection (status verified/unknown), `verified` crop + log on match. Log `FACE_TIMEOUT` and `FACE_UNKNOWN` events. Changed new-track loop to `items()` to access box. |
| Session 4 | **Bug fix A:** Added `UNKNOWN_CONFIRM_COUNT=3` constant and `face_unknown_count{}` dict. `FACE_UNKNOWN` now requires 3 consecutive "unknown" frames — prevents false SECURITY_ALERT when ArcFace misses a single frame while the enrolled person is still present. Count resets on "verified" or non-unknown status. |
| Session 4 | **Bug fix B:** Changed vanish detection from `set(active_tracks.keys())` to `set(tracker.tracks.keys())`. `active_tracks` only holds `gone==0` tracks; a single missed detection frame was incorrectly sending `FACE_TIMEOUT`, blocking the Arduino servo scan trigger. `tracker.tracks` holds all IDs until fully pruned (gone > max_disappeared=45). |
| Session 4 | Changed `ARDUINO_PORT` default from `/dev/ttyUSB0` to `/dev/ttyACM0` (genuine Arduino Mega uses ATmega16U2 → ACM device). Replaced `try/except KeyboardInterrupt` with SIGINT flag (`_shutdown`) + `os._exit(0)` to prevent pycuda core dump on exit. |
| Session 4 | Added `_send(link, msg, note)` helper — wraps `link.send()` with clean `[Jetson→Arduino]` terminal print. Added `last_face_status` dict to suppress repeated face-status prints (only prints on change). Replaced all bare `link.send()` calls with `_send()`. Cleaned startup messages. |
| Session 4 | Added `ANNOUNCE_CONFIRM_FRAMES=24` gate: a new track must be continuously active for 24 consecutive frames before it enters the `announced` set and appears in STATUS packets. Prevents ghost re-ID tracks from triggering false verification cycles. `announce_count{}` dict tracks per-track frame count. |
| Session 4 | **STATUS-packet protocol (replaces individual events):** Removed individual `_send()` calls for `PERSON_DETECTED`, `FACE_VERIFIED:<name>`, `FACE_UNKNOWN`, `FACE_TIMEOUT`. Replaced with a `STATUS:<payload>` packet built and sent every face-ID cycle (~3 Hz). Payload is comma-separated `T{tid}:SCANNING`, `T{tid}:VERIFIED:{name}`, or `T{tid}:UNKNOWN` entries for all tracks in `announced`; `CLEAR` when none. Added `confirmed_unknown` set (replaces `face_unknown_sent`), `verified_played` set (one-shot dedup for snapshot/log). Cleaned up vanish handler: tracks removed from all state when pruned from `tracker.tracks`; no explicit FACE_TIMEOUT event needed. STATUS is only printed to terminal when payload changes (`last_status` dedup). |

### enroll.py
| When | Change |
|---|---|
| Session 1 | Created — `_detect_with_augment()`, `enroll_all()`, calls `fi.close()` at end |
| Session 1 | Fixed method name: `_extract_embedding` → `detect_and_embed` |

### capture_enroll.py
| When | Change |
|---|---|
| Session 1 | Created — interactive photo capture, auto/manual modes, GStreamer pipeline |

### test_face_detect.py
| When | Change |
|---|---|
| Session 1 | Created — YOLO + Haar test |
| Session 1 | Replaced Haar with SSD ResNet-10 |
| Session 1 | Added upper-40% crop + 1-second time-based throttle to reduce lag |

### SESSION_NOTES.md
| When | Change |
|---|---|
| Session 1 | Created — environment, file structure, pipeline, TRT build commands, errors, face detection methods, config values, enrollment workflow |
| Session 2 | Updated pipeline to reflect SCRFD replacing SSD |
| Session 2 | Updated face detection methods table — SCRFD marked as current |
| Session 2 | Added `Failed to create CaptureSession` / `nvargus-daemon` error entry |
| Session 2 | Updated segfault fix — two-part fix (detector + encoder close) |
| Session 2 | Updated "Where Things Were Left Off" — frame drop resolved, SCRFD confirmed working |

### hazard_monitor/hazard_monitor.ino
| When | Change |
|---|---|
| Session 2 | Renamed from `stephen_code/stephen_code.ino` → `hazard_monitor/hazard_monitor.ino`. Arduino Mega firmware: WS2812B 24-LED panel, SG90 servo, passive buzzer, 4× HC-SR04 ultrasonic, 9-element IR flame sensor array, MQ-2 gas sensor, SSD1306 OLED. Serial 115200 baud. |
| Session 3 | Full detail documented in CODE_NOTES.md (pins, thresholds, all functions, serial commands, alert behaviour). No code changes. |
| Session 3 | **Phase 3:** Replaced single-char `handleSerial()` with line-buffered version. Added `dispatchSingleChar()` (preserves h/o/p/b/l/v/c), `dispatchJetsonLine()` (parses `PERSON_DETECTED`, `FACE_VERIFIED:<name>`, `FACE_UNKNOWN`, `FACE_TIMEOUT`, `HEARTBEAT`). Added forward declarations for both. |
| Session 3 | **Phase 4:** Merged motor control from `wall_bounce.ino` (motor pins, drive functions, tuned ratios). Added `RobotMode` enum, `PatrolSubState` enum, `MotorPins` struct. Added mode globals. Added `enterPatrol/FireAlert/Verification/SecurityAlert()`, `runPatrol/FireAlert/Verification/SecurityAlert()`, `runCurrentMode()`. Added `drawVerificationOled()`, `drawSecurityAlertOled()`. Filled in `dispatchJetsonLine()` stubs. Updated `setup()` (adds `setupMotorPins`, `enterPatrol`). Updated `loop()` to call `updateHazardState` + `runCurrentMode` instead of `updateAlertOutputs`. Fire: 2200 Hz buzzer. Security: 3500 Hz buzzer. |

### wall_bounce/wall_bounce.ino
| When | Change |
|---|---|
| Session 3 | Created. 4-motor differential drive + wall-bounce navigation. Phase 1 (motors) and Phase 2 (patrol) implementation. Standalone file — merge into main firmware in Phase 4. |

### arduino_link.py
| When | Change |
|---|---|
| Session 3 | Created. Jetson-side USB serial bridge. Background thread, auto-reconnect, heartbeat, `send()`/`register_command_handler()`. Wired into `detect_people.py`. |
| Session 3 | **Phase 5:** Imported `audio_player`. Updated `default_command_handler` — `PLAY:` now calls `audio_player.play(arg)` instead of stub. |
| Session 3 | **Phase 7:** Imported `snapshot_writer`. Updated `default_command_handler` — `SNAP:` now calls `snapshot_writer.save_current(arg)` instead of stub. |

### hazard_monitor/hazard_monitor.ino (Phase 8 additions)
| When | Change |
|---|---|
| Session 3 | **Phase 8:** Added `BT_BAUD=9600` config. Added `Serial1.begin(BT_BAUD)` in `setup()`. Added `btSend()`, `btSendFireAlert()`, `btSendSecurityAlert()` + forward declarations. Called `btSendFireAlert()` from `enterFireAlert()`, `btSendSecurityAlert()` from `enterSecurityAlert()`. JSON format: fire=`{"type":"fire","subtype":"...","dir":"...","ts":...}`, intruder=`{"type":"intruder","ts":...}`. |
| Session 4 | **Bug fix:** `runVerification()` was calling `enterSecurityAlert()` unconditionally at the end of the servo scan, before reading the serial buffer. During ~8.5 s of blocking delays (`delay()`) `handleSerial()` never ran, so `FACE_VERIFIED` accumulated unread. Fix: after `centerServo()`, call `handleSerial()` to flush buffer; if `gMode` changed (verified), return; otherwise reset `gModeStartMs` and return — post-scan VERIF_TIMEOUT_MS window gives Jetson time to respond. Second `if` block now guarded with `gVerifServoScanned`. |
| Session 4 | Set `streamEnabled = false` by default — stops 500 ms sensor-dump spam on the serial line. Press 'p' in Arduino Serial Monitor to re-enable for sensor debugging. |
| Session 4 | Added `printHazardTrigger()` — called once on `enterFireAlert()`. Prints only the sensors that exceeded their threshold (side/back IR < SIDE_FLAME_THRESHOLD, front IR > FRONT_ACTIVE_TH, gas >= GAS_THRESHOLD) with raw analog values and the applicable threshold, so false-trigger thresholds can be tuned. Added `frontCount=N/FRONT_STABLE_COUNT` to the summary line. |
| Session 4 | **Front IR debounce:** Added `FRONT_STABLE_COUNT=8`, `gFrontCount`, `gFrontAlert`. `updateHazardState()` now counts consecutive reads where any front sensor exceeds `FRONT_ACTIVE_TH`; only sets `gFrontAlert` after 8 consecutive reads (~800 ms). If direction resolves to a FRONT_* string but `gFrontAlert` is false, direction is overridden to "NONE" — prevents a single ambient-IR spike (body heat, lighting) from triggering FIRE_ALERT. Side/back sensors are unaffected. |
| Session 4 | **STATUS-packet protocol (replaces individual events):** Removed `PERSON_DETECTED`, `FACE_VERIFIED`, `FACE_UNKNOWN`, `FACE_TIMEOUT` individual event handlers from `dispatchJetsonLine()`. Replaced with a single `STATUS:<payload>` handler that routes to new `handleStatusPacket()`. Added `playVerifiedNames()` to extract and play `PLAY:verified_<name>` for each verified person when exiting VERIFICATION. Enlarged `sLineBuf` from 80 to 128 bytes to fit STATUS packets. Added forward declarations for both new functions. Removed `gPendingCount` (superseded by STATUS approach). |
| Session 4 | Added `waitOrFireAlert(ms)`: replaces bare `delay()` calls inside the Phase 1 servo scan block. Polls `updateHazardState()` every 20 ms; if `gHazardDetected` becomes true mid-scan, calls `enterFireAlert()` and returns `true` so `runVerification()` can immediately return. This makes fire detection responsive during the entire ~7-second blocking scan window. All five delays in Phase 1 (audio wait, forward sweep steps, hold, return sweep steps, settle) now use this helper. |
| Session 4 | `SERVO_CENTER_DEG` changed 90→100. Verification return sweep replaced `centerServo()` snap-back with a symmetric step loop (same `SERVO_STEP_DEG`/`SERVO_STEP_DELAY_MS` as the forward sweep) so 130→100 is as slow as 100→130. Added `gPreFireMode` global. `enterFireAlert()` saves current mode to `gPreFireMode` before transitioning. `runVerification()` checks `gHazardDetected` at top — if fire detected, calls `enterFireAlert()` and returns. Added `gVerifPendingResolution` bool: set true on first SCANNING packet in VERIFICATION, cleared false when all resolve to VERIFIED or on entering patrol/security/verification. `handleStatusPacket()` CLEAR branch: if `gVerifPendingResolution` → `enterSecurityAlert()` ("ran away"), else `enterPatrol()`. Added `resumeVerification()`: lightweight re-entry after fire interrupt — skips audio/scan (sets `gVerifServoScanned=true`), re-attaches servo, re-centers, resets timeout, does NOT reset `gVerifPendingResolution` so CLEAR still triggers alert if person left during fire. `runFireAlert()` now calls `resumeVerification()` instead of `enterVerification()` when `gPreFireMode==MODE_VERIFICATION`. |
| Session 4 | Fixed servo twitching during FIRE_ALERT and SECURITY_ALERT: `FastLED.show()` disables interrupts ~720 µs every 50 ms for LED blinking, corrupting the Servo library's 20 ms PWM pulse. Fix: `testServo.detach()` in `enterFireAlert()`; `testServo.attach(SERVO_PIN)` at start of `enterVerification()` and `enterSecurityAlert()` (with immediate detach again after centering in security alert). Also increased `VERIF_SERVO_TRIGGER_MS` 3000→6000 ms so `detected_person.wav` finishes before `approach_camera.wav` starts. |

### audio_player.py
| When | Change |
|---|---|
| Session 4 | `default_command_handler`: MODE now prints a separator line for visibility; PLAY prints `[Arduino→Jetson] PLAY:<name>`. `_dispatch`: non-PLAY/MODE/SNAP lines print with `[Arduino]` prefix and blank lines suppressed. |
| Session 3 | Created. Non-blocking `aplay` subprocess wrapper. `play(name)` kills previous clip then starts new one. Auto-generates `verified_<name>.wav` via pico2wave/espeak on first use. `APLAY_DEVICE` config for non-default USB speaker. |

### generate_audio.py
| When | Change |
|---|---|
| Session 4 | Created. Generates `alert_fire.wav`, `alert_intruder.wav`, `detected_person.wav`, `approach_camera.wav` into `audio/` using espeak+sox at 48 kHz. Run once on Jetson before starting main program. |

### snapshot_writer.py
| When | Change |
|---|---|
| Session 3 | **Phase 7:** Created. `update_frame()`, `save()`, `save_current()`, `log_event()`. Saves JPEG crops to `snapshots/{person,face,verified}/`. Appends to `logs/events.csv` (auto-creates header). `JPEG_QUALITY=92`. |

### bt_receiver.py
| When | Change |
|---|---|
| Session 3 | **Phase 8:** Created. Connects to HC-05 paired BT port, reads JSON alert lines, pretty-prints with local timestamp. Auto-reconnects on disconnect. Set `BT_PORT` before running. |

### audio/generate_clips.sh
| When | Change |
|---|---|
| Session 3 | Created. Generates `alert_fire.wav`, `alert_intruder.wav`, `detected_person.wav`, `approach_camera.wav`, `verified.wav` using pico2wave or espeak. Run once on Jetson. |

### Servo_test/Servo_test.ino
| When | Change |
|---|---|
| Session 3 | Added to CODE_NOTES.md. Standalone servo test sketch — smooth 90° ↔ 130° sweep via `u`/`d` serial commands on D39. Reference only. |

### CODE_NOTES.md
| When | Change |
|---|---|
| Session 2 | Created |
| Session 3 | Added `Servo_test/Servo_test.ino` to file index and full detail section. Expanded `hazard_monitor/hazard_monitor.ino` section with full pin table, all functions, serial commands, config values, and alert behaviour. |
| Session 3 | Added `wall_bounce/wall_bounce.ino` and `arduino_link.py` — file index entries, full detail sections, edit history rows. |
