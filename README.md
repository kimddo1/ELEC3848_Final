# Patrol Pro

An autonomous security patrol robot built for HKU **ELEC3848** (Integrated Design Project).
The robot patrols an area, detects people, verifies identity using on-device face recognition,
and raises alerts for intruders or fire/gas hazards.

**Stack:** Jetson Nano (TensorRT, OpenCV, Python 3.6) · Arduino Mega (C++) · YOLO11 · SCRFD · MobileFaceNet

---

## What it does

- Detects people in real time on the Jetson using YOLO11n (TensorRT, FP16)
- Recognizes faces against an enrolled database via SCRFD + MobileFaceNet
- Drives autonomously with ultrasonic obstacle avoidance and two-speed approach
- Detects fire and gas via IR flame sensors and an MQ-2, with debounced thresholds
- Orients toward detected hazards, captures a snapshot, and resumes verification once the hazard clears
- Handles multiple people on screen simultaneously — all must be verified or the robot raises an intruder alert
- Streams live video, snapshots, and an event log over a built-in web dashboard

---

## System architecture

```
   Camera → YOLO11n (TRT) → Multi-person tracker
                                    │
                          SCRFD + MobileFaceNet (TRT)
                                    │
                          STATUS packets (USB serial)
                                    ▼
            Arduino Mega — mode state machine, motor & sensor I/O
                                    │
              Motors · Servo · WS2812B · OLED · Buzzer · HC-05 BT
```

The Jetson handles all perception. The Arduino owns the state machine and every hardware output.
Communication is a compact `STATUS:` packet protocol that lets the Arduino make decisions on full
context rather than reacting to one event at a time.

---

## Technical highlights

- **Multi-person verification protocol** — Jetson sends full-context state snapshots, not events.
  Solves a race condition where one verified + one unknown person on screen could exit verification
  prematurely.
- **Fire alert preemption** — A `STOP_AUDIO` command and an interruptible delay let fire alerts
  cleanly preempt an in-progress face scan, then resume verification afterward.
- **Debounced hazard detection** — Front IR sensors require 8 consecutive readings above
  threshold (~800 ms sustained) to trigger, eliminating ambient false positives.
- **Track confirmation gating** — A 24-frame announce gate filters ghost re-identification artifacts
  from the IoU tracker before they trigger audio cues.
- **Departure detection** — A `FACE_TIMEOUT` event fires when a still-scanning person leaves the
  frame, distinguishing a clean exit from someone running away.
- **On-demand TTS** — Verified-name audio clips are generated automatically the first time they
  are needed (`Hello, David and Alice.`).

---

## Repository layout

| File / directory | Purpose |
|---|---|
| `detect_people.py` | Jetson main loop — detection, tracking, face ID, serial I/O |
| `face_id.py` | SCRFD + MobileFaceNet TensorRT pipeline + face database |
| `arduino_link.py` | Background serial bridge between Jetson and Arduino |
| `audio_player.py` | Non-blocking WAV playback with on-demand TTS generation |
| `snapshot_writer.py` | Saves event snapshots and CSV log |
| `web_dashboard.py` | HTTP dashboard — MJPEG stream, snapshots, event log |
| `enroll.py`, `capture_enroll.py` | Offline and live face enrollment |
| `generate_audio.py` | Generates all required audio clips |
| `hazard_monitor/hazard_monitor.ino` | Arduino firmware — full state machine and hardware I/O |
| `audio/` | Pre-generated WAV clips |
| `Required_Func/` | Standalone Arduino sketches used during motor and sensor calibration |

---

## Quick start

**Arduino**
1. Open `hazard_monitor/hazard_monitor.ino` in the Arduino IDE
2. Install libraries: FastLED, Adafruit SSD1306, Adafruit GFX
3. Upload to Arduino Mega — robot enters patrol mode on boot

**Jetson**
```bash
# 1. Build TRT engines (yolo11n.engine, face_det.engine, face_embed.engine)
#    and place at the paths defined at the top of face_id.py

# 2. Enroll authorized people
python3 enroll.py

# 3. Generate audio clips (one-time)
sudo apt install espeak sox alsa-utils
python3 generate_audio.py

# 4. Run the main pipeline
python3 detect_people.py

# Optional — web dashboard at http://<jetson-ip>:8080
python3 web_dashboard.py
```

Defaults assume Arduino on `/dev/ttyUSB0`. Adjust `ARDUINO_PORT` in `detect_people.py` if needed.

---

## Hardware

| Component | Notes |
|---|---|
| Jetson Nano 4 GB | TensorRT 8.0, CUDA 10.2, OpenCV 4.1.1 |
| Arduino Mega 2560 | Brain — runs the mode state machine |
| IMX219 CSI camera | 1280×720 input, 320×320 inference |
| 4× DC motors + driver | Two-speed patrol drive |
| SG90 servo | Camera tilt for face scanning |
| WS2812B LED ring (24) | Mode status indication |
| SSD1306 OLED 128×64 | I²C status display |
| 8× IR flame sensors + MQ-2 | Hazard detection (front array, sides, rear, gas) |
| 4× HC-SR04 ultrasonic | Patrol obstacle avoidance |
| HC-05 Bluetooth | Remote alert receiver |
| Passive buzzer + USB speaker | Audio alerts |

---

## License

Released under the MIT License.
