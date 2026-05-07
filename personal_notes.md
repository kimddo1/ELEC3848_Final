# Patrol Pro — Project Plan

Autonomous security patrol robot for a 3 m × 3 m demo area.
Detects **fire/gas hazards** and **intruders** (unknown faces); responds with local alerts and a remote signal.

See `SESSION_NOTES.md` (environment, errors, TRT build) and `CODE_NOTES.md` (per-file reference) for supplements.

---

## 1. Hard Requirements (non-negotiable)

1. **Arduino Mega is the brain.** Main loop and mode state live on Arduino. Jetson is a vision sensor that reports events over USB serial.
2. **Default mode = patrol loop.** Everything else is entered on event, run for a bounded time, then returns to patrol.
3. Existing hardware pin assignments in `hazard_monitor/hazard_monitor.ino` and `archive/reference_only/Servo_test/Servo_test.ino` are fixed — reuse them.
4. Source code, comments, and UI strings are English only.

---

## 2. System Architecture

```
┌───────────────────────── Arduino Mega (BRAIN) ─────────────────────────┐
│  State machine: PATROL / FIRE_ALERT / VERIFICATION / SECURITY_ALERT     │
│  Drives: motors, servo, WS2812B LEDs, OLED, buzzer, MQ-2, IR array,     │
│          ultrasonics, HC-05 (remote link)                               │
│  USB-serial link ⇆ Jetson       BT-serial ⇆ remote PC                   │
└─────────────────────────────────────────────────────────────────────────┘
                                    ▲
                        USB serial  │  JSON-lines or K=V events
                                    ▼
┌────────────────────────── Jetson Nano (EYES) ──────────────────────────┐
│  detect_people.py   — YOLO11n + PersonTracker + FaceIdentifier          │
│  Audio player       — aplay/pygame, pre-recorded .wav files             │
│  Snapshot writer    — saves to snapshots/{person,face,verified}/        │
│  Sends events to Arduino on state change only (not every frame)         │
└─────────────────────────────────────────────────────────────────────────┘
```

**Why Arduino is brain:** mode state is tied to physical outputs (LED/OLED/buzzer/motors). Keeping state there avoids race conditions and makes the robot fail-safe if the Jetson crashes.

---

## 3. State Machine (Arduino)

```
            ┌─────────────────┐
            │     PATROL      │◄───────────────┐
            │  (default)      │                │
            └────┬────────┬───┘                │
                 │        │                    │
       fire/gas  │        │  person event     │ timeout (10s)
       detected  │        │  from Jetson      │ or resolved
                 ▼        ▼                    │
     ┌──────────────┐  ┌──────────────────┐   │
     │ FIRE_ALERT   │  │  VERIFICATION    │   │
     │  (10s)       │  │  (up to 8s)      │   │
     └──────┬───────┘  └────┬────────┬────┘   │
            │               │        │        │
            └───────────────┤        │        │
                            │        │        │
                   verified │        │ unknown / runaway
                            ▼        ▼        │
                            │  ┌──────────────┴───┐
                            │  │ SECURITY_ALERT   │
                            │  │  (10s)           │
                            │  └──────────────────┘
                            │
                            └──→ back to PATROL
```

**Concurrency rule (side note 1):** if both fire and person events fire within the same state window, run **FIRE_ALERT** first (life safety > security), then transition to VERIFICATION after.

---

## 4. Per-Mode Behaviour

### 4.1 PATROL
- Move around 3 m × 3 m area (see §7 for navigation options).
- Arduino polls all sensors (`updateHazardState()`) every 100 ms.
- Listens on USB serial for Jetson events.
- LED: solid white. OLED: "PATROL / SAFE".

### 4.2 FIRE_ALERT (10 s, self-terminating)
- Trigger: `gHazardDetected == true` from existing `hazard_monitor` logic.
- Outputs:
  - LED fast-blink red (50 ms, already implemented).
  - OLED: "FIRE" / "GAS" / "FIRE+GAS" + direction (already implemented).
  - Buzzer intermittent tone (new — add to `updateAlertOutputs()`).
  - USB speaker plays `alert_fire.wav` (Jetson side, triggered by serial cmd `PLAY:fire`).
  - Remote link: sends `{type:"fire", subtype:"fire|gas|both", dir:"LEFT", ts:...}`.
- Stop condition: 10 s elapsed → return to PATROL (even if still detected; re-enters next cycle).

### 4.3 VERIFICATION (up to 8 s)
Entered on Jetson serial event `PERSON_DETECTED`.
- LED: solid orange. OLED: "PERSON / DETECTED".
- USB speaker: `detected_person.wav`.
- Jetson runs face detection + recognition (existing pipeline).
- **If face not detected within 3 s:** Arduino commands servo sweep 90° → 130° (slow, from `Servo_test`) to raise camera angle; LED turns red; speaker plays `approach_camera.wav`; give 5 s grace.
- Outcomes from Jetson:
  | Jetson event | Arduino response |
  |---|---|
  | `FACE_VERIFIED:<name>` | LED green 1 s, speaker plays `verified_<name>.wav` (or generic `verified.wav`), log, return to PATROL |
  | `FACE_UNKNOWN` (after 8 s total) | enter SECURITY_ALERT |
  | `FACE_TIMEOUT` (person left frame) | enter SECURITY_ALERT (treat as runaway) |

### 4.4 SECURITY_ALERT (10 s, self-terminating)
- LED fast-blink red. OLED: "INTRUDER / DETECTED".
- Buzzer alarm pattern (different from fire — e.g. two-tone).
- Speaker plays `alert_intruder.wav`.
- Remote link: sends `{type:"intruder", snapshot:"<path>", ts:...}`.
- Return to PATROL after 10 s.

---

## 5. Jetson ↔ Arduino Serial Protocol

USB serial, 115200 baud, newline-terminated ASCII. Keep it dumb — one event per line, key:value or short token.

### Jetson → Arduino
| Event | Meaning | When |
|---|---|---|
| `PERSON_DETECTED` | YOLO saw an unverified person | On new unverified track (throttled, 1/sec max) |
| `FACE_VERIFIED:<name>` | Face matched DB above threshold | On `mark_verified()` |
| `FACE_UNKNOWN` | Face detected but no DB match after N attempts | After 5 s of unknown |
| `FACE_TIMEOUT` | Person left frame without verification | Track expired |
| `HEARTBEAT` | Jetson alive | Every 2 s |

### Arduino → Jetson
| Command | Meaning |
|---|---|
| `PLAY:<name>` | Play a named .wav (fire/intruder/verified/detected_person/approach_camera) |
| `MODE:<state>` | Inform Jetson of current mode (for logging / overlay) |
| `SNAP:<category>` | Request Jetson to save a snapshot (person/face/verified) |

**Implementation note:** on Arduino side add a small line-reader to `handleSerial()` that branches on token type. On Jetson side add a thin `arduino_link.py` module with a background thread reading serial + a queue for sending.

---

## 6. Snapshots & Logging (Jetson)

Directory layout:
```
snapshots/
  person/     # YOLO detected, new track
  face/       # SCRFD detected a face inside that track
  verified/   # Face matched DB
logs/
  events.csv  # timestamp, event, track_id, name, mode
```

Save once per track transition — not per frame. Hook into `detect_people.py` where `tracker.mark_verified()` is called, and one level above for the `person` and `face` categories.

---

## 7. Open Design Decisions — with recommendations

| # | Decision | Options | Recommendation |
|---|---|---|---|
| 1 | Patrol navigation | (a) AprilTag waypoints, (b) dead-reckoning timed square, (c) wall-follow via ultrasonics | **(b) timed square** for demo — simplest, 3 m × 3 m is tiny. Upgrade to (a) if time permits. |
| 2 | Audio source | (a) pre-recorded .wav, (b) on-device TTS (espeak/pico2wave) | **(a) pre-recorded** — deterministic, no latency, no dependency risk. TTS only for name variation (`verified_<name>`). |
| 3 | Remote link | HC-05 BT, WiFi | **HC-05** — Arduino is brain, BT module wires directly to Serial1. No extra stack. |
| 4 | Concurrent fire + person | ignore the second, queue it, or interrupt | **Queue it.** FIRE_ALERT is not interruptible; on exit, check if a pending `PERSON_DETECTED` is still fresh (< 3 s old) and enter VERIFICATION. |
| 5 | Motor hardware | Not yet chosen / wired | **Blocking unknown.** See §8 Phase 1. |

---

## 8. Development Phases

### Phase 1 — Motor control ✅ DONE
Implemented in `archive/reference_only/wall_bounce/wall_bounce.ino`.
- 4-motor differential drive (M1–M4) with PWM, per-motor sign correction, scaling ratios, and trim.
- Functions: `driveForward`, `driveBackward`, `rotateLeft`, `rotateRight`, `stopMotors`.
- Tuned values: `PWM_FORWARD=42`, `TURN_RIGHT_90_MS=2550`, `TURN_LEFT_90_MS=2400`.
- Motor pins: M1 PWM=12/IN1=34/IN2=35 · M2 PWM=8/IN1=37/IN2=36 · M3 PWM=9/IN1=43/IN2=42 · M4 PWM=5/IN1=A4/IN2=A5.
- ⚠️ Merge into main firmware (Phase 4) — currently standalone file.

### Phase 2 — Patrol loop ✅ DONE
Implemented in `archive/reference_only/wall_bounce/wall_bounce.ino`.
- Wall-bounce navigation: drive forward → front ultrasonic poll (`FRONT_WALL_STOP_CM=25`) → stop → optional backup if < 12 cm → rotate 90° right → repeat.
- Non-blocking per loop tick via `delay(LOOP_DELAY_MS=30)`.
- U1 echo pin confirmed D32 — corrected in `wall_bounce.ino`.
- ⚠️ Hazard sensor polling not yet integrated — will merge in Phase 4.

### Phase 3 — Serial protocol ✅ DONE
- `arduino_link.py` created: background thread reader/writer, auto-reconnect, heartbeat every 2 s, `send()` / `register_command_handler()` / `start()` / `stop()`.
- `detect_people.py` wired up:
  - New unverified track → `send("PERSON_DETECTED")` (once per track, tracked via `announced` set).
  - `mark_verified()` → `send("FACE_VERIFIED:<name>")`.
  - Face not found for 2 s (alert status) → `send("FACE_UNKNOWN")`.
  - Track disappeared before verified → `send("FACE_TIMEOUT")`.
  - `link.stop()` called at exit.
- `hazard_monitor.ino` updated: `handleSerial()` now buffers full lines; `dispatchSingleChar()` preserves all existing commands; `dispatchJetsonLine()` parses `PERSON_DETECTED`, `FACE_VERIFIED:<name>`, `FACE_UNKNOWN`, `FACE_TIMEOUT`, `HEARTBEAT` with TODO stubs for Phase 4 actions.
- Audio (`PLAY:`) and snapshot (`SNAP:`) Arduino→Jetson commands are stubbed in `default_command_handler()` — wired in Phases 5 and 7 respectively.

### Phase 4 — Mode state machine ✅ DONE
- Add `enum Mode { PATROL, FIRE_ALERT, VERIFICATION, SECURITY_ALERT }`.
- Refactor `loop()`: dispatch to `runPatrol()`, `runFireAlert()`, `runVerification()`, `runSecurityAlert()`.
- Each mode-runner is non-blocking — it advances its own timer per `loop()` tick.
- Transition rules per §3 diagram.

### Phase 5 — Audio ✅ DONE
- `audio_player.py` created: non-blocking `aplay` wrapper, kills previous clip on new play, auto-generates `verified_<name>.wav` via pico2wave/espeak on first use.
- `audio/generate_clips.sh` created: generates all 5 fixed clips using pico2wave or espeak. Run once on Jetson: `bash audio/generate_clips.sh`.
- `arduino_link.py` updated: `default_command_handler` now calls `audio_player.play(arg)` on `PLAY:` commands.
- If USB speaker is not ALSA default, set `APLAY_DEVICE = "plughw:1,0"` (or correct index from `aplay -l`) in `audio_player.py`.

### Phase 6 — Servo scan during verification ✅ DONE
- After 3 s no face: Arduino sets LED red, sends `PLAY:approach_camera`, blocks 3000 ms (audio duration).
- Sweeps servo 90° → 130° (2°/step, 55 ms/step).
- LED white, holds 3 s at 130°.
- `centerServo()` → immediately calls `enterSecurityAlert()` — no extra wait.
- Constants: `VERIF_SERVO_TRIGGER_MS=3000`, `VERIF_AUDIO_WAIT_MS=3000`, `VERIF_SERVO_HOLD_MS=3000`, `VERIF_SERVO_MAX_DEG=130`.

### Phase 7 — Snapshots & logging ✅ DONE
- `snapshot_writer.py` created: `update_frame()`, `save()`, `save_current()`, `log_event()`.
- Saves JPEG crops to `snapshots/{person,face,verified}/`. Creates dirs on first use.
- Logs to `logs/events.csv` (auto-creates header on first write).
- Wired into `detect_people.py`:
  - New track → `save(frame, box, "person", tid)`.
  - First frame with face detected (status "verified" or "unknown") → `save(frame, box, "face", tid)`. Tracked via `face_snapped` set.
  - Verified → `save(frame, box, "verified", tid, name)` + `log_event("FACE_VERIFIED", ...)`.
  - FACE_TIMEOUT → `log_event("FACE_TIMEOUT", ...)`.
  - FACE_UNKNOWN → `log_event("FACE_UNKNOWN", ...)`.
- `arduino_link.py` updated: `SNAP:<category>` now calls `snapshot_writer.save_current(arg)` (uses last frame from `update_frame`).

### Phase 8 — Remote link ✅ DONE
- HC-05 wires to Arduino Serial3: HC-05 TX → Mega pin 15 (RX3), HC-05 RX → Mega pin 14 (TX3) via 1kΩ+2kΩ divider (3.3 V logic). VCC → 5 V, GND → GND.
- `Serial3.begin(9600)` in `setup()`.
- `btSendFireAlert()` — called from `enterFireAlert()`. Sends: `{"type":"fire","subtype":"FLAME|SMOKE_GAS|BOTH","dir":"<dir>","ts":<ms>}`
- `btSendSecurityAlert()` — called from `enterSecurityAlert()`. Sends: `{"type":"intruder","ts":<ms>}`
- `btSend(msg)` helper mirrors the line to `Serial` (USB) as `[BT->] ...` for debugging.
- `bt_receiver.py` created: connects to paired HC-05 BT serial port, parses JSON lines, prints formatted alert. Auto-reconnects on disconnect.
- Pair HC-05 first (PIN 1234 or 0000), set `BT_PORT` in `bt_receiver.py`, run `python3 bt_receiver.py`.

### Phase 9 — Polish & demo
- Re-enroll Noah with 5 clean photos (pending from Session 2).
- Full dry-run: start robot → walk in → get verified → walk away. Light a match → fire alert.
- Record demo video.

---

## 9. File Plan (to create)

| New file | Location | Purpose |
|---|---|---|
| `arduino_link.py` | `yolo11_jetson/` | USB serial bridge, background reader + sender |
| `audio_player.py` | `yolo11_jetson/` | `aplay` wrapper keyed by name |
| `snapshot_writer.py` | `yolo11_jetson/` | Save frame crops to `snapshots/<cat>/` |
| `audio/*.wav` | `yolo11_jetson/audio/` | Pre-recorded alert/verification clips |

Arduino: extend `hazard_monitor.ino` with mode state machine, motor control, and a line-reader. If it grows past ~1000 lines, split into `patrol_brain/patrol_brain.ino` with tabs for `hazard.ino`, `motors.ino`, `serial_link.ino`, `oled.ino`.

---

## 10. Testing Checklist

- [ ] Motor forward/turn/stop behaves predictably on flat surface
- [ ] Timed square closes within ~20 cm of start after 4 sides
- [ ] Ultrasonic bail-out triggers before collision
- [ ] `PERSON_DETECTED` reaches Arduino within 500 ms of YOLO detection
- [ ] VERIFICATION enters on event, exits cleanly on `FACE_VERIFIED`
- [ ] Servo scan triggers after 3 s no-face, returns to 90° after state exit
- [ ] FIRE_ALERT runs for exactly 10 s, does not block motor loop
- [ ] SECURITY_ALERT enters after 8 s unverified
- [ ] Remote PC receives BT message within 2 s of alert
- [ ] Audio plays on each transition with < 500 ms latency
- [ ] Snapshots land in correct folder per category
- [ ] Full patrol → verify → patrol cycle runs for 5 minutes without hang

---

## 11. Deferred / Nice-to-have

- Battery monitoring + auto-return-to-dock
- Face enrollment GUI (current `capture_enroll.py` is CLI)
- Web dashboard instead of raw BT messages
- Multi-person handling during verification (currently one track at a time)
- Night mode / IR illuminator
