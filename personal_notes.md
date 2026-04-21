# Patrol Pro — Project Plan

Autonomous security patrol robot for a 3 m × 3 m demo area.
Detects **fire/gas hazards** and **intruders** (unknown faces); responds with local alerts and a remote signal.

See `SESSION_NOTES.md` (environment, errors, TRT build) and `CODE_NOTES.md` (per-file reference) for supplements.

---

## 1. Hard Requirements (non-negotiable)

1. **Arduino Mega is the brain.** Main loop and mode state live on Arduino. Jetson is a vision sensor that reports events over USB serial.
2. **Default mode = patrol loop.** Everything else is entered on event, run for a bounded time, then returns to patrol.
3. Existing hardware pin assignments in `hazard_monitor/hazard_monitor.ino` and `Servo_test/Servo_test.ino` are fixed — reuse them.
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

### Phase 1 — Motor control (blocker)
- Decide drivetrain (2-wheel diff drive + caster? 4-wheel?). Pick motor driver (L298N / TB6612).
- Add motor pin definitions to `hazard_monitor.ino` (or split into `patrol_controller.ino` if file gets large).
- Write `driveForward(speed, ms)`, `turnLeft(deg)`, `turnRight(deg)`, `stop()`.
- Test on bench without sensors active.

### Phase 2 — Patrol loop
- Implement timed square: forward 2.5 s → turn 90° → repeat. Tune on floor.
- Add ultrasonic obstacle bail-out: if `U_front < 15 cm`, stop + back up + turn.
- Confirm existing hazard monitoring still runs inline (it already does — `updateAlertOutputs()` is non-blocking).

### Phase 3 — Serial protocol
- Add line-reader to Arduino `handleSerial()` for `PERSON_DETECTED` / `FACE_*` tokens.
- Create `arduino_link.py` on Jetson: background thread reads serial, exposes `send(event)` and `on_command(callback)`.
- Wire into `detect_people.py`:
  - On new unverified track → `send("PERSON_DETECTED")`.
  - On `mark_verified()` → `send(f"FACE_VERIFIED:{name}")`.
  - On track expiry without verification → `send("FACE_TIMEOUT")`.

### Phase 4 — Mode state machine (Arduino)
- Add `enum Mode { PATROL, FIRE_ALERT, VERIFICATION, SECURITY_ALERT }`.
- Refactor `loop()`: dispatch to `runPatrol()`, `runFireAlert()`, `runVerification()`, `runSecurityAlert()`.
- Each mode-runner is non-blocking — it advances its own timer per `loop()` tick.
- Transition rules per §3 diagram.

### Phase 5 — Audio
- Record/generate 6 .wav files:
  - `alert_fire.wav`, `alert_intruder.wav`, `detected_person.wav`, `approach_camera.wav`, `verified.wav`, (optional) `verified_<name>.wav`.
- Add `audio_player.py` on Jetson: simple wrapper around `aplay` (subprocess) keyed by name. Pre-warm by playing a silent 0.1 s on startup.
- Hook into `arduino_link.py` `on_command` for `PLAY:<name>`.

### Phase 6 — Servo scan during verification
- When Arduino enters VERIFICATION, start a 3 s timer.
- If no `FACE_VERIFIED` / `FACE_UNKNOWN` by then, trigger slow servo sweep 90° → 130° (reuse `Servo_test` logic inline).
- Return servo to 90° on state exit.

### Phase 7 — Snapshots & logging
- Add `snapshot_writer.py` on Jetson — saves frame crop + metadata.
- Call at each track transition (see §6).

### Phase 8 — Remote link
- Wire HC-05 to Arduino Serial1 (pins 18 TX / 19 RX on Mega).
- Send JSON-ish lines on entering FIRE_ALERT / SECURITY_ALERT.
- Receiver: simple Python script on remote laptop using `pyserial` over BT-serial port.

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
