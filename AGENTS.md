# AGENTS.md — Patrol Pro

Guidance for Codex agents working in this repo. Read this first.

## What this project is
Patrol Pro — autonomous security patrol robot. Jetson Nano handles vision (YOLO person detection + SCRFD/MobileFaceNet face recognition). Arduino Mega is the **brain**, running the mode state machine and driving all hardware outputs (LEDs, OLED, buzzer, servo, motors, HC-05).

## Read these three documents before touching code
| File | What it tells you |
|---|---|
| `personal_notes.md` | Project plan, state machine, phases, open decisions. **The source of truth for "what we're building next."** |
| `SESSION_NOTES.md` | Environment (Jetson, Python 3.6, TRT 8.0, OpenCV 4.1.1), known errors + fixes, enrollment workflow, TRT engine build commands. |
| `CODE_NOTES.md` | Per-file summary of every source file (classes, functions, configs). Read instead of re-reading full source. |

## Hard rules
1. **English only** — no Korean (or any other non-English) in code, comments, or strings.
2. **Arduino is the brain** — do not move mode state into Jetson. Jetson sends events; Arduino decides.
3. **Do not break the existing pipeline.** SCRFD + MobileFaceNet TRT is confirmed working. Before editing `face_id.py` or `detect_people.py`, check CODE_NOTES.md for the edit history and rationale.
4. **Keep existing pin assignments** from `hazard_monitor/hazard_monitor.ino` and `Servo_test/Servo_test.ino`.
5. **Update CODE_NOTES.md** whenever a source file is edited or added (add an entry to that file's Edit History table).
6. **Update SESSION_NOTES.md** when a new environment issue / error is discovered and fixed.

## When adding new work
- New Python module on Jetson → add to `CODE_NOTES.md` File Index + create a per-file section.
- New Arduino file or major feature → same.
- New open design decision encountered → add to `personal_notes.md` §7 with options + recommendation.

## Things to never do
- Never call `git add -A` / `git add .` — stage files explicitly.
- Never commit unless the user asks.
- Never skip git hooks (`--no-verify` etc.).
- Never `--amend` after a failed commit; make a new commit.
