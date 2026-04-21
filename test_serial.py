# test_serial.py
"""
Mac-side serial tester for Arduino Mega.
Simulates Jetson events so you can verify Phase 3 protocol without the Jetson.

Usage:
    pip3 install pyserial          # one-time
    python3 test_serial.py         # auto-detects port, or:
    python3 test_serial.py /dev/cu.usbmodem14101

Controls (type in terminal, press Enter):
    pd   -> send PERSON_DETECTED
    fv   -> send FACE_VERIFIED:TestUser
    fu   -> send FACE_UNKNOWN
    ft   -> send FACE_TIMEOUT
    hb   -> send HEARTBEAT
    h    -> send 'h' (Arduino help)
    o    -> send 'o' (Arduino snapshot)
    p    -> send 'p' (toggle stream)
    b    -> send 'b' (buzzer beep)
    l    -> send 'l' (LED test)
    v    -> send 'v' (servo sweep)
    c    -> send 'c' (center servo)
    q    -> quit
"""

import sys
import threading
import time
import glob
import serial


BAUD = 115200


# ── port auto-detection ───────────────────────────────────────────────────────

def find_arduino_port():
    """Return first likely Arduino Mega port on Mac, or None."""
    candidates = sorted(
        glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*")
    )
    if candidates:
        return candidates[0]
    return None


# ── reader thread ─────────────────────────────────────────────────────────────

def reader_thread(ser, stop_event):
    """Continuously print lines arriving from Arduino."""
    while not stop_event.is_set():
        try:
            if ser.in_waiting:
                raw = ser.readline()
                line = raw.decode("ascii", errors="ignore").strip()
                if line:
                    print(f"  [Arduino] {line}")
            else:
                time.sleep(0.02)
        except Exception:
            break


# ── command map ───────────────────────────────────────────────────────────────

COMMANDS = {
    "pd":  "PERSON_DETECTED",
    "fv":  "FACE_VERIFIED:TestUser",
    "fu":  "FACE_UNKNOWN",
    "ft":  "FACE_TIMEOUT",
    "hb":  "HEARTBEAT",
    # single-char Arduino interactive commands — sent as-is with newline
    "h":   "h",
    "o":   "o",
    "p":   "p",
    "b":   "b",
    "l":   "l",
    "v":   "v",
    "c":   "c",
}

HELP = """
  pd  -> PERSON_DETECTED
  fv  -> FACE_VERIFIED:TestUser
  fu  -> FACE_UNKNOWN
  ft  -> FACE_TIMEOUT
  hb  -> HEARTBEAT
  h/o/p/b/l/v/c  -> Arduino interactive commands
  q   -> quit
"""


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_arduino_port()

    if not port:
        print("No Arduino port found. Plug in the Mega and retry, or pass port as argument.")
        print("  python3 test_serial.py /dev/cu.usbmodem14101")
        sys.exit(1)

    print(f"Connecting to {port} @ {BAUD}...")
    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)
    except serial.SerialException as e:
        print(f"Failed to open port: {e}")
        sys.exit(1)

    print(f"Connected. Arduino will reset — waiting 2 s...")
    time.sleep(2.0)
    print("Ready.\n" + HELP)

    stop_event = threading.Event()
    t = threading.Thread(target=reader_thread, args=(ser, stop_event), daemon=True)
    t.start()

    try:
        while True:
            try:
                raw = input("> ").strip().lower()
            except EOFError:
                break

            if raw == "q":
                break

            if raw not in COMMANDS:
                print(f"Unknown command. Type one of: {', '.join(COMMANDS)}")
                continue

            msg = COMMANDS[raw] + "\n"
            ser.write(msg.encode("ascii"))
            print(f"  [Sent] {msg.strip()}")

    except KeyboardInterrupt:
        pass

    stop_event.set()
    ser.close()
    print("Closed.")


if __name__ == "__main__":
    main()
