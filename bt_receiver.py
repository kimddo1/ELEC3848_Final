# bt_receiver.py
"""
Remote receiver for Patrol Pro Bluetooth alerts (HC-05).

Alert formats sent by Arduino:
    {"type":"fire","subtype":"FLAME|SMOKE_GAS|BOTH","dir":"LEFT","ts":12345}
    {"type":"intruder","ts":12345}

Platform notes
--------------
macOS   : socket.AF_BLUETOOTH is NOT available. Use an Android phone with
          "Serial Bluetooth Terminal" app (pair HC-05, connect, alerts appear live).
          OR run this script on a Linux machine / the Jetson (with USB BT dongle).

Linux   : set USE_SERIAL_PORT = False, set BT_MAC + BT_CHANNEL. Requires
          socket.AF_BLUETOOTH which is supported natively.

Windows : set USE_SERIAL_PORT = True, set BT_PORT to the outgoing COM port
          (Device Manager → Ports → HC-05). pip install pyserial first.

Demo fallback (no phone/Linux)
------------------------------
btSend() on Arduino mirrors every BT message to USB serial as:
    [BT->] {"type":"fire",...}
So archive/reference_only/test_serial.py on the Mac will show the alert data in its output.
"""

import socket
import json
import time
from datetime import datetime

# ── config ────────────────────────────────────────────────────────────────────
BT_MAC     = "00:25:11:02:83:24"  # HC-05 MAC — colons, not dashes
BT_CHANNEL = 1                    # SPP is always channel 1 on HC-05

# Windows / Linux fallback: set this True and set BT_PORT to the COM port
USE_SERIAL_PORT = False
BT_PORT         = "COM5"          # e.g. COM5 on Windows, /dev/rfcomm0 on Linux
BT_BAUD         = 9600

RECONNECT_DELAY_S = 5.0
# ─────────────────────────────────────────────────────────────────────────────

ALERT_LABELS = {
    "fire":     "FIRE / GAS ALERT",
    "intruder": "INTRUDER ALERT",
}


def format_alert(data: dict) -> str:
    """Pretty-print a parsed alert dict."""
    kind  = data.get("type", "unknown")
    label = ALERT_LABELS.get(kind, f"ALERT [{kind}]")
    parts = [f"*** {label} ***"]

    if kind == "fire":
        parts.append(f"  subtype  : {data.get('subtype', '?')}")
        parts.append(f"  direction: {data.get('dir', '?')}")
    elif kind == "intruder":
        parts.append("  intruder detected — face was not verified")

    parts.append(f"  robot ts : {data.get('ts', '?')} ms")
    return "\n".join(parts)


def handle_line(line: str):
    """Parse one received line and print it."""
    if not line:
        return
    ts = datetime.now().strftime("%H:%M:%S")
    try:
        data = json.loads(line)
        print(f"\n[{ts}] {format_alert(data)}")
    except json.JSONDecodeError:
        print(f"[{ts}] raw: {line}")


# ── RFCOMM socket receiver (macOS / Linux) ────────────────────────────────────

def run_socket():
    """Connect via raw RFCOMM socket — no /dev/cu.* required on macOS."""
    print(f"[BT] connecting to {BT_MAC} channel {BT_CHANNEL} ...")
    print("Press Ctrl-C to quit.\n")

    while True:
        sock = None
        try:
            sock = socket.socket(socket.AF_BLUETOOTH,
                                 socket.SOCK_STREAM,
                                 socket.BTPROTO_RFCOMM)
            sock.settimeout(10.0)
            sock.connect((BT_MAC, BT_CHANNEL))
            sock.settimeout(None)
            print("[BT] connected.\n")

            buf = ""
            while True:
                chunk = sock.recv(256).decode("ascii", errors="ignore")
                if not chunk:
                    raise ConnectionResetError("connection closed by remote")
                buf += chunk
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    handle_line(line.strip())

        except (ConnectionResetError, OSError) as exc:
            print(f"\n[BT] connection lost: {exc}")
            print(f"[BT] retrying in {RECONNECT_DELAY_S} s...")
            time.sleep(RECONNECT_DELAY_S)

        except KeyboardInterrupt:
            print("\n[BT] stopped.")
            break

        finally:
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass


# ── Serial port receiver (Windows / Linux fallback) ───────────────────────────

def run_serial():
    """Connect via /dev/cu.* or COM port — use on Windows or Linux."""
    import serial
    print(f"[BT] connecting to {BT_PORT} @ {BT_BAUD} ...")
    print("Press Ctrl-C to quit.\n")

    while True:
        try:
            with serial.Serial(BT_PORT, BT_BAUD, timeout=1.0) as ser:
                print("[BT] connected.\n")
                while True:
                    raw = ser.readline()
                    if raw:
                        handle_line(raw.decode("ascii", errors="ignore").strip())

        except serial.SerialException as exc:
            print(f"\n[BT] connection lost: {exc}")
            print(f"[BT] retrying in {RECONNECT_DELAY_S} s...")
            time.sleep(RECONNECT_DELAY_S)

        except KeyboardInterrupt:
            print("\n[BT] stopped.")
            break


# ── entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    if USE_SERIAL_PORT:
        run_serial()
    else:
        run_socket()
