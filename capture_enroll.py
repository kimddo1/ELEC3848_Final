# capture_enroll.py
# Interactive photo capture tool for building the enrollment database.
#
# Usage:
#   python3 capture_enroll.py
#
# Controls:
#   SPACE       - capture a photo manually
#   A           - toggle auto-capture mode (captures every AUTO_INTERVAL seconds)
#   Q / ESC     - quit (saves whatever was captured so far)

import cv2
import time
from pathlib import Path

# ── config ───────────────────────────────────────────
ENROLL_DIR    = "/home/nvidia/Desktop/yolo11_jetson/enrollments"
TARGET_COUNT  = 5       # photos to collect per person
AUTO_INTERVAL = 2.0     # seconds between auto-captures

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


def get_name() -> str:
    while True:
        name = input("Enter person's name (no spaces, e.g. Alice): ").strip()
        if name:
            return name
        print("Name cannot be empty.")


def make_save_dir(name: str) -> Path:
    save_dir = Path(ENROLL_DIR) / name
    save_dir.mkdir(parents=True, exist_ok=True)
    return save_dir


def next_index(save_dir: Path) -> int:
    existing = list(save_dir.glob("*.jpg"))
    return len(existing) + 1


def draw_overlay(frame, name: str, count: int, auto: bool, countdown: float):
    h, w = frame.shape[:2]

    # dark bar at top
    cv2.rectangle(frame, (0, 0), (w, 50), (0, 0, 0), -1)

    cv2.putText(frame, f"Person: {name}",
                (10, 33), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    status = f"Captured: {count}/{TARGET_COUNT}"
    cv2.putText(frame, status,
                (w - 280, 33), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 220, 0), 2)

    # dark bar at bottom
    cv2.rectangle(frame, (0, h - 45), (w, h), (0, 0, 0), -1)

    if auto:
        next_in = max(0.0, AUTO_INTERVAL - countdown)
        hint = f"AUTO ON  |  next in {next_in:.1f}s  |  A=off  Q=quit"
        color = (0, 200, 255)
    else:
        hint = "SPACE=capture   A=auto   Q=quit"
        color = (180, 180, 180)

    cv2.putText(frame, hint,
                (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    return frame


def flash(frame):
    bright = cv2.convertScaleAbs(frame, alpha=2.0, beta=50)
    cv2.imshow("Enrollment Capture", bright)
    cv2.waitKey(80)


def capture_for_person(name: str):
    save_dir = make_save_dir(name)
    idx      = next_index(save_dir)
    count    = idx - 1

    print(f"\n[capture] Saving to: {save_dir}")
    print(f"[capture] Already have {count} photo(s). Need {TARGET_COUNT} total.")
    print(f"[capture] SPACE = capture  |  A = auto-mode  |  Q = quit\n")

    cap = cv2.VideoCapture(GST_PIPELINE, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera. Check GStreamer pipeline.")

    auto_mode  = False
    auto_start = time.time()

    while count < TARGET_COUNT:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame")
            break

        elapsed = time.time() - auto_start
        display = draw_overlay(frame.copy(), name, count, auto_mode, elapsed)
        cv2.imshow("Enrollment Capture", display)

        key = cv2.waitKey(1) & 0xFF

        do_capture = False

        if key == ord(' '):
            do_capture = True
        elif key == ord('a'):
            auto_mode  = not auto_mode
            auto_start = time.time()
            print(f"[capture] Auto-capture {'ON' if auto_mode else 'OFF'}")
        elif key in (ord('q'), 27):  # Q or ESC
            print("[capture] Quit early.")
            break

        if auto_mode and elapsed >= AUTO_INTERVAL:
            do_capture = True
            auto_start = time.time()

        if do_capture:
            path = save_dir / f"{idx:02d}.jpg"
            cv2.imwrite(str(path), frame)
            flash(display)
            print(f"  Saved: {path.name}  ({count + 1}/{TARGET_COUNT})")
            count += 1
            idx   += 1

    cap.release()
    cv2.destroyAllWindows()

    print(f"\n[capture] Done. {count} photo(s) saved for '{name}'.")
    if count >= TARGET_COUNT:
        print(f"[capture] Ready to enroll. Run: python3 enroll.py")
    else:
        print(f"[capture] Only {count}/{TARGET_COUNT} captured. "
              f"Run capture again to add more, or run enroll.py now.")


def main():
    print("=== Enrollment Photo Capture ===")
    while True:
        name = get_name()
        capture_for_person(name)

        again = input("\nCapture another person? (y/n): ").strip().lower()
        if again != 'y':
            break

    print("\nAll done. Run  python3 enroll.py  to build the face database.")


if __name__ == "__main__":
    main()
