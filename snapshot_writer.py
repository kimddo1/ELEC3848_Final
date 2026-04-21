# snapshot_writer.py
"""
Saves frame crops and logs events for the Patrol Pro pipeline.

Directory layout (created automatically):
    snapshots/
        person/     — first frame of a new YOLO track
        face/       — first frame where SCRFD detects a face in that track
        verified/   — frame when face is matched in the database
    logs/
        events.csv  — one row per event: timestamp, event, track_id, name, mode

Public API
----------
update_frame(frame)
    Call every main loop iteration with the latest camera frame.
    Keeps a module-level reference so save_current() can use it.

save(frame, box, category, track_id, name="")
    Crop `box` from `frame` and write to snapshots/<category>/.
    Also appends a <category>_SNAP row to events.csv.
    Safe to call with box=None (saves the full frame instead of a crop).

save_current(category, track_id=-1, name="")
    Like save() but uses the last frame passed to update_frame().
    Called from arduino_link default_command_handler on SNAP: commands.

log_event(event, track_id, name, mode)
    Append one row to logs/events.csv without saving an image.
"""

import os
import csv
import cv2
from datetime import datetime

# ── config ────────────────────────────────────────────────────────────────────
_BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SNAP_DIR  = os.path.join(_BASE_DIR, "snapshots")
LOG_DIR   = os.path.join(_BASE_DIR, "logs")
LOG_FILE  = os.path.join(LOG_DIR, "events.csv")

JPEG_QUALITY = 92       # cv2.imwrite JPEG quality (0–100)
# ─────────────────────────────────────────────────────────────────────────────

_current_frame = None   # latest camera frame set by update_frame()


# ── public ────────────────────────────────────────────────────────────────────

def update_frame(frame):
    """Store the latest camera frame for use by save_current()."""
    global _current_frame
    _current_frame = frame


def save(frame, box, category: str, track_id: int, name: str = "") -> str:
    """
    Crop box from frame and save to snapshots/<category>/.

    Parameters
    ----------
    frame     : BGR numpy array (required)
    box       : [x1, y1, x2, y2] in pixel coords, or None for full frame
    category  : "person" | "face" | "verified"
    track_id  : integer track id (used in filename)
    name      : person name (empty string if not yet known)

    Returns the saved file path, or empty string on failure.
    """
    if frame is None:
        print(f"[Snapshot] no frame available — skipping {category} save")
        return ""

    cat_dir = os.path.join(SNAP_DIR, category)
    os.makedirs(cat_dir, exist_ok=True)

    crop = _crop(frame, box)
    if crop is None or crop.size == 0:
        print(f"[Snapshot] empty crop for track {track_id} — skipping")
        return ""

    ts       = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # ms precision
    filename = f"{category}_{track_id}_{ts}.jpg"
    path     = os.path.join(cat_dir, filename)

    ok = cv2.imwrite(path, crop, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    if ok:
        print(f"[Snapshot] saved {path}")
    else:
        print(f"[Snapshot] cv2.imwrite failed for {path}")
        return ""

    log_event(f"{category.upper()}_SNAP", track_id, name, "")
    return path


def save_current(category: str, track_id: int = -1, name: str = "") -> str:
    """
    Save a snapshot using the most recently stored frame (no box crop).
    Intended for SNAP: commands arriving from Arduino.
    """
    return save(_current_frame, None, category, track_id, name)


def log_event(event: str, track_id: int, name: str, mode: str):
    """
    Append one row to logs/events.csv.
    Creates the file with a header row if it does not yet exist.
    """
    os.makedirs(LOG_DIR, exist_ok=True)
    ts          = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    write_header = not os.path.exists(LOG_FILE)

    with open(LOG_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow(["timestamp", "event", "track_id", "name", "mode"])
        writer.writerow([ts, event, track_id, name, mode])


# ── private ───────────────────────────────────────────────────────────────────

def _crop(frame, box):
    """Return a cropped region of frame, clamped to image bounds."""
    if box is None:
        return frame.copy()

    h, w = frame.shape[:2]
    x1 = max(0, int(box[0]))
    y1 = max(0, int(box[1]))
    x2 = min(w, int(box[2]))
    y2 = min(h, int(box[3]))

    if x2 <= x1 or y2 <= y1:
        return None

    return frame[y1:y2, x1:x2].copy()
