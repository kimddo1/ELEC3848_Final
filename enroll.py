# enroll.py
# Offline enrollment script.
# Reads images from:
#   ENROLL_DIR/<person_name>/img1.jpg
#   ENROLL_DIR/<person_name>/img2.jpg
#   ...
# Detects a face in each image, extracts its embedding,
# and saves all embeddings to DB_PATH.
#
# Run once before deploying:
#   python enroll.py
#
# Re-running merges new entries with the existing database.

import cv2
import numpy as np
from pathlib import Path

from face_id import (
    FaceDatabase,
    FaceIdentifier,
    DB_PATH,
)

# ── config ───────────────────────────────────────────
ENROLL_DIR     = "/home/nvidia/Desktop/face_id/enrollments"
SUPPORTED_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}
# ────────────────────────────────────────────────────


def enroll_all():
    enroll_root = Path(ENROLL_DIR)
    if not enroll_root.exists():
        raise FileNotFoundError(
            f"Enrollment directory not found: {ENROLL_DIR}\n"
            f"Expected structure:\n"
            f"  {ENROLL_DIR}/\n"
            f"    Alice/\n"
            f"      front.jpg\n"
            f"      side.jpg\n"
            f"    Bob/\n"
            f"      front.jpg\n"
        )

    person_dirs = sorted([d for d in enroll_root.iterdir() if d.is_dir()])
    if not person_dirs:
        print("No person folders found to enroll.")
        return

    print("[enroll] Loading engines...")
    fi = FaceIdentifier()

    total_added = 0
    for person_dir in person_dirs:
        name = person_dir.name
        image_paths = sorted([
            p for p in person_dir.iterdir()
            if p.suffix.lower() in SUPPORTED_EXTS
        ])

        if not image_paths:
            print(f"  [{name}] No images found, skipping")
            continue

        embeddings = []
        for img_path in image_paths:
            frame = cv2.imread(str(img_path))
            if frame is None:
                print(f"  [{name}] Failed to read: {img_path.name}")
                continue

            h, w = frame.shape[:2]
            result = fi.identify(frame,
                                 person_box=np.array([0, 0, w, h]),
                                 track_id=-1)

            if result["face_box"] is None:
                print(f"  [{name}] No face detected: {img_path.name}")
                continue

            # re-extract embedding directly (identify() also compares DB,
            # but we only need the embedding here)
            fx1, fy1, fx2, fy2 = result["face_box"]
            # face_box is in original frame coords since person_box starts at 0,0
            face_crop = frame[fy1:fy2, fx1:fx2]
            emb = fi._extract_embedding(face_crop)
            embeddings.append(emb)
            print(f"  [{name}] OK {img_path.name}")

        if not embeddings:
            print(f"  [{name}] No valid embeddings, skipping")
            continue

        emb_array = np.stack(embeddings)          # [N, 512]
        fi.db.add(name, emb_array)
        total_added += 1
        print(f"  [{name}] {len(embeddings)} embeddings enrolled")

    if total_added > 0:
        fi.db.save()
        print(f"\n[enroll] Done -- {total_added} people enrolled, DB saved: {DB_PATH}")
    else:
        print("\n[enroll] No new people enrolled")


if __name__ == "__main__":
    enroll_all()
