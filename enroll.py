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
ENROLL_DIR     = "/home/nvidia/Desktop/yolo11_jetson/enrollments"
SUPPORTED_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}
# ────────────────────────────────────────────────────


def _detect_with_augment(fi, image):
    """
    Try multiple scales and flips to maximise Haar detection recall.
    Returns the first embedding found, or None.
    """
    transforms = [
        lambda img: img,
        lambda img: cv2.flip(img, 1),
        lambda img: cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE),
        lambda img: cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE),
    ]
    scales = [1.0, 1.5, 2.0]

    for tfm in transforms:
        t_img = tfm(image)
        for scale in scales:
            if scale != 1.0:
                candidate = cv2.resize(t_img, (0, 0), fx=scale, fy=scale)
            else:
                candidate = t_img
            emb, _ = fi.detect_and_embed(candidate)
            if emb is not None:
                return emb
    return None


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

    print("[enroll] Loading models...")
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

            emb = _detect_with_augment(fi, frame)
            if emb is None:
                print(f"  [{name}] No face detected: {img_path.name}")
                continue

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

    fi.close()


if __name__ == "__main__":
    enroll_all()
