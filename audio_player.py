# audio_player.py
"""
Non-blocking audio player for Jetson.

- Uses aplay (ALSA) to play pre-recorded .wav files from the audio/ folder.
- Only one clip plays at a time — starting a new one stops the previous.
- verified_<name>.wav is generated on demand via TTS if the file doesn't exist.

Called from arduino_link.default_command_handler on PLAY:<name> commands.
"""

import os
import shutil
import subprocess

# ── config ────────────────────────────────────────────────────────────────────
AUDIO_DIR   = os.path.join(os.path.dirname(os.path.abspath(__file__)), "audio")
APLAY_DEVICE = None   # None = system default. Set to e.g. "plughw:1,0" if
                      # USB speaker is not the default ALSA device.
                      # Find your device: aplay -l
# ─────────────────────────────────────────────────────────────────────────────

_current_proc = None   # the running aplay subprocess


def play(name: str):
    """
    Play audio/<name>.wav non-blocking.
    Stops any currently playing clip first.
    Generates verified_<name>.wav via TTS if the file doesn't exist.
    """
    global _current_proc

    # stop previous clip
    if _current_proc and _current_proc.poll() is None:
        _current_proc.terminate()
        _current_proc = None

    wav_path = os.path.join(AUDIO_DIR, f"{name}.wav")

    # auto-generate verified_<name>.wav on first use
    if not os.path.exists(wav_path) and name.startswith("verified_"):
        person_name = name[len("verified_"):]
        _generate_verified(person_name, wav_path)

    if not os.path.exists(wav_path):
        print(f"[Audio] file not found: {wav_path}")
        return

    cmd = ["aplay", "-q"]
    if APLAY_DEVICE:
        cmd += ["-D", APLAY_DEVICE]
    cmd.append(wav_path)

    try:
        _current_proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        print(f"[Audio] playing: {name}.wav")
    except FileNotFoundError:
        print("[Audio] aplay not found — run: sudo apt install alsa-utils")


def stop():
    """Stop the currently playing clip immediately."""
    global _current_proc
    if _current_proc and _current_proc.poll() is None:
        _current_proc.terminate()
        _current_proc = None


def _generate_verified(name: str, wav_path: str):
    """Generate verified_<name>.wav using pico2wave or espeak as fallback."""
    os.makedirs(AUDIO_DIR, exist_ok=True)
    text = f"Identity verified. Welcome, {name}."

    if shutil.which("pico2wave"):
        try:
            subprocess.run(["pico2wave", "-w", wav_path, text],
                           check=True, timeout=10)
            print(f"[Audio] generated {wav_path} via pico2wave")
            return
        except Exception as exc:
            print(f"[Audio] pico2wave failed: {exc}")

    if shutil.which("espeak"):
        try:
            subprocess.run(["espeak", "-w", wav_path, text],
                           check=True, timeout=10)
            print(f"[Audio] generated {wav_path} via espeak")
            return
        except Exception as exc:
            print(f"[Audio] espeak failed: {exc}")

    print("[Audio] no TTS engine — install: sudo apt install libttspico-utils")
