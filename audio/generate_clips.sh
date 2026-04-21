#!/bin/bash
# generate_clips.sh
# Generates all pre-recorded audio clips for Patrol Pro.
#
# Run once on Jetson (from the project root):
#   bash audio/generate_clips.sh
#
# Requirements (install one):
#   sudo apt install libttspico-utils   # pico2wave — better quality
#   sudo apt install espeak             # espeak — fallback

set -e
DIR="$(cd "$(dirname "$0")" && pwd)"

# ── pick TTS engine ───────────────────────────────────────────────────────────
if command -v pico2wave &> /dev/null; then
    ENGINE="pico2wave"
    echo "[TTS] Using pico2wave"
elif command -v espeak &> /dev/null; then
    ENGINE="espeak"
    echo "[TTS] Using espeak"
else
    echo "[TTS] No TTS engine found."
    echo "      Install: sudo apt install libttspico-utils"
    exit 1
fi

generate() {
    local filename="$1"
    local text="$2"
    local path="$DIR/$filename"

    if [ "$ENGINE" = "pico2wave" ]; then
        pico2wave -w "$path" "$text"
    else
        espeak -w "$path" "$text"
    fi
    echo "  Generated: $filename"
}

# ── clips ─────────────────────────────────────────────────────────────────────
echo ""
echo "Generating clips..."

generate "alert_fire.wav"      "Warning. Fire detected. Please evacuate immediately."
generate "alert_intruder.wav"  "Security alert. Intruder detected. Authorities have been notified."
generate "detected_person.wav" "Person detected. Please face the camera for verification."
generate "approach_camera.wav" "Please approach and face the camera directly."
generate "verified.wav"        "Identity verified. Access granted."

echo ""
echo "Done. Files saved to: $DIR"
echo ""
echo "To verify playback:"
echo "  aplay audio/alert_fire.wav"
echo ""
echo "If no sound from USB speaker, find your device with:"
echo "  aplay -l"
echo "Then set APLAY_DEVICE in audio_player.py (e.g. 'plughw:1,0')"
