#!/bin/bash
#
# flash_xiao_station.sh
#
# Flash a connected XIAO ESP32S3 with the LoRa sensor sketch, baking in
# a specific STATION_ID via compile-time flag.
#
# Usage:
#   ./flash_xiao_station.sh <station_id>
#
# Example:
#   ./flash_xiao_station.sh 2
#
# Requirements (one-time):
#   - Run setup_arduino_cli.sh first.
#
# Safety:
#   - If more than one XIAO is detected on USB, the script aborts. Unplug
#     all but the target XIAO before retrying. This prevents accidentally
#     re-flashing an existing station.

set -euo pipefail

# --- Parse arguments --------------------------------------------------------

if [ $# -ne 1 ]; then
    echo "Usage: $0 <station_id>"
    echo "  station_id: integer between 1 and 255"
    exit 1
fi

STATION_ID="$1"
if ! [[ "$STATION_ID" =~ ^[0-9]+$ ]] || \
   [ "$STATION_ID" -lt 1 ] || [ "$STATION_ID" -gt 255 ]; then
    echo "Error: station_id must be an integer 1..255 (got: $STATION_ID)"
    exit 1
fi

# --- Locate sketch ----------------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="$SCRIPT_DIR/xiao_lora_sensor"
FQBN="esp32:esp32:XIAO_ESP32S3"

if [ ! -f "$SKETCH_DIR/xiao_lora_sensor.ino" ]; then
    echo "Error: expected sketch at $SKETCH_DIR/xiao_lora_sensor.ino"
    exit 1
fi

# --- Check arduino-cli is available ----------------------------------------

if ! command -v arduino-cli >/dev/null 2>&1; then
    if [ -x "$HOME/bin/arduino-cli" ]; then
        export PATH="$HOME/bin:$PATH"
    else
        echo "Error: arduino-cli not in PATH"
        echo "  Run setup_arduino_cli.sh first."
        exit 1
    fi
fi

# --- Detect the XIAO --------------------------------------------------------

echo "[1/3] Detecting XIAO ESP32S3..."

# Refresh board list (board detection is sometimes stale right after a hotplug)
sleep 1

PORTS=$(arduino-cli board list --format json 2>/dev/null | python3 -c '
import sys, json
try:
    data = json.load(sys.stdin)
except json.JSONDecodeError:
    sys.exit(0)
candidates = set()
for entry in data.get("detected_ports", []):
    port = entry.get("port", {})
    props = port.get("properties", {})
    # ESP32-S3 native USB JTAG/serial uses Espressif VID 0x303A
    vid = (props.get("vid") or "").lower()
    if vid == "0x303a" or vid == "0x2886" or vid == "2886":
        if port.get("address"):
            candidates.add(port["address"])
    # Also catch any board that arduino-cli explicitly matches as XIAO_ESP32S3
    for board in entry.get("matching_boards", []):
        fqbn = board.get("fqbn", "")
        if "XIAO_ESP32S3" in fqbn:
            if port.get("address"):
                candidates.add(port["address"])
for p in sorted(candidates):
    print(p)
')

NUM_PORTS=$(echo -n "$PORTS" | grep -c . || true)

if [ "$NUM_PORTS" -eq 0 ]; then
    echo "      No XIAO ESP32S3 detected."
    echo ""
    echo "  Make sure the XIAO is plugged in and powered (a small LED should be lit)."
    echo "  If it's plugged in but still not detected, see what arduino-cli sees:"
    echo "        arduino-cli board list"
    exit 1
elif [ "$NUM_PORTS" -gt 1 ]; then
    echo "      ERROR: $NUM_PORTS XIAOs detected:"
    echo "$PORTS" | sed 's/^/        /'
    echo ""
    echo "  Unplug all but the target XIAO and retry."
    echo "  This safety check prevents accidentally re-flashing an existing station."
    exit 1
fi

PORT="$PORTS"
echo "      Found at $PORT"

# --- Compile ----------------------------------------------------------------

echo "[2/3] Compiling with STATION_ID=$STATION_ID..."
COMPILE_LOG=$(mktemp)
if arduino-cli compile \
    --fqbn "$FQBN" \
    --build-property "compiler.cpp.extra_flags=-DSTATION_ID_OVERRIDE=$STATION_ID" \
    "$SKETCH_DIR" \
    > "$COMPILE_LOG" 2>&1; then
    SIZE=$(grep -oP "Sketch uses \K\d+" "$COMPILE_LOG" | head -1 || echo "?")
    echo "      OK (binary: ${SIZE} bytes)"
    rm "$COMPILE_LOG"
else
    echo "      FAILED. Compile log:"
    tail -30 "$COMPILE_LOG"
    rm "$COMPILE_LOG"
    exit 1
fi

# --- Upload -----------------------------------------------------------------

echo "[3/3] Uploading to $PORT..."
UPLOAD_LOG=$(mktemp)
if arduino-cli upload \
    --fqbn "$FQBN" \
    --port "$PORT" \
    "$SKETCH_DIR" \
    > "$UPLOAD_LOG" 2>&1; then
    echo "      OK"
    rm "$UPLOAD_LOG"
else
    echo "      FAILED. Upload log:"
    tail -30 "$UPLOAD_LOG"
    echo ""
    echo "  If you see 'Could not connect to chip', try:"
    echo "    1. Hold BOOT button on the XIAO"
    echo "    2. Press and release RESET while holding BOOT"
    echo "    3. Release BOOT"
    echo "    4. Re-run this script"
    rm "$UPLOAD_LOG"
    exit 1
fi

# --- Done -------------------------------------------------------------------

cat <<EOF

================================================================
Done. XIAO is now station #$STATION_ID.

To watch it boot, run:
    arduino-cli monitor -p $PORT --config baudrate=115200

(Ctrl+C to exit the monitor.)
================================================================
EOF