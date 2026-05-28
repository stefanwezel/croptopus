#!/bin/bash
#
# flash_xiao_station.sh
#
# Flash a connected XIAO ESP32S3 with the LoRa sensor sketch. Identity is now
# derived from the ESP32-S3 factory MAC (read from eFuse at boot) — no manual
# STATION_ID assignment is needed. After upload the script briefly opens the
# serial port to capture the device's MAC + short ID from the boot banner and
# appends a row to ../stations.csv so we have a registry of every flashed unit.
#
# Usage:
#   ./flash_xiao_station.sh [station_label] [--force-id N]
#
# Arguments:
#   station_label   Optional human label recorded in stations.csv ("greenhouse",
#                   "back garden", etc.). Pass quoted if it contains spaces.
#
# Flags:
#   --force-id N    Bypass MAC-derived identity and bake short ID = N (1..65535)
#                   in at compile time. Useful for testing or pinning a known
#                   ID. Default mode (no flag) is MAC-derived.
#
# Examples:
#   ./flash_xiao_station.sh
#   ./flash_xiao_station.sh greenhouse
#   ./flash_xiao_station.sh test-rig --force-id 42
#
# Requirements (one-time):
#   - Run setup_arduino_cli.sh first.
#
# Safety:
#   - If more than one XIAO is detected on USB, the script aborts. Unplug all
#     but the target XIAO before retrying.

set -euo pipefail

# --- Parse arguments --------------------------------------------------------

STATION_LABEL=""
FORCE_ID=""

while [ $# -gt 0 ]; do
    case "$1" in
        --force-id)
            if [ $# -lt 2 ]; then
                echo "Error: --force-id requires a value"
                exit 1
            fi
            FORCE_ID="$2"
            shift 2
            ;;
        --force-id=*)
            FORCE_ID="${1#*=}"
            shift
            ;;
        -h|--help)
            sed -n '3,30p' "$0" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        --*)
            echo "Error: unknown flag: $1"
            exit 1
            ;;
        *)
            if [ -z "$STATION_LABEL" ]; then
                STATION_LABEL="$1"
            else
                echo "Error: unexpected positional argument: $1"
                echo "  station_label can only be given once. Quote it if it contains spaces."
                exit 1
            fi
            shift
            ;;
    esac
done

if [ -n "$FORCE_ID" ]; then
    if ! [[ "$FORCE_ID" =~ ^[0-9]+$ ]] || \
       [ "$FORCE_ID" -lt 1 ] || [ "$FORCE_ID" -gt 65535 ]; then
        echo "Error: --force-id must be an integer 1..65535 (got: $FORCE_ID)"
        exit 1
    fi
fi

# --- Locate sketch + registry ----------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="$SCRIPT_DIR/xiao_lora_sensor"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
REGISTRY="$REPO_ROOT/stations.csv"
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

echo "[1/5] Detecting XIAO ESP32S3..."

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

COMPILE_ARGS=(--fqbn "$FQBN")
if [ -n "$FORCE_ID" ]; then
    echo "[2/5] Compiling with --force-id $FORCE_ID (overrides MAC-derived ID)..."
    COMPILE_ARGS+=(--build-property "compiler.cpp.extra_flags=-DSTATION_ID_OVERRIDE=$FORCE_ID")
else
    echo "[2/5] Compiling (identity will be MAC-derived at boot)..."
fi
COMPILE_ARGS+=("$SKETCH_DIR")

COMPILE_LOG=$(mktemp)
if arduino-cli compile "${COMPILE_ARGS[@]}" > "$COMPILE_LOG" 2>&1; then
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

echo "[3/5] Uploading to $PORT..."
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

# --- Capture identity from boot banner --------------------------------------

echo "[4/5] Capturing boot identity from $PORT (up to 12s)..."

# Give the upload tool a moment to release the port and the chip to reset.
sleep 1

SERIAL_LOG=$(mktemp)
# arduino-cli monitor runs until killed; we cap it with `timeout` and ignore
# the resulting non-zero exit (124 on timeout) so the script keeps going.
timeout 12 arduino-cli monitor \
    -p "$PORT" \
    --config "baudrate=115200" \
    > "$SERIAL_LOG" 2>/dev/null || true

MAC=$(grep -m1 "STATION MAC:" "$SERIAL_LOG" | awk '{print $NF}' || true)
SHORT_ID=$(grep -m1 "STATION SHORT_ID:" "$SERIAL_LOG" | awk '{print $NF}' || true)
rm -f "$SERIAL_LOG"

if [ -z "$MAC" ] || [ -z "$SHORT_ID" ]; then
    echo "      WARNING: boot banner not captured within 12s."
    echo "      Upload succeeded but identity wasn't recorded. Read manually with:"
    echo "        arduino-cli monitor -p $PORT --config baudrate=115200"
    MAC="${MAC:-UNKNOWN}"
    SHORT_ID="${SHORT_ID:-UNKNOWN}"
else
    echo "      MAC=$MAC  SHORT_ID=$SHORT_ID"
fi

# --- Append to registry -----------------------------------------------------

echo "[5/5] Recording to $REGISTRY ..."
if [ ! -f "$REGISTRY" ]; then
    echo "timestamp,mac,short_id,station_label,port" > "$REGISTRY"
fi
TIMESTAMP=$(date -u +"%Y-%m-%dT%H:%M:%SZ")
# Quote the label so commas/spaces in it don't break the CSV.
LABEL_CSV=$(printf '%s' "$STATION_LABEL" | sed 's/"/""/g')
printf '%s,%s,%s,"%s",%s\n' \
    "$TIMESTAMP" "$MAC" "$SHORT_ID" "$LABEL_CSV" "$PORT" >> "$REGISTRY"
echo "      Appended."

# --- Done -------------------------------------------------------------------

cat <<EOF

================================================================
Done.
  MAC:        $MAC
  SHORT_ID:   $SHORT_ID
  Label:      ${STATION_LABEL:-(none)}
  Port:       $PORT
  Registry:   $REGISTRY

To watch it boot again, run:
    arduino-cli monitor -p $PORT --config baudrate=115200

(Ctrl+C to exit the monitor.)
================================================================
EOF
