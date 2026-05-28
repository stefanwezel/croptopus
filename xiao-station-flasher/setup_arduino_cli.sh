#!/bin/bash
#
# setup_arduino_cli.sh
#
# One-time setup for scripted XIAO ESP32S3 flashing:
#   - Install arduino-cli to ~/bin (if not already present)
#   - Initialise its config to use ~/Arduino as the sketchbook
#   - Add Espressif's ESP32 board manager URL
#   - Install the esp32:esp32 core (with XIAO_ESP32S3 support)
#   - Install RadioLib and Seeed_SHT35 libraries
#   - Test-compile the sketch to verify everything works
#
# Run this once on the workstation. After this, use flash_xiao_station.sh
# to flash individual XIAOs.

set -euo pipefail

# --- Paths ------------------------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="$SCRIPT_DIR/xiao_lora_sensor"
INSTALL_DIR="$HOME/bin"
SKETCHBOOK_DIR="$HOME/Arduino"
FQBN="esp32:esp32:XIAO_ESP32S3"

# --- Sanity check: sketch exists -------------------------------------------

if [ ! -f "$SKETCH_DIR/xiao_lora_sensor.ino" ]; then
    echo "Error: expected sketch at $SKETCH_DIR/xiao_lora_sensor.ino"
    echo "  Make sure xiao_lora_sensor.ino is inside a folder named xiao_lora_sensor,"
    echo "  placed alongside this setup script."
    exit 1
fi

# --- Install arduino-cli ----------------------------------------------------

echo "==> Checking arduino-cli..."
if command -v arduino-cli >/dev/null 2>&1; then
    echo "    Already installed: $(arduino-cli version 2>&1 | head -1)"
else
    echo "    Not found — installing to $INSTALL_DIR..."
    mkdir -p "$INSTALL_DIR"
    
    # Corrected pipeline passing BINDIR explicitly to the execution shell
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR="$INSTALL_DIR" sh

    if [ ! -x "$INSTALL_DIR/arduino-cli" ]; then
        echo "    ERROR: installation failed; arduino-cli not in $INSTALL_DIR"
        exit 1
    fi
    export PATH="$INSTALL_DIR:$PATH"
    echo "    Installed."
fi

# --- Verify PATH ------------------------------------------------------------

if ! command -v arduino-cli >/dev/null 2>&1; then
    echo "    arduino-cli installed at $INSTALL_DIR but not in PATH for this session."
    echo "    Run: export PATH=\"\$HOME/bin:\$PATH\""
    echo "    Then re-run this setup script."
    exit 1
fi

if ! [[ ":$PATH:" == *":$INSTALL_DIR:"* ]]; then
    echo "    NOTE: $INSTALL_DIR is not in your shell's persistent PATH."
    echo "    Add this to ~/.bashrc (or ~/.zshrc) so flash_xiao_station.sh works"
    echo "    in future terminal sessions:"
    echo ""
    echo "        export PATH=\"\$HOME/bin:\$PATH\""
    echo ""
fi

# --- Init arduino-cli config -----------------------------------------------

echo "==> Configuring arduino-cli..."
CONFIG_FILE="$HOME/.arduino15/arduino-cli.yaml"
if [ ! -f "$CONFIG_FILE" ]; then
    arduino-cli config init >/dev/null
fi

arduino-cli config set directories.user "$SKETCHBOOK_DIR"
arduino-cli config set library.enable_unsafe_install true

# Espressif ESP32 board manager URL
ESP32_URL="https://espressif.github.io/arduino-esp32/package_esp32_index.json"
arduino-cli config set board_manager.additional_urls "$ESP32_URL"

# --- Install ESP32 core ----------------------------------------------------

echo "==> Updating board index..."
arduino-cli core update-index >/dev/null

echo "==> Installing esp32:esp32 core (may take a few minutes on first run)..."
if arduino-cli core list | grep -q "^esp32:esp32"; then
    echo "    Already installed: $(arduino-cli core list | grep '^esp32:esp32')"
else
    arduino-cli core install esp32:esp32
fi

# Verify XIAO_ESP32S3 board is available
if ! arduino-cli board listall esp32:esp32 2>/dev/null | grep -qi "XIAO_ESP32S3"; then
    echo "    ERROR: XIAO_ESP32S3 not in installed esp32 core."
    echo "    The installed core might be too old. Try:"
    echo "        arduino-cli core upgrade esp32:esp32"
    exit 1
fi

# --- Install libraries -----------------------------------------------------

echo "==> Installing RadioLib..."
arduino-cli lib install "RadioLib" >/dev/null

echo "==> Installing Seeed_SHT35..."
# The exact name in the Arduino library registry is "Grove - Integrated Smart Station - SHT35"
if arduino-cli lib search "Grove - Integrated Smart Station - SHT35" 2>/dev/null | grep -q "SHT35"; then
    arduino-cli lib install "Grove - Integrated Smart Station - SHT35" >/dev/null
else
    echo "    Registry fallback: Installing via Git URL..."
    arduino-cli lib install --git-url https://github.com/Seeed-Studio/Seeed_SHT35.git >/dev/null
fi

# --- Test compile ----------------------------------------------------------

echo "==> Test-compiling sketch..."
TEST_LOG=$(mktemp)
if arduino-cli compile \
    --fqbn "$FQBN" \
    --build-property "compiler.cpp.extra_flags=-DSTATION_ID_OVERRIDE=99" \
    "$SKETCH_DIR" \
    > "$TEST_LOG" 2>&1; then
    SKETCH_SIZE=$(grep -oP "Sketch uses \K\d+" "$TEST_LOG" | head -1)
    echo "    OK — sketch compiles (binary size: ${SKETCH_SIZE} bytes)"
    rm "$TEST_LOG"
else
    echo "    ERROR: test compile failed. Full log:"
    cat "$TEST_LOG"
    rm "$TEST_LOG"
    exit 1
fi

# --- Done -------------------------------------------------------------------

echo ""
echo "================================================================"
echo "Setup complete."
echo ""
echo "To flash a XIAO with station ID N:"
echo "    1. Plug it into USB"
echo "    2. Run:  ./flash_xiao_station.sh N"
echo "================================================================"