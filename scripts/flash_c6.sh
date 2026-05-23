#!/bin/bash
# Flash the ESP32-C6 companion firmware via its PROG_C6 header.
# Usage: scripts/flash_c6.sh [PORT]
# Defaults to /dev/ttyACM2 (the third ACM enumerated by P4-NANO);
# pass the explicit device if your enumeration differs.

set -e
cd "$(dirname "$0")/.."
SCRIPT_DIR="$(pwd)"

PORT="${1:-/dev/ttyACM2}"

if [ -z "$IDF_PATH" ] || [ ! -d "$IDF_PATH" ]; then
    . "$SCRIPT_DIR/esp-idf/export.sh"
fi

cd c6-companion
idf.py -DIDF_TARGET=esp32c6 -p "$PORT" flash
