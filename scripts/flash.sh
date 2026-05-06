#!/usr/bin/env bash
# Flash the ESP32-P4 firmware to the connected board.
#
# Usage:
#   scripts/flash.sh                        # auto-detect port, default baud
#   scripts/flash.sh /dev/ttyACM1           # explicit port
#   scripts/flash.sh /dev/ttyACM0 921600    # port + baud
#
# Defaults: port=$ESP_PORT or first /dev/ttyACM* found, baud=460800.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
APP_DIR="${REPO_DIR}/p4-usb-host"

PORT="${1:-${ESP_PORT:-}}"
BAUD="${2:-460800}"

if [ -z "${PORT}" ]; then
    # Auto-detect: take the first /dev/ttyACM* present.
    PORT="$(ls /dev/ttyACM* 2>/dev/null | head -n 1 || true)"
    if [ -z "${PORT}" ]; then
        echo "error: no /dev/ttyACM* found and no port given" >&2
        echo "  pass a port: scripts/flash.sh /dev/ttyACM0" >&2
        exit 1
    fi
fi

if [ ! -e "${PORT}" ]; then
    echo "error: ${PORT} does not exist" >&2
    exit 1
fi

# Source the project's vendored IDF environment unconditionally — see
# build.sh for rationale.
IDF_EXPORT="${IDF_EXPORT:-${REPO_DIR}/../esp-idf/export.sh}"
if [ ! -f "${IDF_EXPORT}" ]; then
    echo "error: vendored IDF export.sh not found at ${IDF_EXPORT}" >&2
    exit 1
fi
# shellcheck disable=SC1090
source "${IDF_EXPORT}" > /dev/null

cd "${APP_DIR}"

echo "[flash.sh] idf.py -p ${PORT} -b ${BAUD} flash"
exec idf.py -p "${PORT}" -b "${BAUD}" flash
