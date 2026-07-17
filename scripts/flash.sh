#!/usr/bin/env bash
# Flash the ESP32-P4 firmware to the connected board.
#
# Usage:
#   scripts/flash.sh                        # auto-detect port, default baud
#   scripts/flash.sh /dev/ttyACM1           # explicit port
#   scripts/flash.sh /dev/ttyACM0 921600    # port + baud
#
# Defaults: port=$ESP_PORT or first matching P4-NANO port, baud=460800.
#
# Port selection (Waveshare ESP32-P4-NANO has two USB-CDC interfaces):
#   * CH343 USB-UART (WCH 1a86:55d3) — wired to P4 UART0 + EN + GPIO0
#     with proper RTS/DTR auto-reset. THIS IS THE FLASH PORT.
#   * Native USB-Serial-JTAG (Espressif 303a:1001) — debug-only; cannot
#     reliably enter download mode because the host can't strap-reset
#     the chip via this interface once firmware is running.
# We auto-prefer the CH343 over the native USB-JTAG. Auto-detect also
# works when the board is in boot mode (BOOT held during plug-in) since
# both ports re-enumerate identically in either state.
#
# When a port is given explicitly we still validate it points at a known
# P4-NANO interface and warn loudly if not.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
APP_DIR="${REPO_DIR}/p4-usb-host"

PORT="${1:-${ESP_PORT:-}}"
BAUD="${2:-460800}"
SERIAL_LOGGER="${SCRIPT_DIR}/serial_logger.sh"

# macOS has no udevadm, so id_of() maps a /dev/cu.usbmodem* device to its
# vid:pid via pyserial's list_ports. The stock python3 may lack pyserial, so
# pick an interpreter that has it — preferring the vendored IDF venv python.
# (Linux uses the udevadm branch below and ignores PYSERIAL_PY.)
PYSERIAL_PY="python3"
if ! command -v udevadm >/dev/null 2>&1; then
    if ! "${PYSERIAL_PY}" -c "import serial.tools.list_ports" 2>/dev/null; then
        for _cand in "${IDF_PYTHON_ENV_PATH:-}/bin/python" \
                     "${HOME}"/.espressif/python_env/*/bin/python; do
            [ -x "${_cand}" ] || continue
            if "${_cand}" -c "import serial.tools.list_ports" 2>/dev/null; then
                PYSERIAL_PY="${_cand}"; break
            fi
        done
    fi
fi

# vendor:model pairs we recognise on the P4-NANO. The first match wins
# during auto-detect; both are accepted (with a warning for the 2nd) when
# an explicit port is given.
P4_FLASH_ID="1a86:55d3"      # WCH CH343 USB-UART -> P4 UART0 + EN + GPIO0
P4_DEBUG_ID="303a:1001"      # Espressif USB JTAG/serial -> P4 native USB OTG

# Print "vendor:model" for a serial device (empty if it can't be told).
# Linux: udevadm. macOS (no udevadm): pyserial's list_ports vid/pid.
id_of() {
    local d="$1"
    [ -e "$d" ] || { echo ""; return; }
    if command -v udevadm >/dev/null 2>&1; then
        local v m
        v="$(udevadm info "$d" 2>/dev/null | awk -F= '/^E: ID_VENDOR_ID=/{print $2; exit}')"
        m="$(udevadm info "$d" 2>/dev/null | awk -F= '/^E: ID_MODEL_ID=/{print $2; exit}')"
        if [ -n "$v" ] && [ -n "$m" ]; then
            echo "${v}:${m}"
        else
            echo ""
        fi
        return
    fi
    # macOS / no udevadm: map the callout device -> vid:pid via pyserial.
    "${PYSERIAL_PY}" - "$d" <<'PY' 2>/dev/null
import sys
try:
    from serial.tools import list_ports
except Exception:
    sys.exit(0)
dev = sys.argv[1]
for p in list_ports.comports():
    if p.device == dev and p.vid is not None and p.pid is not None:
        print("%04x:%04x" % (p.vid, p.pid))
        break
PY
}

if [ -z "${PORT}" ]; then
    # Auto-detect: prefer the CH343 (flash-capable), fall back to native
    # USB-JTAG only if the CH343 isn't present. Works in both run mode
    # and BOOT-held mode -- the ports re-enumerate identically either way.
    PORT=""
    for d in /dev/ttyACM* /dev/ttyUSB*; do
        [ -e "$d" ] || continue
        if [ "$(id_of "$d")" = "${P4_FLASH_ID}" ]; then
            PORT="$d"
            break
        fi
    done
    if [ -z "${PORT}" ]; then
        # No CH343 found -- try the native USB-JTAG as a fallback, but
        # warn since flashing through it is unreliable.
        for d in /dev/ttyACM* /dev/ttyUSB*; do
            [ -e "$d" ] || continue
            if [ "$(id_of "$d")" = "${P4_DEBUG_ID}" ]; then
                PORT="$d"
                echo "[flash.sh] WARNING: only the native USB-JTAG ($d, ${P4_DEBUG_ID}) was found." >&2
                echo "[flash.sh] WARNING: this port cannot reliably enter download mode. Flashing will likely fail." >&2
                echo "[flash.sh] WARNING: connect the P4-NANO's CH343 UART port (the second USB-C / micro-USB) and retry." >&2
                break
            fi
        done
    fi
    if [ -z "${PORT}" ]; then
        echo "error: no P4-NANO port found (expected ${P4_FLASH_ID} or ${P4_DEBUG_ID})" >&2
        echo "  available tty devices:" >&2
        for d in /dev/ttyACM* /dev/ttyUSB*; do
            [ -e "$d" ] || continue
            echo "    $d  ($(id_of "$d"))" >&2
        done
        exit 1
    fi
fi

if [ ! -e "${PORT}" ]; then
    echo "error: ${PORT} does not exist" >&2
    exit 1
fi

# Validate: must be one of the known P4-NANO interfaces. We allow the
# native USB-JTAG (with a warning) so users can still try if they really
# mean it, but flag anything else as a hard error.
PORT_ID="$(id_of "${PORT}")"
case "${PORT_ID}" in
    "${P4_FLASH_ID}")
        echo "[flash.sh] port ${PORT} (${PORT_ID}) -> P4-NANO CH343 UART (flash-capable)"
        ;;
    "${P4_DEBUG_ID}")
        echo "[flash.sh] WARNING: ${PORT} is the native USB-JTAG (${PORT_ID})." >&2
        echo "[flash.sh] WARNING: flashing via this port usually fails (chip cannot enter download mode)." >&2
        echo "[flash.sh] WARNING: continuing anyway -- pass /dev/ttyACMx of the CH343 to flash reliably." >&2
        ;;
    "")
        echo "error: cannot identify ${PORT} via udevadm. Refusing to flash an unknown device." >&2
        exit 1
        ;;
    *)
        echo "error: ${PORT} reports id ${PORT_ID}, which is not a known P4-NANO interface." >&2
        echo "  expected ${P4_FLASH_ID} (CH343 flash) or ${P4_DEBUG_ID} (native USB-JTAG)." >&2
        echo "  refusing to flash a foreign device." >&2
        exit 1
        ;;
esac

# Source the project's vendored IDF environment unconditionally — see
# build.sh for rationale.
IDF_EXPORT="${IDF_EXPORT:-${REPO_DIR}/esp-idf/export.sh}"
if [ ! -f "${IDF_EXPORT}" ]; then
    echo "error: vendored IDF export.sh not found at ${IDF_EXPORT}" >&2
    exit 1
fi
# shellcheck disable=SC1090
source "${IDF_EXPORT}" > /dev/null

cd "${APP_DIR}"

# Stop the serial logger so it releases the port before esptool connects.
# Port contention during a long erase causes blank-sector corruption.
LOGGER_WAS_RUNNING=0
if [ -x "${SERIAL_LOGGER}" ]; then
    if "${SERIAL_LOGGER}" status &>/dev/null; then
        LOGGER_WAS_RUNNING=1
    fi
    "${SERIAL_LOGGER}" stop 2>/dev/null || true
fi

# If the port is still held (e.g. orphaned cat from a SIGKILL'd logger),
# kill the specific holder(s). We only target ${PORT} — not any other ACM port.
# Use lsof -t (portable): it prints the holder PIDs and nothing when free.
# NOT fuser — macOS/BSD fuser exits 0 even when the file is unheld, which
# would make this guard a permanent false-positive and abort every flash.
# `|| true` because lsof -t exits 1 when the port is free, which would trip
# `set -e` in the `holders=$(...)` assignments below.
port_holders() { lsof -t "${PORT}" 2>/dev/null || true; }
holders="$(port_holders)"
if [ -n "${holders}" ]; then
    echo "[flash.sh] ${PORT} still held after logger stop (pids: ${holders}); killing ..."
    # shellcheck disable=SC2086
    kill -9 ${holders} 2>/dev/null || true
    sleep 0.3
fi
holders="$(port_holders)"
if [ -n "${holders}" ]; then
    echo "error: ${PORT} is still held (pids: ${holders}); aborting flash" >&2
    exit 1
fi

echo "[flash.sh] idf.py -p ${PORT} -b ${BAUD} flash"
idf.py -p "${PORT}" -b "${BAUD}" flash
FLASH_RC=$?

# Restart the logger if it was running before we stopped it.
if [ "${LOGGER_WAS_RUNNING}" -eq 1 ] && [ -x "${SERIAL_LOGGER}" ]; then
    nohup "${SERIAL_LOGGER}" run >> /tmp/p4_serial_logger_restart.log 2>&1 &
    disown $!
    echo "[flash.sh] serial logger restarted (pid=$!)"
fi

exit ${FLASH_RC}
