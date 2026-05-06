#!/usr/bin/env bash
# Read serial output from the connected ESP32-P4 board for a fixed duration
# and print it to stdout. Non-interactive (suitable for piping into grep
# / log analysers, unlike `idf.py monitor` which is curses-based).
#
# Usage:
#   scripts/monitor.sh                       # default port, 15 s
#   scripts/monitor.sh 30                    # default port, 30 s
#   scripts/monitor.sh 15 /dev/ttyACM1       # explicit port
#   scripts/monitor.sh 15 /dev/ttyACM0 reset # also pulse RTS to reset before read
#
# Defaults: duration=15 seconds, baud=115200, port=$ESP_PORT or first /dev/ttyACM*.

set -euo pipefail

DURATION="${1:-15}"
PORT="${2:-${ESP_PORT:-}}"
RESET="${3:-}"
BAUD="${BAUD:-115200}"

if [ -z "${PORT}" ]; then
    PORT="$(ls /dev/ttyACM* 2>/dev/null | head -n 1 || true)"
    if [ -z "${PORT}" ]; then
        echo "error: no /dev/ttyACM* found and no port given" >&2
        exit 1
    fi
fi

if [ ! -e "${PORT}" ]; then
    echo "error: ${PORT} does not exist" >&2
    exit 1
fi

# Use python+pyserial for non-interactive reading. pyserial ships with the
# project's IDF venv. Source the env only if pyserial isn't already importable.
if ! python3 -c "import serial" 2>/dev/null; then
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
    IDF_EXPORT="${IDF_EXPORT:-${REPO_DIR}/esp-idf/export.sh}"
    if [ ! -f "${IDF_EXPORT}" ]; then
        echo "error: pyserial unavailable and IDF export.sh not found" >&2
        exit 1
    fi
    # shellcheck disable=SC1090
    source "${IDF_EXPORT}" > /dev/null
fi

# Run a small inline python — pyserial reads up to <duration> seconds, prints
# every byte as it arrives, then exits cleanly. RESET (any non-empty value)
# pulses RTS first to force a clean boot capture.
exec python3 - "${PORT}" "${BAUD}" "${DURATION}" "${RESET}" <<'PYEOF'
import serial, sys, time
port, baud, duration, reset = sys.argv[1], int(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
s = serial.Serial(port, baud, timeout=0.5)
if reset:
    s.setDTR(False); s.setRTS(True); time.sleep(0.05)
    s.setRTS(False); time.sleep(0.05)
end = time.monotonic() + duration
while time.monotonic() < end:
    try:
        d = s.read(4096)
        if d:
            sys.stdout.buffer.write(d)
            sys.stdout.flush()
    except Exception as e:
        print(f"[monitor.sh ERR] {e}", file=sys.stderr)
        break
PYEOF
