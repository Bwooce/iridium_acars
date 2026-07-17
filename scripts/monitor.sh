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
# project's IDF venv. The stock python3 may lack it (e.g. macOS Homebrew
# python), and `source export.sh` does not reliably put the venv python ahead
# on PATH — so pick an interpreter that actually has pyserial, preferring the
# vendored IDF venv python.
PYSERIAL_PY="python3"
if ! "${PYSERIAL_PY}" -c "import serial" 2>/dev/null; then
    PYSERIAL_PY=""
    for _cand in "${IDF_PYTHON_ENV_PATH:-}/bin/python" \
                 "${HOME}"/.espressif/python_env/*/bin/python; do
        [ -x "${_cand}" ] || continue
        if "${_cand}" -c "import serial" 2>/dev/null; then PYSERIAL_PY="${_cand}"; break; fi
    done
    if [ -z "${PYSERIAL_PY}" ]; then
        echo "error: no python with pyserial found (checked python3 + IDF venv)" >&2
        exit 1
    fi
fi

# Run a small inline python — pyserial reads up to <duration> seconds, prints
# every byte as it arrives, then exits cleanly. RESET (any non-empty value)
# pulses RTS first to force a clean boot capture.
exec "${PYSERIAL_PY}" - "${PORT}" "${BAUD}" "${DURATION}" "${RESET}" <<'PYEOF'
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
