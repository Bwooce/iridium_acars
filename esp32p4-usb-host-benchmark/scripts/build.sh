#!/usr/bin/env bash
# Build the ESP32-P4 firmware. Prefers a direct ninja invocation when the
# build directory is already configured (skips idf.py's ~2 s python startup
# for incremental builds). Falls back to `idf.py build` for first-time
# configure or when CMakeLists changes force a reconfigure.
#
# Usage: scripts/build.sh [extra ninja/idf.py args]

set -euo pipefail

# Resolve project root (two levels up from this script: scripts/.. = repo,
# then into the active firmware app dir).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
APP_DIR="${REPO_DIR}/p4-usb-host"

if [ ! -f "${APP_DIR}/CMakeLists.txt" ]; then
    echo "error: ${APP_DIR}/CMakeLists.txt not found" >&2
    exit 1
fi

# Source the IDF environment if not already active.
if [ -z "${IDF_PATH:-}" ]; then
    IDF_EXPORT="${IDF_EXPORT:-/home/bruce/dev/iridium_acars/esp-idf/export.sh}"
    if [ ! -f "${IDF_EXPORT}" ]; then
        echo "error: IDF_EXPORT not found at ${IDF_EXPORT}" >&2
        echo "  set IDF_EXPORT=/path/to/esp-idf/export.sh or source it manually" >&2
        exit 1
    fi
    # shellcheck disable=SC1090
    source "${IDF_EXPORT}" > /dev/null
fi

cd "${APP_DIR}"

if [ -f build/build.ninja ]; then
    # Incremental: ninja directly is ~2× faster than idf.py for no-op rebuilds.
    echo "[build.sh] ninja -C build $*"
    exec ninja -C build "$@"
else
    # First build / no build dir yet: let idf.py configure + build.
    echo "[build.sh] idf.py build $*"
    exec idf.py build "$@"
fi
