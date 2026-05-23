#!/usr/bin/env bash
# Build the ESP32-C6 companion firmware (D17).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
APP_DIR="${REPO_DIR}/c6-companion"

if [ ! -f "${APP_DIR}/CMakeLists.txt" ]; then
    echo "error: ${APP_DIR}/CMakeLists.txt not found" >&2
    exit 1
fi

IDF_EXPORT="${IDF_EXPORT:-${REPO_DIR}/esp-idf/export.sh}"
if [ ! -f "${IDF_EXPORT}" ]; then
    echo "error: vendored IDF export.sh not found at ${IDF_EXPORT}" >&2
    exit 1
fi
# shellcheck disable=SC1090
source "${IDF_EXPORT}" > /dev/null

cd "${APP_DIR}"
echo "[build_c6.sh] idf.py -DIDF_TARGET=esp32c6 build $*"
exec idf.py -DIDF_TARGET=esp32c6 build "$@"
