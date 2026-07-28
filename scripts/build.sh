#!/usr/bin/env bash
# Build the ESP32-P4 firmware. Prefers a direct ninja invocation when the
# build directory is already configured (skips idf.py's ~2 s python startup
# for incremental builds). Falls back to `idf.py build` for first-time
# configure or when CMakeLists changes force a reconfigure.
#
# Usage: scripts/build.sh [--rev pre_v3|v3_0|v3_1] [extra ninja/idf.py args]
#
#   --rev pre_v3  (default) ESP32-P4 rev v0.x/v1.x engineering samples — the
#                 current, only-tested target. Uses build/ + sdkconfig, base
#                 defaults only: byte-for-byte the pre-variant behaviour.
#   --rev v3_0 / v3_1   ESP32-P4 rev v3.0 / v3.1 — HARDWARE-UNVERIFIED. Layers
#                 sdkconfig.rev_<rev>.defaults over sdkconfig.defaults into an
#                 isolated build-<rev>/ + build-<rev>/sdkconfig. Configures +
#                 builds only; no v3 silicon has run it.
#   See docs/2026-07-28-p4-cpu-revision-variants.md.

set -euo pipefail

# Resolve project root (two levels up from this script: scripts/.. = repo,
# then into the active firmware app dir).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
APP_DIR="${REPO_DIR}/p4-usb-host"

# CPU-revision variant (compile-time). Default pre_v3 preserves the exact
# current behaviour. Strip --rev from the args passed through to ninja/idf.py.
REV="${REV:-pre_v3}"
_args=()
while [ $# -gt 0 ]; do
    case "$1" in
        --rev)   REV="${2:?--rev needs a value}"; shift 2 ;;
        --rev=*) REV="${1#--rev=}"; shift ;;
        *)       _args+=("$1"); shift ;;
    esac
done
set -- ${_args[@]+"${_args[@]}"}   # restore passthrough args (bash 3.2 + set -u safe)

if [ ! -f "${APP_DIR}/CMakeLists.txt" ]; then
    echo "error: ${APP_DIR}/CMakeLists.txt not found" >&2
    exit 1
fi

# Source the project's vendored IDF environment. We deliberately do NOT
# trust a pre-existing IDF_PATH — this project requires v6.1 at ./esp-idf/
# and the user may have a different system-installed IDF on PATH.
IDF_EXPORT="${IDF_EXPORT:-${REPO_DIR}/esp-idf/export.sh}"
if [ ! -f "${IDF_EXPORT}" ]; then
    echo "error: vendored IDF export.sh not found at ${IDF_EXPORT}" >&2
    echo "  expected at <repo>/esp-idf/export.sh; set IDF_EXPORT to override" >&2
    exit 1
fi
# shellcheck disable=SC1090
source "${IDF_EXPORT}" > /dev/null

cd "${APP_DIR}"

case "${REV}" in
    pre_v3)
        # Unchanged path: base defaults only, default build/ + sdkconfig.
        if [ -f build/build.ninja ]; then
            # Incremental: ninja directly is ~2× faster than idf.py for no-op rebuilds.
            echo "[build.sh] ninja -C build $*"
            exec ninja -C build "$@"
        else
            # First build / no build dir yet: let idf.py configure + build.
            echo "[build.sh] idf.py build $*"
            exec idf.py build "$@"
        fi
        ;;
    v3_0|v3_1)
        FRAG="sdkconfig.rev_${REV}.defaults"
        [ -f "${FRAG}" ] || { echo "error: ${FRAG} not found" >&2; exit 1; }
        BD="build-${REV}"
        # SDKCONFIG defaults to <proj>/sdkconfig regardless of -B, and
        # SDKCONFIG_DEFAULTS is applied ONLY when that file is absent — so
        # per-variant isolation needs a per-variant SDKCONFIG path too, else
        # the variant silently inherits the pre_v3 sdkconfig.
        echo "[build.sh] idf.py -B ${BD} (rev=${REV}, HARDWARE-UNVERIFIED) $*"
        exec idf.py -B "${BD}" \
            -D SDKCONFIG="${BD}/sdkconfig" \
            -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;${FRAG}" \
            build "$@"
        ;;
    *)
        echo "error: unknown --rev '${REV}' (expected pre_v3|v3_0|v3_1)" >&2
        exit 1
        ;;
esac
