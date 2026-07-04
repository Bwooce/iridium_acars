#!/usr/bin/env bash
# smoke_run.sh — drive the on-device SMOKE_TEST variants and restore
# production firmware. Toggles CONFIG_SMOKE_TEST_* in the generated
# sdkconfig (there is no fragment), rebuilds, flashes, and captures
# serial per variant. Instrumented: everything tees to per-variant
# logs under $OUTDIR so results are re-readable and never fabricated.
#
# Usage:
#   scripts/smoke_run.sh corpus            # one variant
#   scripts/smoke_run.sh corpus frame raw real
#   scripts/smoke_run.sh restore           # rebuild+flash production
#
# Variants: corpus frame raw real (map to SMOKE_TEST_{CORPUS,
# FRAME_DECODER,RAW_IRIDIUM,REAL_IRIDIUM}). Each is built from a
# pristine copy of the production sdkconfig so variants can't cross-
# contaminate. Env: PORT=/dev/ttyACM0, CAP_SECS=90.

set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
APP="$REPO/p4-usb-host"
SDK="$APP/sdkconfig"
PORT="${PORT:-/dev/ttyACM0}"
CAP_SECS="${CAP_SECS:-90}"
OUTDIR="${OUTDIR:-/tmp/smoke_run}"
BAK="$OUTDIR/sdkconfig.prod.bak"
mkdir -p "$OUTDIR"

log() { printf '[%s] %s\n' "$(date -Iseconds)" "$*"; }

declare -A SYM=(
    [corpus]=CONFIG_SMOKE_TEST_CORPUS
    [frame]=CONFIG_SMOKE_TEST_FRAME_DECODER
    [raw]=CONFIG_SMOKE_TEST_RAW_IRIDIUM
    [real]=CONFIG_SMOKE_TEST_REAL_IRIDIUM
)

# Snapshot the production sdkconfig ONCE (only if it isn't already a
# smoke config — refuse to back up a smoke-tainted config).
snapshot_prod() {
    # A pristine backup already taken in an earlier variant run is
    # authoritative — reuse it even if the live sdkconfig is currently
    # smoke-tainted (variants run back-to-back without restoring between).
    if [ -f "$BAK" ]; then
        if grep -q '^CONFIG_SMOKE_TEST_MODE=y' "$BAK"; then
            log "REFUSING: existing backup $BAK is itself smoke-tainted — restore manually."
            return 1
        fi
        log "reusing existing pristine backup -> $BAK"
        return 0
    fi
    if grep -q '^CONFIG_SMOKE_TEST_MODE=y' "$SDK"; then
        log "REFUSING to snapshot: $SDK is already a smoke config and no backup exists."
        log "  Restore production sdkconfig manually before running smoke variants."
        return 1
    fi
    cp "$SDK" "$BAK"
    log "snapshotted production sdkconfig -> $BAK"
}

# set_config <extra-sym|"">  — from the pristine backup, enable
# SMOKE_TEST_MODE + the one extra symbol; all other smoke syms off.
set_config() {
    local extra="$1"
    cp "$BAK" "$SDK"
    [ -z "$extra" ] && { log "config = PRODUCTION"; return; }
    python3 - "$SDK" "$extra" <<'PY'
import sys, re
path, extra = sys.argv[1], sys.argv[2]
syms = ['CONFIG_SMOKE_TEST_MODE','CONFIG_SMOKE_TEST_CORPUS',
        'CONFIG_SMOKE_TEST_FRAME_DECODER','CONFIG_SMOKE_TEST_RAW_IRIDIUM',
        'CONFIG_SMOKE_TEST_REAL_IRIDIUM','CONFIG_SMOKE_TEST_PIE_PLACEMENT',
        'CONFIG_SMOKE_TEST_LIVE_SDR']
enable = {'CONFIG_SMOKE_TEST_MODE', extra}
def canon(s): return (f"{s}=y" if s in enable else f"# {s} is not set")
lines = open(path).read().splitlines()
seen, out = set(), []
for ln in lines:
    m = re.match(r'^(?:# )?(CONFIG_SMOKE_TEST_[A-Z_]+)(?: is not set|=.*)$', ln)
    if m and m.group(1) in syms:
        seen.add(m.group(1)); out.append(canon(m.group(1)))
    else:
        out.append(ln)
for s in syms:
    if s not in seen: out.append(canon(s))
open(path, 'w').write("\n".join(out) + "\n")
print("enabled:", sorted(enable))
PY
}

build_flash_capture() {  # $1 = label (variant name or "production")
    local label="$1"
    local blog="$OUTDIR/build_${label}.log"
    local flog="$OUTDIR/flash_${label}.log"
    local slog="$OUTDIR/serial_${label}.log"

    log "[$label] building..."
    if ! ( cd "$APP" && "$REPO/scripts/build.sh" ) >"$blog" 2>&1; then
        log "[$label] BUILD FAILED — see $blog"; tail -20 "$blog"; return 2
    fi
    log "[$label] build ok ($(grep -c . "$blog") lines) -> $blog"

    log "[$label] flashing $PORT..."
    if ! "$REPO/scripts/flash.sh" "$PORT" >"$flog" 2>&1; then
        log "[$label] FLASH FAILED — see $flog"; tail -20 "$flog"; return 3
    fi
    log "[$label] flash ok -> $flog"

    log "[$label] capturing ${CAP_SECS}s serial -> $slog"
    "$REPO/scripts/monitor.sh" "$CAP_SECS" "$PORT" reset >"$slog" 2>&1 || true

    if grep -q "SMOKE_PASS" "$slog"; then
        log "[$label] RESULT: SMOKE_PASS"
    elif grep -q "SMOKE_FAIL\|PLACEMENT_FAIL" "$slog"; then
        log "[$label] RESULT: SMOKE_FAIL"
    elif [ "$label" = production ]; then
        grep -qE "class_driver|LIBUSB|rate>" "$slog" && log "[production] streaming path alive" || log "[production] (no stream markers in ${CAP_SECS}s — check $slog)"
    else
        log "[$label] RESULT: NO VERDICT in ${CAP_SECS}s — inspect $slog"
    fi
}

# ---- main ----
if [ "$#" -eq 0 ]; then
    echo "usage: $0 <corpus|frame|raw|real ...> | restore"; exit 1
fi

if [ "$1" = restore ]; then
    log "=== RESTORE production firmware ==="
    if [ ! -f "$BAK" ]; then log "no backup at $BAK — nothing to restore from"; exit 1; fi
    set_config ""
    build_flash_capture production
    log "restore done. (backup kept at $BAK)"
    exit 0
fi

snapshot_prod || exit 1
for v in "$@"; do
    sym="${SYM[$v]:-}"
    if [ -z "$sym" ]; then log "unknown variant '$v' (want corpus|frame|raw|real)"; continue; fi
    log "=== SMOKE variant: $v ($sym) ==="
    set_config "$sym"
    build_flash_capture "$v"
done
log "=== variants done. Run 'scripts/smoke_run.sh restore' to return to production. ==="
