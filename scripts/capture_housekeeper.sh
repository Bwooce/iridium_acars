#!/usr/bin/env bash
# capture_housekeeper.sh — rotate the capture_monitor log and FIFO-
# prune old .u8 / decode-* files in WORKDIR. Runs independently of
# capture_monitor.sh so it can be added to a live run without
# interrupting it.
#
# Defaults:
#   LOGFILE=/tmp/capture_monitor.log
#   WORKDIR=/tmp/capture_monitor
#   LOG_MAX_BYTES=10485760     # 10 MB — rotate when bigger
#   LOG_KEEP=3                 # keep .log + .log.1 + .log.2
#   KEEP_IQ=8                  # keep newest N iq-*.u8 (oldest deleted)
#   POLL_SECONDS=30
#
# summary.tsv is small (one line per 10-min download cycle ≈ a few KB
# over a 4 h run) and is the canonical record — never deleted.
#
# Usage:
#   scripts/capture_housekeeper.sh > /tmp/capture_housekeeper.log 2>&1 &

set -uo pipefail

LOGFILE="${LOGFILE:-/tmp/capture_monitor.log}"
WORKDIR="${WORKDIR:-/tmp/capture_monitor}"
LOG_MAX_BYTES="${LOG_MAX_BYTES:-10485760}"
LOG_KEEP="${LOG_KEEP:-3}"
KEEP_IQ="${KEEP_IQ:-8}"
POLL_SECONDS="${POLL_SECONDS:-30}"

log() { printf '[%s] %s\n' "$(date -u +%FT%TZ)" "$*"; }

rotate_log() {
    [ -f "$LOGFILE" ] || return 0
    local sz
    sz=$(stat -c %s "$LOGFILE" 2>/dev/null || echo 0)
    if [ "$sz" -lt "$LOG_MAX_BYTES" ]; then
        return 0
    fi
    log "rotating $LOGFILE (size=$sz)"
    # Walk old rotations from highest to lowest, shifting up.
    local i=$((LOG_KEEP - 1))
    while [ "$i" -ge 1 ]; do
        local src="$LOGFILE.$i"
        local dst="$LOGFILE.$((i + 1))"
        if [ -f "$src" ]; then
            if [ "$i" -eq "$((LOG_KEEP - 1))" ]; then
                rm -f "$src"      # drop the oldest
            else
                mv "$src" "$dst"
            fi
        fi
        i=$((i - 1))
    done
    # Copy-truncate the live log so the producer (capture_monitor) keeps
    # writing to the same fd without losing entries to the gap that a
    # plain mv would introduce.
    cp "$LOGFILE" "$LOGFILE.1" && : > "$LOGFILE"
}

prune_iq() {
    [ -d "$WORKDIR" ] || return 0
    # FIFO: list iq-*.u8 newest-first by mtime, keep KEEP_IQ, delete the rest.
    # For each delete, also remove the matching decode-*.csv/.txt if present.
    local stale_count
    stale_count=$(ls -1t "$WORKDIR"/iq-*.u8 2>/dev/null | tail -n +"$((KEEP_IQ + 1))" | wc -l)
    [ "$stale_count" -eq 0 ] && return 0
    log "pruning $stale_count old iq files (keeping $KEEP_IQ newest)"
    ls -1t "$WORKDIR"/iq-*.u8 2>/dev/null | tail -n +"$((KEEP_IQ + 1))" | while read -r fp; do
        local base
        base=$(basename "$fp" .u8)
        rm -f "$fp" "$WORKDIR/$base.csv" "$WORKDIR/$base.txt"
    done
}

log "housekeeper start — LOGFILE=$LOGFILE WORKDIR=$WORKDIR"
log "config: LOG_MAX_BYTES=$LOG_MAX_BYTES LOG_KEEP=$LOG_KEEP KEEP_IQ=$KEEP_IQ POLL=$POLL_SECONDS"
mkdir -p "$WORKDIR"

while true; do
    rotate_log
    prune_iq
    sleep "$POLL_SECONDS"
done
