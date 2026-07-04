#!/usr/bin/env bash
# stability_logger.sh — long-run health poller for the live P4-NANO.
# Polls GET /status every INTERVAL seconds, appends one raw JSON line
# (with a host wall-clock timestamp) to a daily NDJSON runlog, and a
# compact human-readable TSV row to a summary file. Detects device
# reboots (uptime_s decreasing) and stream stalls, and flags them.
#
# Append + flush every tick so the runlog is safe to re-read live and
# survives the poller being killed. Canonical record = the NDJSON.
#
# Usage:
#   scripts/stability_logger.sh > /tmp/stability_logger.out 2>&1 &
#
# Env:
#   HOST=192.168.1.235   device IP
#   INTERVAL=60          seconds between polls
#   OUTDIR=/tmp/stability_logger   runlog + summary dir

set -uo pipefail

HOST="${HOST:-192.168.1.235}"
INTERVAL="${INTERVAL:-60}"
OUTDIR="${OUTDIR:-/tmp/stability_logger}"
mkdir -p "$OUTDIR"

SUMMARY="$OUTDIR/summary.tsv"
if [ ! -s "$SUMMARY" ]; then
    printf 'host_ts\tuptime_s\tusb_completed\trb_full_drops\tstatus_errors\tshort_xfers\tstream_live\tstream_stalls\tmsgs_total\tacars\tsbd\tevent\n' >> "$SUMMARY"
fi

log() { printf '[%s] %s\n' "$(date -u +%FT%TZ)" "$*"; }

prev_uptime=-1
polls=0
reboots=0
log "stability_logger start host=$HOST interval=${INTERVAL}s outdir=$OUTDIR"

while true; do
    host_ts="$(date -u +%FT%TZ)"
    day="$(date -u +%F)"
    ndjson="$OUTDIR/status_${day}.ndjson"
    json="$(curl -s --max-time 8 "http://$HOST/status" 2>/dev/null)"

    if [ -z "$json" ] || ! printf '%s' "$json" | grep -q '"uptime_s"'; then
        printf '{"host_ts":"%s","event":"unreachable"}\n' "$host_ts" >> "$ndjson"
        printf '%s\t\t\t\t\t\t\t\t\t\t\tUNREACHABLE\n' "$host_ts" >> "$SUMMARY"
        log "UNREACHABLE (poll $polls)"
        sleep "$INTERVAL"
        polls=$((polls+1))
        continue
    fi

    # Prepend host timestamp into the raw object and append to NDJSON.
    printf '{"host_ts":"%s",%s\n' "$host_ts" "${json#\{}" >> "$ndjson"

    # Extract fields with a small python helper (jq may be absent).
    read -r uptime usb_c rb_drops st_err short_x stream_live stream_stalls msgs acars sbd <<EOF
$(printf '%s' "$json" | python3 -c '
import sys,json
d=json.load(sys.stdin)
u=d.get("usb",{}); dec=d.get("decode",{}); hw=d.get("health_wdt",{})
print(d.get("uptime_s",-1), u.get("completed",-1), u.get("rb_full_drops",-1),
      u.get("status_errors",-1), u.get("short_xfers",-1),
      str(hw.get("stream_live","?")).lower(), hw.get("stream_stalls",-1),
      dec.get("messages_total",-1), dec.get("acars_decoded",-1), dec.get("sbd_complete",-1))
' 2>/dev/null)
EOF

    event=""
    if [ "$prev_uptime" -ge 0 ] && [ "$uptime" -ge 0 ] && [ "$uptime" -lt "$prev_uptime" ]; then
        reboots=$((reboots+1))
        event="REBOOT(prev_uptime=${prev_uptime})"
        log "!! REBOOT detected: uptime $prev_uptime -> $uptime (total reboots=$reboots)"
    fi
    if [ "$stream_live" != "true" ]; then
        event="${event:+$event,}STREAM_DOWN"
        log "!! stream_live=$stream_live stalls=$stream_stalls"
    fi

    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
        "$host_ts" "$uptime" "$usb_c" "$rb_drops" "$st_err" "$short_x" \
        "$stream_live" "$stream_stalls" "$msgs" "$acars" "$sbd" "${event:-ok}" >> "$SUMMARY"

    if [ $((polls % 30)) -eq 0 ]; then
        log "tick $polls up=${uptime}s usb=${usb_c} rb_drops=${rb_drops} st_err=${st_err} stalls=${stream_stalls} msgs=${msgs} reboots=${reboots}"
    fi

    prev_uptime="$uptime"
    polls=$((polls+1))
    sleep "$INTERVAL"
done
