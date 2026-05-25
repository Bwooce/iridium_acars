#!/usr/bin/env bash
# capture_monitor.sh — long-running burst-mode capture watchdog +
# offline-decode summariser. Runs against the live P4-NANO firmware.
#
# Pair with capture_housekeeper.sh to keep /tmp from filling up:
#   scripts/capture_monitor.sh     > /tmp/capture_monitor.log     2>&1 &
#   scripts/capture_housekeeper.sh > /tmp/capture_housekeeper.log 2>&1 &
#
# Loop:
#   1. Ensure capture is armed on the device
#   2. Wait POLL_INTERVAL seconds
#   3. If the device dropped active=false (crash/reboot/circuit
#      breaker), log it and re-arm
#   4. Periodically (DOWNLOAD_INTERVAL): stop capture, download the
#      file, run decode_burst_capture on it, append a summary row,
#      delete the local copy if it grew, re-arm.
#   5. If the SD card is filling up (>SD_FULL_BYTES), reformat — only
#      reaction triggered automatically.
#
# Output:
#   * Tick log to stdout (and the redirected log file from caller)
#   * /tmp/capture_monitor/summary.tsv — one row per download cycle
#   * /tmp/capture_monitor/iq-<seq>.u8 — most recent download
#   * /tmp/capture_monitor/decode-<seq>.csv — per-burst CSV
#   * /tmp/capture_monitor/decode-<seq>.txt — summary lines
#
# Designed to be invoked as:
#   scripts/capture_monitor.sh > /tmp/capture_monitor.log 2>&1 &

set -uo pipefail

DEVICE="${DEVICE:-http://192.168.1.235}"
WORKDIR="${WORKDIR:-/tmp/capture_monitor}"
DECODER="${DECODER:-$(dirname "$(readlink -f "$0")")/../tests/host/build/decode_burst_capture}"
POLL_INTERVAL="${POLL_INTERVAL:-60}"        # status check cadence (s)
DOWNLOAD_INTERVAL="${DOWNLOAD_INTERVAL:-600}"   # 10 min — stop+download+decode
SD_FULL_BYTES="${SD_FULL_BYTES:-2000000000}"    # ~2 GB → reformat
MAX_RUNTIME_S="${MAX_RUNTIME_S:-14400}"         # 4 h total cap

mkdir -p "$WORKDIR"
SUMMARY="$WORKDIR/summary.tsv"
if [ ! -s "$SUMMARY" ]; then
    printf 'ts\tup_s\tfile\tsize_bytes\tbursts\tdecode_ok_pct\tframes\tms\ttl\tbc\tlw\tra\tunk\tdl\tul\tnote\n' > "$SUMMARY"
fi

log() { printf '[%s] %s\n' "$(date -u +%FT%TZ)" "$*"; }

curl_q() { curl -s --connect-timeout 3 --max-time 30 "$@"; }
curl_p() { curl -s --connect-timeout 3 --max-time 600 "$@"; }   # long ops (format/download)

device_uptime() {
    curl_q "$DEVICE/status" | python3 -c \
        'import json,sys;d=json.load(sys.stdin);print(d.get("uptime_s",0))' 2>/dev/null || echo 0
}

capture_status_active() {
    curl_q "$DEVICE/capture/status" | python3 -c \
        'import json,sys;d=json.load(sys.stdin);print("1" if d.get("active",False) else "0")' 2>/dev/null || echo 0
}

capture_status_bytes() {
    curl_q "$DEVICE/capture/status" | python3 -c \
        'import json,sys;d=json.load(sys.stdin);print(d.get("bytes_captured",0))' 2>/dev/null || echo 0
}

capture_status_path() {
    curl_q "$DEVICE/capture/status" | python3 -c \
        'import json,sys;d=json.load(sys.stdin);print(d.get("path","").split("/")[-1])' 2>/dev/null || echo ""
}

sd_list_size_total() {
    curl_q "$DEVICE/sd/list" | python3 -c \
        'import json,sys;d=json.load(sys.stdin);print(sum(f["size"] for f in d if f["size"]>0))' 2>/dev/null || echo 0
}

ensure_mounted() {
    curl_q -X POST "$DEVICE/sd/mount" >/dev/null
}

start_capture() {
    curl_q -X POST "$DEVICE/capture/start" \
        -H "Content-Type: application/json" \
        -d '{"mode":"bursts"}' | head -c 200; echo
}

stop_capture() {
    curl_q -X POST "$DEVICE/capture/stop" | head -c 200; echo
}

download_and_decode() {
    local fn="$1"
    local out_iq="$WORKDIR/$fn"
    local out_csv="$WORKDIR/${fn%.u8}.csv"
    local out_txt="$WORKDIR/${fn%.u8}.txt"
    log "downloading $fn"
    local dl_status
    dl_status=$(curl_p -o "$out_iq" -w "%{http_code}|%{size_download}" \
                  "$DEVICE/capture/file?name=$fn" 2>/dev/null)
    log "download $fn: $dl_status"
    local size
    size=$(stat -c %s "$out_iq" 2>/dev/null || echo 0)
    if [ "$size" -lt 64 ]; then
        log "download too small ($size) — skipping decode"
        return 0
    fi

    log "decoding $fn ($size bytes)"
    if [ ! -x "$DECODER" ]; then
        log "decoder $DECODER not built — skipping decode"
        return 0
    fi
    # Decoder writes CSV to stdout, summary to stderr.
    "$DECODER" "$out_iq" --csv > "$out_csv" 2> "$out_txt"
    # Extract summary numbers from the txt.
    local bursts ok_pct frames ms tl bc lw ra unk dl ul
    bursts=$(awk -F': *' '/^  bursts /{print $2; exit}' "$out_txt")
    ok_pct=$(awk -F'[() %]+' '/^  decode ok/{for(i=1;i<=NF;i++)if($i~/^[0-9.]+$/&&$(i+1)~/%/){print $i; exit}}' "$out_txt")
    frames=$(awk -F': *' '/^  frames /{print $2; exit}' "$out_txt")
    ms=$(awk -F'[/ ]+' '/MS\/TL\/BC\/LW\/RA/{print $5}' "$out_txt")
    tl=$(awk -F'[/ ]+' '/MS\/TL\/BC\/LW\/RA/{print $6}' "$out_txt")
    bc=$(awk -F'[/ ]+' '/MS\/TL\/BC\/LW\/RA/{print $7}' "$out_txt")
    lw=$(awk -F'[/ ]+' '/MS\/TL\/BC\/LW\/RA/{print $8}' "$out_txt")
    ra=$(awk -F'[/ ]+' '/MS\/TL\/BC\/LW\/RA/{print $9}' "$out_txt")
    unk=$(awk -F'unknown: *' '/unknown:/{print $2; exit}' "$out_txt")
    dl=$(awk -F'[/ ]+' '/DL\/UL/{print $4}' "$out_txt")
    ul=$(awk -F'[/ ]+' '/DL\/UL/{print $5}' "$out_txt")

    local up; up=$(device_uptime)
    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
        "$(date -u +%FT%TZ)" "$up" "$fn" "$size" \
        "${bursts:-0}" "${ok_pct:-0}" "${frames:-0}" \
        "${ms:-0}" "${tl:-0}" "${bc:-0}" "${lw:-0}" "${ra:-0}" "${unk:-0}" \
        "${dl:-0}" "${ul:-0}" "ok" \
        >> "$SUMMARY"
    log "decode summary: bursts=${bursts:-0} frames=${frames:-0} MS=${ms:-0} TL=${tl:-0} LW=${lw:-0}"
}

# ---- main loop --------------------------------------------------------

start_epoch=$(date +%s)
last_download=$start_epoch
last_reboot_up=0

log "monitor starting against $DEVICE (workdir=$WORKDIR)"
log "poll=${POLL_INTERVAL}s download=${DOWNLOAD_INTERVAL}s max=${MAX_RUNTIME_S}s"

ensure_mounted
start_capture

while true; do
    now=$(date +%s)
    if [ $((now - start_epoch)) -gt "$MAX_RUNTIME_S" ]; then
        log "max runtime reached — stopping"
        stop_capture
        break
    fi

    up=$(device_uptime)
    if [ "$up" -lt "$last_reboot_up" ]; then
        log "device REBOOTED (uptime ${last_reboot_up} -> $up)"
        ensure_mounted
        sleep 2
        start_capture
        last_reboot_up=$up
        sleep "$POLL_INTERVAL"
        continue
    fi
    last_reboot_up=$up

    active=$(capture_status_active)
    bytes=$(capture_status_bytes)
    sd_used=$(sd_list_size_total)
    log "tick: up=${up}s active=$active bytes=$bytes sd_used=$sd_used"

    if [ "$active" = "0" ]; then
        log "capture is INACTIVE — re-arming"
        start_capture
        sleep "$POLL_INTERVAL"
        continue
    fi

    # Download cycle?
    if [ $((now - last_download)) -ge "$DOWNLOAD_INTERVAL" ] && [ "$bytes" -gt 100000 ]; then
        fn=$(capture_status_path)
        if [ -n "$fn" ]; then
            log "download cycle: stopping to flush $fn"
            stop_capture
            sleep 5
            download_and_decode "$fn"
            last_download=$now
            log "re-arming"
            start_capture
        fi
    fi

    # SD getting full?
    if [ "$sd_used" -gt "$SD_FULL_BYTES" ]; then
        log "SD used ${sd_used} > ${SD_FULL_BYTES} — reformatting"
        stop_capture
        curl_p -X POST "$DEVICE/sd/format" >/dev/null
        ensure_mounted
        start_capture
    fi

    sleep "$POLL_INTERVAL"
done

log "monitor finished"
