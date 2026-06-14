#!/usr/bin/env bash
# serial_logger.sh — long-running serial-log capture with auto-reconnect.
#
# Replaces the ad-hoc `nohup cat /dev/ttyACMx >> /tmp/p4_serial.log &`
# pattern that dies every reflash / USB re-enumeration. This script:
#
#   1. Auto-detects the P4-NANO flash port (CH343, USB ID 1a86:55d3) by
#      vendor:product so it survives ACM-index reshuffles across host
#      reboots / unplugs (per memory note project_p4_nano_flash_port.md).
#   2. Loops cat with stty re-config on every connect, so a flash.sh
#      reset doesn't permanently kill capture.
#   3. Inserts a marked reconnect line on each disconnect so post-hoc
#      log inspection can see when the device dropped.
#   4. Optionally rotates the log when it crosses LOG_MAX_BYTES (default
#      50 MB) so 24-h soaks don't fill /tmp.
#   5. PID-file locked — running twice is a no-op (and re-uses the
#      existing instance).
#
# Usage:
#   scripts/serial_logger.sh                                 # background-run via nohup
#   scripts/serial_logger.sh stop                            # kill the running instance
#   scripts/serial_logger.sh status                          # is it alive? where's the log?
#   LOG_FILE=/tmp/p4_serial.log scripts/serial_logger.sh     # override log path
#   PORT=/dev/ttyACM0 scripts/serial_logger.sh               # skip auto-detect
#
# Environment knobs (with defaults):
#   LOG_FILE        /tmp/p4_serial.log
#   PID_FILE        /tmp/p4_serial_logger.pid
#   LOG_MAX_BYTES   52428800   (50 MB; 0 = disable rotation)
#   LOG_KEEP        3          rotated copies kept (.log.1 .. .log.N)
#   RECONNECT_S     2          sleep between cat retries

set -uo pipefail

LOG_FILE="${LOG_FILE:-/tmp/p4_serial.log}"
PID_FILE="${PID_FILE:-/tmp/p4_serial_logger.pid}"
PORT_FILE="${PORT_FILE:-/tmp/p4_serial_logger.port}"
LOG_MAX_BYTES="${LOG_MAX_BYTES:-52428800}"
LOG_KEEP="${LOG_KEEP:-3}"
RECONNECT_S="${RECONNECT_S:-2}"

P4_FLASH_ID="1a86:55d3"      # CH343 USB-UART -> P4 UART0 (flash port)

# --- helpers ---

find_p4_port() {
    # Prefer explicit override, else scan /dev/ttyACM* by vendor:product.
    if [ -n "${PORT:-}" ]; then
        echo "$PORT"; return 0
    fi
    for n in 0 1 2 3 4; do
        local dev="/dev/ttyACM$n"
        [ -e "$dev" ] || continue
        local id
        id=$(udevadm info -q property -n "$dev" 2>/dev/null | awk -F= '
            /^ID_VENDOR_ID=/  { v=$2 }
            /^ID_MODEL_ID=/   { m=$2 }
            END { if (v && m) print v":"m }
        ')
        if [ "$id" = "$P4_FLASH_ID" ]; then
            echo "$dev"; return 0
        fi
    done
    return 1
}

stamp() { date -u +"%Y-%m-%dT%H:%M:%SZ"; }

rotate_if_needed() {
    [ "$LOG_MAX_BYTES" -eq 0 ] && return 0
    [ -f "$LOG_FILE" ] || return 0
    local sz
    sz=$(stat -c %s "$LOG_FILE" 2>/dev/null || echo 0)
    [ "$sz" -lt "$LOG_MAX_BYTES" ] && return 0
    # Shift .log.N-1 -> .log.N; .log -> .log.1
    local i
    for ((i=LOG_KEEP-1; i>=1; i--)); do
        if [ -f "${LOG_FILE}.$i" ]; then
            mv -f "${LOG_FILE}.$i" "${LOG_FILE}.$((i+1))" 2>/dev/null
        fi
    done
    mv -f "$LOG_FILE" "${LOG_FILE}.1"
    : > "$LOG_FILE"
    printf "[%s] LOG ROTATED (previous moved to %s.1; > %d bytes)\n" \
        "$(stamp)" "$LOG_FILE" "$LOG_MAX_BYTES" >> "$LOG_FILE"
}

# --- subcommands ---

cmd_status() {
    if [ ! -f "$PID_FILE" ]; then
        echo "not running (no PID file at $PID_FILE)"; exit 1
    fi
    local pid; pid=$(cat "$PID_FILE")
    if kill -0 "$pid" 2>/dev/null; then
        echo "running pid=$pid  log=$LOG_FILE"
        if [ -f "$LOG_FILE" ]; then
            echo "  log size: $(stat -c %s "$LOG_FILE") bytes"
        fi
        exit 0
    else
        echo "stale PID file (pid $pid not alive); remove with: scripts/serial_logger.sh stop"
        exit 1
    fi
}

cmd_stop() {
    if [ ! -f "$PID_FILE" ]; then
        echo "no PID file"; exit 0
    fi
    local pid; pid=$(cat "$PID_FILE")
    if kill -0 "$pid" 2>/dev/null; then
        kill -TERM "$pid" 2>/dev/null
        sleep 1
        kill -KILL "$pid" 2>/dev/null
        echo "killed pid=$pid"
    fi
    rm -f "$PID_FILE"
    # Kill any orphaned cat children on our specific port only (not other agents' monitors).
    if [ -f "$PORT_FILE" ]; then
        local owned_port; owned_port=$(cat "$PORT_FILE")
        pkill -f "cat $owned_port" 2>/dev/null || true
        rm -f "$PORT_FILE"
    fi
    exit 0
}

cmd_run() {
    # Single-instance: PID file lock.
    if [ -f "$PID_FILE" ]; then
        local pid; pid=$(cat "$PID_FILE")
        if kill -0 "$pid" 2>/dev/null; then
            echo "already running pid=$pid (log=$LOG_FILE); exit"
            exit 0
        fi
        rm -f "$PID_FILE"
    fi
    # Trap on exit: clean up children + PID/port files.
    cleanup() {
        pkill -P $$ 2>/dev/null || true
        rm -f "$PID_FILE" "$PORT_FILE"
    }
    trap cleanup EXIT INT TERM

    echo $$ > "$PID_FILE"
    : >> "$LOG_FILE"  # ensure exists, don't truncate
    printf "[%s] serial_logger START pid=%d\n" "$(stamp)" "$$" >> "$LOG_FILE"

    while true; do
        rotate_if_needed
        local port
        if ! port=$(find_p4_port); then
            printf "[%s] no P4 port (vendor %s) found; retry in %ds\n" \
                "$(stamp)" "$P4_FLASH_ID" "$RECONNECT_S" >> "$LOG_FILE"
            sleep "$RECONNECT_S"
            continue
        fi
        # Record which port we own so flash.sh (and cmd_stop) can target cleanup precisely.
        echo "$port" > "$PORT_FILE"
        # Reconfigure each connect — the device may have re-enumerated.
        stty -F "$port" 115200 raw -echo -hupcl clocal 2>/dev/null || true
        printf "[%s] CONNECT %s\n" "$(stamp)" "$port" >> "$LOG_FILE"
        # cat blocks until EOF / disconnect / error
        cat "$port" >> "$LOG_FILE" 2>/dev/null
        local rc=$?
        printf "[%s] DISCONNECT %s (cat rc=%d), reconnect in %ds\n" \
            "$(stamp)" "$port" "$rc" "$RECONNECT_S" >> "$LOG_FILE"
        sleep "$RECONNECT_S"
    done
}

# --- entry ---

case "${1:-run}" in
    run)    cmd_run    ;;
    status) cmd_status ;;
    stop)   cmd_stop   ;;
    *)
        echo "usage: $0 [run|stop|status]" >&2
        exit 2
        ;;
esac
