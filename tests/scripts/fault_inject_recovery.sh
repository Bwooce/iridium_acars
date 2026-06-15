#!/usr/bin/env bash
# Fault-injection recovery test for #122.
#
# Drives each synthetic fault site on a live device (built with
# CONFIG_FAULT_INJECT=y) and asserts that:
#   1. the matching /diag/recovery_counters value climbs by >= the injected
#      count (>= because real events may also tick concurrently),
#   2. the USB stream keeps advancing (usb.completed climbs) — no #106-class
#      deadlock, and
#   3. the device does not reboot (uptime_s keeps increasing).
#
# Usage:
#   tests/scripts/fault_inject_recovery.sh [board_ip]
#   BOARD_IP=192.168.1.235 tests/scripts/fault_inject_recovery.sh
#
# Requires: a device flashed with a CONFIG_FAULT_INJECT=y build, reachable
# over HTTP, and python3 on the host. Exits nonzero on any assertion failure.

set -uo pipefail

BOARD_IP="${1:-${BOARD_IP:-192.168.1.235}}"
BASE="http://${BOARD_IP}"
WAIT_S="${WAIT_S:-8}"        # seconds to let injected faults fire
CURL="curl -s -m 6"

# site name -> "json_path_in_recovery_counters injected_count"
# json_path is dot-notation into /diag/recovery_counters.
declare -A SITE_COUNTER=(
  [dma_submit]="signal_buffer.stash_alloc_recoveries"
  [dma_submit_wrap]="signal_buffer.stash_alloc_fails"
  [dispatch_queue]="ingest_core1.dispatch_drops"
  [take_converted]="ingest_core1.slow_waits"
  [urb_submit]="esp_libusb.xfer_pool_lost"
)
declare -A SITE_COUNT=(
  [dma_submit]=5
  [dma_submit_wrap]=5
  [dispatch_queue]=5
  [take_converted]=5
  [urb_submit]=3
)

jq_path() { # $1=json  $2=dotted.path -> value (via python, no jq dependency)
  printf '%s' "$1" | python3 -c "import sys,json;d=json.load(sys.stdin)
p='$2'.split('.')
for k in p: d=d[k]
print(d)"
}

status_field() { # $1=json $2=dotted.path
  jq_path "$1" "$2"
}

fail=0
echo "== fault-injection recovery test against ${BASE} =="

# Preflight: confirm the endpoint exists (CONFIG_FAULT_INJECT build).
probe=$($CURL -o /dev/null -w "%{http_code}" -X POST "${BASE}/debug/fault_inject?site=__probe__" || echo 000)
if [ "$probe" = "000" ]; then
  echo "FATAL: ${BASE} unreachable"; exit 2
fi
if [ "$probe" = "404" ]; then
  echo "FATAL: /debug/fault_inject not found — device not built with CONFIG_FAULT_INJECT=y"; exit 2
fi

for site in dma_submit dma_submit_wrap dispatch_queue take_converted urb_submit; do
  path="${SITE_COUNTER[$site]}"
  n="${SITE_COUNT[$site]}"
  echo "--- site=${site} (expect ${path} += >=${n}) ---"

  st0=$($CURL "${BASE}/status"); rc0=$($CURL "${BASE}/diag/recovery_counters")
  c0=$(status_field "$rc0" "$path")
  up0=$(status_field "$st0" "uptime_s")
  usb0=$(status_field "$st0" "usb.completed")

  resp=$($CURL -X POST "${BASE}/debug/fault_inject?site=${site}&count=${n}")
  echo "  request: ${resp}"

  sleep "$WAIT_S"

  st1=$($CURL "${BASE}/status"); rc1=$($CURL "${BASE}/diag/recovery_counters")
  c1=$(status_field "$rc1" "$path")
  up1=$(status_field "$st1" "uptime_s")
  usb1=$(status_field "$st1" "usb.completed")

  delta=$(( c1 - c0 ))
  # 1. counter climbed by >= n
  if [ "$delta" -ge "$n" ]; then
    echo "  PASS counter ${path}: ${c0} -> ${c1} (+${delta} >= ${n})"
  else
    echo "  FAIL counter ${path}: ${c0} -> ${c1} (+${delta} < ${n})"; fail=1
  fi
  # 2. no reboot (uptime monotonic)
  if [ "$up1" -ge "$up0" ]; then
    echo "  PASS no reboot: uptime ${up0} -> ${up1}"
  else
    echo "  FAIL rebooted: uptime ${up0} -> ${up1}"; fail=1
  fi
  # 3. stream alive (usb.completed advanced)
  if [ "$usb1" -gt "$usb0" ]; then
    echo "  PASS stream alive: usb.completed ${usb0} -> ${usb1}"
  else
    echo "  FAIL stream stalled: usb.completed ${usb0} -> ${usb1}"; fail=1
  fi
done

echo "== $([ $fail -eq 0 ] && echo ALL PASS || echo FAILURES) =="
exit $fail
