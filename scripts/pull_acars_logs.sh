#!/usr/bin/env bash
#
# pull_acars_logs.sh — pull ALL decoded-ACARS NDJSON logs off the P4 SDR
# device's SD card and consolidate them into ONE deduplicated file.
#
# The device (p4-usb-host) writes one JSON object per decoded ACARS message
# to /sdcard/acars/log-*.ndjson (see p4-usb-host/main/sd_log.c). Each line:
#   {"id":..,"t_us":..,"dir":"UL|DL","mode":"c","label":"..","block":"c",
#    "msg_num":"..","flight":"..","crc":bool,"peak_bin":..,"snr_db":..,"txt":".."}
#
# HTTP API used (all READ-ONLY except the safe POST /sd/mount):
#   GET  /sd/list                       -> JSON array [{name,size,mtime},...]
#   POST /sd/mount                       -> mounts card if lazy-unmounted (safe,
#                                           allow_format=false)
#   GET  /capture/file?name=<basename>   -> streams any file under /sdcard/acars/
#                                           (basename only; rejects '/' and '..').
#                                           This is the general SD-file fetch: it
#                                           serves log-*.ndjson too, not just IQ
#                                           captures. It only 409s if the *capture*
#                                           writer holds the file — the decode-log
#                                           writer never triggers that, so the
#                                           currently-open log is fetchable (may
#                                           lack the last <1 s of unflushed lines).
#
# DEDUP KEY: the full, whitespace-trimmed JSON line. Robust because `id` is
# always 0 on the VDL2 path and `t_us` is a per-boot monotonic tick (collides
# across reboots / across log files). The complete line — including t_us,
# flight, txt, snr, peak_bin — is effectively unique per real message.
#
# IDEMPOTENT: re-running merges any new lines into the consolidated file
# without duplicating. First-seen order is preserved.
#
# READ-ONLY on the device: this script only issues HTTP GETs plus at most one
# POST /sd/mount. It never reflashes, reboots, formats, deletes, or retunes.

set -euo pipefail

export PATH="/opt/homebrew/bin:$PATH"

# ---- config -----------------------------------------------------------------
DEVICE="${DEVICE:-http://192.168.1.235}"
OUT="${OUT:-$HOME/iridium_capture/acars_consolidated.ndjson}"
CURL_MAX_TIME="${CURL_MAX_TIME:-60}"
CURL=(curl -sS --max-time "$CURL_MAX_TIME")

WORKDIR="$(mktemp -d "${TMPDIR:-/tmp}/pull_acars.XXXXXX")"
trap 'rm -rf "$WORKDIR"' EXIT

mkdir -p "$(dirname "$OUT")"
touch "$OUT"

log() { printf '%s\n' "$*" >&2; }

# ---- 1. ensure SD is mounted ------------------------------------------------
# /sd/list returns a JSON array on success, or {"error":...} / non-JSON if the
# card is not mounted. Try a list; if it doesn't look like an array, mount and
# retry once.
fetch_list() {
    "${CURL[@]}" "$DEVICE/sd/list" 2>/dev/null || true
}

LIST_JSON="$(fetch_list)"
if [[ "${LIST_JSON:0:1}" != "[" ]]; then
    log "SD not listable yet (got: ${LIST_JSON:0:80}); POST /sd/mount ..."
    "${CURL[@]}" -X POST "$DEVICE/sd/mount" >/dev/null 2>&1 || true
    sleep 2
    LIST_JSON="$(fetch_list)"
fi
if [[ "${LIST_JSON:0:1}" != "[" ]]; then
    log "ERROR: /sd/list did not return a JSON array after mount. Response:"
    log "$LIST_JSON"
    exit 1
fi

# ---- 2. extract the ACARS log filenames -------------------------------------
# (read loop instead of mapfile so this runs on macOS's bundled bash 3.2)
LOGFILES=()
while IFS= read -r n; do
    [[ -n "$n" ]] && LOGFILES+=("$n")
done < <(printf '%s' "$LIST_JSON" | python3 -c '
import sys, json
try:
    arr = json.load(sys.stdin)
except Exception as e:
    sys.stderr.write("failed to parse /sd/list JSON: %s\n" % e); sys.exit(1)
for e in arr:
    n = e.get("name","")
    if n.startswith("log-") and n.endswith(".ndjson"):
        print(n)
')

if [[ ${#LOGFILES[@]} -eq 0 ]]; then
    log "No log-*.ndjson files found on the card. Nothing to do."
    log "(files present: $(printf '%s' "$LIST_JSON" | python3 -c 'import sys,json;print(", ".join(e.get("name","") for e in json.load(sys.stdin)))'))"
    exit 0
fi

log "Found ${#LOGFILES[@]} ACARS log file(s): ${LOGFILES[*]}"

# ---- 3. download each log file ----------------------------------------------
PULLED=0
DL_ALL="$WORKDIR/downloaded.ndjson"
: > "$DL_ALL"
for name in "${LOGFILES[@]}"; do
    dest="$WORKDIR/$name"
    if "${CURL[@]}" -o "$dest" "$DEVICE/capture/file?name=$name"; then
        n=$(wc -l < "$dest" | tr -d ' ')
        log "  pulled $name ($n lines)"
        cat "$dest" >> "$DL_ALL"
        PULLED=$((PULLED+1))
    else
        log "  WARN: failed to fetch $name — skipping"
    fi
done

# ---- 4. merge + dedup into the consolidated file ----------------------------
# Prior consolidated lines come FIRST so their first-seen order is preserved;
# newly downloaded lines are appended and deduped against the prior set.
# "Newly added" = final unique count - prior unique count.
STATS="$(python3 - "$OUT" "$DL_ALL" <<'PY'
import sys, os

out_path, dl_path = sys.argv[1], sys.argv[2]

def load(path):
    lines = []
    try:
        with open(path, "r", errors="replace") as f:
            for ln in f:
                s = ln.strip()
                if s:
                    lines.append(s)
    except FileNotFoundError:
        pass
    return lines

prior = load(out_path)
downloaded = load(dl_path)

seen = set()
merged = []
for s in prior + downloaded:
    if s not in seen:
        seen.add(s)
        merged.append(s)

prior_unique = len(set(prior))
final_unique = len(merged)
newly_added = final_unique - prior_unique

# Atomic rewrite of the consolidated file.
tmp = out_path + ".tmp"
with open(tmp, "w") as f:
    for s in merged:
        f.write(s + "\n")
os.replace(tmp, out_path)

# Machine-readable numbers for the shell summary (stdout).
print(f"{len(downloaded)}\t{final_unique}\t{newly_added}\t{prior_unique}")
PY
)"
IFS=$'\t' read -r DL_TOTAL FINAL_UNIQUE NEW_ADDED PRIOR_UNIQUE <<<"$STATS"

# ---- 5. summary -------------------------------------------------------------
log ""
log "==================== SUMMARY ===================="
log "Device:               $DEVICE"
log "Log files pulled:     $PULLED / ${#LOGFILES[@]}"
log "Lines downloaded:     $DL_TOTAL (this run, pre-dedup)"
log "Consolidated file:    $OUT"
log "Unique messages:      $FINAL_UNIQUE (was $PRIOR_UNIQUE before this run)"
log "Newly added this run: $NEW_ADDED"
log "================================================="
