#!/usr/bin/env bash
# mendeley_run_all.sh — orchestrate the full CFO peak-distribution
# characterization workflow: fetch → characterize → analyze. The
# expensive step is the fetch (4.67 GB download + 14 GB decompression).
# Characterize streams (constant memory); analyze loads the CSV.
#
# Total runtime expected: 15–25 min on a typical machine with reasonable
# Internet (fetch dominates at ~10–15 min; analyze is seconds).
#
# Usage:
#   tests/scripts/mendeley_run_all.sh                # full 3.8 M bursts
#   tests/scripts/mendeley_run_all.sh --files 1109-0910_20_parsed.txt  # half
#   tests/scripts/mendeley_run_all.sh --max 100000   # smoke run
#   IQ_CACHE_ROOT=/big/disk tests/scripts/mendeley_run_all.sh
#
# Results land in:
#   $IQ_CACHE_ROOT/cfo_stats.csv          per-burst statistics
#   $IQ_CACHE_ROOT/cfo_analysis.md        the report
#   $IQ_CACHE_ROOT/cfo_plots/*.png        histograms (if matplotlib)

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CACHE="${IQ_CACHE_ROOT:-$HOME/iq_cache}"
mkdir -p "$CACHE"

echo "[1/3] fetch ..."
python3 "$HERE/fetch_mendeley_iridium.py"

echo "[2/2] decode_validate → $CACHE/decode_stats.csv ..."
# NOTE 2026-05-31: the original cfo_characterize.py / cfo_analyze.py
# pair is the WRONG tool for Mendeley because Mendeley's IQ turned out
# to be post-PLL symbol-rate (110 samples @ 25 ksps), not raw
# wideband at 250 ksps as initially assumed. CFO peak distribution on
# 110-sample zero-padded windows is degenerate. Those scripts are kept
# for when ALBQ-style wideband data is the target.
#
# The right tool for Mendeley is decode_validate.py — diff-decode +
# BCH + IRA-classify + ground-truth comparison. Note: current decode
# alignment achieves only 14% bch_ok / 1% sat_match against Mendeley
# metadata; getting it to >95% requires reverse-engineering the exact
# iridium-toolkit frame layout (multi-hour task, deferred).
python3 "$HERE/mendeley_decode_validate.py" "$@" > "$CACHE/decode_stats.csv"

echo "    rows produced: $(($(wc -l < "$CACHE/decode_stats.csv") - 1))"

echo
echo "DONE. Inspect:"
echo "  $CACHE/decode_stats.csv      — per-burst decode + match results"
echo "  Summary on stderr above showed both-BCH-OK and sat_id-match rates."
echo
echo "(Analyzer for this CSV is TBD — current decode rate too low to be"
echo " a useful regression signal until the iridium-toolkit frame-layout"
echo " work is done. See tests/scripts/mendeley_decode_validate.py docstring.)"
