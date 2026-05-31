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

echo "[2/3] characterize → $CACHE/cfo_stats.csv ..."
python3 "$HERE/mendeley_cfo_characterize.py" "$@" > "$CACHE/cfo_stats.csv"

echo "    rows produced: $(($(wc -l < "$CACHE/cfo_stats.csv") - 1))"

echo "[3/3] analyze → $CACHE/cfo_analysis.md ..."
python3 "$HERE/mendeley_cfo_analyze.py" \
    --plot-dir "$CACHE/cfo_plots" \
    < "$CACHE/cfo_stats.csv" \
    > "$CACHE/cfo_analysis.md"

echo
echo "DONE. Open:"
echo "  $CACHE/cfo_analysis.md       — the report"
echo "  $CACHE/cfo_plots/*.png       — histograms"
echo
echo "When happy, commit a slimmed copy of cfo_analysis.md to docs/."
