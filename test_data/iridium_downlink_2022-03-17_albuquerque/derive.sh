#!/bin/bash
# Reproduce all derivations from iridium_cf32.sigmf-data.
# Idempotent: skips a step if its primary output already exists.
#
# Requires:
#   - iridium-extractor on PATH (apt install gnuradio gnuradio-dev libvolk-dev
#     pybind11-dev libsndfile1-dev gr-osmosdr; then build gr-iridium and
#     'sudo cmake --install build' from the repo's gr-iridium/ dir)
#   - python3-pyproj python3-crcmod (apt)

set -e

HERE="$(cd "$(dirname "$0")" && pwd)"
RAW="$HERE/iridium_cf32.sigmf-data"
OUT="$HERE/derivations"
TOOLKIT="$HERE/../../iridium-toolkit"

mkdir -p "$OUT"

if [ ! -f "$RAW" ]; then
    if [ -f "$RAW.zst" ]; then
        echo "[0/3] decompressing $(basename "$RAW.zst") -> $(basename "$RAW") ..."
        zstd -d -q "$RAW.zst" -o "$RAW"
    else
        echo "Missing raw data: neither $RAW nor $RAW.zst exists." >&2
        exit 1
    fi
fi

# 1. Extract bursts via gr-iridium
if [ ! -s "$OUT/iridium.bits" ]; then
    echo "[1/3] iridium-extractor ..."
    iridium-extractor -r 12000000 -c 1621500000 -f cf32_le "$RAW" \
        2> "$OUT/iridium-extractor.log" \
        | grep -v "fft_burst_tagger\|vmcircbuf_prefs" \
        > "$OUT/iridium.bits"
fi

# 2. Parse + classify
if [ ! -s "$OUT/iridium.parsed" ]; then
    echo "[2/3] iridium-parser.py ..."
    python3 "$TOOLKIT/iridium-parser.py" \
        < "$OUT/iridium.bits" \
        > "$OUT/iridium.parsed" \
        2> "$OUT/iridium-parser.log"
fi

# 3. Reassemble + extract derived artefacts
echo "[3/3] reassembler ..."
python3 "$TOOLKIT/reassembler.py" -i "$OUT/iridium.parsed" -m acars \
    > "$OUT/acars_reassembly.txt" 2>&1
python3 "$TOOLKIT/reassembler.py" -i "$OUT/iridium.parsed" -m ida \
    > "$OUT/ida_fragments.txt" 2>&1
awk '{print $1}' "$OUT/iridium.parsed" | sort | uniq -c | sort -rn \
    > "$OUT/frame_type_histogram.txt"

echo "Done. See $OUT/."
