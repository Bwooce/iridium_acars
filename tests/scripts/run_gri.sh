#!/usr/bin/env bash
# run_gri.sh — decode RTL-SDR cu8 captures with gr-iridium (iridium-extractor)
# + iridium-toolkit (iridium-parser), for a P4-vs-gr-iridium comparison against
# tests/host/decode_continuous_capture (which runs the SAME captures through the
# P4's own decode chain). Both consume the identical raw-uint8 IQ file.
#
# Capture format: RTL-SDR **cu8** — raw uint8, interleaved I,Q, DC offset 128 —
# which is exactly what the P4 firmware's continuous SD capture writes
# (p4-usb-host/main/sd_capture.h). Native 2.5 MSPS, so NO resample (2500000 is
# divisible by 100000, which iridium-extractor requires).
#
# gr-iridium is NOT a Homebrew formula; it's a source build. Install used
# 2026-07-19 on the Mac (see reference_mac_env_and_serial memory):
#   brew install gnuradio                       # 3.10.12, Homebrew python3.14
#   python3.14 -m venv --system-site-packages $GRI_TOOLS/venv
#   $GRI_TOOLS/venv/bin/pip install pybind11 scipy crcmod
#   # build muccc/gr-iridium into the venv prefix:
#   cmake .. -DCMAKE_INSTALL_PREFIX=$GRI_TOOLS/venv -DCMAKE_BUILD_TYPE=Release \
#            -DPython3_EXECUTABLE=$GRI_TOOLS/venv/bin/python \
#            -Dpybind11_DIR=$($GRI_TOOLS/venv/bin/python -c 'import pybind11;print(pybind11.get_cmake_dir())')
#   make -j && make install                     # -> venv/bin/iridium-extractor
#   git clone https://github.com/muccc/iridium-toolkit $GRI_TOOLS/iridium-toolkit
#
# Usage:
#   [GRI_TOOLS=~/dev/gri-tools] [FS=2500000] run_gri.sh <center_hz> <cap.u8> [...]
# Example (the 2026-07-19 captures were LO 1620.6 MHz):
#   run_gri.sh 1620600000 ~/Downloads/iq-*.u8
# Output: a bursts/frames table + per-type histogram; .bits/.parsed/.stderr in
# $OUTDIR (default ./gri_out).
set -euo pipefail

GRI_TOOLS="${GRI_TOOLS:-/Users/bruce/dev/gri-tools}"
VENV="$GRI_TOOLS/venv"
TK="$GRI_TOOLS/iridium-toolkit"
PY="$VENV/bin/python"
EXTRACT="$VENV/bin/iridium-extractor"
PARSE="$TK/iridium-parser.py"
FS="${FS:-2500000}"

[ -x "$EXTRACT" ] || { echo "iridium-extractor not found at $EXTRACT — set GRI_TOOLS or install (see header)"; exit 1; }
CENTER="${1:?usage: run_gri.sh <center_hz> <file.u8> [...]}"; shift
OUTDIR="${OUTDIR:-$(pwd)/gri_out}"; mkdir -p "$OUTDIR"

printf '%-24s %10s %10s\n' file bursts frames
printf '%s\n' "--------------------------------------------------"
for f in "$@"; do
  b=$(basename "$f" .u8)
  "$PY" "$EXTRACT" -c "$CENTER" -r "$FS" -f cu8 -o "$f" \
      2>"$OUTDIR/$b.stderr" | grep '^RAW:' > "$OUTDIR/$b.bits" || true
  bursts=$(wc -l < "$OUTDIR/$b.bits" | tr -d ' ')
  # NOTE: the installed iridium-extractor emits "RAW:" bits in the *non-swapped*
  # symbol order, but this iridium-toolkit parser treats "RAW:" as needing a
  # per-symbol swap (swapped = tag != "RWA") and applies symbol_reverse(), which
  # corrupts every payload -> 0 frames. (The 24-sym UW is swap-invariant, so it
  # still matched and hid the bug.) Relabel RAW->RWA so the parser leaves the
  # bits as-is. Also force "-o line": with stdout redirected (not a tty) the
  # parser defaults to output=file and hijacks stdout into its own <base>.parsed,
  # so the shell redirect below would otherwise capture nothing. Drop "-p"
  # (--perfect): it contradicts --uw-ec and discards error-corrected frames.
  sed 's/^RAW:/RWA:/' "$OUTDIR/$b.bits" \
      | "$PY" "$PARSE" -o line --uw-ec /dev/stdin 2>/dev/null > "$OUTDIR/$b.parsed" || true
  # frames = successfully decoded (typed) lines; failed bursts print "... ERR:".
  frames=$(grep -cv ' ERR:' "$OUTDIR/$b.parsed" || true)
  printf '%-24s %10s %10s\n' "$b" "$bursts" "$frames"
  if [ "$frames" -gt 0 ]; then
    grep -v ' ERR:' "$OUTDIR/$b.parsed" | awk '{print $1}' | sort | uniq -c | sort -rn | sed 's/^/    /'
  fi
done
