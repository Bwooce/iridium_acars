#!/bin/bash
# One HydraSDR -> POA (acarsdec) + VDL2 (dumpvdl2) decoded in parallel via the
# numpy channelizer (channelize.py). VHF ACARS coverage check across both
# technologies from a single 10 MSPS capture. Logs per-band decode output.
#
# Prereqs (macOS): hydrasdr_rx, sox, python3+numpy on PATH; dumpvdl2 +
# acarsdec built under ~/dev/vdl2-tools (see repo memory). Antenna on the
# HydraSDR (VHF airband). Usage: run_dualband.sh [logdir]
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGDIR="${1:-/tmp/vhf-dualband}"
mkdir -p "$LOGDIR"

export DYLD_LIBRARY_PATH="$HOME/dev/vdl2-tools/prefix/lib"
DUMPVDL2="$HOME/dev/vdl2-tools/prefix/bin/dumpvdl2"
ACARSDEC="$HOME/dev/vdl2-tools/acarsdec/build/acarsdec"
PY="$(command -v python3 || echo /opt/homebrew/bin/python3)"
FIFO="$LOGDIR/poa.wav"
rm -f "$FIFO"; mkfifo "$FIFO"

echo "logs -> $LOGDIR ; Ctrl-C to stop"

# POA reader (blocks on the fifo until the channelizer opens the write end)
"$ACARSDEC" -o 1 -i BF-YSSY-ACARS -f "$FIFO" > "$LOGDIR/poa.log" 2>&1 &

# VDL2 chain: channelizer -> sox 500k->420k -> dumpvdl2 (6 VDL2 channels)
( "$PY" "$HERE/channelize.py" "$FIFO" 2>"$LOGDIR/chan.err" \
  | sox -t raw -r 500000 -e signed -b 16 -c 2 - -t raw -r 420000 -c 2 - 2>"$LOGDIR/sox.err" \
  | "$DUMPVDL2" --iq-file - --sample-format S16_LE --centerfreq 136900000 --oversample 4 \
      --utc --milliseconds \
      136700000 136725000 136775000 136800000 136975000 137000000 \
      > "$LOGDIR/vdl2.log" 2>&1 ) &

wait
