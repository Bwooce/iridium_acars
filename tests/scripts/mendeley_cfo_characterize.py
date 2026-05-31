#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""
mendeley_cfo_characterize.py — process the FULL Mendeley Iridium dataset
(3.8 M IRA bursts) through cfo_fine_estimate's squared-FFT peak-finding
logic, emitting per-burst statistics to CSV. The analyzer in
mendeley_cfo_analyze.py then derives a data-driven threshold for #115.

Why this exists: #115's CFO peak-confidence filter (4× peak/second-peak)
was guessed, not measured. It rejected the LEGITIMATE target burst on
real RF and tanked decode (RAW 3→0, REAL 6→0 — see commit 78f0d57's
revert message). Re-applying it needs an empirical peak distribution
across many bursts. 3.8 M is overkill but the dataset is what it is;
the script is streaming (constant memory) so size doesn't matter.

Reads from the cache populated by fetch_mendeley_iridium.py. Streaming
parser: one record at a time, never holds more than one burst's IQ in
memory.

Output CSV columns (one row per burst):
  ts, sat_id, beam_id, freq_hz, n_complex,
  peak_mag, second_mag, mean_mag, median_mag,
  peak_over_second, peak_over_mean, peak_over_median

The downstream analyzer turns these into a histogram + percentile
table + threshold recommendation.

Usage:
    python3 tests/scripts/mendeley_cfo_characterize.py \\
        > ~/iq_cache/cfo_stats.csv
    # Optionally only one of the two files (faster):
    python3 tests/scripts/mendeley_cfo_characterize.py \\
        --files 1109-0910_20_parsed.txt \\
        > ~/iq_cache/cfo_stats.csv
"""

import argparse
import os
import re
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import fetch_mendeley_iridium as fetch     # noqa: E402

# Match cfo_fine_estimate's FFT size on host/device.
# Source: common/iridium_decoder/uw_correlator.c:736  #define CFO_FFT_N 4096
CFO_FFT_N = 4096
# Main-lobe exclusion radius for second-peak search (uw_correlator.c #115).
LOBE_RADIUS = 2

# Regex matching one complex sample: (re+imJ) where re/im may be in
# scientific notation, sign included.
RE_COMPLEX = re.compile(
    r"\(\s*([+-]?\d+\.?\d*(?:[eE][+-]?\d+)?)"
    r"\s*([+-])\s*"
    r"(\d+\.?\d*(?:[eE][+-]?\d+)?)\s*j\s*\)"
)


def parse_iq(text: str) -> np.ndarray:
    """Parse a 'IQ-cell' string into a complex64 array."""
    parts = RE_COMPLEX.findall(text)
    if not parts:
        return np.empty(0, dtype=np.complex64)
    n = len(parts)
    re_arr = np.empty(n, dtype=np.float32)
    im_arr = np.empty(n, dtype=np.float32)
    for i, (re_s, sign, im_s) in enumerate(parts):
        re_arr[i] = float(re_s)
        im_arr[i] = float(im_s) * (1.0 if sign == "+" else -1.0)
    return re_arr + 1j * im_arr


def cfo_stats(iq: np.ndarray) -> dict:
    """Replicate cfo_fine_estimate's squared-FFT peak-finding statistics.

    Sequence mirrors uw_correlator.c:cfo_fine_estimate (the float path,
    CFO_USE_Q15_FFT=0):
      1. Square the input (s = iq * iq)
      2. Window (we use a Hann to approximate; the production code uses
         s_cfo_window_full_f — for STATISTICS this is close enough)
      3. Zero-pad to CFO_FFT_N
      4. Magnitude² of the FFT bins
      5. Peak (max), second-peak (max outside ±LOBE_RADIUS of main),
         mean, median

    Returns a dict; caller decides which to emit."""
    if iq.size == 0:
        return None

    # Step 1: square (Iridium preamble is BPSK — squaring doubles the
    # carrier and produces a DC peak in the squared signal).
    sq = iq * iq

    # Step 2: window. Production uses an asymmetric truncated window;
    # for distribution characterization Hann is sufficient.
    n_in = min(sq.size, CFO_FFT_N)
    win = np.hanning(n_in).astype(np.float32)
    x = np.zeros(CFO_FFT_N, dtype=np.complex64)
    x[:n_in] = sq[:n_in] * win

    # Step 4: FFT and magnitude².
    X = np.fft.fft(x)
    mag2 = (X.real ** 2 + X.imag ** 2).astype(np.float64)

    # Step 5: peak finding, with main-lobe-excluded second peak.
    peak_k = int(np.argmax(mag2))
    peak_mag = float(mag2[peak_k])
    if peak_mag <= 0.0:
        return None

    # Mask the main lobe (peak_k ± LOBE_RADIUS, mod N).
    mask = np.ones(CFO_FFT_N, dtype=bool)
    for dk in range(-LOBE_RADIUS, LOBE_RADIUS + 1):
        mask[(peak_k + dk) % CFO_FFT_N] = False
    off_peak = mag2[mask]
    second_mag = float(off_peak.max())
    mean_mag = float(off_peak.mean())
    median_mag = float(np.median(off_peak))

    return {
        "n_complex": sq.size,
        "peak_mag": peak_mag,
        "second_mag": second_mag,
        "mean_mag": mean_mag,
        "median_mag": median_mag,
        "peak_over_second": peak_mag / second_mag if second_mag > 0 else float("inf"),
        "peak_over_mean":   peak_mag / mean_mag   if mean_mag > 0   else float("inf"),
        "peak_over_median": peak_mag / median_mag if median_mag > 0 else float("inf"),
    }


def stream_records(src_files: list, max_records: int):
    """Yield (meta_dict, iq_str_position) per record. The IQ string is
    NOT parsed at this stage — too expensive to do speculatively for
    records we may filter on metadata first."""
    seen = 0
    for src in src_files:
        with open(src, "r", buffering=1 << 20, errors="replace") as f:
            for line in f:
                if seen >= max_records:
                    return
                line = line.rstrip("\n")
                # Tab-separated metadata followed by the IQ cell.
                tab9 = -1
                for i in range(9):
                    p = line.find("\t", tab9 + 1)
                    if p < 0:
                        break
                    tab9 = p
                if tab9 < 0:
                    continue
                meta_str = line[:tab9]
                iq_str = line[tab9 + 1:]
                fields = meta_str.split("\t")
                if len(fields) < 9:
                    continue
                try:
                    meta = {
                        "ts":      float(fields[0]),
                        "pkt_id":  int(fields[1]),
                        "sat_id":  int(fields[2]),
                        "beam_id": int(fields[3]),
                        "lat":     float(fields[4]),
                        "lon":     float(fields[5]),
                        "alt":     float(fields[6]),
                        "conf":    float(fields[7]),
                        "freq_hz": float(fields[8]),
                    }
                except ValueError:
                    continue
                yield meta, iq_str
                seen += 1


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--files", nargs="+", default=None,
                   help="restrict to specific filename(s) under the extracted dir")
    p.add_argument("--max", type=int, default=10**9,
                   help="max records to process (default: all)")
    p.add_argument("--progress-every", type=int, default=10000,
                   help="emit progress to stderr every N records")
    args = p.parse_args()

    ed = fetch.extract_dir()
    if not ed.exists() or not any(ed.iterdir()):
        sys.stderr.write(f"ERROR: extracted dir {ed} missing; run "
                         "fetch_mendeley_iridium.py first\n")
        return 1

    if args.files:
        src_files = [ed / f for f in args.files]
        for f in src_files:
            if not f.exists():
                sys.stderr.write(f"ERROR: {f} not found\n")
                return 1
    else:
        src_files = sorted(ed.glob("*.txt"))

    sys.stderr.write(f"processing: {[str(f.name) for f in src_files]}\n")
    sys.stderr.write(f"max records: {args.max}\n")

    cols = ("ts", "sat_id", "beam_id", "freq_hz", "n_complex",
            "peak_mag", "second_mag", "mean_mag", "median_mag",
            "peak_over_second", "peak_over_mean", "peak_over_median")
    print(",".join(cols))

    n_processed = 0
    n_skipped = 0
    t0 = time.time()
    for meta, iq_str in stream_records(src_files, args.max):
        iq = parse_iq(iq_str)
        stats = cfo_stats(iq)
        if stats is None:
            n_skipped += 1
            continue
        print(",".join((
            f"{meta['ts']:.3f}",
            str(meta["sat_id"]),
            str(meta["beam_id"]),
            str(int(meta["freq_hz"])),
            str(stats["n_complex"]),
            f"{stats['peak_mag']:.6e}",
            f"{stats['second_mag']:.6e}",
            f"{stats['mean_mag']:.6e}",
            f"{stats['median_mag']:.6e}",
            f"{stats['peak_over_second']:.3f}",
            f"{stats['peak_over_mean']:.3f}",
            f"{stats['peak_over_median']:.3f}",
        )))
        n_processed += 1
        if n_processed % args.progress_every == 0:
            dt = time.time() - t0
            rate = n_processed / dt
            sys.stderr.write(f"  {n_processed:>10} processed  ({rate:.0f}/s, "
                             f"{n_skipped} skipped)\n")
            sys.stderr.flush()

    dt = time.time() - t0
    sys.stderr.write(f"DONE: {n_processed} processed, {n_skipped} skipped "
                     f"in {dt:.1f} s ({n_processed/dt:.0f}/s)\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
