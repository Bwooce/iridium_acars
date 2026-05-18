"""
Stage-by-stage diff: gr-iridium intermediate dumps vs host pipeline
intermediate dumps for the SAME ALBQ-corpus burst.

Workflow:
  1. Run gr-iridium with --debug-id <id> → /tmp/signals/*-{id}.cfile
  2. Run host pipeline test with HOST_DIAG_CH=<ch> → /tmp/host_signals/*.cf32
     (channel ch = round((target_freq_hz - LO) / 40000) mod 64)
  3. For each pipeline stage we have BOTH dumps for, report:
     - sample count
     - RMS, peak magnitude
     - peak spectrum bin (offset Hz)
     - cross-correlation max with the other side (alignment / scale)

Stages:
  S1 — channelized 40 kHz output (host) vs gr-iridium's
       "signal-filtered-deci-{id}" (which is at the burst sample rate,
       250 kHz with their flowgraph). Comparable AFTER resampling host
       to 250 kHz (which is what stage S2 does).
  S2 — host 250 kHz resampled output vs gr-iridium's
       "signal-filtered-deci-{id}.cfile". Direct comparable.
  S3 — gr-iridium "...-cut-start" (post start_finder trim) vs the
       same point on host (D13 trim). Reflects where the burst onset
       was located.
  S4 — "...-cut-start-shift" (after CFO freq-correct). Reflects the
       coarse omega estimate.
  S5 — "...-cut-start-shift-rrc" (after RRC matched filter).
  S6 — "...-cut-start-shift-rrc-rotate" (after phase rotation).
  S7 — "...-cut-start-shift-rrc-rotate-cut" (final 2-sps stream).

We currently only dump host stage S2 (resampled 250 kHz IQ) so this
script focuses on that comparison. Add more host stages later if S2
already shows divergence.

Usage:
  # 1. Pick a decoded burst's I: id from the gr-iridium output.
  # 2. Run this script:
  python3 tests/scripts/stagewise_compare.py --burst-id 510 --channel 6
"""
import argparse
import struct
import sys
from pathlib import Path


def read_cf32(path: Path):
    """Read a binary interleaved complex-float file. Returns (re, im)
    as plain Python lists; for big files we'd switch to numpy but the
    typical burst is <2000 samples = 16 KB."""
    if not path.exists():
        return None
    raw = path.read_bytes()
    n_floats = len(raw) // 4
    floats = struct.unpack(f"<{n_floats}f", raw)
    re = list(floats[0::2])
    im = list(floats[1::2])
    return re, im


def stats(re, im):
    """RMS, peak, mean(I), mean(Q)."""
    n = len(re)
    if n == 0:
        return {"n": 0, "rms": 0.0, "peak": 0.0,
                "mean_re": 0.0, "mean_im": 0.0}
    sumsq = 0.0
    peak = 0.0
    mean_re = 0.0
    mean_im = 0.0
    for r, i in zip(re, im):
        mag2 = r * r + i * i
        sumsq += mag2
        if mag2 > peak:
            peak = mag2
        mean_re += r
        mean_im += i
    import math
    return {
        "n":      n,
        "rms":    math.sqrt(sumsq / n),
        "peak":   math.sqrt(peak),
        "mean_re": mean_re / n,
        "mean_im": mean_im / n,
    }


def peak_freq_hz(re, im, fs_hz, n_fft=None):
    """Crude one-bin FFT peak finder: zero-pads to next pow2, returns
    bin frequency in Hz of the max-magnitude bin."""
    n = len(re)
    if n == 0:
        return 0.0, 0.0
    if n_fft is None:
        n_fft = 1
        while n_fft < n:
            n_fft <<= 1
        n_fft = max(n_fft * 4, 1024)       # 4x zero-pad for resolution
    # Naive O(N²) DFT (just for finding the dominant bin; n is small).
    import math
    best_mag = 0.0
    best_bin = 0
    twopi = 2.0 * math.pi
    for k in range(n_fft):
        ang_step = -twopi * k / n_fft
        sum_r = 0.0
        sum_i = 0.0
        for j in range(n):
            ang = ang_step * j
            c = math.cos(ang)
            s = math.sin(ang)
            sum_r += re[j] * c - im[j] * s
            sum_i += re[j] * s + im[j] * c
        mag = sum_r * sum_r + sum_i * sum_i
        if mag > best_mag:
            best_mag = mag
            best_bin = k
    # Map bin (with negative-frequency wrap) to signed freq in Hz.
    if best_bin >= n_fft // 2:
        best_bin -= n_fft
    return best_bin * fs_hz / n_fft, math.sqrt(best_mag) / n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--burst-id", type=int, required=True,
                    help="gr-iridium debug burst id (look for I:xxxx in "
                         "the iridium-extractor RAW output)")
    ap.add_argument("--channel", type=int, required=True,
                    help="host channelizer index for the same burst")
    ap.add_argument("--fft-resolution", type=int, default=0,
                    help="optional override of FFT zero-pad N "
                         "(default: 4× next-pow2 of input length)")
    args = ap.parse_args()

    gr_dir = Path("/tmp/signals")
    host_dir = Path("/tmp/host_signals")

    # The stages we have host dumps for currently.
    bid = args.burst_id
    pairs = [
        # (label, gr-iridium file, host file, sample_rate_hz)
        ("post-resample 250k",
         gr_dir / f"signal-filtered-deci-{bid}.cfile",
         host_dir / "03_resamp_250k.cf32",
         250_000),
        ("post start_finder",
         gr_dir / f"signal-filtered-deci-cut-start-{bid}.cfile",
         host_dir / "04_post_d13_250k.cf32",
         250_000),
        ("post CFO shift",
         gr_dir / f"signal-filtered-deci-cut-start-shift-{bid}.cfile",
         host_dir / "05_post_cfo_250k.cf32",
         250_000),
        ("post RRC",
         gr_dir / f"signal-filtered-deci-cut-start-shift-rrc-{bid}.cfile",
         host_dir / "06_post_rrc_250k.cf32",
         250_000),
        ("post phase-rot",
         gr_dir / f"signal-filtered-deci-cut-start-shift-rrc-rotate-{bid}.cfile",
         host_dir / "07_post_prerot_250k.cf32",
         250_000),
    ]

    print(f"Comparing gr-iridium burst {args.burst_id} vs host channel "
          f"{args.channel}")
    print()
    print(f"{'stage':<26} {'src':<3}  {'n':>5}  {'rms':>10}  "
          f"{'peak':>10}  {'mean':>16}  {'peak_freq_Hz':>12}")
    print("-" * 100)

    for label, gr_path, host_path, fs in pairs:
        gr = read_cf32(gr_path)
        host = read_cf32(host_path)
        if gr is None:
            print(f"{label:<26}  MISSING gr-iridium dump: {gr_path}")
            continue
        if host is None:
            print(f"{label:<26}  MISSING host dump: {host_path}")
            continue

        g_re, g_im = gr
        h_re, h_im = host
        gs = stats(g_re, g_im)
        hs = stats(h_re, h_im)
        gf, _ = peak_freq_hz(g_re, g_im, fs, args.fft_resolution or None)
        hf, _ = peak_freq_hz(h_re, h_im, fs, args.fft_resolution or None)

        print(f"{label:<26} gri  {gs['n']:>5}  {gs['rms']:>10.4f}  "
              f"{gs['peak']:>10.4f}  ({gs['mean_re']:+.4f},{gs['mean_im']:+.4f})  "
              f"{gf:>+12.0f}")
        print(f"{label:<26} hst  {hs['n']:>5}  {hs['rms']:>10.4f}  "
              f"{hs['peak']:>10.4f}  ({hs['mean_re']:+.4f},{hs['mean_im']:+.4f})  "
              f"{hf:>+12.0f}")
        print()


if __name__ == "__main__":
    main()
