"""
Apply the gr-iridium squared-FFT CFO math to BOTH the gr-iridium
post-D13 dump and the host post-D13 dump for the same burst. Plot
the magnitude-squared spectrum from each — if both pipelines see
the same signal at this stage, the spectra should be near-identical
and the peak bin should match.

This isolates whether the host's CFO is picking a different peak
because of input-signal differences (precision, alignment) or
because of the FFT/window/peak-picker implementation.

Usage:
  python3 tests/scripts/squared_fft_compare.py --burst-id 30

Requires (gr-iridium and host) post-D13 dumps in /tmp/signals and
/tmp/host_signals respectively.
"""
import argparse
import struct
from pathlib import Path

CFO_FFT_N = 4096        # matches gr-iridium d_cfo_est_fft_size × fft_over_size_facor
CFO_INPUT_N = 256       # gr-iridium d_cfo_est_fft_size (= 2.56 MSPS × log2(...))
FS_HZ = 250_000


def read_cf32(path: Path):
    if not path.exists():
        return None
    raw = path.read_bytes()
    n = len(raw) // 8
    floats = struct.unpack(f"<{n*2}f", raw)
    return [complex(floats[2*i], floats[2*i+1]) for i in range(n)]


def squared_fft(x, n_in, n_fft):
    """Apply Blackman window over first n_in samples, square, FFT to
    n_fft bins (zero-padded). Returns magnitude² array of length n_fft."""
    import math
    if len(x) < n_in:
        n_in = len(x)
    # Blackman window: w[n] = 0.42 - 0.5*cos(2πn/(N-1)) + 0.08*cos(4πn/(N-1))
    win = [0.42 - 0.5*math.cos(2*math.pi*n/(n_in-1))
                 + 0.08*math.cos(4*math.pi*n/(n_in-1)) for n in range(n_in)]
    # Square: (a+jb)^2 = (a²-b²) + j·2ab
    sq = [None] * n_fft
    for n in range(n_fft):
        if n < n_in:
            r, i = x[n].real, x[n].imag
            sq_r = r*r - i*i
            sq_i = 2*r*i
            sq[n] = complex(sq_r * win[n], sq_i * win[n])
        else:
            sq[n] = 0
    # DFT via numpy (the only non-stdlib piece, but keeps this fast)
    import numpy as np
    arr = np.array(sq, dtype=np.complex128)
    spec = np.fft.fft(arr)
    return np.abs(spec) ** 2


def peak_with_neighbors(mag_sq, n_top=10):
    """Top n_top bins by magnitude², with negative-freq wrap."""
    import numpy as np
    n = len(mag_sq)
    idx = np.argsort(mag_sq)[::-1][:n_top]
    out = []
    for k in idx:
        signed = k if k < n // 2 else k - n
        # Squared-FFT bin → carrier residual in Hz: peak at 2·f_c, so
        # freq_hz = signed × fs / N, carrier_residual = freq_hz / 2.
        f_2c = signed * FS_HZ / n
        carrier = f_2c / 2.0
        out.append((signed, f_2c, carrier, float(mag_sq[k])))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--burst-id", type=int, required=True)
    args = ap.parse_args()

    gr_path = Path(f"/tmp/signals/signal-filtered-deci-cut-start-{args.burst_id}.cfile")
    host_path = Path("/tmp/host_signals/04_post_d13_250k.cf32")

    gr = read_cf32(gr_path)
    host = read_cf32(host_path)
    if gr is None:
        print(f"missing {gr_path}")
        return
    if host is None:
        print(f"missing {host_path}")
        return

    print(f"gr-iridium burst {args.burst_id}: post-D13 n={len(gr)}, "
          f"host post-D13 n={len(host)}")
    print()

    gr_spec = squared_fft(gr, CFO_INPUT_N, CFO_FFT_N)
    host_spec = squared_fft(host, CFO_INPUT_N, CFO_FFT_N)

    gr_peaks = peak_with_neighbors(gr_spec, 10)
    host_peaks = peak_with_neighbors(host_spec, 10)

    print(f"gr-iridium squared-FFT top peaks (window={CFO_INPUT_N}, fft={CFO_FFT_N}):")
    print(f"  {'bin':>5}  {'f_2c_Hz':>9}  {'carrier_Hz':>11}  magnitude²")
    for k, f2c, fc, mag in gr_peaks:
        print(f"  {k:>+5}  {f2c:>+9.0f}  {fc:>+11.0f}  {mag:.3e}")
    print()
    print(f"host squared-FFT top peaks:")
    print(f"  {'bin':>5}  {'f_2c_Hz':>9}  {'carrier_Hz':>11}  magnitude²")
    for k, f2c, fc, mag in host_peaks:
        print(f"  {k:>+5}  {f2c:>+9.0f}  {fc:>+11.0f}  {mag:.3e}")

    # Same peak?
    if gr_peaks[0][0] == host_peaks[0][0]:
        print(f"\n✓ Peak bin matches: {gr_peaks[0][0]} "
              f"(carrier residual = {gr_peaks[0][2]:+.0f} Hz)")
    else:
        print(f"\n✗ DIVERGE: gri peak at bin {gr_peaks[0][0]} "
              f"({gr_peaks[0][2]:+.0f} Hz), host at bin {host_peaks[0][0]} "
              f"({host_peaks[0][2]:+.0f} Hz)")
        # Magnitude ratio gri.peak / gri.host's peak's-magnitude-on-gri-spec
        host_bin_on_gri = gr_spec[host_peaks[0][0] if host_peaks[0][0] >= 0
                                  else CFO_FFT_N + host_peaks[0][0]]
        print(f"  → host's peak bin on gri spectrum: mag² = {host_bin_on_gri:.3e} "
              f"(gri's actual peak: {gr_peaks[0][3]:.3e})")


if __name__ == "__main__":
    main()
