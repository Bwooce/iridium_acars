#!/usr/bin/env python3
"""Resample a phaseb ci16 cut (10 MSPS, center 1622 MHz) to the device's
2.56 MSPS ingest rate for host-chain validation.

The device front end ingests cu8/ci16 at ~2.56 MSPS (RTL-SDR style),
then resample_256_to_250 (125/128) → 2.5 MSPS feeds the FFT burst
tagger. This script produces the 2.56 MSPS int16 IQ stream that
test_phaseb_cut.c consumes (it runs the firmware resample_256_to_250
itself, mirroring test_pipeline_wideband_resampled).

Steps:
  1. Read the 10 MSPS ci16 cut (interleaved int16 I/Q).
  2. Mix so the chosen device LO sits at DC.
  3. resample_poly 10 MSPS → 2.56 MSPS (up=32, down=125 → exact).
  4. Scale to int16 peak ~20000 (linear, SNR-preserving, no clipping).
  5. Write interleaved int16 to the output path.

Usage:
  python3 tests/scripts/phaseb_cut_to_256.py IN.ci16 OUT.ci16 [LO_HZ]
"""
import sys
import numpy as np
from scipy.signal import resample_poly

SRC_RATE = 10_000_000
SRC_CENTER = 1_622_000_000
DST_RATE = 2_560_000          # 10e6 * 32/125 = 2_560_000 exactly
UP, DOWN = 32, 125
DEFAULT_LO = 1_620_600_000    # device's typical LO (freq-scanner note)
PEAK_TARGET = 20000.0


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    in_path = sys.argv[1]
    out_path = sys.argv[2]
    lo_hz = int(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_LO

    x = np.fromfile(in_path, dtype=np.int16).astype(np.float32)
    cf = (x[0::2] + 1j * x[1::2]).astype(np.complex64)
    print(f"loaded {len(cf)} complex @ {SRC_RATE/1e6:g} MSPS center "
          f"{SRC_CENTER/1e6:g} MHz ({len(cf)/SRC_RATE:.3f} s)")

    # Mix chosen LO to DC. Baseband freq of a signal at f_abs is
    # (f_abs - SRC_CENTER). To move LO to DC we shift by -(LO - CENTER).
    shift = lo_hz - SRC_CENTER
    n = np.arange(len(cf), dtype=np.float64)
    phasor = np.exp(-2j * np.pi * shift / SRC_RATE * n).astype(np.complex64)
    mixed = cf * phasor
    print(f"mixed LO={lo_hz/1e6:.3f} MHz to DC (shift {shift/1e3:+.1f} kHz)")

    dec = resample_poly(mixed, up=UP, down=DOWN).astype(np.complex64)
    print(f"resample up={UP} down={DOWN} → {len(dec)} complex @ "
          f"{SRC_RATE*UP/DOWN/1e6:.4f} MSPS")

    peak = float(np.max(np.abs(dec)))
    g = PEAK_TARGET / max(1.0, peak)
    di = np.clip(np.real(dec) * g, -32767, 32767).astype(np.int16)
    dq = np.clip(np.imag(dec) * g, -32767, 32767).astype(np.int16)
    print(f"scale gain={g:.4f} (peak {peak:.0f} → {PEAK_TARGET:.0f})")

    out = np.empty(2 * len(dec), dtype=np.int16)
    out[0::2] = di
    out[1::2] = dq
    out.tofile(out_path)
    print(f"wrote {out_path}: {len(dec)} complex int16 @ {DST_RATE} Hz")

    # Report where the two VH-8IC fragments should land in the 2.56 MSPS
    # stream (for sanity vs the tagger output). Manifest: fragments at
    # abs 1620751919 / 1620751909 Hz.
    for f_abs in (1620751919, 1620751909):
        rel = f_abs - lo_hz
        print(f"  burst @ {f_abs} Hz → {rel/1e3:+.1f} kHz from LO "
              f"(bin offset ~{rel/(DST_RATE*125/128/2048):.1f} @ 2.5MSPS/2048)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
