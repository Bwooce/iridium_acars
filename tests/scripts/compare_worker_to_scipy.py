#!/usr/bin/env python3
"""
Compare OUR worker DSP chain (stage-1 FIR + stage-2 polyphase resampler)
against scipy's resample_poly on the SAME input.

This is the equivalence test the user asked for: if our worker DSP is
algorithmically equivalent to gr-iridium / scipy, the per-sample
outputs should match within numerical noise. Big SNR differences
indicate a real DSP bug, not just precision quirks.

We synthesise a known input (preamble + UW pattern at the Iridium
symbol rate, with band-shaping), pass it through:
  - Path A: scipy.signal.resample_poly (high-quality polyphase
            with anti-aliasing — what gr-iridium effectively does
            when it converts SDR sample rates)
  - Path B: our worker chain: 64-tap Hamming-windowed sinc LP +
            decim 32, then 64×25-tap polyphase resampler 25/8

Then compares the outputs by:
  - RMS ratio (gain match)
  - Cross-correlation peak (signal alignment)
  - Per-sample max abs diff (precision)

Run: python3 tests/scripts/compare_worker_to_scipy.py
"""
import numpy as np
from scipy.signal import resample_poly

# Sample-rate configuration mirrors the worker.
FS_IN = 2_560_000
DECIM_STAGE1 = 32                # 2.56M -> 80k
INTERP_STAGE2 = 25               # 80k * 25 -> 2M
DECIM_STAGE2 = 8                 # 2M / 8 -> 250k
POST_CORR_DECIM = 5              # 250k / 5 -> 50k = 2 sps
FS_2SPS = FS_IN // DECIM_STAGE1 * INTERP_STAGE2 // DECIM_STAGE2 // POST_CORR_DECIM
assert FS_2SPS == 50_000, FS_2SPS

# Our stage-1 FIR design — mirrors worker_core1.c init.
FIR_TAPS = 64
POLYCHAN_M = 64
RESAMPLE_TAPS = 64

def build_stage1_fir():
    omega_c = np.pi / POLYCHAN_M
    n = np.arange(FIR_TAPS) - (FIR_TAPS - 1) / 2.0
    sinc = np.where(np.abs(n) < 1e-9, omega_c / np.pi,
                    np.sin(omega_c * n) / (np.pi * n))
    hamming = 0.5 * (1.0 - np.cos(2.0 * np.pi * np.arange(FIR_TAPS) / (FIR_TAPS - 1)))
    taps = sinc * hamming
    return taps / taps.sum()   # DC gain 1.0

def build_stage2_resampler():
    rN = RESAMPLE_TAPS * INTERP_STAGE2
    stage2_poly_rate = (FS_IN / DECIM_STAGE1) * INTERP_STAGE2     # 2 MHz
    channel_half_bw = FS_IN / (2.0 * POLYCHAN_M)                  # 20 kHz
    omega_c = 2.0 * np.pi * channel_half_bw / stage2_poly_rate
    n = np.arange(rN) - (rN - 1) / 2.0
    sinc = np.where(np.abs(n) < 1e-9, omega_c / np.pi,
                    np.sin(omega_c * n) / (np.pi * n))
    hamming = 0.5 * (1.0 - np.cos(2.0 * np.pi * np.arange(rN) / (rN - 1)))
    taps = sinc * hamming
    return taps / taps.sum() * INTERP_STAGE2     # gain matches polyphase convention

def our_worker_chain(x, stage1_taps, stage2_taps):
    """Replicate worker_core1.c stage-1 + stage-2 in pure numpy.
    No esp-dsp involved; just the DSP design itself."""
    # Stage 1: LP + decim by 32 (full convolution then take every 32nd sample).
    y1_full = np.convolve(x, stage1_taps, mode='same')
    y1 = y1_full[::DECIM_STAGE1]
    # Stage 2: polyphase resampler 25/8. scipy's resample_poly is the
    # canonical reference for this — passing our designed taps directly.
    y2 = resample_poly(y1, up=INTERP_STAGE2, down=DECIM_STAGE2, window=stage2_taps)
    # Post-correlation decim by 5 (worker does this after pre-rotation).
    y3 = y2[::POST_CORR_DECIM]
    return y1, y2, y3

def scipy_direct_chain(x):
    """Reference: scipy.resample_poly takes 2.56M -> 50k directly
    (up=1, down=2560000/50000=51.2 — not integer, so do it as two
    stages with rational ratios).
    2.56M -> 50k = factor 51.2 (256/5). Use up=5, down=256."""
    y = resample_poly(x, up=5, down=256)
    return y

def main():
    # Synthesise a test signal: BPSK-modulated pulses on the Iridium
    # symbol grid (25 kHz) with RRC pulse shape — what an Iridium
    # burst looks like at our channel rate. Add some white noise to
    # make SNR measurement meaningful.
    np.random.seed(42)
    duration_s = 0.020   # 20 ms = covers ~one TDMA burst
    n = int(duration_s * FS_IN)
    t = np.arange(n) / FS_IN

    # 30 symbols at 25 ksym/s with RRC β=0.4
    n_syms = 50
    syms = np.random.choice([-1+1j, 1-1j, 1+1j, -1-1j], n_syms)
    sym_period = FS_IN // 25_000     # 102.4 samples — not integer but
                                       # numpy convolve handles it
    sym_positions = np.zeros(n, dtype=np.complex64)
    for i, s in enumerate(syms):
        idx = int(i * sym_period)
        if idx < n:
            sym_positions[idx] = s

    # RRC pulse shape at FS_IN (long impulse response)
    rrc_len = 2048
    rrc_t = (np.arange(rrc_len) - rrc_len // 2) / FS_IN
    sym_T = 1.0 / 25_000
    beta = 0.4
    tau = rrc_t / sym_T
    # Standard RRC formula
    eps = 1e-9
    num = np.sin(np.pi * tau * (1 - beta)) + 4 * beta * tau * np.cos(np.pi * tau * (1 + beta))
    den = np.pi * tau * (1 - (4 * beta * tau) ** 2)
    rrc = np.where(np.abs(tau) < eps, 1 - beta + 4*beta/np.pi, num / den)
    rrc[np.abs(den) < eps] = 0
    rrc /= np.sqrt(np.sum(rrc**2))   # unit energy

    signal = np.convolve(sym_positions, rrc, mode='same')

    # Add white noise at 25 dB SNR (per-sample, after pulse shaping)
    sig_pow = np.mean(np.abs(signal)**2)
    noise_pow = sig_pow * 10**(-25 / 10)
    noise = (np.random.randn(n) + 1j * np.random.randn(n)) * np.sqrt(noise_pow / 2)
    x = (signal + noise).astype(np.complex64)

    # Pass through both chains
    stage1_taps = build_stage1_fir()
    stage2_taps = build_stage2_resampler()
    y1, y2, y_ours = our_worker_chain(x, stage1_taps, stage2_taps)
    y_scipy = scipy_direct_chain(x)

    # Trim to common length (resample_poly produces slightly different lengths)
    n_common = min(len(y_ours), len(y_scipy))
    y_ours = y_ours[:n_common]
    y_scipy = y_scipy[:n_common]

    # Compare
    print(f"Input length: {n} samples at {FS_IN} Hz = {duration_s*1000:.0f} ms")
    print(f"Output length: {n_common} samples at {FS_2SPS} Hz = {n_common/FS_2SPS*1000:.1f} ms")
    print()
    print(f"Stage 1 (LP + decim 32) output RMS: {np.sqrt(np.mean(np.abs(y1)**2)):.4f}")
    print(f"Stage 2 (resample 25/8) output RMS: {np.sqrt(np.mean(np.abs(y2)**2)):.4f}")
    print(f"Final (decim 5)         output RMS: {np.sqrt(np.mean(np.abs(y_ours)**2)):.4f}")
    print(f"Scipy direct (5/256)    output RMS: {np.sqrt(np.mean(np.abs(y_scipy)**2)):.4f}")
    print()

    # Cross-correlate to align (the two paths may have different group delays)
    corr = np.correlate(y_ours, y_scipy, mode='full')
    peak_idx = np.argmax(np.abs(corr))
    lag = peak_idx - (len(y_scipy) - 1)
    print(f"Cross-correlation peak lag: {lag} samples")

    # Align and compare
    if lag > 0:
        a = y_ours[lag:]
        b = y_scipy[:len(a)]
    else:
        b = y_scipy[-lag:]
        a = y_ours[:len(b)]

    rms_a = np.sqrt(np.mean(np.abs(a)**2))
    rms_b = np.sqrt(np.mean(np.abs(b)**2))
    print(f"Aligned RMS: ours={rms_a:.4f}, scipy={rms_b:.4f}, ratio={rms_a/rms_b:.4f}")
    # Normalize for comparison
    a_n = a / rms_a
    b_n = b / rms_b
    diff = a_n - b_n
    rms_diff = np.sqrt(np.mean(np.abs(diff)**2))
    print(f"Normalized RMS difference: {rms_diff:.6f}")
    print(f"Normalized max abs diff:   {np.max(np.abs(diff)):.6f}")
    snr_match_db = 20.0 * np.log10(1.0 / (rms_diff + 1e-12))
    print(f"Equivalence SNR (1/diff): {snr_match_db:.1f} dB")

    if snr_match_db > 30:
        print("\nPASS: paths agree to better than 30 dB (effectively equivalent).")
    elif snr_match_db > 15:
        print(f"\nMARGINAL: {snr_match_db:.1f} dB equivalence — bandlimit/transient differences.")
    else:
        print(f"\nFAIL: {snr_match_db:.1f} dB equivalence — significant DSP divergence.")


if __name__ == "__main__":
    main()
