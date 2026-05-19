// rotate_to_dc.h — shared absolute-phase Q15 IQ rotation for the
// wideband direct-IF front end. Used by:
//   - p4-usb-host/main/worker_core1.c  (firmware per-burst path)
//   - tests/host/test_pipeline_wideband_albq.c  (host wideband test)
//
// Why a shared module instead of two local copies: the algorithm has
// a subtle correctness trap — a naïve Q15 incremental phasor
// `p ← p · exp(j·dphi)` with `>>15` rounding decays magnitude by
// ~0.012%/sample (memory/feedback_q15_incremental_phasor_decays.md),
// collapsing the output amplitude over the 100k-sample window of a
// wideband burst. Both call sites must compute the phasor from
// ABSOLUTE phase per sample (`cs = cos(k·dphi), ss = sin(k·dphi)`)
// to match gr-iridium's volk path. Co-locating the implementation
// makes that contract explicit and harder to lose under refactor.

#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Rotate `iq` (n_complex × interleaved int16 IQ) by exp(j·dphi·k) in
// place, where k is the sample index (0..n_complex-1).
//
// `phase_step` is the per-sample phase advance in radians. For a
// burst tagged at FFT bin `b` with FFT size N, to shift it to DC,
// pass `phase_step = -2·π · (b - N/2) / N`.
//
// Implementation: per-sample cosf/sinf in float (matches gri's volk
// rotator) and float Q15 saturation. No Q15 incremental phasor — see
// module header for why. Reference implementation — accurate but
// slow on RV-32IMF (cosf/sinf fall back to a software polynomial
// since the FPU has no hardware sin/cos). ~10 µs per sample on P4.
// Use rotate_to_dc_lut() for the same operation ~10× faster.
void rotate_to_dc(int16_t *iq, int n_complex, double phase_step);

// Q15 incremental phasor variant: same input/output contract as
// rotate_to_dc, but advances a Q15 phasor incrementally
// (4 muls + 1 shift per sample) and renormalises against an
// absolute-phase cosf/sinf reference every ROT_RENORM_PERIOD
// samples to prevent the magnitude-decay trap documented in the
// module header.
//
// Validated against rotate_to_dc to NMSE ≤ -50 dB. ~30× faster
// than the cosf/sinf reference on RV-32IMF (per-sample work is
// 4 int16 muls instead of 2 software-polynomial trig calls).
//
// This kernel is structurally what the eventual PIE int16 SIMD
// asm will implement — the same Q15 complex-multiply per sample
// against a periodically-refreshed phasor, just vectorised 8 lanes
// wide. Keeping the scalar version here both as a host-testable
// reference and as the fallback when PIE isn't available.
void rotate_to_dc_q15_inc(int16_t *iq, int n_complex, double phase_step);

// Convenience: convert (FFT center bin, FFT size) to the per-sample
// phase_step that rotate_to_dc expects.
double rotate_to_dc_phase_step_from_bin(int center_bin, int fft_size);

#ifdef __cplusplus
}
#endif
