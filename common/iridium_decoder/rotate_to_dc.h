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
// Validated against rotate_to_dc to NMSE ≤ -40 dB. ~5× faster
// than the cosf/sinf reference on RV-32IMF (per-sample work is
// 4 int16 muls instead of 2 software-polynomial trig calls).
void rotate_to_dc_q15_inc(int16_t *iq, int n_complex, double phase_step);

// Same as rotate_to_dc_q15_inc but treats `iq[0]` as if it were
// sample `sample_offset` in a larger logical stream — i.e. uses
// phase `(sample_offset + k) * phase_step` for output sample k.
// Used by the worker_core1 chunk loop to maintain phase continuity
// across chunks: a burst is rotated in N internal-SRAM chunks of
// CHUNK samples each, with this function called per chunk and
// sample_offset = chunk_index * CHUNK so the absolute-phase
// renorm at each ROT_RENORM_PERIOD boundary lines up exactly
// with what a single all-in-one rotate_to_dc_q15_inc would have
// produced (within Q15 saturation rounding).
void rotate_to_dc_q15_inc_at(int16_t *iq, int n_complex,
                              double phase_step, int sample_offset);

// Chunked scalar reference for the PIE int16 SIMD path. Same
// numerics as rotate_to_dc_q15_inc but structured in 8-sample
// chunks to match how the PIE asm pipelines: per chunk, first
// build an 8-pack of phasor values (cs[0..7], ss[0..7]) by Q15-
// incrementing an internal accumulator, then apply a vector
// complex-multiply across the 8 input samples. The asm version
// (rotate_to_dc_q15_simd_arp4_at, P4-only) does the inner SIMD
// multiply in PIE; this scalar reference does it as 8 separate
// Q15 muls so the same code can be host-tested.
//
// `sample_offset` is the burst-global sample index at which iq[0]
// sits — needed by the chunk loop in worker_core1.c so phase
// continuity holds when the worker rotates a burst piece-by-piece.
// Pass 0 for single-shot whole-buffer rotation.
//
// Validated against rotate_to_dc_q15_inc_at to NMSE ≤ -60 dB
// (bit-exact except for ordering of saturation rounding) — see
// tests/host/test_rotate_to_dc.c.
//
// Use this on the host AND as the on-target fallback when PIE
// isn't available. The P4 firmware swaps to the PIE asm via a
// thin wrapper (see rotate_to_dc_q15_simd_at below).
void rotate_to_dc_q15_simd_ref_at(int16_t *iq, int n_complex,
                                   double phase_step, int sample_offset);

// Backwards-compat wrapper: sample_offset = 0.
static inline void rotate_to_dc_q15_simd_ref(int16_t *iq, int n_complex,
                                              double phase_step)
{
    rotate_to_dc_q15_simd_ref_at(iq, n_complex, phase_step, 0);
}

// Platform-best dispatcher: P4 firmware calls the PIE asm
// (rotate_to_dc_q15_simd_arp4_at), host build calls the scalar
// reference. Same input/output contract; `sample_offset` semantics
// as above. This is what worker_core1's chunk loop calls.
void rotate_to_dc_q15_simd_at(int16_t *iq, int n_complex,
                               double phase_step, int sample_offset);

// Backwards-compat wrapper: sample_offset = 0.
static inline void rotate_to_dc_q15_simd(int16_t *iq, int n_complex,
                                          double phase_step)
{
    rotate_to_dc_q15_simd_at(iq, n_complex, phase_step, 0);
}

// Width of the SIMD chunk (in complex samples). Exposed so callers
// or test harnesses can pad inputs to a multiple of this if they
// want to avoid the per-call tail-handling cost.
#define ROT_SIMD_LANES 8

// Convenience: convert (FFT center bin, FFT size) to the per-sample
// phase_step that rotate_to_dc expects.
double rotate_to_dc_phase_step_from_bin(int center_bin, int fft_size);

#ifdef __cplusplus
}
#endif
