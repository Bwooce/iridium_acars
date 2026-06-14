// rotate_to_dc.c — see header for design and why this lives in
// common/iridium_decoder/ rather than as two local copies.

#include "rotate_to_dc.h"
#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---- Q32 fraction-of-a-turn phase --------------------------------------
//
// The renorm used to compute `cos(phase_step * (double)index)` with
// double trig. P4's FPU is single-precision only, so every double op is
// soft-float — at one renorm per 128 samples that was ~625 soft-double
// trig pairs per 40 k-sample window and ~10 k per worst-case multi-frame
// burst, on the hottest worker stage. Plain cosf/sinf of the raw product
// is NOT a fix: phase reaches ~2e6 rad on long windows where float ULP
// is ~0.25 rad.
//
// Instead, keep phase as a 32-bit fraction of a turn. dphi_q32 is
// phase_step expressed in turns × 2^32; `dphi_q32 * index` then wraps
// modulo 2^32 — i.e. modulo one turn — EXACTLY, in integer math. The
// reduced [0,1)-turn fraction is small enough for single-precision trig
// (float resolution 2^-24 turns ≈ 4e-7 rad, far below the Q15 floor).
// Residual error vs the old double path: dphi quantisation ≤ 2^-33
// turns/sample ≈ 0.5 mrad accumulated over a 625 k-sample burst —
// irrelevant to DQPSK, which is differential.
static inline uint32_t rot_dphi_q32(double phase_step)
{
    double f = phase_step * (1.0 / (2.0 * M_PI)); // turns/sample
    f -= floor(f);                                // [0, 1)
    // llround may yield 2^32 when f rounds up from 1-eps; the uint32_t
    // conversion is defined modulo 2^32, wrapping that to 0 — correct.
    return (uint32_t)(unsigned long long)llround(f * 4294967296.0);
}

// Q15 phasor at absolute sample index `index` (modular: only the low 32
// bits of the index matter, by construction of the Q32 representation).
static inline void rot_phasor_q15(uint32_t dphi_q32, uint32_t index,
                                  int16_t *pr, int16_t *pi)
{
    uint32_t ph_q32 = dphi_q32 * index; // exact mod-one-turn
    float    rad    = (float)ph_q32 * (6.28318530717958647692f / 4294967296.0f);
    *pr             = (int16_t)lrintf(cosf(rad) * 32767.0f);
    *pi             = (int16_t)lrintf(sinf(rad) * 32767.0f);
}

void rotate_to_dc(int16_t *iq, int n_complex, double phase_step)
{
    // float (single-precision) cosf/sinf: P4 has only a single-
    // precision FPU (RV-32IMAFC), so double trig would be soft-
    // emulated. Single-precision epsilon (~1e-7) is well below the
    // Q15 quantisation floor (~3e-5 for amplitude 32767), so the
    // numerical accuracy difference vs the host's prior double
    // implementation is negligible. NMSE re-validated against gri at
    // step rebuild — see tests/host/test_pipeline_wideband_albq.
    // Q32 turn-fraction reduction (see rot_dphi_q32): `dphi_f * k` in
    // raw float lost precision once k·dphi exceeded a few thousand rad
    // (float ULP at 2e6 rad is ~0.25 rad).
    const uint32_t dphi_q32 = rot_dphi_q32(phase_step);
    for (int k = 0; k < n_complex; k++) {
        uint32_t ph_q32 = dphi_q32 * (uint32_t)k;
        float    phase  = (float)ph_q32 * (6.28318530717958647692f / 4294967296.0f);
        float    cs     = cosf(phase);
        float    ss     = sinf(phase);
        int32_t  r      = iq[k * 2 + 0];
        int32_t  v      = iq[k * 2 + 1];
        float    nr     = (float)r * cs - (float)v * ss;
        float    ni     = (float)r * ss + (float)v * cs;
        if (nr > 32767.0f) nr = 32767.0f;
        if (nr < -32768.0f) nr = -32768.0f;
        if (ni > 32767.0f) ni = 32767.0f;
        if (ni < -32768.0f) ni = -32768.0f;
        iq[k * 2 + 0] = (int16_t)lrintf(nr);
        iq[k * 2 + 1] = (int16_t)lrintf(ni);
    }
}

double rotate_to_dc_phase_step_from_bin(int center_bin, int fft_size)
{
    double rel_f = ((double)center_bin - (double)fft_size / 2.0) / (double)fft_size;
    return -2.0 * M_PI * rel_f;
}

// Renormalise the phasor every this many samples. The two error
// sources we trade off here:
//   - Q15 quantisation of (cs_q, ss_q): each per-step multiply loses
//     ~½ LSB to truncation/rounding, accumulating linearly. Over N
//     steps the per-sample output error grows ~N LSB.
//   - Cost of the renorm: one cos() + one sin() per renorm.
// 128 samples gives ~-50 dB NMSE on a long window (one renorm per
// 128 input samples; per-step ~½ LSB × 128 ≈ 64 LSB peak error).
// At burst-window scales (40k samples) that's ~312 renormalisations,
// total cos/sin cost ~6000 cycles — negligible vs the ~640k cycles
// for the per-sample work itself.
//
// For the future PIE int16 SIMD asm (task #58 step 2), 128 maps to
// a 16-iteration outer loop of 8-lane vector mul-add (16 × 8 = 128),
// keeping the renorm + inner-loop structure clean in assembly.
#define ROT_RENORM_PERIOD 128

// Q15 multiplication with round-to-nearest instead of truncation:
// add 2^14 before >>15. Halves the per-step bias of the incremental
// phasor (truncation has ~0.5 LSB negative drift per step; rounding
// has ~0 LSB systematic drift, only zero-mean random ±0.5 LSB).
// Pre-saturation; downstream caller saturates the int32 to int16.
static inline int32_t q15_mul_round(int32_t a, int32_t b)
{
    return (a * b + (1 << 14)) >> 15;
}

static inline int16_t q15_sat(int32_t x)
{
    if (x > 32767) return 32767;
    if (x < -32768) return -32768;
    return (int16_t)x;
}

// Chunked scalar reference for the PIE int16 SIMD path. The asm
// pipelines 8-lane Q15 complex-multiply via PIE's vmulas.s16; this C
// version does the same algorithmic work as 8 scalar muls per chunk
// so the chunk structure is bit-validatable on host (and as the
// fallback when ESP_PLATFORM isn't defined).
//
// Per chunk:
//   1. Build cs[0..7] / ss[0..7] by Q15-incrementing the running
//      phasor 8 steps forward (same path as rotate_to_dc_q15_inc).
//      Renormalise at the chunk boundary if we've reached the
//      renorm period — chunk-aligned, lower cost than the per-sample
//      renorm check.
//   2. For each lane k=0..7: out = in × (cs[k] + j·ss[k]) >> 15
//      with Q15 round-to-nearest and saturation, identical to
//      q15_inc's per-sample code.
//
// Tail samples (n_complex not a multiple of ROT_SIMD_LANES) are
// processed scalar at the end via the existing q15_inc per-sample
// path so the boundary doesn't need extra masking in the asm.
void rotate_to_dc_q15_simd_ref_at(int16_t *iq, int n_complex,
                                  double phase_step, int sample_offset)
{
    // Per-step phasor exp(j·phase_step) in Q15 via the reduced-argument
    // float path (index 1 of the Q32 turn accumulator). No double trig:
    // P4 soft-floats every double op, and these functions are called
    // per chunk (~157×/burst from the worker's chunked rotate).
    const uint32_t dphi_q32 = rot_dphi_q32(phase_step);
    int16_t        cs_q, ss_q;
    rot_phasor_q15(dphi_q32, 1u, &cs_q, &ss_q);

    int16_t pr_q               = 32767;
    int16_t pi_q               = 0;
    int     n_chunks_to_renorm = 0;

    int n_chunks   = n_complex / ROT_SIMD_LANES;
    int tail_start = n_chunks * ROT_SIMD_LANES;

    int16_t cs_lane[ROT_SIMD_LANES];
    int16_t ss_lane[ROT_SIMD_LANES];

    for (int c = 0; c < n_chunks; c++) {
        // Chunk-aligned renorm: replace pr_q, pi_q with the exact
        // absolute-phase quantisation at the start of this chunk.
        // sample_offset shifts the absolute-phase reference so the
        // chunked-with-offset path is bit-equivalent to a single
        // all-buffer rotate. Renorm cadence (every
        // ROT_RENORM_PERIOD/ROT_SIMD_LANES chunks) matches q15_inc's
        // ROT_RENORM_PERIOD samples.
        if (n_chunks_to_renorm == 0) {
            rot_phasor_q15(dphi_q32,
                           (uint32_t)(sample_offset + c * ROT_SIMD_LANES),
                           &pr_q, &pi_q);
            n_chunks_to_renorm = ROT_RENORM_PERIOD / ROT_SIMD_LANES;
        }
        n_chunks_to_renorm--;

        // Build the per-lane phasor table by incrementing the
        // accumulator 8 steps. Same Q15 incremental as q15_inc.
        for (int k = 0; k < ROT_SIMD_LANES; k++) {
            cs_lane[k]  = pr_q;
            ss_lane[k]  = pi_q;
            int32_t npr = q15_mul_round(pr_q, cs_q) - q15_mul_round(pi_q, ss_q);
            int32_t npi = q15_mul_round(pr_q, ss_q) + q15_mul_round(pi_q, cs_q);
            pr_q        = q15_sat(npr);
            pi_q        = q15_sat(npi);
        }

        // SIMD-style complex multiply across the 8 lanes. The asm
        // version does this 8-wide in one vmulas.s16 sequence; here
        // we just unroll it scalar so the test stays portable.
        int16_t *p = iq + (size_t)(c * ROT_SIMD_LANES) * 2;
        for (int k = 0; k < ROT_SIMD_LANES; k++) {
            int32_t r    = p[k * 2 + 0];
            int32_t v    = p[k * 2 + 1];
            int32_t nr   = q15_mul_round(r, cs_lane[k]) - q15_mul_round(v, ss_lane[k]);
            int32_t ni   = q15_mul_round(r, ss_lane[k]) + q15_mul_round(v, cs_lane[k]);
            p[k * 2 + 0] = q15_sat(nr);
            p[k * 2 + 1] = q15_sat(ni);
        }
    }

    // Tail (fewer than ROT_SIMD_LANES remaining samples). Reuse the
    // q15_inc per-sample path on the tail. Recompute the phasor at
    // the correct absolute phase first so the tail aligns with the
    // chunk-processed prefix. sample_offset shifts the absolute phase
    // so chunk-loop callers stay consistent across chunks.
    if (tail_start < n_complex) {
        rot_phasor_q15(dphi_q32, (uint32_t)(sample_offset + tail_start),
                       &pr_q, &pi_q);
        for (int k = tail_start; k < n_complex; k++) {
            int32_t r     = iq[k * 2 + 0];
            int32_t v     = iq[k * 2 + 1];
            int32_t nr    = q15_mul_round(r, pr_q) - q15_mul_round(v, pi_q);
            int32_t ni    = q15_mul_round(r, pi_q) + q15_mul_round(v, pr_q);
            iq[k * 2 + 0] = q15_sat(nr);
            iq[k * 2 + 1] = q15_sat(ni);
            int32_t npr   = q15_mul_round(pr_q, cs_q) - q15_mul_round(pi_q, ss_q);
            int32_t npi   = q15_mul_round(pr_q, ss_q) + q15_mul_round(pi_q, cs_q);
            pr_q          = q15_sat(npr);
            pi_q          = q15_sat(npi);
        }
    }
}

// ESP_PLATFORM PIE SIMD path. The 8-lane inner kernel
// rotate_q15_chunk_arp4 lives in rotate_to_dc_arp4.S; this C wrapper
// owns the outer per-chunk loop (renorm + phasor advance + de/inter-
// leave) so we can iterate on the inner kernel without re-doing the
// boilerplate. Same numerical contract as rotate_to_dc_q15_simd_ref_at
// within Q15 saturation rounding; validated on target via the
// scalar-vs-PIE diff diagnostic in smoke_test (see ROT_SIMD_DIAG).
#ifdef ESP_PLATFORM
#define ROT_SIMD_ARP4_AVAILABLE 1

extern void rotate_q15_chunk_arp4(const int16_t *I, const int16_t *Q,
                                  const int16_t *cs, const int16_t *ss,
                                  int16_t *nI, int16_t *nQ);

void rotate_to_dc_q15_simd_arp4_at(int16_t *iq, int n_complex,
                                   double phase_step, int sample_offset)
{
    // Per-step phasor exp(j·phase_step) in Q15 via the reduced-argument
    // float path (index 1 of the Q32 turn accumulator). No double trig:
    // P4 soft-floats every double op, and these functions are called
    // per chunk (~157×/burst from the worker's chunked rotate).
    const uint32_t dphi_q32 = rot_dphi_q32(phase_step);
    int16_t        cs_q, ss_q;
    rot_phasor_q15(dphi_q32, 1u, &cs_q, &ss_q);

    int16_t pr_q               = 32767;
    int16_t pi_q               = 0;
    int     n_chunks_to_renorm = 0;

    int n_chunks   = n_complex / ROT_SIMD_LANES;
    int tail_start = n_chunks * ROT_SIMD_LANES;

    // Stack-resident, 16-byte aligned scratch — PIE vld.128 needs
    // alignment, and we set the unaligned cfg bit anyway as a safety
    // net (see rotate_to_dc_arp4.S). The deinterleave + interleave
    // cost is ~16 cycles/chunk vs the ~8-lane SIMD inner kernel's
    // ~12 cycles/chunk; net per-chunk ~30 cycles vs scalar ~210.
    int16_t I_lane[ROT_SIMD_LANES] __attribute__((aligned(16)));
    int16_t Q_lane[ROT_SIMD_LANES] __attribute__((aligned(16)));
    int16_t cs_lane[ROT_SIMD_LANES] __attribute__((aligned(16)));
    int16_t ss_lane[ROT_SIMD_LANES] __attribute__((aligned(16)));
    int16_t nI[ROT_SIMD_LANES] __attribute__((aligned(16)));
    int16_t nQ[ROT_SIMD_LANES] __attribute__((aligned(16)));

    for (int c = 0; c < n_chunks; c++) {
        if (n_chunks_to_renorm == 0) {
            rot_phasor_q15(dphi_q32,
                           (uint32_t)(sample_offset + c * ROT_SIMD_LANES),
                           &pr_q, &pi_q);
            n_chunks_to_renorm = ROT_RENORM_PERIOD / ROT_SIMD_LANES;
        }
        n_chunks_to_renorm--;

        // Build the 8 per-lane phasor values via incremental Q15
        // advance — same exact arithmetic as the scalar reference so
        // a phasor-mismatch can't be the source of any output diff.
        for (int k = 0; k < ROT_SIMD_LANES; k++) {
            cs_lane[k]  = pr_q;
            ss_lane[k]  = pi_q;
            int32_t npr = q15_mul_round(pr_q, cs_q) - q15_mul_round(pi_q, ss_q);
            int32_t npi = q15_mul_round(pr_q, ss_q) + q15_mul_round(pi_q, cs_q);
            pr_q        = q15_sat(npr);
            pi_q        = q15_sat(npi);
        }

        // Deinterleave the 8 IQ pairs into split I/Q lanes for the
        // SIMD kernel (which operates on real-only int16 vectors).
        int16_t *p = iq + (size_t)(c * ROT_SIMD_LANES) * 2;
        for (int k = 0; k < ROT_SIMD_LANES; k++) {
            I_lane[k] = p[k * 2 + 0];
            Q_lane[k] = p[k * 2 + 1];
        }

        rotate_q15_chunk_arp4(I_lane, Q_lane, cs_lane, ss_lane, nI, nQ);

        // Re-interleave.
        for (int k = 0; k < ROT_SIMD_LANES; k++) {
            p[k * 2 + 0] = nI[k];
            p[k * 2 + 1] = nQ[k];
        }
    }

    // Tail (fewer than ROT_SIMD_LANES remaining samples) — scalar
    // per-sample, identical to _simd_ref_at's tail.
    if (tail_start < n_complex) {
        rot_phasor_q15(dphi_q32, (uint32_t)(sample_offset + tail_start),
                       &pr_q, &pi_q);
        for (int k = tail_start; k < n_complex; k++) {
            int32_t r     = iq[k * 2 + 0];
            int32_t v     = iq[k * 2 + 1];
            int32_t nr    = q15_mul_round(r, pr_q) - q15_mul_round(v, pi_q);
            int32_t ni    = q15_mul_round(r, pi_q) + q15_mul_round(v, pr_q);
            iq[k * 2 + 0] = q15_sat(nr);
            iq[k * 2 + 1] = q15_sat(ni);
            int32_t npr   = q15_mul_round(pr_q, cs_q) - q15_mul_round(pi_q, ss_q);
            int32_t npi   = q15_mul_round(pr_q, ss_q) + q15_mul_round(pi_q, cs_q);
            pr_q          = q15_sat(npr);
            pi_q          = q15_sat(npi);
        }
    }
}
#endif /* ESP_PLATFORM */

// Platform-best dispatcher. P4 firmware calls the PIE asm if it has
// been linked in (ROT_SIMD_ARP4_AVAILABLE); otherwise both paths
// land on the scalar chunked reference so behaviour is identical
// on host and target. Same input/output contract, including
// sample_offset semantics — see rotate_to_dc.h.
void rotate_to_dc_q15_simd_at(int16_t *iq, int n_complex,
                              double phase_step, int sample_offset)
{
#if defined(ESP_PLATFORM) && defined(ROT_SIMD_ARP4_AVAILABLE)
    rotate_to_dc_q15_simd_arp4_at(iq, n_complex, phase_step, sample_offset);
#else
    rotate_to_dc_q15_simd_ref_at(iq, n_complex, phase_step, sample_offset);
#endif
}

void rotate_to_dc_q15_inc(int16_t *iq, int n_complex, double phase_step)
{
    // Per-step phasor multiplier: exp(j·phase_step). Quantise to Q15.
    // Per-step phasor exp(j·phase_step) in Q15 via the reduced-argument
    // float path (index 1 of the Q32 turn accumulator). No double trig:
    // P4 soft-floats every double op, and these functions are called
    // per chunk (~157×/burst from the worker's chunked rotate).
    const uint32_t dphi_q32 = rot_dphi_q32(phase_step);
    int16_t        cs_q, ss_q;
    rot_phasor_q15(dphi_q32, 1u, &cs_q, &ss_q);

    // Running phasor, starts at (1, 0) ≡ exp(j·0).
    int16_t pr_q        = 32767;
    int16_t pi_q        = 0;
    int     n_to_renorm = ROT_RENORM_PERIOD; // force first iteration to set
                                             // pr/pi from the absolute phase
                                             // at k=0 (which is just (1,0),
                                             // but the same code path covers
                                             // start-from-nonzero callers if
                                             // we ever extend that)

    for (int k = 0; k < n_complex; k++) {
        // Renormalise periodically: replace the drifting Q15 phasor
        // with a fresh quantisation of cos(k·dphi), sin(k·dphi).
        if (n_to_renorm >= ROT_RENORM_PERIOD) {
            n_to_renorm = 0;
            rot_phasor_q15(dphi_q32, (uint32_t)k, &pr_q, &pi_q);
        }

        // Multiply input sample by phasor: out = in × (pr + j·pi) / 32768
        // Rounded Q15 × Q15: (a·b + 2^14) >> 15.
        int32_t r     = iq[k * 2 + 0];
        int32_t v     = iq[k * 2 + 1];
        int32_t nr    = q15_mul_round(r, pr_q) - q15_mul_round(v, pi_q);
        int32_t ni    = q15_mul_round(r, pi_q) + q15_mul_round(v, pr_q);
        iq[k * 2 + 0] = q15_sat(nr);
        iq[k * 2 + 1] = q15_sat(ni);

        // Advance phasor: p ← p · (cs_q + j·ss_q) / 32768.
        // Rounded Q15 multiplies (zero-mean per-step error vs the
        // ~½ LSB negative bias of truncation). The renormalisation
        // at ROT_RENORM_PERIOD restores |p| = 1.
        int32_t npr = q15_mul_round(pr_q, cs_q) - q15_mul_round(pi_q, ss_q);
        int32_t npi = q15_mul_round(pr_q, ss_q) + q15_mul_round(pi_q, cs_q);
        pr_q        = q15_sat(npr);
        pi_q        = q15_sat(npi);

        n_to_renorm++;
    }
}

void rotate_to_dc_q15_inc_at(int16_t *iq, int n_complex,
                             double phase_step, int sample_offset)
{
    // Same algorithm as rotate_to_dc_q15_inc but the absolute-phase
    // renorm uses (sample_offset + k) instead of k, so a burst rotated
    // chunk-by-chunk by the worker_core1 chunk loop matches what a
    // single all-in-one call would have produced (modulo Q15 saturation
    // rounding ordering). Each chunk's first sample force-renorms via
    // n_to_renorm = ROT_RENORM_PERIOD on entry — no inter-chunk state
    // is carried in the running phasor, the cos/sin reference fully
    // re-establishes it.
    // Per-step phasor exp(j·phase_step) in Q15 via the reduced-argument
    // float path (index 1 of the Q32 turn accumulator). No double trig:
    // P4 soft-floats every double op, and these functions are called
    // per chunk (~157×/burst from the worker's chunked rotate).
    const uint32_t dphi_q32 = rot_dphi_q32(phase_step);
    int16_t        cs_q, ss_q;
    rot_phasor_q15(dphi_q32, 1u, &cs_q, &ss_q);

    int16_t pr_q        = 32767;
    int16_t pi_q        = 0;
    int     n_to_renorm = ROT_RENORM_PERIOD;

    for (int k = 0; k < n_complex; k++) {
        if (n_to_renorm >= ROT_RENORM_PERIOD) {
            n_to_renorm = 0;
            rot_phasor_q15(dphi_q32, (uint32_t)(sample_offset + k),
                           &pr_q, &pi_q);
        }
        int32_t r     = iq[k * 2 + 0];
        int32_t v     = iq[k * 2 + 1];
        int32_t nr    = q15_mul_round(r, pr_q) - q15_mul_round(v, pi_q);
        int32_t ni    = q15_mul_round(r, pi_q) + q15_mul_round(v, pr_q);
        iq[k * 2 + 0] = q15_sat(nr);
        iq[k * 2 + 1] = q15_sat(ni);
        int32_t npr   = q15_mul_round(pr_q, cs_q) - q15_mul_round(pi_q, ss_q);
        int32_t npi   = q15_mul_round(pr_q, ss_q) + q15_mul_round(pi_q, cs_q);
        pr_q          = q15_sat(npr);
        pi_q          = q15_sat(npi);
        n_to_renorm++;
    }
}
