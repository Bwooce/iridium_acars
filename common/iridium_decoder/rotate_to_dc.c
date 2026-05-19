// rotate_to_dc.c — see header for design and why this lives in
// common/iridium_decoder/ rather than as two local copies.

#include "rotate_to_dc.h"
#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void rotate_to_dc(int16_t *iq, int n_complex, double phase_step)
{
    // float (single-precision) cosf/sinf: P4 has only a single-
    // precision FPU (RV-32IMAFC), so double trig would be soft-
    // emulated. Single-precision epsilon (~1e-7) is well below the
    // Q15 quantisation floor (~3e-5 for amplitude 32767), so the
    // numerical accuracy difference vs the host's prior double
    // implementation is negligible. NMSE re-validated against gri at
    // step rebuild — see tests/host/test_pipeline_wideband_albq.
    const float dphi_f = (float)phase_step;
    for (int k = 0; k < n_complex; k++) {
        float phase = dphi_f * (float)k;
        float cs = cosf(phase);
        float ss = sinf(phase);
        int32_t r = iq[k * 2 + 0];
        int32_t v = iq[k * 2 + 1];
        float nr = (float)r * cs - (float)v * ss;
        float ni = (float)r * ss + (float)v * cs;
        if (nr >  32767.0f) nr =  32767.0f;
        if (nr < -32768.0f) nr = -32768.0f;
        if (ni >  32767.0f) ni =  32767.0f;
        if (ni < -32768.0f) ni = -32768.0f;
        iq[k * 2 + 0] = (int16_t)lrintf(nr);
        iq[k * 2 + 1] = (int16_t)lrintf(ni);
    }
}

double rotate_to_dc_phase_step_from_bin(int center_bin, int fft_size)
{
    double rel_f = ((double)center_bin - (double)fft_size / 2.0)
                   / (double)fft_size;
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
    if (x >  32767) return  32767;
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
void rotate_to_dc_q15_simd_ref(int16_t *iq, int n_complex,
                                double phase_step)
{
    double cs_d = cos(phase_step);
    double ss_d = sin(phase_step);
    int16_t cs_q = (int16_t)lrint(cs_d * 32767.0);
    int16_t ss_q = (int16_t)lrint(ss_d * 32767.0);

    int16_t pr_q = 32767;
    int16_t pi_q = 0;
    int n_chunks_to_renorm = 0;

    int n_chunks = n_complex / ROT_SIMD_LANES;
    int tail_start = n_chunks * ROT_SIMD_LANES;

    int16_t cs_lane[ROT_SIMD_LANES];
    int16_t ss_lane[ROT_SIMD_LANES];

    for (int c = 0; c < n_chunks; c++) {
        // Chunk-aligned renorm: replace pr_q, pi_q with the exact
        // absolute-phase quantisation at the start of this chunk.
        // Doing this every ROT_RENORM_PERIOD/ROT_SIMD_LANES chunks
        // keeps the renorm cadence equivalent to q15_inc's
        // ROT_RENORM_PERIOD samples.
        if (n_chunks_to_renorm == 0) {
            double phase = phase_step * (double)(c * ROT_SIMD_LANES);
            pr_q = (int16_t)lrint(cos(phase) * 32767.0);
            pi_q = (int16_t)lrint(sin(phase) * 32767.0);
            n_chunks_to_renorm = ROT_RENORM_PERIOD / ROT_SIMD_LANES;
        }
        n_chunks_to_renorm--;

        // Build the per-lane phasor table by incrementing the
        // accumulator 8 steps. Same Q15 incremental as q15_inc.
        for (int k = 0; k < ROT_SIMD_LANES; k++) {
            cs_lane[k] = pr_q;
            ss_lane[k] = pi_q;
            int32_t npr = q15_mul_round(pr_q, cs_q) - q15_mul_round(pi_q, ss_q);
            int32_t npi = q15_mul_round(pr_q, ss_q) + q15_mul_round(pi_q, cs_q);
            pr_q = q15_sat(npr);
            pi_q = q15_sat(npi);
        }

        // SIMD-style complex multiply across the 8 lanes. The asm
        // version does this 8-wide in one vmulas.s16 sequence; here
        // we just unroll it scalar so the test stays portable.
        int16_t *p = iq + (size_t)(c * ROT_SIMD_LANES) * 2;
        for (int k = 0; k < ROT_SIMD_LANES; k++) {
            int32_t r = p[k * 2 + 0];
            int32_t v = p[k * 2 + 1];
            int32_t nr = q15_mul_round(r, cs_lane[k])
                       - q15_mul_round(v, ss_lane[k]);
            int32_t ni = q15_mul_round(r, ss_lane[k])
                       + q15_mul_round(v, cs_lane[k]);
            p[k * 2 + 0] = q15_sat(nr);
            p[k * 2 + 1] = q15_sat(ni);
        }
    }

    // Tail (fewer than ROT_SIMD_LANES remaining samples). Reuse the
    // q15_inc per-sample path on the tail. Recompute the phasor at
    // the correct absolute phase first so the tail aligns with the
    // chunk-processed prefix.
    if (tail_start < n_complex) {
        double phase = phase_step * (double)tail_start;
        pr_q = (int16_t)lrint(cos(phase) * 32767.0);
        pi_q = (int16_t)lrint(sin(phase) * 32767.0);
        for (int k = tail_start; k < n_complex; k++) {
            int32_t r = iq[k * 2 + 0];
            int32_t v = iq[k * 2 + 1];
            int32_t nr = q15_mul_round(r, pr_q) - q15_mul_round(v, pi_q);
            int32_t ni = q15_mul_round(r, pi_q) + q15_mul_round(v, pr_q);
            iq[k * 2 + 0] = q15_sat(nr);
            iq[k * 2 + 1] = q15_sat(ni);
            int32_t npr = q15_mul_round(pr_q, cs_q) - q15_mul_round(pi_q, ss_q);
            int32_t npi = q15_mul_round(pr_q, ss_q) + q15_mul_round(pi_q, cs_q);
            pr_q = q15_sat(npr);
            pi_q = q15_sat(npi);
        }
    }
}

// Platform-best dispatcher. On the P4 firmware this would forward to
// the PIE asm (rotate_to_dc_q15_simd_arp4 — to be added under
// ESP_PLATFORM). For now both paths use the scalar reference so the
// dispatcher is functional on host AND target without the asm in
// place yet.
void rotate_to_dc_q15_simd(int16_t *iq, int n_complex, double phase_step)
{
#if defined(ESP_PLATFORM) && defined(ROT_SIMD_ARP4_AVAILABLE)
    rotate_to_dc_q15_simd_arp4(iq, n_complex, phase_step);
#else
    rotate_to_dc_q15_simd_ref(iq, n_complex, phase_step);
#endif
}

void rotate_to_dc_q15_inc(int16_t *iq, int n_complex, double phase_step)
{
    // Per-step phasor multiplier: exp(j·phase_step). Quantise to Q15.
    double cs_d = cos(phase_step);
    double ss_d = sin(phase_step);
    int16_t cs_q = (int16_t)lrint(cs_d * 32767.0);
    int16_t ss_q = (int16_t)lrint(ss_d * 32767.0);

    // Running phasor, starts at (1, 0) ≡ exp(j·0).
    int16_t pr_q = 32767;
    int16_t pi_q = 0;
    int n_to_renorm = ROT_RENORM_PERIOD;   // force first iteration to set
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
            double phase = phase_step * (double)k;
            pr_q = (int16_t)lrint(cos(phase) * 32767.0);
            pi_q = (int16_t)lrint(sin(phase) * 32767.0);
        }

        // Multiply input sample by phasor: out = in × (pr + j·pi) / 32768
        // Rounded Q15 × Q15: (a·b + 2^14) >> 15.
        int32_t r = iq[k * 2 + 0];
        int32_t v = iq[k * 2 + 1];
        int32_t nr = q15_mul_round(r, pr_q) - q15_mul_round(v, pi_q);
        int32_t ni = q15_mul_round(r, pi_q) + q15_mul_round(v, pr_q);
        iq[k * 2 + 0] = q15_sat(nr);
        iq[k * 2 + 1] = q15_sat(ni);

        // Advance phasor: p ← p · (cs_q + j·ss_q) / 32768.
        // Rounded Q15 multiplies (zero-mean per-step error vs the
        // ~½ LSB negative bias of truncation). The renormalisation
        // at ROT_RENORM_PERIOD restores |p| = 1.
        int32_t npr = q15_mul_round(pr_q, cs_q) - q15_mul_round(pi_q, ss_q);
        int32_t npi = q15_mul_round(pr_q, ss_q) + q15_mul_round(pi_q, cs_q);
        pr_q = q15_sat(npr);
        pi_q = q15_sat(npi);

        n_to_renorm++;
    }
}
