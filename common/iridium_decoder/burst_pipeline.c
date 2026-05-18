// See burst_pipeline.h for design rationale and pipeline ordering.

#include "burst_pipeline.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef UW_SPS
// Tests/host stubs don't always pull the same UW_SPS source as the
// production headers; keep a local copy synchronised with
// uw_correlator.h.
#define UW_SPS  10
#endif

#define POST_CORR_DECIM 5

static inline int16_t q15_saturate(int32_t x)
{
    if (x > INT16_MAX) return (int16_t)INT16_MAX;
    if (x < INT16_MIN) return (int16_t)INT16_MIN;
    return (int16_t)x;
}

static inline int16_t q15_from_float(float f)
{
    int32_t q = (int32_t)lrintf(f * 32767.0f);
    return q15_saturate(q);
}

// Apply a Q15 linear phase ramp exp(+j·dphi·n) in place to a complex
// int16 IQ stream. Used both for the coarse pre-RRC freq-correction
// and (with peak-phase initial value + sub-sample interp) for the
// final post-UW pre-rotation step.
static void q15_freq_shift_inplace(int16_t *iq, int n_complex,
                                    int16_t pr_q_init, int16_t pi_q_init,
                                    int16_t cs_q, int16_t ss_q)
{
    int16_t pr_q = pr_q_init;
    int16_t pi_q = pi_q_init;
    for (int i = 0; i < n_complex; i++) {
        int32_t r = iq[i * 2 + 0];
        int32_t v = iq[i * 2 + 1];
        int32_t nr = ((int32_t)r * pr_q - (int32_t)v * pi_q) >> 15;
        int32_t ni = ((int32_t)r * pi_q + (int32_t)v * pr_q) >> 15;
        iq[i * 2 + 0] = q15_saturate(nr);
        iq[i * 2 + 1] = q15_saturate(ni);
        // Advance phasor: p ← p · (cs_q + j·ss_q) = p · exp(j·dphi).
        int32_t npr = ((int32_t)pr_q * cs_q - (int32_t)pi_q * ss_q) >> 15;
        int32_t npi = ((int32_t)pr_q * ss_q + (int32_t)pi_q * cs_q) >> 15;
        pr_q = q15_saturate(npr);
        pi_q = q15_saturate(npi);
    }
}

bool burst_pipeline_process_250khz(int16_t *iq250, int n_complex,
                                    burst_pipeline_result_t *result)
{
    memset(result, 0, sizeof(*result));

    // 1. D13 envelope start_finder. Search the entire burst (the
    //    start_finder's internal LP filter smooths noise, so wider
    //    search doesn't add false positives).
    int burst_start = uw_correlator_find_burst_start(
                          iq250, n_complex, /*search_max=*/n_complex);
    result->burst_start = burst_start;
    int16_t *adj_burst = iq250 + burst_start * 2;
    int adj_n = n_complex - burst_start;
    if (adj_n < UW_SPS * 28) {
        // Less than one full sync word worth — can't even run the
        // matched filter; bail out cleanly.
        return false;
    }

    // 2. Pre-RRC squared-FFT CFO estimate on the trimmed burst.
    float omega_coarse = uw_correlator_estimate_cfo(adj_burst, adj_n);
    result->omega_coarse = omega_coarse;

    // 3. Phase-correct adj_burst in place by exp(+j·omega/sps·n).
    //    Sign convention matches uw_correlator_estimate_cfo: the
    //    returned omega is the NEGATIVE of the actual offset, so
    //    multiplying by exp(+j·omega/sps·n) cancels it.
    if (omega_coarse != 0.0f) {
        float dphi = omega_coarse / (float)UW_SPS;
        q15_freq_shift_inplace(adj_burst, adj_n,
                               q15_from_float(1.0f),     // initial pr
                               q15_from_float(0.0f),     // initial pi
                               q15_from_float(cosf(dphi)),
                               q15_from_float(sinf(dphi)));
    }

    // 4. RRC matched filter.
    uw_correlator_apply_rrc(adj_burst, adj_burst, adj_n);

    // 5. UW correlator (matched filter peak + direction + residual CFO).
    uw_correlator_find(adj_burst, adj_n,
                       /*search_complex=*/adj_n - 24,
                       &result->uw_res);
    if (result->uw_res.direction == UW_DIR_UNKNOWN) {
        return true;            // pipeline ran, no decode
    }

    // 6. Sub-sample timing correction + peak-phase pre-rotation +
    //    residual-omega linear phase ramp, all combined in one pass
    //    with linear interpolation between adjacent samples.
    float true_pos = (float)result->uw_res.uw_offset
                   + result->uw_res.correction;
    int   int_base = (int)floorf(true_pos);
    float interp_frac = true_pos - (float)int_base;
    if (int_base < 0) { int_base = 0; interp_frac = 0.0f; }
    int16_t *src = adj_burst + int_base * 2;
    int n_rot = adj_n - int_base;

    float pmag = sqrtf(result->uw_res.peak_re * result->uw_res.peak_re
                     + result->uw_res.peak_im * result->uw_res.peak_im);
    if (pmag <= 1e-3f) {
        return true;            // pipeline ran, peak too weak to rotate
    }
    float rot_re = result->uw_res.peak_re / pmag;
    float rot_im = result->uw_res.peak_im / pmag;
    float dphi   = result->uw_res.omega_per_sym / (float)UW_SPS;

    int16_t pr_q = q15_from_float(rot_re);
    int16_t pi_q = q15_from_float(rot_im);
    int16_t cs_q = q15_from_float(cosf(dphi));
    int16_t ss_q = q15_from_float(sinf(dphi));
    int16_t a_q  = q15_from_float(1.0f - interp_frac);
    int16_t b_q  = q15_from_float(interp_frac);

    // n_rot includes src[0..n_rot-1]; the linear interp uses src[i+1]
    // so the loop bound is n_rot - 1.
    int n_cplx = n_rot - 1;
    if (n_cplx < 0) n_cplx = 0;
    for (int i = 0; i < n_cplx; i++) {
        int32_t re = ((int32_t)a_q * src[i * 2 + 0]
                    + (int32_t)b_q * src[(i + 1) * 2 + 0]) >> 15;
        int32_t im = ((int32_t)a_q * src[i * 2 + 1]
                    + (int32_t)b_q * src[(i + 1) * 2 + 1]) >> 15;
        int32_t nr = ((int32_t)re * pr_q - (int32_t)im * pi_q) >> 15;
        int32_t ni = ((int32_t)re * pi_q + (int32_t)im * pr_q) >> 15;
        src[i * 2 + 0] = q15_saturate(nr);
        src[i * 2 + 1] = q15_saturate(ni);
        int32_t npr = ((int32_t)pr_q * cs_q - (int32_t)pi_q * ss_q) >> 15;
        int32_t npi = ((int32_t)pr_q * ss_q + (int32_t)pi_q * cs_q) >> 15;
        pr_q = q15_saturate(npr);
        pi_q = q15_saturate(npi);
    }

    // 7. 5:1 decimation 10 sps → 2 sps. In-place: read every 5th
    //    complex sample, write back to the start of `src`.
    int n_post_cplx = n_cplx / POST_CORR_DECIM;
    for (int i = 0; i < n_post_cplx; i++) {
        int j = i * POST_CORR_DECIM;
        src[i * 2 + 0] = src[j * 2 + 0];
        src[i * 2 + 1] = src[j * 2 + 1];
    }
    result->n_post_2sps = n_post_cplx;

    // 8. qpsk_demod.
    if (qpsk_demod_process(src, n_post_cplx * 2, &result->frame)) {
        result->demod_ok = true;
    }
    return true;
}
