// See burst_pipeline.h for design rationale and pipeline ordering.

#include "burst_pipeline.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// One-shot diagnostic dump state. Set via burst_pipeline_set_dump_once();
// after the next process_250khz call consumes it, it's auto-cleared so
// only one burst gets dumped.
static char s_dump_dir[256] = {0};
static int  s_dump_pending = 0;

// One-shot D13 override. -1 = use D13 normally.
static int  s_force_burst_start = -1;

void burst_pipeline_force_start_once(int sample_idx)
{
    s_force_burst_start = sample_idx;
}

void burst_pipeline_set_dump_once(const char *dir)
{
    if (dir) {
        strncpy(s_dump_dir, dir, sizeof(s_dump_dir) - 1);
        s_dump_dir[sizeof(s_dump_dir) - 1] = 0;
        s_dump_pending = 1;
    } else {
        s_dump_dir[0] = 0;
        s_dump_pending = 0;
    }
}

static void dump_iq_cf32(const char *fname, const int16_t *iq, int n_complex)
{
    if (!s_dump_pending || s_dump_dir[0] == 0) return;
    char path[400];
    snprintf(path, sizeof(path), "%s/%s.cf32", s_dump_dir, fname);
    FILE *f = fopen(path, "wb");
    if (!f) return;
    for (int i = 0; i < n_complex; i++) {
        float fr = (float)iq[i * 2 + 0] / 32768.0f;
        float fi = (float)iq[i * 2 + 1] / 32768.0f;
        fwrite(&fr, 4, 1, f);
        fwrite(&fi, 4, 1, f);
    }
    fclose(f);
    fprintf(stderr, "    [burst-pipeline-dump] %s: %d cplx\n",
            path, n_complex);
}

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

    // 1. D13 envelope start_finder. Search depth matches gr-iridium's
    //    burst_downmix exactly: 0.007 × burst_sample_rate = 1750
    //    samples at 250 ksps (= ~7 ms). With our 17.6 ms padded
    //    window (gr-iridium-aligned: 1.6 ms pre + 16 ms post) a wider
    //    search lets D13 lock onto a SECOND burst that happens to
    //    follow the original within the post-padding window — its
    //    envelope peak then becomes the argmax → wrong start position
    //    → matched filter operates on the wrong burst → 0 decodes.
    //    Cap at 7 ms so start_finder always finds the FIRST envelope
    //    rise (the actual burst we got tagged for).
    int burst_start;
    if (s_force_burst_start >= 0) {
        burst_start = s_force_burst_start;
        if (burst_start >= n_complex) burst_start = 0;
        s_force_burst_start = -1;
    } else {
        const int SEARCH_DEPTH_SAMPLES = 1750;
        int search_depth = SEARCH_DEPTH_SAMPLES;
        if (search_depth > n_complex) search_depth = n_complex;
        burst_start = uw_correlator_find_burst_start(
                          iq250, n_complex, /*search_max=*/search_depth);
    }
    result->burst_start = burst_start;
    int16_t *adj_burst = iq250 + burst_start * 2;
    int adj_n = n_complex - burst_start;
    if (adj_n < UW_SPS * 28) {
        // Less than one full sync word worth — can't even run the
        // matched filter; bail out cleanly.
        return false;
    }

    dump_iq_cf32("04_post_d13_250k", adj_burst, adj_n);

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

    dump_iq_cf32("05_post_cfo_250k", adj_burst, adj_n);

    // 4. RRC matched filter.
    uw_correlator_apply_rrc(adj_burst, adj_burst, adj_n);

    dump_iq_cf32("06_post_rrc_250k", adj_burst, adj_n);

    // 5. UW correlator (matched filter peak + direction + residual CFO).
    //    Limit the search range to (PREAMBLE_LONG + UW + 8 syms) × sps
    //    = (64 + 12 + 8) × 10 = 840 samples, matching gr-iridium's
    //    d_sync_search_len exactly. Wider search lets noise beat the
    //    real UW peak — particularly painful for DL whose constant-
    //    carrier preamble gives a wide triangular autocorrelation
    //    plateau (~32 samples wide) instead of a sharp peak.
    const int SYNC_SEARCH_LEN = (64 + 12 + 8) * UW_SPS;     // 840
    int search_complex = SYNC_SEARCH_LEN;
    if (search_complex > adj_n - 24) search_complex = adj_n - 24;
    uw_correlator_find(adj_burst, adj_n,
                       search_complex,
                       &result->uw_res);
    if (result->uw_res.direction == UW_DIR_UNKNOWN) {
        return true;            // pipeline ran, no decode
    }

    // 6a. Constant-phase pre-rotation on the FULL adj_burst, matching
    //     gr-iridium burst_downmix_impl.cc:692-694 exactly:
    //       d_r.set_phase_incr(exp(gr_complex(0, 0)));    // no freq ramp
    //       d_r.set_phase(std::conj(corr/abs(corr)));     // peak conj
    //       d_r.rotateN(d_tmp_b, d_tmp_a, frame_size);    // rotate ALL
    //     gri then trims the rotated buffer by uw_start later. We
    //     mirror that order: rotate everything in adj_burst first,
    //     then apply our sub-sample interp + trim as a separate
    //     downstream-only step (step 6b). This keeps our
    //     07_post_prerot dump comparable to gri's
    //     signal-filtered-deci-cut-start-shift-rrc-rotate-<id>.cfile
    //     (same window boundaries, same sample rate).
    float pmag = sqrtf(result->uw_res.peak_re * result->uw_res.peak_re
                     + result->uw_res.peak_im * result->uw_res.peak_im);
    // Match the legacy worker: when pmag is too small to derive a
    // clean rotation, still run the decim + qpsk_demod on the
    // un-rotated data — a peak with near-zero phase already sits on
    // the constellation, and the demod's own PLL can pull small
    // residuals.
    if (pmag > 1e-3f) {
        float rot_re = result->uw_res.peak_re / pmag;
        float rot_im = result->uw_res.peak_im / pmag;
        int16_t pr_q = q15_from_float(rot_re);
        int16_t pi_q = q15_from_float(rot_im);
        for (int i = 0; i < adj_n; i++) {
            int32_t re = adj_burst[i * 2 + 0];
            int32_t im = adj_burst[i * 2 + 1];
            int32_t nr = ((int32_t)re * pr_q - (int32_t)im * pi_q) >> 15;
            int32_t ni = ((int32_t)re * pi_q + (int32_t)im * pr_q) >> 15;
            adj_burst[i * 2 + 0] = q15_saturate(nr);
            adj_burst[i * 2 + 1] = q15_saturate(ni);
        }
    }

    // Dump POST-rotation 10 sps stream — same window as adj_burst
    // (post-D13), matches gr-iridium's "rotate" dump in time-window
    // boundaries. Note gri's dump is its frame_size which may be
    // smaller than adj_n; lengths can still differ, but the START
    // points align now.
    dump_iq_cf32("07_post_prerot_250k", adj_burst, adj_n);

    // 6b. Sub-sample timing correction + UW-start trim — our addition
    //     (not in gri's burst_downmix; gri leaves the sub-sample offset
    //     in PDU metadata for the downstream Python qpsk_demod).
    //     Apply linear-interp fractional delay across the WHOLE
    //     adj_burst, then point src at the int_base sample position.
    float true_pos = (float)result->uw_res.uw_offset
                   + result->uw_res.correction;
    int   int_base = (int)floorf(true_pos);
    float interp_frac = true_pos - (float)int_base;
    if (int_base < 0) { int_base = 0; interp_frac = 0.0f; }
    int16_t *src = adj_burst + int_base * 2;
    int n_rot = adj_n - int_base;
    // Cap the post-UW-cut window to gr-iridium's
    //   frame_size = min(adj_n - int_base, uw_start + max_frame_length)
    //   max_frame_length = MAX_FRAME_LENGTH_NORMAL × sps = 191 × 10 = 1910
    // for normal (non-simplex) bursts. Bursts in our ALBQ corpus are
    // all <1626 MHz (= normal). This trims spurious post-burst noise
    // that would otherwise feed qpsk_demod and confuse PLL lock.
    //
    // TODO: for simplex bursts (≥1626 MHz, paging) the cap is
    // MAX_FRAME_LENGTH_SIMPLEX × 10 = 4440. burst_pipeline doesn't
    // currently know the burst frequency; either plumb it through or
    // detect SIMPLEX from burst length itself.
    const int MAX_FRAME_LEN_NORMAL_10SPS = 191 * UW_SPS;     // 1910
    if (n_rot > MAX_FRAME_LEN_NORMAL_10SPS) {
        n_rot = MAX_FRAME_LEN_NORMAL_10SPS;
    }
    if (interp_frac != 0.0f) {
        int16_t a_q = q15_from_float(1.0f - interp_frac);
        int16_t b_q = q15_from_float(interp_frac);
        int n_rot_cplx_interp = n_rot - 1;
        if (n_rot_cplx_interp < 0) n_rot_cplx_interp = 0;
        for (int i = 0; i < n_rot_cplx_interp; i++) {
            int32_t re = ((int32_t)a_q * src[i * 2 + 0]
                        + (int32_t)b_q * src[(i + 1) * 2 + 0]) >> 15;
            int32_t im = ((int32_t)a_q * src[i * 2 + 1]
                        + (int32_t)b_q * src[(i + 1) * 2 + 1]) >> 15;
            src[i * 2 + 0] = q15_saturate(re);
            src[i * 2 + 1] = q15_saturate(im);
        }
    }

    // Dump the post-trim 250 kHz stream — this is the gri-equivalent
    // of signal-filtered-deci-cut-start-shift-rrc-rotate-cut, matching
    // gri's pre-qpsk-demod 10-sps buffer. Useful for stagewise compare
    // to verify our trim + sub-sample interp landed in the right spot.
    dump_iq_cf32("07b_post_rotate_cut_250k", src,
                 n_rot - 1 > 0 ? n_rot - 1 : 0);

    // 7. 5:1 decimation 10 sps → 2 sps. In-place: read every 5th
    //    complex sample, write back to the start of `src`.
    //    Use `n_rot` (not `n_rot - 1`) to match the legacy worker's
    //    decim count exactly.
    int n_post_cplx = n_rot / POST_CORR_DECIM;
    for (int i = 0; i < n_post_cplx; i++) {
        int j = i * POST_CORR_DECIM;
        src[i * 2 + 0] = src[j * 2 + 0];
        src[i * 2 + 1] = src[j * 2 + 1];
    }
    result->n_post_2sps = n_post_cplx;

    dump_iq_cf32("08_decim_2sps", src, n_post_cplx);

    // 8. qpsk_demod.
    if (qpsk_demod_process(src, n_post_cplx * 2, &result->frame)) {
        result->demod_ok = true;
    }
    // Clear the one-shot dump trigger so subsequent bursts don't dump.
    s_dump_pending = 0;
    return true;
}
