// See burst_pipeline.h for design rationale and pipeline ordering.

#include "burst_pipeline.h"
#include "rotate_to_dc.h"

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

#define SYNC_RRC_LEN_GUARD     280   // SYNC_LENGTH × sps — minimum slice
                                      //   that the matched filter can
                                      //   operate on without running off
                                      //   the end of the buffer.
#define SYNC_SEARCH_LEN_GUARD  300   // a few-symbol margin past
                                      //   SYNC_RRC_LEN_GUARD so the
                                      //   matched-filter peak isn't at
                                      //   the very last possible bin.

// Per-frame matched-filter + pre-rotation + decim + demod. Mirrors the
// inside of gr-iridium's process_next_frame() so the multi-frame loop
// in handle_burst can call this repeatedly with advancing search_start
// positions. Returns true if qpsk_demod produced a frame.
//
// `adj_burst` must already be post-D13 (trimmed), post-CFO (carrier
// removed), post-RRC (matched-filter shaped). For sub-frame iterations,
// the same buffer is reused with `search_start` advanced — gri only
// re-runs steps 5-8 per frame, with one initial CFO+RRC pass for the
// whole PDU (line 890-905 of burst_downmix_impl.cc). We follow that
// pattern except for the per-iteration CFO refinement which gri does
// inside process_next_frame; if a future port needs the per-frame CFO
// (some bursts have measurable carrier drift across frames), it can be
// added inside this helper before the matched filter.
static bool try_decode_frame(int16_t *adj_burst, int adj_n,
                              int search_start,
                              burst_pipeline_result_t *result, bool dump)
{
    // Search range = just under one frame (191 sym × 10 sps - 1 = 1909).
    //
    // gr-iridium uses 840 = (64 preamble + 12 UW + 8 margin) × sps for
    // its tight burst windows, where the UW lands within the first ~80
    // symbols of adj_burst. Our wideband tagger gone-event windows are
    // 2.4x wider than gri's (task #70), so frame 0's UW can be deeper
    // in the buffer. We scale the search range accordingly, but cap it
    // STRICTLY below one frame length so the correlator cannot
    // accidentally pick frame N+1's UW (which would land at +1910
    // samples past frame 0's UW).
    //
    // Tested values: 840 (gri-exact) regresses BER 1.30 -> 2.12% on this
    // corpus because our wider buffer pushes some UWs past 840. Values
    // 1909, 2520, 3000 all give identical BER 1.30% -- 1909 is the
    // smallest that captures all UWs while remaining < 1 frame.
    //
    // Long-term, task #70 fix (tighten the tagger gone-event window)
    // would let this revert to 840 to match gri exactly.
    const int SYNC_SEARCH_LEN = 191 * UW_SPS - 1;          // 1909
    int remaining = adj_n - search_start;
    if (remaining < SYNC_RRC_LEN_GUARD) return false;
    int search_complex = SYNC_SEARCH_LEN;
    if (search_complex > remaining - 24) search_complex = remaining - 24;
    if (search_complex <= 2) return false;

    // NOTE: gr-iridium runs a per-iteration squared-FFT CFO inside
    // process_next_frame. Tried it here for retry iterations
    // (search_start > 0); it didn't recover the last path-C miss
    // (gri 461) and regressed path A from 17/99 to 7/99 because the
    // CFO finds spurious peaks on the channelizer's noisier signals
    // and the resulting rotation breaks otherwise-decoding bursts.
    // Skipped — the main pipeline's one-shot CFO is good enough for
    // the cases this multi-frame loop catches. Per-iter CFO can be
    // revisited once path A's channelizer SNR loss (D7+) is addressed.

    uw_corr_result_t tmp;
    memset(&tmp, 0, sizeof(tmp));
    uw_correlator_find(adj_burst + search_start * 2, remaining,
                       search_complex, &tmp);
    if (tmp.direction == UW_DIR_UNKNOWN) return false;
    result->uw_res = tmp;

    // Pre-rotation: rotate adj_burst[search_start..adj_n] by
    // conj(peak/|peak|). gri rotates frame_size of d_tmp_a into d_tmp_b
    // (lines 692-694) — equivalent to in-place on our slice.
    float pmag = sqrtf(tmp.peak_re * tmp.peak_re + tmp.peak_im * tmp.peak_im);
    if (pmag > 1e-3f) {
        float rot_re = tmp.peak_re / pmag;
        float rot_im = tmp.peak_im / pmag;
        int16_t pr_q = q15_from_float(rot_re);
        int16_t pi_q = q15_from_float(rot_im);
        for (int i = search_start; i < adj_n; i++) {
            int32_t re = adj_burst[i * 2 + 0];
            int32_t im = adj_burst[i * 2 + 1];
            int32_t nr = ((int32_t)re * pr_q - (int32_t)im * pi_q) >> 15;
            int32_t ni = ((int32_t)re * pi_q + (int32_t)im * pr_q) >> 15;
            adj_burst[i * 2 + 0] = q15_saturate(nr);
            adj_burst[i * 2 + 1] = q15_saturate(ni);
        }
    }
    if (dump) {
        dump_iq_cf32("07_post_prerot_250k",
                     adj_burst + search_start * 2, remaining);
    }

    // Sub-sample interp + UW-start trim.
    float true_pos = (float)tmp.uw_offset + tmp.correction;
    int   int_base = (int)floorf(true_pos);
    float interp_frac = true_pos - (float)int_base;
    if (int_base < 0) { int_base = 0; interp_frac = 0.0f; }
    int16_t *src = adj_burst + (search_start + int_base) * 2;
    int n_rot = remaining - int_base;
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
    if (dump) {
        dump_iq_cf32("07b_post_rotate_cut_250k", src,
                     n_rot - 1 > 0 ? n_rot - 1 : 0);
    }

    int n_post_cplx = n_rot / POST_CORR_DECIM;
    for (int i = 0; i < n_post_cplx; i++) {
        int j = i * POST_CORR_DECIM;
        src[i * 2 + 0] = src[j * 2 + 0];
        src[i * 2 + 1] = src[j * 2 + 1];
    }
    result->n_post_2sps = n_post_cplx;
    if (dump) dump_iq_cf32("08_decim_2sps", src, n_post_cplx);

    if (qpsk_demod_process(src, n_post_cplx * 2, &result->frame)) {
        result->demod_ok = true;
        return true;
    }
    return false;
}

#ifdef ESP_PLATFORM
#include "esp_timer.h"
#define PROFILE_T0()           int64_t _pt0 = esp_timer_get_time()
#define PROFILE_NOW()          esp_timer_get_time()
#define PROFILE_LOG(name)      do { \
    int64_t _pt_now = esp_timer_get_time(); \
    s_profile_us[BP_##name] += (uint32_t)(_pt_now - _pt0); \
    _pt0 = _pt_now; \
} while (0)
#else
#define PROFILE_T0()           do { } while (0)
#define PROFILE_LOG(name)      do { } while (0)
#endif

enum {
    BP_D13, BP_CFO, BP_PREROT, BP_RRC, BP_LOOP_FIRST, BP_LOOP_RETRY,
    BP_N
};
static volatile uint32_t s_profile_us[BP_N] = { 0 };
static volatile uint32_t s_profile_loops_first = 0;
static volatile uint32_t s_profile_loops_retry = 0;

void burst_pipeline_get_stage_us(uint32_t out[6], uint32_t *first_calls,
                                  uint32_t *retry_calls)
{
    for (int i = 0; i < BP_N; i++) {
        out[i] = s_profile_us[i];
        s_profile_us[i] = 0;
    }
    if (first_calls) { *first_calls = s_profile_loops_first; s_profile_loops_first = 0; }
    if (retry_calls) { *retry_calls = s_profile_loops_retry; s_profile_loops_retry = 0; }
}

// Internal helper for the legacy single-frame wrapper.
typedef struct {
    burst_pipeline_result_t *out;
    int n_seen;
} legacy_first_frame_ctx_t;

static void legacy_first_frame_cb(burst_pipeline_result_t *res, void *ctx)
{
    legacy_first_frame_ctx_t *lc = (legacy_first_frame_ctx_t *)ctx;
    if (lc->n_seen == 0) {
        // Shallow-copy: result->frame.bits ownership transfers to caller.
        *lc->out = *res;
    } else {
        // Extra frames -- caller of legacy API doesn't take them.
        free(res->frame.bits);
    }
    lc->n_seen++;
}

bool burst_pipeline_process_250khz(int16_t *iq250, int n_complex,
                                    burst_pipeline_result_t *result)
{
    memset(result, 0, sizeof(*result));
    legacy_first_frame_ctx_t lc = { .out = result, .n_seen = 0 };
    burst_pipeline_process_burst(iq250, n_complex,
                                  legacy_first_frame_cb, &lc);
    // Match the historical return contract: true means "pipeline ran end-
    // to-end". The caller checks result->demod_ok separately.
    return true;
}

int burst_pipeline_process_burst(int16_t *iq250, int n_complex,
                                  burst_pipeline_frame_cb cb, void *ctx)
{
    PROFILE_T0();

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
    int16_t *adj_burst = iq250 + burst_start * 2;
    int adj_n = n_complex - burst_start;
    if (adj_n < UW_SPS * 28) {
        // Less than one full sync word worth — can't even run the
        // matched filter; bail out cleanly.
        return 0;
    }
    PROFILE_LOG(D13);

    dump_iq_cf32("04_post_d13_250k", adj_burst, adj_n);

    // 2. Pre-RRC squared-FFT CFO estimate on the trimmed burst.
    float omega_coarse = uw_correlator_estimate_cfo(adj_burst, adj_n);
    PROFILE_LOG(CFO);

    // 3. Phase-correct adj_burst in place by exp(+j·omega/sps·n).
    //    Sign convention matches uw_correlator_estimate_cfo: the
    //    returned omega is the NEGATIVE of the actual offset, so
    //    multiplying by exp(+j·omega/sps·n) cancels it.
    //
    // Use absolute-phase rotation (rotate_to_dc_q15_simd_at) rather
    // than Q15 incremental phasor multiplication. The incremental
    // p ← p · exp(j·dphi) with >>15 truncation loses ~0.012%
    // magnitude per sample; over 7900-sample bursts that decays to
    // ~5% of initial, collapsing the modulation into a constant-phase
    // tone for large-residual-carrier bursts. See memory note
    // `feedback_q15_incremental_phasor_decays.md`.
    if (omega_coarse != 0.0f) {
        float dphi = omega_coarse / (float)UW_SPS;
        rotate_to_dc_q15_simd_at(adj_burst, adj_n,
                                  (double)dphi, 0);
    }
    PROFILE_LOG(PREROT);

    dump_iq_cf32("05_post_cfo_250k", adj_burst, adj_n);

    // 4. RRC matched filter.
    uw_correlator_apply_rrc(adj_burst, adj_burst, adj_n);
    PROFILE_LOG(RRC);

    dump_iq_cf32("06_post_rrc_250k", adj_burst, adj_n);

    // 5-8. Per-frame matched filter + pre-rotate + decim + demod.
    //
    // Multi-frame loop mirrors gri's handle_multiple_frames_per_burst
    // (lib/burst_downmix_impl.cc:890-905):
    //   * First frame: try at search_start=0. If qpsk_demod's UW
    //     direction check rejects, fall through to a half-frame retry
    //     loop -- needed for our wider tagger windows (task #70) where
    //     the first try sometimes lands on a non-UW correlation peak.
    //   * Subsequent frames: advance the search position by one frame
    //     length (191 sym × 10 sps = 1910 samples) from each emitted
    //     frame's UW position, and try again. gri advances even on
    //     failure; we do the same so the loop reaches all sub-frame
    //     positions in a multi-frame burst.
    //
    // Each successful frame fires the callback with its result and
    // freshly malloc'd frame.bits. The callback owns the bits.
    const int FRAME_LEN_SAMPLES = 191 * UW_SPS;             // 1910
    const int RETRY_STEP_10SPS  = (131 * UW_SPS) / 2;       // 655

    int n_emitted        = 0;
    int next_search_start = -1;    // -1 = haven't found first frame yet

    burst_pipeline_result_t res;
    memset(&res, 0, sizeof(res));
    res.burst_start  = burst_start;
    res.omega_coarse = omega_coarse;

    // First frame: try at 0, then retry on failure.
    bool found = try_decode_frame(adj_burst, adj_n, 0, &res,
                                   /*dump=*/true);
    s_profile_loops_first++;
    int first_used_search_start = 0;
    if (!found) {
        for (int retry_start = RETRY_STEP_10SPS;
             retry_start + SYNC_SEARCH_LEN_GUARD <= adj_n;
             retry_start += RETRY_STEP_10SPS) {
            s_profile_loops_retry++;
            if (try_decode_frame(adj_burst, adj_n, retry_start,
                                  &res, /*dump=*/false)) {
                found = true;
                first_used_search_start = retry_start;
                break;
            }
        }
    }
    PROFILE_LOG(LOOP_FIRST);

    if (found) {
        res.burst_start  = burst_start;
        res.omega_coarse = omega_coarse;
        // uw_res.uw_offset is relative to first_used_search_start; convert
        // to an absolute position within adj_burst so the multi-frame
        // step can advance by FRAME_LEN_SAMPLES correctly.
        int first_uw_abs = first_used_search_start
                         + (int)res.uw_res.uw_offset;
        cb(&res, ctx);   // ownership of res.frame.bits transfers
        n_emitted++;
        next_search_start = first_uw_abs + FRAME_LEN_SAMPLES;
    }

    // Multi-frame: iterate until burst no longer has room for another
    // frame. gri uses MIN_FRAME_LENGTH_NORMAL × sps = 1310 samples as
    // the cutoff (burst_downmix_impl.cc: "if (burst_size - start <
    // min_frame_length) return 0"). Using SYNC_SEARCH_LEN_GUARD (300)
    // here would let the loop iterate 4-5 times across the post-burst
    // padding for single-frame bursts -- wasted try_decode_frame work
    // at ~18 ms each. 1310 matches gri's bound and saves ~45 ms per
    // single-frame burst.
    const int MIN_FRAME_LEN_REMAINING = 131 * UW_SPS;       // 1310
    while (found && next_search_start >= 0 &&
           next_search_start + MIN_FRAME_LEN_REMAINING <= adj_n) {
        memset(&res, 0, sizeof(res));
        res.burst_start  = burst_start;
        res.omega_coarse = omega_coarse;
        s_profile_loops_retry++;
        bool ok = try_decode_frame(adj_burst, adj_n, next_search_start,
                                    &res, /*dump=*/false);
        if (ok) {
            int uw_abs = next_search_start + (int)res.uw_res.uw_offset;
            cb(&res, ctx);
            n_emitted++;
            // Anchor next search to THIS frame's UW position so timing
            // drift across sub-frames doesn't accumulate.
            next_search_start = uw_abs + FRAME_LEN_SAMPLES;
        } else {
            // gri advances by frame_size even when a frame fails to
            // decode -- the loop continues to the next sub-frame slot.
            next_search_start += FRAME_LEN_SAMPLES;
        }
    }
    PROFILE_LOG(LOOP_RETRY);

    // Clear the one-shot dump trigger so subsequent bursts don't dump.
    s_dump_pending = 0;
    return n_emitted;
}
