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
static int  s_dump_pending  = 0;

// One-shot D13 override. -1 = use D13 normally.
static int s_force_burst_start = -1;

void burst_pipeline_force_start_once(int sample_idx)
{
    s_force_burst_start = sample_idx;
}

void burst_pipeline_set_dump_once(const char *dir)
{
    if (dir) {
        strncpy(s_dump_dir, dir, sizeof(s_dump_dir) - 1);
        s_dump_dir[sizeof(s_dump_dir) - 1] = 0;
        s_dump_pending                     = 1;
    } else {
        s_dump_dir[0]  = 0;
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
#define UW_SPS 10
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

// (q15_freq_shift_inplace used to live here — the Q15 phase-ramp
// helper from the channelizer era. Both its roles are now served by
// rotate_to_dc_q15_simd_at / the in-line pre-rotation in
// try_decode_frame; removed as dead code. Doc references in
// direct_if_decim.h / test_pipeline_wideband_albq.c describe the
// rotation step generically.)

#define SYNC_RRC_LEN_GUARD 280    // SYNC_LENGTH × sps — minimum slice
                                  //   that the matched filter can
                                  //   operate on without running off
                                  //   the end of the buffer.
#define SYNC_SEARCH_LEN_GUARD 300 // a few-symbol margin past
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

// Profile-counter wiring used by both try_decode_frame and the outer
// burst_pipeline_process_burst stages. Definitions live up here so the
// PROFILE_T0/PROFILE_LOG macros are in scope inside try_decode_frame
// (which is declared above the outer driver).
#ifdef ESP_PLATFORM
#include "esp_timer.h"
#include "esp_attr.h"
#define PROFILE_T0() int64_t _pt0 = esp_timer_get_time()
#define PROFILE_NOW() esp_timer_get_time()
#define PROFILE_LOG(name)                                      \
    do {                                                       \
        int64_t _pt_now = esp_timer_get_time();                \
        s_profile_us[BP_##name] += (uint32_t)(_pt_now - _pt0); \
        _pt0 = _pt_now;                                        \
    } while (0)
#else
#define PROFILE_T0() \
    do {             \
    } while (0)
#define PROFILE_LOG(name) \
    do {                  \
    } while (0)
#ifndef EXT_RAM_BSS_ATTR
#define EXT_RAM_BSS_ATTR
#endif
#endif

enum {
    BP_D13,
    BP_CFO,
    BP_PREROT,
    BP_RRC,
    BP_LOOP_FIRST,
    BP_LOOP_RETRY,
    // try_decode_frame substages (accumulated across all calls — first +
    // retries — within the run). Lets us see what dominates the 35.5 ms
    // first-call cost vs the 11.8 ms retry cost.
    BP_TDF_UW,
    BP_TDF_PREROT,
    BP_TDF_DECIM,
    BP_TDF_QPSK,
    BP_N
};
static volatile uint32_t s_profile_us[BP_N]    = {0};
static volatile uint32_t s_profile_loops_first = 0;
static volatile uint32_t s_profile_loops_retry = 0;

// Decode scratch for try_decode_frame: fused interp+decim output, at most
// one frame at 2 sps (1910 / POST_CORR_DECIM = 382 complex ≈ 1.6 KB).
// Static (single-threaded worker / host test), +16 int16 trailing pad for
// the PIE rotate kernel's vector look-ahead.
#define TDF_POST_MAX (191 * UW_SPS / POST_CORR_DECIM)
static EXT_RAM_BSS_ATTR int16_t s_tdf_post[TDF_POST_MAX * 2 + 16];

static bool try_decode_frame(int16_t *adj_burst, int adj_n,
                             int                      search_start,
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
    // 1909, 2520, 3000 all give identical BER 1.30% -- because they all
    // exceed the correlator's HARD cap: a 2048-pt FFT with a 271-sample
    // reference yields only 2048 - 271 + 1 = 1778 alias-free lags, and
    // uw_correlator_find's search loop breaks at k + 270 >= 2048. So the
    // effective search range was ALWAYS 1778, never the 1909 "just under
    // one frame" this constant used to claim — UWs at offsets 1778..1908
    // in a window are only found by the 655-step retry loop (~18 ms per
    // retry). Set the constant to the real cap so the intent matches the
    // behaviour; full 1909-lag coverage would need CORR_FFT_N = 4096
    // (probably not worth it given the retries).
    //
    // Long-term, task #70 fix (tighten the tagger gone-event window)
    // would let this revert to 840 to match gri exactly.
    const int SYNC_SEARCH_LEN = 2048 - 271 + 1; // 1778: correlator's alias-free lag count
    int       remaining       = adj_n - search_start;
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

    PROFILE_T0();
    uw_corr_result_t tmp;
    memset(&tmp, 0, sizeof(tmp));
    uw_correlator_find(adj_burst + search_start * 2, remaining,
                       search_complex, &tmp);
    PROFILE_LOG(TDF_UW);
    if (tmp.direction == UW_DIR_UNKNOWN) return false;
    result->uw_res = tmp;

    // Pre-rotation: rotate adj_burst[search_start..adj_n] by
    // conj(peak/|peak|). gri rotates frame_size of d_tmp_a into d_tmp_b
    // (lines 692-694) — equivalent to in-place on our slice.
    float pmag = sqrtf(tmp.peak_re * tmp.peak_re + tmp.peak_im * tmp.peak_im);
    if (pmag > 1e-3f) {
        float   rot_re = tmp.peak_re / pmag;
        float   rot_im = tmp.peak_im / pmag;
        int16_t pr_q   = q15_from_float(rot_re);
        int16_t pi_q   = q15_from_float(rot_im);
        for (int i = search_start; i < adj_n; i++) {
            int32_t re           = adj_burst[i * 2 + 0];
            int32_t im           = adj_burst[i * 2 + 1];
            int32_t nr           = ((int32_t)re * pr_q - (int32_t)im * pi_q) >> 15;
            int32_t ni           = ((int32_t)re * pi_q + (int32_t)im * pr_q) >> 15;
            adj_burst[i * 2 + 0] = q15_saturate(nr);
            adj_burst[i * 2 + 1] = q15_saturate(ni);
        }
    }
    if (dump) {
        dump_iq_cf32("07_post_prerot_250k",
                     adj_burst + search_start * 2, remaining);
    }
    PROFILE_LOG(TDF_PREROT);

    // Sub-sample interp + UW-start trim.
    float true_pos    = (float)tmp.uw_offset + tmp.correction;
    int   int_base    = (int)floorf(true_pos);
    float interp_frac = true_pos - (float)int_base;
    if (int_base < 0) {
        int_base    = 0;
        interp_frac = 0.0f;
    }
    int16_t  *src                        = adj_burst + (search_start + int_base) * 2;
    int       n_rot                      = remaining - int_base;
    const int MAX_FRAME_LEN_NORMAL_10SPS = 191 * UW_SPS; // 1910
    if (n_rot > MAX_FRAME_LEN_NORMAL_10SPS) {
        n_rot = MAX_FRAME_LEN_NORMAL_10SPS;
    }
    if (dump) {
        // Pre-interp view of the UW-trimmed slice. (Used to be the
        // post-interp buffer when interp ran in place; the fused
        // interp+decim below no longer materialises that intermediate.)
        dump_iq_cf32("07b_post_rotate_cut_250k", src,
                     n_rot - 1 > 0 ? n_rot - 1 : 0);
    }

    // Fused sub-sample interpolation + POST_CORR_DECIM:1 decimation into
    // the dedicated s_tdf_post scratch. This used to run IN PLACE on
    // adj_burst, mutilating [uw .. uw+n_rot): when qpsk_demod rejected
    // the frame (the designed diffs>2 noise gate), the retry loop then
    // correlated against low-pass-interpolated, decimation-compacted
    // garbage — silently lowering multi-frame / retry recall. adj_burst
    // now stays intact for the retries (the pre-rotation above is pure
    // phase: it doesn't affect correlation magnitude and each retry
    // re-estimates phase from the current buffer, so it may stay
    // in place).
    //
    // Equivalence to the old interp-then-compact: output i took
    // interp(src[j], src[j+1]) with j = i*POST_CORR_DECIM, and
    // j+1 <= n_rot-4 for every kept sample, so no out-of-bounds read.
    int n_post_cplx = n_rot / POST_CORR_DECIM;
    if (n_post_cplx > TDF_POST_MAX) n_post_cplx = TDF_POST_MAX; // can't trip: n_rot <= 1910
    if (interp_frac != 0.0f) {
        int16_t a_q = q15_from_float(1.0f - interp_frac);
        int16_t b_q = q15_from_float(interp_frac);
        for (int i = 0; i < n_post_cplx; i++) {
            int     j             = i * POST_CORR_DECIM;
            int32_t re            = ((int32_t)a_q * src[j * 2 + 0] + (int32_t)b_q * src[(j + 1) * 2 + 0]) >> 15;
            int32_t im            = ((int32_t)a_q * src[j * 2 + 1] + (int32_t)b_q * src[(j + 1) * 2 + 1]) >> 15;
            s_tdf_post[i * 2 + 0] = q15_saturate(re);
            s_tdf_post[i * 2 + 1] = q15_saturate(im);
        }
    } else {
        for (int i = 0; i < n_post_cplx; i++) {
            int j                 = i * POST_CORR_DECIM;
            s_tdf_post[i * 2 + 0] = src[j * 2 + 0];
            s_tdf_post[i * 2 + 1] = src[j * 2 + 1];
        }
    }
    int16_t *post       = s_tdf_post;
    result->n_post_2sps = n_post_cplx;
    if (dump) dump_iq_cf32("08_decim_2sps", post, n_post_cplx);
    PROFILE_LOG(TDF_DECIM);

    // Apply post-UW CFO refinement. omega_per_sym from uw_correlator_find
    // is the residual frequency offset measured with a correctly-anchored
    // CFO window (preamble+UW, not adj_burst[0..255]); the pre-RRC blind
    // omega_coarse can be wrong on low-CFO bursts where the squared tone
    // lands near DC and the noise-dominated input window biases the peak.
    // Without this, the first-order PLL (alpha=0.2, beta=0) has to chase
    // the residual within 12 UW symbols and fails the diffs<=2 check on
    // borderline cases (gri_id=0 in the ALBQ corpus -- task #78).
    // src is at sps=2 after the POST_CORR_DECIM step, so phase_step =
    // omega_per_sym / 2.
    if (tmp.omega_per_sym != 0.0f) {
        rotate_to_dc_q15_simd_at(post, n_post_cplx,
                                 (double)tmp.omega_per_sym * 0.5,
                                 0);
    }
    if (dump) dump_iq_cf32("08b_post_uwcfo_2sps", post, n_post_cplx);

    bool ok = qpsk_demod_process(post, n_post_cplx * 2, &result->frame);
    PROFILE_LOG(TDF_QPSK);
    if (ok) {
        result->demod_ok = true;
        return true;
    }
    return false;
}

void burst_pipeline_get_stage_us(uint32_t out[10], uint32_t *first_calls,
                                 uint32_t *retry_calls)
{
    for (int i = 0; i < BP_N; i++) {
        out[i]          = s_profile_us[i];
        s_profile_us[i] = 0;
    }
    if (first_calls) {
        *first_calls          = s_profile_loops_first;
        s_profile_loops_first = 0;
    }
    if (retry_calls) {
        *retry_calls          = s_profile_loops_retry;
        s_profile_loops_retry = 0;
    }
}

// Internal helper for the legacy single-frame wrapper.
typedef struct {
    burst_pipeline_result_t *out;
    int                      n_seen;
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
        free(res->frame.soft_bits); // #112
    }
    lc->n_seen++;
}

bool burst_pipeline_process_250khz(int16_t *iq250, int n_complex,
                                   burst_pipeline_result_t *result)
{
    memset(result, 0, sizeof(*result));
    legacy_first_frame_ctx_t lc = {.out = result, .n_seen = 0};
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

    // 0. Per-burst DC removal (#113, moved here from worker_core1.c
    //    2026-05-31 as part of #128 host-harness parity work). The RTL-
    //    SDR R820T tuner doesn't auto-calibrate DC; the byte-128 bias
    //    correction at ingest_core1 subtracts only the nominal centre,
    //    leaving residual DC that varies with temperature and gain.
    //    Inside the per-burst window the DC lands at the channel centre
    //    as a coherent tone — biases the squared-FFT CFO estimator
    //    (uw_correlator below) and distorts the matched-filter peak.
    //    Subtract the burst's mean in place. Two passes (~1.6 ms at 16k
    //    samples ≈ 3% of typical burst budget on P4). Cross-validation
    //    safe: gri fixtures pass through a DC blocker by convention so
    //    they already have ~zero DC and this is a no-op there.
    //
    //    By living here, host pipeline tests (test_pipeline_drift_snr,
    //    test_pipeline_wideband_albq, test_pipeline_direct_if_albq,
    //    test_pipeline_wideband_resampled) all get the same pre-CFO
    //    statistic as the device worker — closing the gap that hid the
    //    #115 regression where host PASS didn't predict device PASS.
    if (n_complex > 0) {
        // int64 accumulators: at WB_DECIM_MAX (~62,639 complex) a
        // full-scale saturated burst sums to 2.05e9 — within 5% of
        // INT32_MAX, i.e. signed-overflow UB one buffer-size bump away.
        int64_t sum_re = 0, sum_im = 0;
        for (int i = 0; i < n_complex; i++) {
            sum_re += iq250[i * 2 + 0];
            sum_im += iq250[i * 2 + 1];
        }
        int16_t dc_re = (int16_t)(sum_re / n_complex);
        int16_t dc_im = (int16_t)(sum_im / n_complex);
        for (int i = 0; i < n_complex; i++) {
            iq250[i * 2 + 0] -= dc_re;
            iq250[i * 2 + 1] -= dc_im;
        }
    }

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
        int       search_depth         = SEARCH_DEPTH_SAMPLES;
        if (search_depth > n_complex) search_depth = n_complex;
        burst_start = uw_correlator_find_burst_start(
            iq250, n_complex, /*search_max=*/search_depth);
    }
    int16_t *adj_burst = iq250 + burst_start * 2;
    int      adj_n     = n_complex - burst_start;
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
    const int FRAME_LEN_SAMPLES = 191 * UW_SPS;       // 1910
    const int RETRY_STEP_10SPS  = (131 * UW_SPS) / 2; // 655

    int n_emitted         = 0;
    int next_search_start = -1; // -1 = haven't found first frame yet

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
                found                   = true;
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
        int first_uw_abs = first_used_search_start + (int)res.uw_res.uw_offset;
        cb(&res, ctx); // ownership of res.frame.bits transfers
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
    const int MIN_FRAME_LEN_REMAINING = 131 * UW_SPS; // 1310
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
