// See fft_burst_tagger.h. Implementation closely mirrors
// gr-iridium/lib/fft_burst_tagger_impl.cc:
//   work(): per-FFT-size step iteration
//   update_filters_pre/post(): baseline EMA via subtract-old + add-new
//   update_bursts(): bump last_active for each currently-tracked burst
//   extract_peaks(): scan all bins for above-threshold candidates
//   create_new_bursts(): allocate burst records from peaks
//   delete_gone_bursts(): timeout expired bursts
//
// Differences from gri's implementation that matter for correctness:
//   - All-Q15 hot path. magnitude² fits in int32; baseline_sum
//     accumulates history_size × magnitude² → int32 still fits
//     (max 32k² × 512 ~ 5e11 — needs int64). Actually use int32 with
//     a per-step scale, see note below.
//   - No GNU Radio scheduler. Caller manages frame indexing via
//     fft_burst_tagger_set_start + step calls.
//   - Threshold expressed as mult relative to noise EMA, simpler in
//     fixed point.
//
// Q15 / dynamic-range note:
//   After fft_sc16_2048 (per-stage right shift), bin values are
//   bounded by ~±32k/2048 × sqrt(2048) ≈ ±700 for full-scale Q15
//   input. Magnitude² then ≤ ~ 1e6, fits int32 easily.
//   baseline_sum = sum of last HISTORY_SIZE=512 magnitudes ≤ 5e8,
//   also int32. The threshold comparison mag² × HISTORY_SIZE >
//   threshold_int × baseline_sum uses int64 to avoid overflow.

#include "fft_burst_tagger.h"
#include "fft_sc16_2048.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#if defined(ESP_PLATFORM)
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#define FBT_NOW_US() ((uint64_t)esp_timer_get_time())
#else
#include <time.h>
static inline uint64_t fbt_now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}
#define FBT_NOW_US() fbt_now_us()
#endif

#define N FBT_FFT_SIZE

// Per-stage timer accumulators (single-tagger process — fine for our
// usage). Order matches fft_burst_tagger_get_stage_us() docs.
static uint64_t s_acc_wind_us;
static uint64_t s_acc_fft_us;
static uint64_t s_acc_mag_us;
static uint64_t s_acc_detect_us;
static uint64_t s_acc_base_us;
static uint32_t s_acc_steps;

struct fft_burst_tagger_s {
    int      burst_pre_len;
    int      burst_post_len;
    int      burst_width;       // in bins
    int32_t  threshold_q15;     // 10^(db/10) in Q15 (1.0 = 2^15)
                                // — we use this so the comparison is
                                // mag² × HISTORY_SIZE > (baseline_sum × threshold_q15) >> 15

    int16_t  window[N];         // Q15 Blackman
    int16_t  fft_buf[2 * N];    // scratch (rotation + FFT)
    int32_t  magnitude_shifted[N];   // mag² of FFT output, FFT-shifted to DC-centred
    int32_t  baseline_sum[N];        // sum of last HISTORY_SIZE magnitudes (per bin)
    uint8_t  burst_mask[N];          // 1 = bin is allowed to declare new burst

    int32_t *baseline_history;       // [HISTORY_SIZE × N], caller-owned (PSRAM)
    int      history_index;
    bool     history_primed;

    fbt_burst_t  bursts[FBT_MAX_BURSTS];
    int          n_bursts;

    // Per-step peak workspace (used inside create_new_bursts_internal).
    // Lives in the struct (heap-backed) rather than on the stack
    // because at sort_key = int64 it's 32 KB — too big for the smoke
    // task's stack and just wasteful churn on every step regardless.
    struct {
        int     bin;
        int64_t sort_key;   // relative_magnitude × HISTORY (gri sort order)
    } peaks[N];

    float    window_enbw;        // Blackman ENBW, computed at init
    uint64_t d_index;            // sample index of CURRENT FFT step's start
    uint64_t burst_id;
};

// Build a Q15 Blackman window of length N.
static void build_blackman_q15(int16_t *w_out)
{
    const double PI = 3.14159265358979323846;
    for (int i = 0; i < N; i++) {
        double t = (double)i / (double)(N - 1);
        double v = 0.42 - 0.5 * cos(2.0 * PI * t)
                        + 0.08 * cos(4.0 * PI * t);
        // Scale to Q15. Blackman peaks at ~1.0 (mid-window), edges ~0.
        // Multiplying by INT16_MAX preserves dynamic range.
        double q = v * (double)INT16_MAX;
        if (q > (double)INT16_MAX) q = (double)INT16_MAX;
        if (q < 0) q = 0;
        w_out[i] = (int16_t)(q + 0.5);
    }
}

// Effective Noise Bandwidth of the Blackman(N) window, used as
// gri does in fft_burst_tagger_impl::create_new_bursts to scale
// magnitude_db so a burst's reported SNR is the SNR an integrator
// of the underlying noise process would see (not the per-bin
// FFT-output magnitude relative to the per-bin EMA). For Blackman
// a0=0.42, a1=0.5, a2=0.08: ENBW = N · Σw² / (Σw)² ≈ 1.7268. 10·log10
// of that is +2.37 dB — magnitude_db without this factor reads
// ~2.4 dB lower than gri reports for the same physical burst.
//
// Computed once at init and cached, because rebuild on every step
// would be wasteful and the value is fixed for our Blackman window.
static float compute_window_enbw(const int16_t *w, int n)
{
    double sum = 0, sum_sq = 0;
    for (int i = 0; i < n; i++) {
        double v = (double)w[i] / 32767.0;
        sum    += v;
        sum_sq += v * v;
    }
    return (float)((double)n * sum_sq / (sum * sum));
}

fft_burst_tagger_t *fft_burst_tagger_init(int burst_pre_len,
                                           int burst_post_len,
                                           int burst_width,
                                           float threshold_mult_db,
                                           int32_t *baseline_history_ext)
{
    if (!baseline_history_ext) return NULL;
#if defined(ESP_PLATFORM)
    // AUDIT (2026-05-22): plain calloc was spilling to PSRAM when
    // internal SRAM was fragmented (e.g. by task stacks). Tagger
    // fields then lived in PSRAM, decode silently regressed.
    // Force MALLOC_CAP_INTERNAL so the alloc FAILS LOUDLY if there
    // isn't room — and log the address so we can verify placement.
    fft_burst_tagger_t *t = (fft_burst_tagger_t *)
        heap_caps_calloc(1, sizeof(*t),
                         MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (t) {
        ESP_LOGI("FBT_INIT",
                 "fft_burst_tagger_t alloc OK at %p (size=%u, INTERNAL, free_int_after=%u)",
                 t, (unsigned)sizeof(*t),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    } else {
        ESP_LOGE("FBT_INIT",
                 "fft_burst_tagger_t INTERNAL alloc FAILED (size=%u, free_int=%u, largest_int=%u)",
                 (unsigned)sizeof(*t),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    }
#else
    fft_burst_tagger_t *t = (fft_burst_tagger_t *)calloc(1, sizeof(*t));
#endif
    if (!t) return NULL;
    t->burst_pre_len  = burst_pre_len;
    t->burst_post_len = burst_post_len;
    t->burst_width    = burst_width;

    build_blackman_q15(t->window);
    t->window_enbw = compute_window_enbw(t->window, N);
    fft_sc16_2048_init();

    // threshold_q15: 10^(dB/10) × 2^15. For 10 dB → 10 × 32768 = 327680.
    // For 15 dB → ~31.6 × 32768 ~ 1.0e6. Fits int32.
    //
    // Note: we deliberately do NOT include the window_enbw factor here.
    // gri's effective threshold is HISTORY×ENBW × less strict than the
    // equivalent dB number suggests (its d_threshold = pow(10, thr/10)
    // / HISTORY / ENBW compared against mag²/baseline). Our above_threshold
    // already factors HISTORY into the LHS but omits ENBW — so at the
    // same threshold_db we're effectively 10·log10(ENBW) ≈ 2.4 dB
    // stricter than gri. We could match gri exactly by dividing
    // threshold_q15 by ENBW, but that loosens the threshold and INCREASES
    // burst count (verified by experiment: 146 → 1345). The 2× FP rate
    // we see vs gri is NOT a threshold issue — it's somewhere else in
    // the detector logic.
    double t_lin = pow(10.0, (double)threshold_mult_db / 10.0);
    t->threshold_q15 = (int32_t)(t_lin * 32768.0 + 0.5);

    t->baseline_history = baseline_history_ext;
    memset(t->baseline_history, 0,
           sizeof(int32_t) * N * FBT_HISTORY_SIZE);
    memset(t->baseline_sum, 0, sizeof(t->baseline_sum));
    for (int i = 0; i < N; i++) t->burst_mask[i] = 1;
    t->history_index = 0;
    t->history_primed = false;
    t->n_bursts = 0;
    t->d_index = 0;
    t->burst_id = 0;
    return t;
}

void fft_burst_tagger_destroy(fft_burst_tagger_t *t)
{
    free(t);
}

void fft_burst_tagger_flush(fft_burst_tagger_t *t,
                             fbt_burst_t *out_gone, int *n_gone)
{
    int max = (n_gone && *n_gone > 0) ? *n_gone : 0;
    int emitted = 0;
    for (int b = 0; b < t->n_bursts && emitted < max; b++) {
        t->bursts[b].stop = t->d_index;       // force-close at current sample
        out_gone[emitted++] = t->bursts[b];
    }
    if (n_gone) *n_gone = emitted;
    // Drop all active bursts; mask is full-rebuild on next step.
    t->n_bursts = 0;
    for (int k = 0; k < N; k++) t->burst_mask[k] = 1;
}

void fft_burst_tagger_set_start(fft_burst_tagger_t *t, uint64_t start)
{
    t->d_index = start;
}

// Window-multiply the input into the FFT scratch buffer. Q15 × Q15 → Q15.
static void window_multiply(fft_burst_tagger_t *t, const int16_t *input)
{
    for (int i = 0; i < N; i++) {
        int32_t re = (int32_t)input[i * 2 + 0] * (int32_t)t->window[i];
        int32_t im = (int32_t)input[i * 2 + 1] * (int32_t)t->window[i];
        t->fft_buf[i * 2 + 0] = (int16_t)(re >> 15);
        t->fft_buf[i * 2 + 1] = (int16_t)(im >> 15);
    }
}

// Compute |FFT output|² with FFT-shift (bin 0 = -fs/2, bin N/2 = DC,
// bin N-1 = +fs/2 - bin_width). gri does this internally; we do it
// here so downstream uses DC-centred bin numbering.
//
// The FFT-shift swaps the upper and lower halves of fft_buf. Rather
// than the modulo-based gather (which defeats vectorisation), we
// split into two contiguous mag² passes: src=N/2..N-1 → dst=0..N/2-1
// and src=0..N/2-1 → dst=N/2..N-1. Each inner loop is a simple
// re²+im² → int32 store and -O3 will SLP-vectorise it.
static inline void mag_sq_pass(const int16_t * __restrict__ src_iq,
                                int32_t * __restrict__ dst, int n)
{
    for (int k = 0; k < n; k++) {
        int32_t re = src_iq[2 * k + 0];
        int32_t im = src_iq[2 * k + 1];
        dst[k] = re * re + im * im;
    }
}

static void compute_magnitude_shifted(fft_burst_tagger_t *t)
{
    const int16_t *fb = t->fft_buf;
    int32_t       *out = t->magnitude_shifted;
    // Pass A: upper half of fft_buf → lower half of output.
    mag_sq_pass(fb + 2 * (N / 2), out, N / 2);
    // Pass B: lower half of fft_buf → upper half of output.
    mag_sq_pass(fb, out + N / 2, N / 2);
}

// Returns true if mag² is above threshold at this bin.
// Comparison: (mag² × HISTORY_SIZE) > (baseline_sum × threshold_q15) >> 15.
// Both sides as int64 to avoid overflow.
static inline bool above_threshold(int32_t mag2, int32_t baseline_sum,
                                    int32_t threshold_q15)
{
    int64_t lhs = (int64_t)mag2 * (int64_t)FBT_HISTORY_SIZE;
    int64_t rhs = ((int64_t)baseline_sum * (int64_t)threshold_q15) >> 15;
    return lhs > rhs;
}

// Update existing bursts' last_active timestamp.
static void update_bursts_internal(fft_burst_tagger_t *t)
{
    for (int b = 0; b < t->n_bursts; b++) {
        int cb = t->bursts[b].center_bin;
        // Check ±1 bin around center (gri's update_bursts).
        for (int dk = -1; dk <= 1; dk++) {
            int bin = cb + dk;
            if (bin < 0 || bin >= N) continue;
            if (above_threshold(t->magnitude_shifted[bin],
                                t->baseline_sum[bin],
                                t->threshold_q15)) {
                t->bursts[b].last_active = t->d_index;
                break;
            }
        }
    }
}

// Mask out bins around an active burst so we don't double-detect it.
static void mask_burst(fft_burst_tagger_t *t, int center_bin)
{
    int lo = center_bin - t->burst_width / 2;
    int hi = center_bin + t->burst_width / 2;
    if (lo < 0) lo = 0;
    if (hi > N - 1) hi = N - 1;
    for (int k = lo; k <= hi; k++) t->burst_mask[k] = 0;
}

static void rebuild_burst_mask(fft_burst_tagger_t *t)
{
    for (int k = 0; k < N; k++) t->burst_mask[k] = 1;
    for (int b = 0; b < t->n_bursts; b++) {
        mask_burst(t, t->bursts[b].center_bin);
    }
}

// Scan all bins for new bursts. Bins must be above threshold AND
// not masked by an existing burst. gri sorts peaks by
// relative_magnitude (mag² / baseline) — NOT by raw mag² — so the
// peak with the highest SNR-above-local-noise wins when overlapping.
// A consistently-noisy bin can have high raw mag² but also a high
// baseline; sorting by relative_magnitude avoids letting it mask a
// quieter-but-cleaner real burst nearby. This matches
// fft_burst_tagger_impl::extract_peaks() in gr-iridium and is the
// key to keeping false-positive burst count close to gri's ~65/s on
// the ALBQ fixture (previously we were at ~133/s with raw mag² sort).
//
// sort_key is computed once at peak-detection time as
//   (mag² × HISTORY_SIZE) / (baseline_sum + 1)
// — same numerator we already form in above_threshold, so the cost
// added by the sort-key swap is a single int64 divide per peak.
static int create_new_bursts_internal(fft_burst_tagger_t *t,
                                       fbt_burst_t *out_new, int max_new)
{
    int n_peaks = 0;

    int margin = t->burst_width / 2;
    for (int bin = margin; bin < N - margin; bin++) {
        if (!t->burst_mask[bin]) continue;
        int32_t mag2 = t->magnitude_shifted[bin];
        int32_t base = t->baseline_sum[bin];
        if (above_threshold(mag2, base, t->threshold_q15)) {
            t->peaks[n_peaks].bin = bin;
            // Sort key = relative_magnitude × (HISTORY_SIZE × 32768)
            // to keep an int64 representation that's stable for
            // ordering. Same ordering as the float ratio mag²/baseline.
            t->peaks[n_peaks].sort_key =
                ((int64_t)mag2 * (int64_t)FBT_HISTORY_SIZE)
                / ((int64_t)base + 1);
            n_peaks++;
        }
    }

    // Sort peaks by relative magnitude descending. Bubble sort —
    // n_peaks is typically small (<50), insertion would be marginal.
    for (int i = 0; i < n_peaks - 1; i++) {
        for (int j = i + 1; j < n_peaks; j++) {
            if (t->peaks[j].sort_key > t->peaks[i].sort_key) {
                int     bin_tmp = t->peaks[i].bin;
                int64_t key_tmp = t->peaks[i].sort_key;
                t->peaks[i] = t->peaks[j];
                t->peaks[j].bin = bin_tmp;
                t->peaks[j].sort_key = key_tmp;
            }
        }
    }

    int n_emitted = 0;
    for (int p = 0; p < n_peaks; p++) {
        int bin = t->peaks[p].bin;
        if (!t->burst_mask[bin]) continue;     // got masked by an earlier peak in this loop
        if (t->n_bursts >= FBT_MAX_BURSTS) break;

        fbt_burst_t *b = &t->bursts[t->n_bursts++];
        b->id = t->burst_id;
        t->burst_id += 10;
        b->center_bin = bin;
        b->start = t->d_index - t->burst_pre_len;
        b->last_active = b->start;
        b->stop = 0;
        // Magnitude in dB, with the window ENBW scaling that gr-iridium
        // applies (fft_burst_tagger_impl.cc:303):
        //   magnitude = 10·log10(rel × HISTORY × window_enbw)
        // ENBW corrects for the fact that the Blackman-windowed FFT
        // bin output reflects integrated power over more than just
        // a rectangular bin width — the noise per bin is wider than
        // the FFT's nominal bin spacing. Without the factor our
        // magnitude_db reads ~2.4 dB low vs gri for the same burst.
        double rel = (double)t->magnitude_shifted[bin]
                     * (double)FBT_HISTORY_SIZE
                     / ((double)t->baseline_sum[bin] + 1.0);
        b->magnitude_db = (float)(10.0 * log10(
                          rel * (double)t->window_enbw + 1e-12));
        b->noise_db = (float)(10.0 * log10(
            (double)t->baseline_sum[bin] / (double)FBT_HISTORY_SIZE + 1e-12));

        mask_burst(t, bin);

        if (out_new && n_emitted < max_new) {
            out_new[n_emitted++] = *b;
        }
    }
    return n_emitted;
}

// Erase bursts whose last_active is older than burst_post_len. Emit
// the timed-out bursts in out_gone.
static int delete_gone_bursts_internal(fft_burst_tagger_t *t,
                                        fbt_burst_t *out_gone, int max_gone)
{
    int n_emitted = 0;
    int dst = 0;
    bool any_removed = false;
    for (int src = 0; src < t->n_bursts; src++) {
        fbt_burst_t *b = &t->bursts[src];
        if (b->last_active + t->burst_post_len <= t->d_index) {
            b->stop = t->d_index;
            if (out_gone && n_emitted < max_gone) {
                out_gone[n_emitted++] = *b;
            }
            any_removed = true;
            // skip — burst is removed
        } else {
            if (dst != src) t->bursts[dst] = t->bursts[src];
            dst++;
        }
    }
    t->n_bursts = dst;
    if (any_removed) {
        // Rebuild burst_mask to free bins we no longer protect.
        rebuild_burst_mask(t);
    }
    return n_emitted;
}

#define HIST(t, i) ((t)->baseline_history + (size_t)(i) * N)

// EMA update: subtract oldest, add newest, advance index. gri only
// updates when no bursts are active OR a long-burst forces a refresh;
// we mirror that — if any burst is active, freeze the EMA.
//
// The hot path reads `old_slot` once (8 KB from PSRAM), updates the
// in-SRAM baseline_sum, then writes the new magnitude back into the
// same PSRAM slot. We fuse the two scans of baseline_sum into one,
// load each `old_slot[k]` exactly once, and overlap the PSRAM write
// with the same iteration so memcpy() isn't a separate sequential
// pass. restrict + -O3 SLPvec the int32 inner.
static inline void ema_step_inner(int32_t * __restrict__ bsum,
                                   int32_t * __restrict__ slot,
                                   const int32_t * __restrict__ mag,
                                   int n)
{
    for (int k = 0; k < n; k++) {
        int32_t old = slot[k];                  // one PSRAM read (L2 cached)
        int32_t cur = mag[k];                   // in-SRAM read
        bsum[k] = bsum[k] - old + cur;          // in-SRAM RMW
        slot[k] = cur;                          // one PSRAM write (L2 writeback)
    }
}

// EMA update: subtract oldest, add newest, advance index. gri only
// updates when no bursts are active OR a long-burst forces a refresh;
// we mirror that — if any burst is active, freeze the EMA.
//
// Fused per-bin read+RMW+write loop. -O3 + restrict make this L2-
// prefetch-friendly (sequential PSRAM access pattern), beating any
// "bulk memcpy + in-SRAM operate + bulk memcpy back" rewrite by
// ~25 µs/step (opp #3 in opt doc was tried 2026-05-22, regressed
// base 73 → 99 µs and was reverted).
static void update_baseline_ema(fft_burst_tagger_t *t)
{
    if (t->n_bursts > 0) return;     // burst active → freeze EMA

    int32_t *old_slot = HIST(t, t->history_index);
    ema_step_inner(t->baseline_sum, old_slot, t->magnitude_shifted, N);

    t->history_index++;
    if (t->history_index >= FBT_HISTORY_SIZE) {
        t->history_index = 0;
        t->history_primed = true;
    }
}

bool fft_burst_tagger_step(fft_burst_tagger_t *t,
                            const int16_t *input,
                            const int16_t *lookback,
                            fbt_burst_t *out_new_bursts, int *n_new,
                            fbt_burst_t *out_gone_bursts, int *n_gone)
{
    (void)lookback;     // reserved for future per-burst-cut step

    int max_new = (n_new && out_new_bursts) ? *n_new : 0;
    int max_gone = (n_gone && out_gone_bursts) ? *n_gone : 0;
    if (n_new) *n_new = 0;
    if (n_gone) *n_gone = 0;

    uint64_t t0 = FBT_NOW_US();
    window_multiply(t, input);
    uint64_t t1 = FBT_NOW_US();
    fft_sc16_2048(t->fft_buf);
    uint64_t t2 = FBT_NOW_US();
    compute_magnitude_shifted(t);
    uint64_t t3 = FBT_NOW_US();

    s_acc_wind_us += (t1 - t0);
    s_acc_fft_us  += (t2 - t1);
    s_acc_mag_us  += (t3 - t2);
    s_acc_steps   += 1;

    if (!t->history_primed) {
        uint64_t b0 = FBT_NOW_US();
        update_baseline_ema(t);
        s_acc_base_us += (FBT_NOW_US() - b0);
        t->d_index += N;
        return false;
    }

    uint64_t d0 = FBT_NOW_US();
    update_bursts_internal(t);
    int n_new_out = create_new_bursts_internal(t, out_new_bursts, max_new);
    int n_gone_out = delete_gone_bursts_internal(t, out_gone_bursts, max_gone);
    uint64_t d1 = FBT_NOW_US();
    s_acc_detect_us += (d1 - d0);

    if (n_new) *n_new = n_new_out;
    if (n_gone) *n_gone = n_gone_out;

    uint64_t b0 = FBT_NOW_US();
    update_baseline_ema(t);
    s_acc_base_us += (FBT_NOW_US() - b0);

    t->d_index += N;
    return true;
}

void fft_burst_tagger_get_stage_us(uint64_t out[5], uint32_t *steps)
{
    if (out) {
        out[0] = s_acc_wind_us;
        out[1] = s_acc_fft_us;
        out[2] = s_acc_mag_us;
        out[3] = s_acc_detect_us;
        out[4] = s_acc_base_us;
    }
    if (steps) *steps = s_acc_steps;
    s_acc_wind_us = 0;
    s_acc_fft_us = 0;
    s_acc_mag_us = 0;
    s_acc_detect_us = 0;
    s_acc_base_us = 0;
    s_acc_steps = 0;
}
