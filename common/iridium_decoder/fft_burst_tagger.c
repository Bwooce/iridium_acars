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
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#define FBT_NOW_US() ((uint64_t)esp_timer_get_time())
#define FBT_HOT IRAM_ATTR
#else
#include <time.h>
static inline uint64_t fbt_now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}
#define FBT_NOW_US() fbt_now_us()
#define FBT_HOT
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

#if defined(ESP_PLATFORM)
// Pipelined helper task (Core 1): does mag+detect+EMA on the
// fft_buf that tagger JUST FFT'd, in parallel with tagger doing
// window+FFT for the NEXT step on the OTHER fft_buf.
//
// Tagger and helper communicate via xTaskNotify. Single helper
// instance; tagger waits for helper completion at the START of
// each call (returning previous step's bursts to caller).
//
// Priority 9 — above ingest_coord (8) — so helper preempts ingest
// for its ~58 µs of compute per FFT step. Lower priority gets
// starved by ingest's 2.5 ms resample chunks.
static TaskHandle_t        s_pipe_helper_task = NULL;
static TaskHandle_t        s_pipe_coord_task  = NULL;
static fft_burst_tagger_t *s_pipe_state_t     = NULL;

static void tagger_pipe_post_fft(fft_burst_tagger_t *t);

static void fbt_pipe_helper_task(void *arg)
{
    (void)arg;
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (s_pipe_state_t) tagger_pipe_post_fft(s_pipe_state_t);
        __sync_synchronize();
        if (s_pipe_coord_task) xTaskNotifyGive(s_pipe_coord_task);
    }
}
#endif

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
    //
    // Attempted to move this to PSRAM (2026-05-22) to shrink the
    // struct's contiguous internal-SRAM footprint from 65 → 33 KB.
    // Decode collapsed (matched 61 → 44, recall 93.8% → 67.7%) for
    // reasons not fully diagnosed — possibly bubble-sort PSRAM
    // bandwidth pressure or write-pattern aliasing. Keep inline.
    struct {
        int     bin;
        int64_t sort_key;   // relative_magnitude × HISTORY (gri sort order)
    } peaks[N];

    float    window_enbw;        // Blackman ENBW, computed at init
    uint64_t d_index;            // sample index of CURRENT FFT step's start
    uint64_t burst_id;

    // Pipelined helper state (ESP_PLATFORM only).
    //
    // Two fft_buf instances ping-pong: tagger window+FFTs the
    // "active" buffer on Core 0; helper does mag+detect+EMA on the
    // "pending" buffer on Core 1, IN PARALLEL. Each step, swap
    // roles. Helper output is staged here and returned via the
    // NEXT call to fft_burst_tagger_step (1-step API latency).
    //
    // fft_buf (inline above, internal SRAM) is one of the two
    // buffers. fft_buf_alt (PSRAM heap, fft_sc16_2048 bounces both
    // through its internal scratch so PSRAM is fine) is the other.
    int16_t      *fft_buf_alt;
    int           pipe_active_idx;       // 0 or 1: which buf tagger writes
    bool          pipe_in_flight;        // helper currently processing
    int           pipe_pending_idx;      // which buf helper is processing
    uint64_t      pipe_pending_d_index;  // d_index snapshot at dispatch
    // Staged outputs — heap-allocated (PSRAM) to keep the struct
    // size unchanged from baseline. Inline arrays would grow the
    // struct by ~6 KB, shifting downstream allocations enough to
    // trip the P4 PIE position-sensitivity bug in unknown ways.
    fbt_burst_t  *staged_new;
    int           staged_n_new;
    int           staged_max_new;        // upper bound caller passed
    fbt_burst_t  *staged_gone;
    int           staged_n_gone;
    int           staged_max_gone;
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

#if defined(ESP_PLATFORM)
    // Pipelined helper: alt fft_buf in PSRAM.
    //
    // EXPERIMENT (2026-05-23) tried INTERNAL SRAM: fft_buf_alt
    // landed at 0x4ff6b6b0 (middle of main RAM, PIE-broken zone),
    // decode collapsed matched=61 → 41. fft_buf_alt is a PIE-
    // touched buffer (window writes, fft_sc16_2048 reads via
    // memcpy) and inherits the position-sensitivity bug. Small-RAM
    // safe zone is already full (s_coeffs_pp + s_w_table +
    // s_fft_scratch = 16 KB of the 18 KB region). PSRAM is the
    // only reliable placement until the silicon bug is root-caused.
    t->fft_buf_alt = (int16_t *)heap_caps_aligned_alloc(
        16, 2 * N * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (!t->fft_buf_alt) {
        ESP_LOGE("FBT_INIT", "fft_buf_alt PSRAM alloc failed (pipelining disabled)");
    }
    // Staged outputs in PSRAM, keeps the struct small.
    t->staged_new  = (fbt_burst_t *)heap_caps_calloc(
        FBT_MAX_BURSTS, sizeof(fbt_burst_t), MALLOC_CAP_SPIRAM);
    t->staged_gone = (fbt_burst_t *)heap_caps_calloc(
        FBT_MAX_BURSTS, sizeof(fbt_burst_t), MALLOC_CAP_SPIRAM);
    if (!t->staged_new || !t->staged_gone) {
        ESP_LOGE("FBT_INIT", "staged_new/gone PSRAM alloc failed");
    }
    t->pipe_active_idx       = 0;       // tagger starts writing to fft_buf (inline)
    t->pipe_in_flight        = false;
    t->pipe_pending_idx      = 0;
    t->pipe_pending_d_index  = 0;
    t->staged_n_new          = 0;
    t->staged_n_gone         = 0;
    t->staged_max_new        = FBT_MAX_BURSTS;
    t->staged_max_gone       = FBT_MAX_BURSTS;

    if (s_pipe_helper_task == NULL && t->fft_buf_alt) {
        BaseType_t ok = xTaskCreatePinnedToCoreWithCaps(
            fbt_pipe_helper_task, "fbt_pipe", 4096, NULL,
            /*prio=*/ 9, &s_pipe_helper_task,
            /*core=*/ 1, MALLOC_CAP_SPIRAM);
        if (ok != pdPASS) {
            ESP_LOGW("FBT_INIT", "pipe helper spawn failed; sequential fallback");
            s_pipe_helper_task = NULL;
        } else {
            ESP_LOGI("FBT_INIT", "pipe helper on Core 1 (window+FFT ‖ mag+detect+EMA)");
        }
    }
#endif
    return t;
}

void fft_burst_tagger_destroy(fft_burst_tagger_t *t)
{
    if (!t) return;
#if defined(ESP_PLATFORM)
    if (t->fft_buf_alt) heap_caps_free(t->fft_buf_alt);
    if (t->staged_new)  heap_caps_free(t->staged_new);
    if (t->staged_gone) heap_caps_free(t->staged_gone);
    heap_caps_free(t);
#else
    free(t);
#endif
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

// Window-multiply the input into a caller-provided FFT scratch
// buffer. Q15 × Q15 → Q15. Buffer-pointer parameter so pipelined
// mode can ping-pong between two fft_buf instances.
static FBT_HOT void window_multiply(fft_burst_tagger_t *t,
                             const int16_t *input,
                             int16_t *fb_out)
{
    for (int i = 0; i < N; i++) {
        int32_t re = (int32_t)input[i * 2 + 0] * (int32_t)t->window[i];
        int32_t im = (int32_t)input[i * 2 + 1] * (int32_t)t->window[i];
        fb_out[i * 2 + 0] = (int16_t)(re >> 15);
        fb_out[i * 2 + 1] = (int16_t)(im >> 15);
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

static FBT_HOT void compute_magnitude_shifted(fft_burst_tagger_t *t,
                                       const int16_t *fb)
{
    int32_t *out = t->magnitude_shifted;
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
static FBT_HOT void update_bursts_internal(fft_burst_tagger_t *t)
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
static FBT_HOT int create_new_bursts_internal(fft_burst_tagger_t *t,
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
static FBT_HOT int delete_gone_bursts_internal(fft_burst_tagger_t *t,
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
static FBT_HOT void update_baseline_ema(fft_burst_tagger_t *t)
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

// Helper: pick fft_buf pointer by index.
static inline int16_t *fbt_buf_at(fft_burst_tagger_t *t, int idx)
{
#if defined(ESP_PLATFORM)
    return (idx == 0) ? t->fft_buf : t->fft_buf_alt;
#else
    (void)idx;
    return t->fft_buf;
#endif
}

// Post-FFT pipeline body: mag + detect + EMA, against the
// fft_buf indexed by t->pipe_pending_idx, using the snapshotted
// d_index. Used only by the Core 1 helper task.
//
// d_index handling: tagger advances t->d_index BEFORE notifying
// (see step function); helper saves the post-advance value, sets
// t->d_index to the snapshot for the duration of post-FFT work,
// and restores afterwards. So step N+1's tagger sees the correct
// post-advance value when it next reads t->d_index.
static FBT_HOT void tagger_pipe_post_fft(fft_burst_tagger_t *t)
{
    int16_t *fb = fbt_buf_at(t, t->pipe_pending_idx);
    uint64_t saved_d = t->d_index;
    t->d_index = t->pipe_pending_d_index;

    uint64_t t2 = FBT_NOW_US();
    compute_magnitude_shifted(t, fb);
    uint64_t t3 = FBT_NOW_US();
    s_acc_mag_us += (t3 - t2);

    if (!t->history_primed) {
        uint64_t b0 = FBT_NOW_US();
        update_baseline_ema(t);
        s_acc_base_us += (FBT_NOW_US() - b0);
        t->staged_n_new  = 0;
        t->staged_n_gone = 0;
    } else {
        uint64_t d0 = FBT_NOW_US();
        update_bursts_internal(t);
        t->staged_n_new  = create_new_bursts_internal(
            t, t->staged_new, t->staged_max_new);
        t->staged_n_gone = delete_gone_bursts_internal(
            t, t->staged_gone, t->staged_max_gone);
        uint64_t d1 = FBT_NOW_US();
        s_acc_detect_us += (d1 - d0);

        uint64_t b0 = FBT_NOW_US();
        update_baseline_ema(t);
        s_acc_base_us += (FBT_NOW_US() - b0);
    }

    t->d_index = saved_d;
}

FBT_HOT bool fft_burst_tagger_step(fft_burst_tagger_t *t,
                            const int16_t *input,
                            const int16_t *lookback,
                            fbt_burst_t *out_new_bursts, int *n_new,
                            fbt_burst_t *out_gone_bursts, int *n_gone)
{
    (void)lookback;     // reserved for future per-burst-cut step

    int max_new  = (n_new  && out_new_bursts ) ? *n_new  : 0;
    int max_gone = (n_gone && out_gone_bursts) ? *n_gone : 0;
    if (n_new)  *n_new  = 0;
    if (n_gone) *n_gone = 0;

#if defined(ESP_PLATFORM)
    // PIPELINED PATH: helper task running on Core 1 in parallel.
    //
    // Tagger does window+FFT for step N on Core 0 while helper
    // does mag+detect+EMA for step N-1 on Core 1. Caller gets
    // step N-1's bursts (1-step API latency).
    if (s_pipe_helper_task && t->fft_buf_alt) {
        bool was_primed = t->history_primed;

        // 1. Drain previous helper run, copy staged bursts to caller.
        if (t->pipe_in_flight) {
            ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
            t->pipe_in_flight = false;
            int n_new_out  = (t->staged_n_new  < max_new ) ? t->staged_n_new  : max_new;
            int n_gone_out = (t->staged_n_gone < max_gone) ? t->staged_n_gone : max_gone;
            for (int i = 0; i < n_new_out;  i++) out_new_bursts [i] = t->staged_new [i];
            for (int i = 0; i < n_gone_out; i++) out_gone_bursts[i] = t->staged_gone[i];
            if (n_new ) *n_new  = n_new_out;
            if (n_gone) *n_gone = n_gone_out;
        }

        // 2. Window+FFT this step into the buffer NOT held by helper.
        int next_idx = 1 - t->pipe_active_idx;
        int16_t *fb  = fbt_buf_at(t, next_idx);

        uint64_t t0 = FBT_NOW_US();
        window_multiply(t, input, fb);
        uint64_t t1 = FBT_NOW_US();
        fft_sc16_2048(fb);
        uint64_t t2 = FBT_NOW_US();
        s_acc_wind_us += (t1 - t0);
        s_acc_fft_us  += (t2 - t1);
        s_acc_steps   += 1;

        // 3. Snapshot pending state, ADVANCE d_index, then notify.
        //    Race-fix: t->d_index must be at its post-advance value
        //    BEFORE the helper task starts (since helper does
        //    `saved_d = t->d_index; ...; t->d_index = saved_d` and
        //    would overwrite a post-notify advance).
        t->pipe_pending_idx     = next_idx;
        t->pipe_pending_d_index = t->d_index;     // step N's value
        t->staged_max_new       = max_new;
        t->staged_max_gone      = max_gone;
        s_pipe_state_t          = t;
        s_pipe_coord_task       = xTaskGetCurrentTaskHandle();
        t->pipe_active_idx      = next_idx;
        t->pipe_in_flight       = true;
        t->d_index             += N;              // advance BEFORE notify
        xTaskNotifyGive(s_pipe_helper_task);

        // Return value: was the step BEFORE this one "primed" enough
        // to produce bursts? On the FIRST call (no in-flight previous),
        // history isn't primed yet; later calls reflect the state
        // observed before the drain.
        return was_primed;
    }
#endif

    // SEQUENTIAL FALLBACK (host build, or pipeline unavailable).
    uint64_t t0 = FBT_NOW_US();
    window_multiply(t, input, t->fft_buf);
    uint64_t t1 = FBT_NOW_US();
    fft_sc16_2048(t->fft_buf);
    uint64_t t2 = FBT_NOW_US();
    compute_magnitude_shifted(t, t->fft_buf);
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
    int n_new_out  = create_new_bursts_internal(t, out_new_bursts, max_new);
    int n_gone_out = delete_gone_bursts_internal(t, out_gone_bursts, max_gone);
    uint64_t d1 = FBT_NOW_US();
    s_acc_detect_us += (d1 - d0);

    if (n_new)  *n_new  = n_new_out;
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
