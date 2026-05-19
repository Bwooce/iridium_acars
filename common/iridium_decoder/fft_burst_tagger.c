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

#define N FBT_FFT_SIZE

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

fft_burst_tagger_t *fft_burst_tagger_init(int burst_pre_len,
                                           int burst_post_len,
                                           int burst_width,
                                           float threshold_mult_db,
                                           int32_t *baseline_history_ext)
{
    if (!baseline_history_ext) return NULL;
    fft_burst_tagger_t *t = (fft_burst_tagger_t *)calloc(1, sizeof(*t));
    if (!t) return NULL;
    t->burst_pre_len  = burst_pre_len;
    t->burst_post_len = burst_post_len;
    t->burst_width    = burst_width;
    // threshold_q15: 10^(dB/10) × 2^15. For 10 dB → 10 × 32768 = 327680.
    // For 15 dB → ~31.6 × 32768 ~ 1.0e6. Fits int32.
    double t_lin = pow(10.0, (double)threshold_mult_db / 10.0);
    t->threshold_q15 = (int32_t)(t_lin * 32768.0 + 0.5);

    build_blackman_q15(t->window);
    fft_sc16_2048_init();

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
static void compute_magnitude_shifted(fft_burst_tagger_t *t)
{
    for (int k = 0; k < N; k++) {
        // FFT-shift: bin k in output → bin (k + N/2) % N in shifted view.
        int src = (k + N / 2) % N;
        int32_t re = t->fft_buf[src * 2 + 0];
        int32_t im = t->fft_buf[src * 2 + 1];
        t->magnitude_shifted[k] = re * re + im * im;
    }
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
// not masked by an existing burst. gri sorts peaks by magnitude
// (strongest first) before creating bursts; we do the same so the
// strongest peak gets the burst record when two bursts overlap.
static int create_new_bursts_internal(fft_burst_tagger_t *t,
                                       fbt_burst_t *out_new, int max_new)
{
    typedef struct { int bin; int32_t mag2; } peak_t;
    peak_t peaks[N];
    int n_peaks = 0;

    int margin = t->burst_width / 2;
    for (int bin = margin; bin < N - margin; bin++) {
        if (!t->burst_mask[bin]) continue;
        if (above_threshold(t->magnitude_shifted[bin],
                            t->baseline_sum[bin],
                            t->threshold_q15)) {
            peaks[n_peaks].bin = bin;
            peaks[n_peaks].mag2 = t->magnitude_shifted[bin];
            n_peaks++;
        }
    }

    // Sort peaks by mag² descending — bubble (n_peaks typically small).
    for (int i = 0; i < n_peaks - 1; i++) {
        for (int j = i + 1; j < n_peaks; j++) {
            if (peaks[j].mag2 > peaks[i].mag2) {
                peak_t tmp = peaks[i]; peaks[i] = peaks[j]; peaks[j] = tmp;
            }
        }
    }

    int n_emitted = 0;
    for (int p = 0; p < n_peaks; p++) {
        int bin = peaks[p].bin;
        if (!t->burst_mask[bin]) continue;     // got masked by an earlier peak in this loop
        if (t->n_bursts >= FBT_MAX_BURSTS) break;

        fbt_burst_t *b = &t->bursts[t->n_bursts++];
        b->id = t->burst_id;
        t->burst_id += 10;
        b->center_bin = bin;
        b->start = t->d_index - t->burst_pre_len;
        b->last_active = b->start;
        b->stop = 0;
        // Magnitude (relative_magnitude × HISTORY_SIZE in linear). Convert
        // to dB. Match gri's `10·log10(relative × HISTORY)` (window ENBW
        // term applied at the dB level — caller scales separately if
        // ENBW matters).
        double rel = (double)peaks[p].mag2 * (double)FBT_HISTORY_SIZE
                     / ((double)t->baseline_sum[bin] + 1.0);
        b->magnitude_db = (float)(10.0 * log10(rel + 1e-12));
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
static void update_baseline_ema(fft_burst_tagger_t *t)
{
    if (t->n_bursts > 0) return;     // burst active → freeze EMA

    int32_t *old_slot = HIST(t, t->history_index);
    for (int k = 0; k < N; k++) {
        t->baseline_sum[k] -= old_slot[k];
        t->baseline_sum[k] += t->magnitude_shifted[k];
    }
    memcpy(old_slot, t->magnitude_shifted, sizeof(int32_t) * N);

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

    window_multiply(t, input);
    fft_sc16_2048(t->fft_buf);
    compute_magnitude_shifted(t);

    if (!t->history_primed) {
        update_baseline_ema(t);
        t->d_index += N;
        return false;
    }

    update_bursts_internal(t);
    int n_new_out = create_new_bursts_internal(t, out_new_bursts, max_new);
    int n_gone_out = delete_gone_bursts_internal(t, out_gone_bursts, max_gone);
    if (n_new) *n_new = n_new_out;
    if (n_gone) *n_gone = n_gone_out;

    update_baseline_ema(t);

    t->d_index += N;
    return true;
}
