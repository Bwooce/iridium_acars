// fft_burst_tagger.h — C port of gr-iridium's fft_burst_tagger block,
// reduced to the surface area we actually need on P4.
//
// What it does: takes a stream of int16 IQ samples at the input
// sample rate (2.5 MSPS), runs a 2048-pt FFT every 2048 samples
// (no overlap, matching gri's set_output_multiple(d_fft_size)), tracks
// a per-bin noise-floor estimate via a rolling EMA, and reports
// detected bursts as (start_sample, center_bin) tuples.
//
// Differences from gr-iridium's gnuradio block (deliberate):
//   - Single-call API instead of a streaming block — caller hands one
//     FFT-size chunk at a time, gets new/gone bursts via output
//     arrays. Removes GNU Radio scheduler dependency.
//   - Q15/sc16 hot path (window-mul, FFT, magnitude², EMA) so it fits
//     in the P4's PIE-int16 SIMD budget (~18% core load at N=2048 vs
//     ~110% at fc32 — see Phase 3.6.M feasibility report in the plan).
//   - baseline_history buffer is the caller's allocation (so it can
//     be placed in PSRAM on P4, the per-FFT working buffers stay in
//     internal SRAM).
//   - Threshold expressed as a multiplier of the EMA (e.g. 32× = 15
//     dB above noise floor) rather than gri's dB-with-ENBW formula
//     — equivalent at fixed window/history params, simpler in fixed
//     point.
//
// Algorithm summary (gri lib/fft_burst_tagger_impl.cc:213-356):
//   per FFT step:
//     1. window-multiply input × Blackman window
//     2. radix-2 FFT (fft_sc16_2048)
//     3. FFT-shift the output and compute magnitude² per bin
//     4. update each existing burst's last_active if its center_bin
//        (or ±1) is above threshold × baseline_sum[bin]
//     5. for each bin, if magnitude² > threshold × baseline_sum AND
//        the bin isn't masked by an active burst, declare a new burst
//     6. erase bursts whose last_active is older than burst_post_len
//     7. if no bursts are currently active (or always, depending on
//        the gri "force noise update" path), update baseline_sum by
//        an EMA step: subtract baseline_history[history_index],
//        add the current magnitude², store the current magnitude²
//        at baseline_history[history_index], advance index.
//
// Output: caller polls `out_new_bursts` and `out_gone_bursts` after
// each step. Each new-burst record carries the absolute start_sample
// (caller's frame of reference, set via fft_burst_tagger_set_start)
// and the FFT bin index. Caller converts bin → relative_frequency
// = (center_bin - fft_size/2) / fft_size (cycles/sample) and from
// that to Hz at the input sample rate.

#pragma once
#include <stdint.h>
#include <stdbool.h>

#define FBT_FFT_SIZE 2048    // matches fft_sc16_2048
#define FBT_MAX_BURSTS 64    // gri default at fs=2.5MHz / burst_width=40k * 0.8 = 50, round up
#define FBT_HISTORY_SIZE 512 // gri default (set in iridium_extractor_flowgraph.py)

// Maximum tracked burst length, in input samples. gri passes
// max_burst_len = int(input_sample_rate * 0.09) into the tagger
// (iridium_extractor_flowgraph.py:509; rationale at :143-144: "After
// 90 ms there needs to be a pause in the frame structure"). At our
// fixed 2.5 MSPS input rate: int(2.5e6 * 0.09) = 225000. A burst whose
// last_active - start exceeds this is force-closed AND triggers a
// forced noise-floor refresh (gri fft_burst_tagger_impl.cc:265-285) —
// the refresh is what stops a persistent carrier from freezing the
// baseline EMA forever (n_bursts > 0 blocks the normal update path).
#define FBT_MAX_BURST_LEN 225000

// Burst-squelch threshold: squelch when the number of tracked bursts
// EXCEEDS this (strictly greater, gri fft_burst_tagger_impl.cc:327).
// gri computes it for max_bursts=0 (the iridium-extractor default,
// apps/iridium-extractor:124) as
//   d_max_bursts = (sample_rate / burst_width) * 0.8
// (fft_burst_tagger_impl.cc:176-182) with INTEGER division of the Hz
// values: (2500000 / 40000) * 0.8 = 62 * 0.8 = 49.6 → int → 49.
// (FBT_MAX_BURSTS=64 above is only the tracking-array capacity.)
#define FBT_SQUELCH_MAX_BURSTS 49

// One detected burst (output record).
typedef struct {
    // id shrunk to 32-bit (no downstream reader) so width_bins fits in the
    // freed 4 bytes — keeps sizeof(fbt_burst_t) BYTE-IDENTICAL. Growing this
    // struct (it's inlined 64× via bursts[FBT_MAX_BURSTS] in the tagger)
    // shifts internal-SRAM allocations and trips the P4 PIE position-
    // sensitivity bug — see the staged_new/staged_gone note in the .c.
    uint32_t id;           // monotonically increasing, +10 per detection (diag)
    int      width_bins;   // T60: contiguous above-threshold bins around the
                           // peak at detection (~34 = one Iridium channel;
                           // broadband RFI is far wider). For the worker PQ.
    uint64_t start;        // absolute sample index in caller's frame
                           //  (= d_index - burst_pre_len, see gri:306)
    uint64_t last_active;  // last FFT step where the burst was seen above threshold
    uint64_t stop;         // set when the burst is removed (last_active + burst_post_len)
    int      center_bin;   // FFT bin, 0..N-1 (DC-centred after FFT-shift)
    float    magnitude_db; // 10·log10(peak relative magnitude)
    float    noise_db;     // 10·log10(EMA baseline at center_bin)
} fbt_burst_t;

typedef struct fft_burst_tagger_s fft_burst_tagger_t;

// Allocate + init a tagger. `baseline_history_ext` must point to an
// int32_t[FBT_FFT_SIZE * FBT_HISTORY_SIZE] buffer that the caller
// owns (4 MB at the default params — place in PSRAM on P4). All
// other working buffers are allocated internally (~120 KB internal
// SRAM total).
//
// `burst_pre_len` and `burst_post_len` are in INPUT samples (at the
// caller's sample rate). gri defaults: pre = 2*fft_size = 4096,
// post = sample_rate * 16e-3 = 40000 at 2.5 MSPS.
//
// `burst_width` is in FFT bins (= input_sample_rate / fft_bin_width
// / 2 gives the half-width of one Iridium channel). For 40 kHz channels
// at 2.5 MSPS with 2048-pt FFT → bin_width = 1220 Hz → 40e3/1220 ≈ 32
// bins. gri default formula matches.
//
// `threshold_mult_db` is in dB. gri uses ~10 dB by default for
// Iridium; bursts must be >threshold_mult_db above the baseline EMA
// to be tagged.
//
// Returns NULL on allocation failure.
fft_burst_tagger_t *fft_burst_tagger_init(int      burst_pre_len,
                                          int      burst_post_len,
                                          int      burst_width,
                                          float    threshold_mult_db,
                                          int32_t *baseline_history_ext);

void fft_burst_tagger_destroy(fft_burst_tagger_t *t);

// Set the caller's absolute starting sample. The first FFT step will
// be tagged with sample positions starting at this value. Subsequent
// steps advance by FBT_FFT_SIZE each. Call once after init.
void fft_burst_tagger_set_start(fft_burst_tagger_t *t, uint64_t start);

// Set the near-DC new-burst exclusion window. `lo`/`hi` are signed FFT-bin
// offsets from DC (N/2); bins in [DC+lo, DC+hi] can never spawn a new burst.
// `lo > hi` disables the window (default). Process-global (single detector);
// safe to call live to re-tune. Does not affect the noise-floor EMA — only
// new-burst declaration.
void fft_burst_tagger_set_dc_mask(fft_burst_tagger_t *t, int lo, int hi);

// Process one FFT-size chunk of complex int16 samples.
//
// `input` must point to 2*FBT_FFT_SIZE int16 (FBT_FFT_SIZE complex IQ).
// `lookback` must point to 2*burst_pre_len int16 — the
// `burst_pre_len` samples IMMEDIATELY PRECEDING `input`. (gri's
// `set_history(burst_pre_len + 1)` provides this from the GR
// scheduler; here the caller manages it.) lookback is currently
// unused in the detection math itself; it's reserved for a future
// PDU-cut step.
//
// On return:
//   - `out_new_bursts` is filled with up to *n_new (in/out) bursts
//     that crossed threshold this step
//   - `out_gone_bursts` is filled with up to *n_gone bursts that
//     timed out this step (last_active + burst_post_len <= d_index)
//   - *n_new and *n_gone are updated to the actual counts emitted
//
// Returns true on success, false if FFT history isn't primed yet
// (first HISTORY_SIZE steps) — caller should just keep feeding and
// the EMA will fill in.
bool fft_burst_tagger_step(fft_burst_tagger_t *t,
                           const int16_t      *input,
                           const int16_t      *lookback,
                           fbt_burst_t *out_new_bursts, int *n_new,
                           fbt_burst_t *out_gone_bursts, int *n_gone);

// Force-emit any bursts still active. Used at end-of-stream in
// offline tests / when an input source closes — without this, the
// last few bursts in the stream never reach a `gone` event (because
// their last_active + burst_post_len > final d_index) and would be
// silently dropped. Sets stop = current d_index for each.
//
// `out_gone_bursts` is filled with up to *n_gone bursts; *n_gone
// is updated to the actual count emitted. Internal state is cleared
// so subsequent _step calls start fresh.
void fft_burst_tagger_flush(fft_burst_tagger_t *t,
                            fbt_burst_t *out_gone_bursts, int *n_gone);

// Reset the per-bin noise-floor EMA and clear active bursts so the tagger
// re-learns the floor over FBT_HISTORY_SIZE steps (used after a live LO
// retune — the old band's floor is meaningless at the new center). step()
// returns false until re-primed. d_index and burst_id are preserved so
// sample-position and burst-id continuity are unbroken.
void fft_burst_tagger_reset_baseline(fft_burst_tagger_t *t);

// Early-boot allocation of the PIE detect-scan pre-screen flags buffer
// (N/4 int32 = 2 KB). Call from the early PIE pin block, BEFORE the USB stack
// fragments internal SRAM — the buffer is PIE-written and inherits the P4
// heap-position sensitivity (a non-DRAM placement silently mis-detects). It is
// DRAM-guarded; on failure the detect scan falls back to the scalar path. No-op
// on host / non-PIE builds. Independent of fft_burst_tagger_init (process-wide,
// like fft_sc16_2048_init).
void fft_burst_tagger_prealloc_screen(void);

// Diagnostic — read accumulated per-stage wall-times (µs) since the
// last call, plus the number of steps that contributed. Order:
//   out[0] = window_multiply
//   out[1] = fft_sc16_2048
//   out[2] = compute_magnitude_shifted
//   out[3] = update_bursts + create_new_bursts + delete_gone_bursts
//   out[4] = update_baseline_ema (incl. PSRAM history slot R/W)
// Counters reset after read. Costs are accumulated across all step()
// calls; per-step values are out[i] / *steps. Safe to call without
// instrumentation enabled — returns zeros.
void fft_burst_tagger_get_stage_us(uint64_t out[5], uint32_t *steps);

// Diagnostic — read-and-reset the per-stage MINIMUM wall time (µs)
// observed since the last call. Same stage ordering as
// fft_burst_tagger_get_stage_us(). The min is the uncontended compute
// floor: preemption can only ADD to a step's wall time, so the minimum
// over many steps is a tight lower bound on real compute — a stable
// perf-regression signal immune to scheduler jitter (unlike the mean,
// which a single preemption outlier inflates). Stages with no clean
// step in the window read back as 0.
void fft_burst_tagger_get_stage_min_us(uint64_t out[5]);

// Diagnostic — read-and-reset the squelch visibility counters
// accumulated since the last call (same process-wide idiom as
// fft_burst_tagger_get_stage_us):
//   squelch_events   steps where the burst squelch fired
//                    (n_bursts > FBT_SQUELCH_MAX_BURSTS)
//   squelch_dropped  bursts force-closed by the squelch while still
//                    ACTIVE (above threshold at the squelch step).
//                    These are counted but NOT emitted as gone events —
//                    a deliberate divergence from gr-iridium, which
//                    dispatches them into an effectively unbounded
//                    parallel downstream; our bounded single-worker PQ
//                    was monopolised by them under live interference.
//                    Already-quiet (post-pad) closures still dispatch.
//                    See the squelch block in create_new_bursts_internal.
//   noise_resets     squelch-driven noise-estimate resets
//                    (squelch_count >= 10 → reset_baseline)
// Any pointer may be NULL.
void fft_burst_tagger_get_squelch_stats(uint32_t *squelch_events,
                                        uint32_t *squelch_dropped,
                                        uint32_t *noise_resets);

// Scalar Q15 window-multiply kernel — the tagger's window stage AND
// the bit-exact REFERENCE for any SIMD replacement (T50 golden
// harness, tests/host/test_window_multiply_golden.c). For each
// complex sample i in [0, n_complex):
//   out_iq[2i+0] = (int16_t)(((int32_t)input_iq[2i+0] * window[i]) >> 15)
//   out_iq[2i+1] = (int16_t)(((int32_t)input_iq[2i+1] * window[i]) >> 15)
// i.e. Q15 multiply with TRUNCATING (floor) shift, no rounding, no
// saturation. The device window table is a Q15 Blackman (values in
// [0, 32767], never negative). Do not change this arithmetic without
// regenerating the golden fixture — and don't do that to make a
// candidate kernel pass (no test-fitting).
void fbt_window_multiply_q15(const int16_t *input_iq,
                             const int16_t *window,
                             int16_t *out_iq, int n_complex);
