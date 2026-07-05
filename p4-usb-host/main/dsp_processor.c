// dsp_processor — wideband burst detector. Thin shim around
// fft_burst_tagger (common/iridium_decoder/), which runs the gr-iridium
// wideband spectrogram detector against the full 2.5 MSPS input
// subband. The Phase 3.6.M cutover (commit bf0bb9f) replaced an
// earlier single-FFT detector and a polyphase channelizer with this
// gri-aligned path; both predecessors were removed from the tree.
// Each tagged burst carries an exact relative-frequency tag (FFT
// bin → Hz), so the worker's rotation is sub-bin precise and the
// downstream burst_pipeline PLL only tracks residual scatter.
//
// #120: state moved from file-scope statics into a heap-allocated
// dsp_processor_t handle so the module is reentrant. The current
// firmware creates exactly one and threads the handle through
// feed/flush/stats; cross-task diagnostic readers use
// dsp_processor_default(). No PIE buffers live here (the tagger owns
// its own FFT scratch), so the handle is heap-safe — none of the
// project_heap_position_decode_bug placement concerns apply.

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdatomic.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "dsp_processor.h"
#include "fft_burst_tagger.h"
#include "worker_core1.h"
#include "app_config.h"

static const char *TAG = "DSP_PROC";

// Tagger threshold over the EMA baseline.
//
// Host wideband test results across the threshold/window sweep:
//   thr  | window | tagged | decoded | % of gri-65
//   ----|--------|--------|---------|-------------
//   10  | fixed  |  133   |   58    |   89%
//   10  | gone   |  133   |   60    |   92%   ← host peak
//   12  | gone   |   74   |   57    |   88%
//   14  | gone   |   70   |   57    |   88%
//
// 10 dB is the closest gri-aligned match given our tagger ENBW
// handling (see fft_burst_tagger.c:131-141 and
// test_pipeline_wideband_albq.c). Smoke at 10 dB: matched 61 -> 63,
// recall 93.8% -> 96.9%, dropped=0/147.
//
// History: 14 dB was a perf workaround through 2026-05-24 morning.
// At 10 dB the worker couldn't keep up under bench-noise conditions
// (tagger fires ~145/sec on noise spikes); the worker monopolised
// Core 1 long enough to starve frame_decoder past the 5 s task
// watchdog, and the firmware aborted. The fix wasn't perf — task
// #58 had already cut Core 1 ingest cost ~10% which was plenty —
// it was a priority inversion: worker (5) preempted frame_decoder
// (4) and status_logger (1). Dropping worker to 3 (below decoder)
// and bumping logger to 6 lets the scheduler keep the WDT-watched
// task alive and the observability lines flowing even when the
// worker has a backlog. With those in place, live USB rate at
// 10 dB measured higher than at 14 dB (4.5 vs 4.0 MB/s) because
// Core 1 spends less time worker-monopolised.
#define FBT_THRESHOLD_DB 10.0f

// Burst window padding in INPUT samples (at FS_DETECT_HZ). gri's
// defaults: pre = 2*fft_size = 4096, post = sample_rate * 16e-3 =
// 40000. Same numbers work here because FS_DETECT_HZ matches gri's
// nominal 2.5 MSPS.
#define FBT_BURST_PRE_LEN (2 * FBT_FFT_SIZE)
#define FBT_BURST_POST_LEN 40000

// Burst width in FFT bins (= half an Iridium channel). 40 kHz /
// (2.5 MSPS / 2048) ≈ 32 bins.
#define FBT_BURST_WIDTH 32

#define FBT_NEW_BUF_SIZE 16
#define FBT_GONE_BUF_SIZE 16

// Detector instance. Formerly a bag of file-scope statics (#120).
struct dsp_processor {
    fft_burst_tagger_t *tagger;
    int32_t            *baseline_history; // PSRAM, 4 MB
    burst_detected_cb_t user_cb;

    // Accumulator for chunks smaller than FBT_FFT_SIZE complex samples.
    // dsp_processor_feed receives variable-length buffers from class_driver
    // (typically ~8000 complex after the 125/128 resample of a 16 KB USB
    // transfer), so we batch into FFT_SIZE-aligned chunks.
    int16_t accum[2 * FBT_FFT_SIZE] __attribute__((aligned(16)));
    int     accum_n; // complex samples currently in accum

    // Absolute sample index of the NEXT chunk we'll feed to the tagger.
    // Matches signal_buffer's head modulo SIGNAL_BUF cap.
    uint64_t next_sample_idx;

    // Diagnostic accumulators (reset by get_stage_stats once per second).
    // T48: dsp_processor_feed()/process_chunk() (the writers) now run on
    // the dsp_feed task while dsp_processor_get_stage_stats() (the
    // reader+resetter) runs on usb_pump — before the split both were the
    // same task, so plain volatile was safe w.r.t. this specific
    // reader/writer pair. Now that a higher-priority task (pump, prio
    // pump_prio) can preempt the writer (dsp_feed, prio pump_prio-1)
    // mid read-modify-write, a 64-bit accumulator could tear on this
    // 32-bit core. _Atomic + relaxed ordering (matches usbring.c /
    // sd_capture.c's existing idiom) costs nothing on the decode path —
    // these are diagnostics-only — and removes the tear.
    _Atomic(uint64_t) acc_step_us;
    _Atomic(uint32_t) acc_input_samples;
    _Atomic(uint32_t) acc_new_bursts;
    _Atomic(uint32_t) acc_gone_bursts;

    // Non-resetting cumulative counter for callers that compute their own
    // deltas (e.g. /diag/dsp_health's 2-second window — #127). Readers
    // snapshot at t0/t1 and subtract. Same cross-task tearing concern as
    // above post-T48; _Atomic removes it.
    _Atomic(uint64_t) total_input_samples;
};

// Process "default" instance for cross-task diagnostic getters that
// can't be handed the owner's handle (httpd /diag/dsp_health). Set by
// create(). The firmware only ever makes one detector.
static dsp_processor_t *s_default = NULL;

// Push one gone burst out to the user callback. Converts fbt_burst_t
// (FFT bin space, uint64 sample idx) into detected_burst_t (signed
// rel_freq_hz, uint32 sample idx for signal_buffer).
//
// Dispatch fires on GONE events (not new) to match gr-iridium's
// tagged_burst_to_pdu_impl.cc:205-227 semantics. The gone event
// carries `stop = last_active + burst_post_len`, so the burst
// length passed to the worker is variable — long enough for
// multi-frame bursts where gri's handle_multiple_frames_per_burst
// would publish the whole PDU.
static void dispatch_gone_burst(dsp_processor_t *p, const fbt_burst_t *b)
{
    if (!p->user_cb) return;

    int   signed_bin  = b->center_bin - FBT_FFT_SIZE / 2;
    float rel_freq_hz = (float)signed_bin * (float)FS_DETECT_HZ / (float)FBT_FFT_SIZE;

    // length = stop - start, variable per burst. Clamp at uint32 max
    // to be safe; the worker further clamps to WB_EXTRACT_MAX.
    uint64_t length     = b->stop - b->start;
    uint32_t length_u32 = length > 0xFFFFFFFFu
                              ? 0xFFFFFFFFu
                              : (uint32_t)length;

    // fft_burst_tagger's magnitude_db is ALREADY the SNR
    // (10·log10(mag² · HISTORY / baseline_sum) — see
    // fft_burst_tagger.c). Don't subtract noise_db.
    detected_burst_t out = {
        .start_sample_idx = b->start, // T44: keep full 64-bit cumulative index
        .length_samples   = length_u32,
        .rel_freq_hz      = rel_freq_hz,
        .peak_snr_db      = b->magnitude_db,
        .magnitude_db     = b->magnitude_db,
        .noise_db         = b->noise_db,
        // T60: pack center_bin (low 16) + width_bins (high 16) — keeps
        // detected_burst_t byte-identical (no PIE-position perturbation).
        .peak_bin = BURST_PACK_BIN_WIDTH(b->center_bin, b->width_bins),
    };
    p->user_cb(&out);
}

// Process one FFT-aligned chunk: hand it to the tagger, advance the
// sample index, dispatch gone-burst events.
static void process_chunk(dsp_processor_t *p, const int16_t *chunk_iq)
{
    fbt_burst_t new_bursts[FBT_NEW_BUF_SIZE];
    fbt_burst_t gone_bursts[FBT_GONE_BUF_SIZE];
    int         n_new  = FBT_NEW_BUF_SIZE;
    int         n_gone = FBT_GONE_BUF_SIZE;

    int64_t t0 = esp_timer_get_time();
    bool    ok = fft_burst_tagger_step(p->tagger, chunk_iq, NULL,
                                       new_bursts, &n_new,
                                       gone_bursts, &n_gone);
    int64_t t1 = esp_timer_get_time();
    atomic_fetch_add_explicit(&p->acc_step_us, (uint64_t)(t1 - t0), memory_order_relaxed);

    if (ok) {
        for (int i = 0; i < n_gone; i++)
            dispatch_gone_burst(p, &gone_bursts[i]);
        atomic_fetch_add_explicit(&p->acc_new_bursts, (uint32_t)n_new, memory_order_relaxed);
        atomic_fetch_add_explicit(&p->acc_gone_bursts, (uint32_t)n_gone, memory_order_relaxed);
    }

    p->next_sample_idx += FBT_FFT_SIZE;
}

// End-of-stream flush. Forces any still-active bursts to emit their
// gone-event with stop = current d_index. Use at end of an offline
// fixture or when the SDR source closes; on a live RF feed, bursts
// naturally time out and reach the gone callback in the steady-state
// step loop — no flush needed.
void dsp_processor_flush(dsp_processor_t *p)
{
    if (!p || !p->tagger) return;
    fbt_burst_t flushed[FBT_GONE_BUF_SIZE];
    int         n = FBT_GONE_BUF_SIZE;
    fft_burst_tagger_flush(p->tagger, flushed, &n);
    for (int i = 0; i < n; i++)
        dispatch_gone_burst(p, &flushed[i]);
    atomic_fetch_add_explicit(&p->acc_gone_bursts, (uint32_t)n, memory_order_relaxed);
    ESP_LOGI(TAG, "fbt flush: emitted %d residual bursts", n);
}

dsp_processor_t *dsp_processor_create(burst_detected_cb_t cb)
{
    // Pull threshold from NVS-backed config (D18). Falls back to the
    // compile-time default if app_config wasn't initialised.
    app_config_t cfg;
    app_config_snapshot(&cfg);
    float thr = cfg.tagger_threshold_db;
    if (thr <= 0.0f || thr > 30.0f) thr = FBT_THRESHOLD_DB; // sanity
    ESP_LOGI(TAG,
             "Creating wideband fft_burst_tagger (N=%d, fs=%u Hz, thr=%.1f dB)",
             FBT_FFT_SIZE, (unsigned)FS_DETECT_HZ, (double)thr);

    // Internal SRAM for the handle so the hot-path accum[] stays fast
    // (it was file-scope .bss / internal before). The struct holds no
    // PIE buffers, so its placement is perf-only, not correctness.
    dsp_processor_t *p = heap_caps_calloc(1, sizeof(*p), MALLOC_CAP_INTERNAL);
    if (!p) {
        ESP_LOGE(TAG, "dsp_processor handle alloc (%zu bytes internal) failed",
                 sizeof(*p));
        return NULL;
    }
    p->user_cb = cb;

    // 4 MB baseline_history in PSRAM. Internal SRAM doesn't have room
    // (int32 × FFT_SIZE × HISTORY_SIZE = 2048 × 512 × 4 = 4 MB).
    size_t bytes        = (size_t)FBT_FFT_SIZE * FBT_HISTORY_SIZE * sizeof(int32_t);
    p->baseline_history = (int32_t *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
    if (!p->baseline_history) {
        ESP_LOGE(TAG, "baseline_history alloc %zu bytes (PSRAM) failed", bytes);
        heap_caps_free(p);
        return NULL;
    }

    p->tagger = fft_burst_tagger_init(FBT_BURST_PRE_LEN, FBT_BURST_POST_LEN,
                                      FBT_BURST_WIDTH, thr,
                                      p->baseline_history);
    if (!p->tagger) {
        ESP_LOGE(TAG, "fft_burst_tagger_init failed");
        heap_caps_free(p->baseline_history);
        heap_caps_free(p);
        return NULL;
    }

    fft_burst_tagger_set_start(p->tagger, 0);
    s_default = p; // publish for cross-task diagnostic readers
    return p;
}

dsp_processor_t *dsp_processor_default(void)
{
    return s_default;
}

void dsp_processor_feed(dsp_processor_t *p, const int16_t *samples, size_t n_samples)
{
    // n_samples is complex IQ pairs. At FS_DETECT_HZ the typical USB
    // transfer (16 KB raw / 2 bytes per complex) is ~8000 complex
    // after the 125/128 resample in ingest_core1.
    if (!p || !p->tagger) return;

    atomic_fetch_add_explicit(&p->acc_input_samples, (uint32_t)n_samples, memory_order_relaxed);
    atomic_fetch_add_explicit(&p->total_input_samples, (uint64_t)n_samples, memory_order_relaxed); // #127, never reset

    size_t off = 0;
    while (off < n_samples) {
        size_t space   = FBT_FFT_SIZE - (size_t)p->accum_n;
        size_t to_copy = n_samples - off;
        if (to_copy > space) to_copy = space;

        memcpy(p->accum + (size_t)p->accum_n * 2,
               samples + off * 2,
               to_copy * 2 * sizeof(int16_t));
        p->accum_n += (int)to_copy;
        off += to_copy;

        if (p->accum_n == FBT_FFT_SIZE) {
            process_chunk(p, p->accum);
            p->accum_n = 0;
        }
    }
}

void dsp_processor_get_stage_stats(dsp_processor_t *p, dsp_stage_stats_t *out)
{
    if (!p) {
        memset(out, 0, sizeof(*out));
        return;
    }
    // T48: read-and-reset each accumulator in one atomic_exchange so
    // there's no load-then-clear window a concurrent dsp_feed increment
    // could fall into (this getter runs on usb_pump; the increments run
    // on dsp_feed — see the struct's field comments). Snapshot every
    // value up front and derive the rest from the snapshot, matching the
    // original same-task semantics exactly.
    uint64_t acc_step_us_snap = atomic_exchange_explicit(&p->acc_step_us, 0, memory_order_relaxed);
    uint32_t acc_input_snap   = atomic_exchange_explicit(&p->acc_input_samples, 0, memory_order_relaxed);
    uint32_t acc_new_snap     = atomic_exchange_explicit(&p->acc_new_bursts, 0, memory_order_relaxed);
    uint32_t acc_gone_snap    = atomic_exchange_explicit(&p->acc_gone_bursts, 0, memory_order_relaxed);

    uint32_t frames          = acc_input_snap / FBT_FFT_SIZE;
    uint64_t tag_stage_us[5] = {0};
    uint32_t tag_steps       = 0;
    fft_burst_tagger_get_stage_us(tag_stage_us, &tag_steps);

    if (frames == 0) {
        memset(out, 0, sizeof(*out));
    } else {
        float fn         = (float)frames;
        float ts         = (tag_steps > 0) ? (float)tag_steps : 1.0f;
        out->frames      = frames;
        out->wind_us     = (float)tag_stage_us[0] / ts;
        out->fft_us      = (float)tag_stage_us[1] / ts;
        out->mag_us      = (float)tag_stage_us[2] / ts;
        out->detect_us   = (float)tag_stage_us[3] / ts;
        out->baseline_us = (float)tag_stage_us[4] / ts;
        out->total_us    = (float)acc_step_us_snap / fn;
    }
    // Raw accumulators for the `fbt:` line. No ESP_LOGI here: this
    // getter runs on usb_pump's 1 Hz snapshot and log formatting belongs
    // on Core 1 — status_logger emits the line from these fields.
    out->new_bursts  = acc_new_snap;
    out->gone_bursts = acc_gone_snap;
    out->step_us     = (uint32_t)acc_step_us_snap;
    out->tag_steps   = tag_steps;
}

// #127: race-free cumulative FFT-frames count. Callers that compute
// their own delta over a wall-clock window (e.g. /diag/dsp_health)
// must use this instead of dsp_processor_get_stage_stats — the latter
// resets its accumulator on every call, so any second reader (e.g.
// status_logger at 1 Hz) destroys the snapshot.
uint64_t dsp_processor_get_total_fft_frames(dsp_processor_t *p)
{
    if (!p) return 0;
    return atomic_load_explicit(&p->total_input_samples, memory_order_relaxed) / FBT_FFT_SIZE;
}
