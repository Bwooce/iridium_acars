// dsp_processor — wideband burst detector. Thin shim around
// fft_burst_tagger (common/iridium_decoder/), which runs the gr-iridium
// wideband spectrogram detector against the full 2.5 MSPS input
// subband. The Phase 3.6.M cutover (commit bf0bb9f) replaced an
// earlier single-FFT detector and a polyphase channelizer with this
// gri-aligned path; both predecessors were removed from the tree.
// Each tagged burst carries an exact relative-frequency tag (FFT
// bin → Hz), so the worker's rotation is sub-bin precise and the
// downstream burst_pipeline PLL only tracks residual scatter.

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "dsp_processor.h"
#include "fft_burst_tagger.h"
#include "worker_core1.h"

static const char *TAG = "DSP_PROC";

// Tagger threshold over the EMA baseline. 10 dB recovers the best
// decode count we've measured: 59/65 = 91% of gri's reference 65
// on the ALBQ fixture, vs 56/65 = 86% at the 14 dB setting we used
// during the queue-overload era. The 14 dB choice cost 3 real
// decodes (5% absolute) to halve burst count and stop queue
// overflow; the 280-tap PIE FIR fix (commit 7eab12a) made the
// per-burst cost low enough that the worker keeps up at 133
// bursts/sec on slow-feed smoke. Setting back to 10 to recover
// the 5% absolute decode rate.
#define FBT_THRESHOLD_DB    10.0f

// Burst window padding in INPUT samples (at FS_DETECT_HZ). gri's
// defaults: pre = 2*fft_size = 4096, post = sample_rate * 16e-3 =
// 40000. Same numbers work here because FS_DETECT_HZ matches gri's
// nominal 2.5 MSPS.
#define FBT_BURST_PRE_LEN   (2 * FBT_FFT_SIZE)
#define FBT_BURST_POST_LEN  40000

// Burst width in FFT bins (= half an Iridium channel). 40 kHz /
// (2.5 MSPS / 2048) ≈ 32 bins.
#define FBT_BURST_WIDTH     32

static fft_burst_tagger_t *s_tagger = NULL;
static int32_t            *s_baseline_history = NULL;   // PSRAM, 4 MB
// (lookback argument to fft_burst_tagger_step is currently unused in
// the detection math — the header documents it as reserved for a
// future PDU-cut step, and the host test passes NULL. We pass NULL
// here too rather than burn 16 KB of internal SRAM heap on a
// placeholder buffer.)
static burst_detected_cb_t s_user_cb = NULL;

// Accumulator for chunks smaller than FBT_FFT_SIZE complex samples.
// dsp_processor_feed receives variable-length buffers from class_driver
// (typically ~8000 complex after the 125/128 resample of a 16 KB USB
// transfer), so we batch into FFT_SIZE-aligned chunks.
static int16_t  s_accum[2 * FBT_FFT_SIZE]
    __attribute__((aligned(16)));
static int      s_accum_n = 0;     // complex samples currently in s_accum

// Absolute sample index of the NEXT chunk we'll feed to the tagger.
// Matches signal_buffer's head modulo SIGNAL_BUF cap — the worker
// uses signal_buffer_extract(start_sample_idx, ...) and depends on
// this lining up exactly with what signal_buffer_push saw.
static uint64_t s_next_sample_idx = 0;

// Tagger reports new and gone bursts after every FFT step. We
// dispatch new bursts to the worker via the user callback; gone
// bursts are informational (used to update length_samples for the
// already-pushed burst record). For step 1 cutover we publish the
// new burst with length=BURST_POST_LEN as a generous initial
// estimate; the worker's extract window covers the whole period.
//
// gri's burst_downmix actually consumes the (start, stop) tuple
// from the GONE record; our worker doesn't yet wait for the gone
// record before processing. Step 4 (false-positive reduction) can
// switch to gone-record-triggered processing if the early-publish
// model produces too many wasted worker cycles.
#define FBT_NEW_BUF_SIZE   16
#define FBT_GONE_BUF_SIZE  16

// Diagnostic accumulators.
static volatile uint64_t s_acc_step_us       = 0;
static volatile uint32_t s_acc_input_samples = 0;
static volatile uint32_t s_acc_new_bursts    = 0;
static volatile uint32_t s_acc_gone_bursts   = 0;

// Push one gone burst out to the user callback. Converts fbt_burst_t
// (FFT bin space, uint64 sample idx) into detected_burst_t (signed
// rel_freq_hz, uint32 sample idx for signal_buffer).
//
// Dispatch fires on GONE events (not new) to match gr-iridium's
// tagged_burst_to_pdu_impl.cc:205-227 semantics. The gone event
// carries `stop = last_active + burst_post_len`, so the burst
// length passed to the worker is variable — long enough for
// multi-frame bursts where gri's handle_multiple_frames_per_burst
// would publish the whole PDU. Previously we triggered on new
// with a fixed 16 ms length and truncated multi-frame bursts to
// their first frame.
static void dispatch_gone_burst(const fbt_burst_t *b)
{
    if (!s_user_cb) return;

    int signed_bin = b->center_bin - FBT_FFT_SIZE / 2;
    float rel_freq_hz = (float)signed_bin * (float)FS_DETECT_HZ
                         / (float)FBT_FFT_SIZE;

    // length = stop - start, variable per burst. Clamp at uint32 max
    // to be safe; the worker further clamps to WB_EXTRACT_MAX.
    uint64_t length = b->stop - b->start;
    uint32_t length_u32 = length > 0xFFFFFFFFu
                          ? 0xFFFFFFFFu : (uint32_t)length;

    // fft_burst_tagger's magnitude_db is ALREADY the SNR
    // (10·log10(mag² · HISTORY / baseline_sum) — see
    // fft_burst_tagger.c). Don't subtract noise_db.
    detected_burst_t out = {
        .start_sample_idx = (uint32_t)b->start,
        .length_samples   = length_u32,
        .rel_freq_hz      = rel_freq_hz,
        .peak_snr_db      = b->magnitude_db,
        .magnitude_db     = b->magnitude_db,
        .noise_db         = b->noise_db,
        .peak_bin         = b->center_bin,
    };
    s_user_cb(&out);
}

// Process one FFT-aligned chunk: hand it to the tagger, advance the
// sample index, dispatch gone-burst events.
static void process_chunk(const int16_t *chunk_iq)
{
    fbt_burst_t new_bursts [FBT_NEW_BUF_SIZE];
    fbt_burst_t gone_bursts[FBT_GONE_BUF_SIZE];
    int n_new  = FBT_NEW_BUF_SIZE;
    int n_gone = FBT_GONE_BUF_SIZE;

    int64_t t0 = esp_timer_get_time();
    bool ok = fft_burst_tagger_step(s_tagger, chunk_iq, NULL,
                                     new_bursts,  &n_new,
                                     gone_bursts, &n_gone);
    int64_t t1 = esp_timer_get_time();
    s_acc_step_us += (uint64_t)(t1 - t0);

    if (ok) {
        for (int i = 0; i < n_gone; i++) dispatch_gone_burst(&gone_bursts[i]);
        s_acc_new_bursts  += (uint32_t)n_new;
        s_acc_gone_bursts += (uint32_t)n_gone;
    }

    s_next_sample_idx += FBT_FFT_SIZE;
}

// End-of-stream flush. Forces any still-active bursts to emit their
// gone-event with stop = current d_index. Use at end of an offline
// fixture or when the SDR source closes; on a live RF feed, bursts
// naturally time out and reach the gone callback in the steady-state
// step loop — no flush needed.
void dsp_processor_flush(void)
{
    if (!s_tagger) return;
    fbt_burst_t flushed[FBT_GONE_BUF_SIZE];
    int n = FBT_GONE_BUF_SIZE;
    fft_burst_tagger_flush(s_tagger, flushed, &n);
    for (int i = 0; i < n; i++) dispatch_gone_burst(&flushed[i]);
    s_acc_gone_bursts += (uint32_t)n;
    ESP_LOGI(TAG, "fbt flush: emitted %d residual bursts", n);
}

esp_err_t dsp_processor_init(burst_detected_cb_t cb)
{
    ESP_LOGI(TAG,
             "Initializing wideband fft_burst_tagger (N=%d, fs=%u Hz, thr=%.1f dB)",
             FBT_FFT_SIZE, (unsigned)FS_DETECT_HZ, (double)FBT_THRESHOLD_DB);
    s_user_cb = cb;

    if (s_tagger) {
        fft_burst_tagger_destroy(s_tagger);
        s_tagger = NULL;
    }

    // 4 MB baseline_history in PSRAM. Internal SRAM doesn't have room
    // (each int32 × FFT_SIZE × HISTORY_SIZE = 2048 × 512 × 4 = 4 MB).
    if (!s_baseline_history) {
        size_t bytes = (size_t)FBT_FFT_SIZE * FBT_HISTORY_SIZE
                       * sizeof(int32_t);
        s_baseline_history = (int32_t *)heap_caps_malloc(bytes,
                                                          MALLOC_CAP_SPIRAM);
        if (!s_baseline_history) {
            ESP_LOGE(TAG, "baseline_history alloc %zu bytes (PSRAM) failed",
                     bytes);
            return ESP_ERR_NO_MEM;
        }
    }

    s_tagger = fft_burst_tagger_init(FBT_BURST_PRE_LEN, FBT_BURST_POST_LEN,
                                      FBT_BURST_WIDTH, FBT_THRESHOLD_DB,
                                      s_baseline_history);
    if (!s_tagger) {
        ESP_LOGE(TAG, "fft_burst_tagger_init failed");
        return ESP_ERR_NO_MEM;
    }

    s_next_sample_idx   = 0;
    s_accum_n           = 0;
    fft_burst_tagger_set_start(s_tagger, 0);

    s_acc_step_us       = 0;
    s_acc_input_samples = 0;
    s_acc_new_bursts    = 0;
    s_acc_gone_bursts   = 0;
    return ESP_OK;
}

void dsp_processor_feed(const int16_t *samples, size_t n_samples)
{
    // n_samples is complex IQ pairs. At FS_DETECT_HZ the typical USB
    // transfer (16 KB raw / 2 bytes per complex) is ~8000 complex
    // after the 125/128 resample in ingest_core1.
    if (!s_tagger) return;

    s_acc_input_samples += (uint32_t)n_samples;

    size_t off = 0;
    while (off < n_samples) {
        size_t space   = FBT_FFT_SIZE - (size_t)s_accum_n;
        size_t to_copy = n_samples - off;
        if (to_copy > space) to_copy = space;

        memcpy(s_accum + (size_t)s_accum_n * 2,
               samples + off * 2,
               to_copy * 2 * sizeof(int16_t));
        s_accum_n += (int)to_copy;
        off       += to_copy;

        if (s_accum_n == FBT_FFT_SIZE) {
            process_chunk(s_accum);
            s_accum_n = 0;
        }
    }
}

void dsp_processor_get_stage_stats(dsp_stage_stats_t *out)
{
    uint32_t frames = s_acc_input_samples / FBT_FFT_SIZE;
    if (frames == 0) {
        memset(out, 0, sizeof(*out));
    } else {
        float fn = (float)frames;
        out->frames      = frames;
        out->wind_us     = 0.0f;
        out->fft_us      = (float)s_acc_step_us / fn;  // tagger step total
        out->mag_us      = 0.0f;
        out->detect_us   = 0.0f;
        out->baseline_us = 0.0f;
        out->total_us    = out->fft_us;
    }

    ESP_LOGI(TAG,
             "fbt: new=%u gone=%u frames=%u step_us=%lu",
             (unsigned)s_acc_new_bursts,
             (unsigned)s_acc_gone_bursts,
             (unsigned)frames,
             (unsigned long)s_acc_step_us);

    s_acc_step_us       = 0;
    s_acc_input_samples = 0;
    s_acc_new_bursts    = 0;
    s_acc_gone_bursts   = 0;
}
