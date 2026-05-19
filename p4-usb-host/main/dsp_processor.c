// dsp_processor — wideband burst detector. As of the Phase 3.6.M
// cutover this is a thin shim around fft_burst_tagger
// (common/iridium_decoder/), which runs the gr-iridium wideband
// spectrogram detector against the full 2.5 MSPS input subband.
//
// History:
//   1. Single 2048-pt FFT with per-bin EMA threshold (legacy). One
//      strongest bin across the whole subband; collapsed two
//      concurrent bursts that overlapped in time.
//   2. Polyphase channelizer + per-channel detector (D7). Bounded
//      centre-frequency error at ½ × 40 kHz = 20 kHz per channel.
//      Hit real-time on P4 but had ~8 dB SNR loss vs gri and 40 kHz
//      frequency quantisation that swamped the PLL's capture range.
//   3. (this) Wideband fft_burst_tagger + per-burst direct_if_decim.
//      gr-iridium-equivalent per stage on host (Phase 3.6.M, commit
//      de72f24). Each burst carries an exact relative-frequency tag
//      (FFT bin → Hz), so the worker's rotation is sub-bin precise
//      and the burst_pipeline PLL only needs to track residual scatter.
//
// The channelizer files (channelizer_detector, polyphase_channelizer,
// polyphase_mac_arp4) and the legacy windowing/magnitude PIE kernels
// (dsp_window_arp4.S, dsp_mag_arp4.S) remain in-tree as references
// for any future PIE work; orphan sections drop in the linker output.

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

// Tagger threshold over the EMA baseline. gr-iridium's default is
// ~10 dB; matches the threshold_mult parameter discussed in the
// fft_burst_tagger header.
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
static int16_t            *s_lookback = NULL;           // internal SRAM,
                                                         //  2*FBT_BURST_PRE_LEN
                                                         //  int16
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

// Push one new burst out to the user callback. Converts fbt_burst_t
// (FFT bin space, uint64 sample idx) into detected_burst_t (signed
// rel_freq_hz, uint32 sample idx for signal_buffer).
static void dispatch_new_burst(const fbt_burst_t *b)
{
    if (!s_user_cb) return;

    // FFT-bin → Hz. center_bin is in fft-shift space (DC-centred); the
    // signed offset from band centre is (center_bin - FFT_SIZE/2).
    int signed_bin = b->center_bin - FBT_FFT_SIZE / 2;
    float rel_freq_hz = (float)signed_bin * (float)FS_DETECT_HZ
                         / (float)FBT_FFT_SIZE;

    detected_burst_t out = {
        .start_sample_idx = (uint32_t)b->start,
        .length_samples   = FBT_BURST_POST_LEN,  // see note above
        .rel_freq_hz      = rel_freq_hz,
        .peak_snr_db      = b->magnitude_db - b->noise_db,
        .magnitude_db     = b->magnitude_db,
        .noise_db         = b->noise_db,
        .peak_bin         = b->center_bin,
    };
    s_user_cb(&out);
}

// Process one FFT-aligned chunk: hand it to the tagger, advance the
// sample index, copy chunk into lookback buffer for the next step.
static void process_chunk(const int16_t *chunk_iq)
{
    fbt_burst_t new_bursts [FBT_NEW_BUF_SIZE];
    fbt_burst_t gone_bursts[FBT_GONE_BUF_SIZE];
    int n_new  = FBT_NEW_BUF_SIZE;
    int n_gone = FBT_GONE_BUF_SIZE;

    int64_t t0 = esp_timer_get_time();
    bool ok = fft_burst_tagger_step(s_tagger, chunk_iq, s_lookback,
                                     new_bursts,  &n_new,
                                     gone_bursts, &n_gone);
    int64_t t1 = esp_timer_get_time();
    s_acc_step_us += (uint64_t)(t1 - t0);

    if (ok) {
        for (int i = 0; i < n_new; i++) dispatch_new_burst(&new_bursts[i]);
        s_acc_new_bursts  += (uint32_t)n_new;
        s_acc_gone_bursts += (uint32_t)n_gone;
    }

    // Slide lookback forward by FBT_FFT_SIZE: old[FFT_SIZE..2*FFT_SIZE-1]
    // becomes new[0..FFT_SIZE-1]; chunk becomes new[FFT_SIZE..2*FFT_SIZE-1].
    memmove(s_lookback,
            s_lookback + 2 * FBT_FFT_SIZE,
            2 * FBT_FFT_SIZE * sizeof(int16_t));
    memcpy(s_lookback + 2 * FBT_FFT_SIZE, chunk_iq,
           2 * FBT_FFT_SIZE * sizeof(int16_t));

    s_next_sample_idx += FBT_FFT_SIZE;
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

    // 2 × burst_pre_len int16 in internal SRAM for the FFT history.
    // burst_pre_len is 4096, so this is 16 KB. The tagger header
    // contract: lookback points to 2*burst_pre_len int16
    // = 2 × 4096 = 8192 int16 = 16 KB. Allocate once.
    if (!s_lookback) {
        size_t bytes = 2 * FBT_BURST_PRE_LEN * sizeof(int16_t);
        s_lookback = (int16_t *)heap_caps_aligned_alloc(16, bytes,
                                                         MALLOC_CAP_INTERNAL
                                                         | MALLOC_CAP_8BIT);
        if (!s_lookback) {
            ESP_LOGE(TAG, "lookback alloc %zu bytes (internal) failed",
                     bytes);
            return ESP_ERR_NO_MEM;
        }
        memset(s_lookback, 0, bytes);
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
