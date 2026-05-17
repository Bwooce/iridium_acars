// dsp_processor — front-end burst detector. As of D7 this is a thin
// shim around channelizer_detector (common/iridium_decoder/), which
// runs a 64-channel polyphase channelizer + per-channel envelope
// detector in place of the legacy 2048-bin single-FFT detector.
//
// The motivation: the single-FFT detector picked one strongest bin
// across the whole 2.56 MHz subband. When two concurrent bursts on
// nearby channels overlapped in time, the bin average landed between
// them, putting the residual carrier offset outside the PLL's
// capture range. Per-channel detection bounds the centre-frequency
// error at ½ × 40 kHz = 20 kHz, well inside the PLL.
//
// The legacy windowing / magnitude / baseline-EMA PIE kernels
// (dsp_window_arp4.S, dsp_mag_arp4.S) are still in this directory;
// they're templates for the D20 int16/PIE rewrite of the channelizer
// hot path. They're built but unused at runtime; the linker drops
// the orphan sections from the final image.

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "dsp_processor.h"
#include "channelizer_detector.h"
#include "polyphase_channelizer.h"

static const char *TAG = "DSP_PROC";

// Front-end sample rate. The SDR is locked to 2.56 MSPS; this is the
// same rate the channelizer expects. polyphase_channelizer_create
// validates it internally.
// FS_IN_HZ now lives in dsp_processor.h (shared with worker_core1.c).

// Detector threshold. 16 dB matches the legacy float setpoint
// (40× linear, kept across the conversion to uint32 arithmetic).
#define DETECTOR_DB     16.0f

// Mapping from channelizer channel index → fftshift bin. The smoke
// test and the worker's freq-centring still operate on peak_bin in
// the conventional fftshift order (bin FFT_SIZE/2 = DC, bin > FFT_SIZE/2
// = positive freq). With M=64 channels in FFT_SIZE=2048 bins, each
// channel maps to 32 contiguous bins; we pick the channel centre.
#define BIN_PER_CHAN    (FFT_SIZE / POLYCHAN_M)

static channelizer_detector_t *s_det = NULL;
static burst_detected_cb_t     s_user_cb = NULL;

// Per-feed total wall time + total samples consumed. The status
// logger calls dsp_processor_get_stage_stats once per second and
// expects a per-FFT-frame breakdown (wind/fft/mag/detect/baseline).
// Most of those stages don't exist any more; we report the combined
// channelize+detect cost in the fft_us slot so the existing log
// line has meaningful data, and zero the others. Document this in
// dsp_processor.h.
static volatile uint64_t s_acc_feed_us       = 0;
static volatile uint32_t s_acc_input_samples = 0;

// User callback adapter: channelizer_burst_t → detected_burst_t.
static void on_channelizer_burst(const channelizer_burst_t *cb,
                                  void *user)
{
    (void)user;
    if (!s_user_cb) return;

    int signed_off = (cb->channel > POLYCHAN_M / 2)
                     ? (cb->channel - POLYCHAN_M)
                     : cb->channel;
    int peak_bin = ((FFT_SIZE / 2) + signed_off * BIN_PER_CHAN)
                   & (FFT_SIZE - 1);

    detected_burst_t out = {
        .start_sample_idx = cb->start_sample_idx,
        .length_samples   = cb->length_samples,
        .peak_bin         = peak_bin,
        .peak_snr_db      = cb->snr_db,
    };
    s_user_cb(&out);
}

esp_err_t dsp_processor_init(burst_detected_cb_t cb)
{
    ESP_LOGI(TAG,
             "Initializing Channelizer Detector (M=%d, fs=%u Hz, thr=%.1f dB)",
             POLYCHAN_M, (unsigned)FS_IN_HZ, (double)DETECTOR_DB);
    s_user_cb = cb;

    if (s_det) {
        channelizer_detector_destroy(s_det);
        s_det = NULL;
    }
    s_det = channelizer_detector_create(FS_IN_HZ, DETECTOR_DB,
                                         on_channelizer_burst, NULL);
    if (!s_det) {
        ESP_LOGE(TAG, "channelizer_detector_create failed");
        return ESP_ERR_NO_MEM;
    }

    s_acc_feed_us       = 0;
    s_acc_input_samples = 0;
    return ESP_OK;
}

void dsp_processor_feed(const int16_t *samples, size_t n_samples)
{
    // n_samples is complex samples (I,Q pairs). At 2.56 MSPS the
    // typical USB transfer is 16 KB = 8192 IQ pairs per call.
    if (!s_det) return;
    int64_t t0 = esp_timer_get_time();
    channelizer_detector_feed_int16(s_det, samples, n_samples);
    int64_t t1 = esp_timer_get_time();
    s_acc_feed_us       += (uint64_t)(t1 - t0);
    s_acc_input_samples += (uint32_t)n_samples;
}

void dsp_processor_get_stage_stats(dsp_stage_stats_t *out)
{
    // Normalise to FFT_SIZE-sample "frames" (2048 input samples) so
    // the printed numbers stay comparable to historical FFT-detector
    // logs. With 8192 samples per feed call we expect ~4 frame-eq's
    // per call.
    uint32_t frames = s_acc_input_samples / FFT_SIZE;
    if (frames == 0) {
        memset(out, 0, sizeof(*out));
    } else {
        float fn = (float)frames;
        out->frames      = frames;
        out->wind_us     = 0.0f;
        out->fft_us      = (float)s_acc_feed_us / fn;   // channelize + detect
        out->mag_us      = 0.0f;
        out->detect_us   = 0.0f;
        out->baseline_us = 0.0f;
        out->total_us    = out->fft_us;
    }

    // Side-band: emit a one-liner with channelizer-specific stats
    // (peak # active channels, bursts emitted in this window) so we
    // can see multi-burst concurrency on the real-RF feed without
    // changing the status_logger struct.
    if (s_det) {
        channelizer_detector_stats_t cs;
        channelizer_detector_get_stats(s_det, &cs);
        ESP_LOGI(TAG,
                 "channelizer: bursts=%u chans_active_peak=%u "
                 "cycles=%u",
                 (unsigned)cs.bursts_detected,
                 (unsigned)cs.channels_active_peak,
                 (unsigned)cs.cycles_processed);
    }

    s_acc_feed_us       = 0;
    s_acc_input_samples = 0;
}
