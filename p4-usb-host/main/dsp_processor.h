#ifndef DSP_PROCESSOR_H
#define DSP_PROCESSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define FFT_SIZE 2048

// Central rate and frequency constants. Single source of truth for the
// whole target DSP chain; derive everything else from these (FFT bin
// width, decimation cutoffs, etc.).
//
// FS_IN_HZ is the RTL-SDR's native rate (2.56 MSPS). The wideband
// front end actually runs at 2.5 MSPS to match gr-iridium's
// reference settings exactly; the conversion happens in ingest_core1
// before signal_buffer_push, so everything DOWNSTREAM of ingest sees
// FS_DETECT_HZ. Keeping FS_IN_HZ for code that still reasons about
// the SDR's nominal rate (status logger, etc.).
#define FS_IN_HZ                2560000u
#define FS_DETECT_HZ            2500000u    // wideband path rate
#define IRIDIUM_CENTER_FREQ_HZ  1626000000u // SDR tuned LO (mid-band of
                                            // Iridium downlink 1616-1626)
#define IRIDIUM_CHANNEL_HZ      41666.67f   // Iridium channel grid spacing

// One detected burst, as emitted by the wideband fft_burst_tagger
// front end. Coordinates are at FS_DETECT_HZ (2.5 MSPS) in signal_buffer
// frame. rel_freq_hz is the signed offset of the burst's center bin
// from band center (positive = above LO).
typedef struct {
    uint32_t start_sample_idx;   // signal_buffer frame, FS_DETECT_HZ units
    uint32_t length_samples;     // FS_DETECT_HZ samples
    float    rel_freq_hz;        // signed offset from band centre
    float    peak_snr_db;        // magnitude_db - noise_db
    float    magnitude_db;
    float    noise_db;
    int      peak_bin;           // FFT bin, 0..FFT_SIZE-1 (diagnostic)
} detected_burst_t;

typedef void (*burst_detected_cb_t)(const detected_burst_t *burst);

esp_err_t dsp_processor_init(burst_detected_cb_t cb);
void dsp_processor_feed(const int16_t *samples, size_t n_samples);

// End-of-stream flush. Forces still-active bursts to emit their
// gone callback with stop = current sample index. Use at end of
// an offline fixture / when the SDR source closes. Not needed on
// a live feed: real bursts naturally time out via burst_post_len.
void dsp_processor_flush(void);

// Diagnostic stats: average per-frame time in each stage (microseconds),
// computed over frames seen since the last call. Calling this resets the
// internal accumulators so the next call covers a fresh window.
typedef struct {
    uint32_t frames;          // FBT FFT frames in this window
    float    wind_us;         // unused under wideband tagger
    float    fft_us;          // combined fft_burst_tagger_step wall time
    float    mag_us;          // unused
    float    detect_us;       // unused
    float    baseline_us;     // unused
    float    total_us;        // total per-frame
} dsp_stage_stats_t;

void dsp_processor_get_stage_stats(dsp_stage_stats_t *out);

#endif
