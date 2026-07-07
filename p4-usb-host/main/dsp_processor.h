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
#define FS_IN_HZ 2500000u     // Path A: RTL-SDR sampled DIRECTLY at 2.5 MSPS
                              // (== FS_DETECT_HZ) so NO 2.56->2.5 resample is
                              // needed in production. Removes the 2nd PIE owner
                              // (wedge), the resample CPU cost, and the R3 merge.
                              // resample_256_to_250 stays in the lib for the host
                              // gri-parity tests (2.56 MSPS fixtures).
#define FS_DETECT_HZ 2500000u // wideband path rate
// SDR tuned LO. NOTE: 1626 MHz is NOT mid-band — it's the boundary between
// the duplex band (1616.0-1626.0 MHz, 240 user channels @ 41.667 kHz, where
// SBD / aircraft ACARS traffic lives) and the simplex band (1626.0-1626.5
// MHz: Ring Alert @ 1626.2708, IBC broadcast, paging). With FS_IN_HZ this
// LO covers ~1624.72-1627.28 MHz: ALL of simplex + only the top ~1.3 MHz of
// duplex. So this tuning targets Iridium SYSTEM frames (RA/IBC/TL — 24/7,
// no aircraft needed); most duplex USER channels are out of window. To favor
// user/ACARS traffic, move the LO lower (gr-iridium suggests ~1622 MHz),
// trading away the simplex/ring-alert band. Runtime-settable via POST
// /tune?hz=<lo_freq_hz> (NVS-persisted, reboots to apply).
#define IRIDIUM_CENTER_FREQ_HZ 1626000000u
#define IRIDIUM_CHANNEL_HZ 41666.67f // Iridium channel grid spacing

// One detected burst, as emitted by the wideband fft_burst_tagger
// front end. Coordinates are at FS_DETECT_HZ (2.5 MSPS) in signal_buffer
// frame. rel_freq_hz is the signed offset of the burst's center bin
// from band center (positive = above LO).
typedef struct {
    uint64_t start_sample_idx; // cumulative FS_DETECT_HZ index (T44: 64-bit
                               // so signal_buffer_burst_valid can disambiguate
                               // ring laps; ring offset is this % total_cap)
    uint32_t length_samples;   // FS_DETECT_HZ samples
    float    rel_freq_hz;      // signed offset from band centre
    float    peak_snr_db;      // magnitude_db - noise_db
    float    magnitude_db;
    float    noise_db;
    // peak_bin packs TWO 11-bit values to keep this struct byte-identical
    // (growing it shifts the worker's s_pq[] and can perturb PIE state under
    // preemption): low 16 bits = FFT bin (0..FFT_SIZE-1); high 16 bits = T60
    // spectral width in bins (~34 = one Iridium channel; wide = broadband RFI).
    // Read via BURST_PEAK_BIN(b) / BURST_WIDTH_BINS(b). The (int16_t) cast at
    // the PDU pack site already yields the low 16 (bin) for free.
    int peak_bin;
} detected_burst_t;

#define BURST_PEAK_BIN(b) ((b)->peak_bin & 0xFFFF)
#define BURST_WIDTH_BINS(b) (((b)->peak_bin >> 16) & 0xFFFF)
#define BURST_PACK_BIN_WIDTH(bin, width) (((bin) & 0xFFFF) | (((width) & 0xFFFF) << 16))

typedef void (*burst_detected_cb_t)(const detected_burst_t *burst);

// Wideband burst detector instance (#120). All state that used to be
// file-scope static now lives in this context, so the module is reentrant
// and could host multiple detectors (e.g. the multi-receiver SPI
// aggregator). The current firmware runs exactly one; create it in the
// owner task and thread the handle through feed/flush/stats.
typedef struct dsp_processor dsp_processor_t;

// Create a detector. cb fires (in the feed caller's context) once per
// completed burst. Returns NULL on alloc failure. Also published as the
// process "default" instance for cross-task diagnostic readers (see
// dsp_processor_default()).
dsp_processor_t *dsp_processor_create(burst_detected_cb_t cb);

// Most-recently-created instance, for cross-task diagnostic getters (e.g.
// httpd /diag/dsp_health) that legitimately can't be handed the owner's
// handle. NULL before the first create. Hot paths use their own handle.
dsp_processor_t *dsp_processor_default(void);

void dsp_processor_feed(dsp_processor_t *p, const int16_t *samples, size_t n_samples);

// End-of-stream flush. Forces still-active bursts to emit their
// gone callback with stop = current sample index. Use at end of
// an offline fixture / when the SDR source closes. Not needed on
// a live feed: real bursts naturally time out via burst_post_len.
void dsp_processor_flush(dsp_processor_t *p);

// Diagnostic stats: average per-frame time in each stage (microseconds),
// computed over frames seen since the last call. Calling this resets the
// internal accumulators so the next call covers a fresh window.
typedef struct {
    uint32_t frames;      // FBT FFT frames in this window
    float    wind_us;     // unused under wideband tagger
    float    fft_us;      // combined fft_burst_tagger_step wall time
    float    mag_us;      // unused
    float    detect_us;   // unused
    float    baseline_us; // unused
    float    total_us;    // total per-frame
    // Raw window accumulators for the `fbt:` diagnostic line. Formatted
    // and emitted by status_logger on Core 1 — the getter itself must
    // stay log-free (it runs on Core 0's hot loop).
    uint32_t new_bursts;  // tagger bursts started in this window
    uint32_t gone_bursts; // tagger bursts ended in this window
    uint32_t step_us;     // total fft_burst_tagger_step wall time
    uint32_t tag_steps;   // tagger steps in this window
    uint32_t coalesced;   // gone bursts suppressed by the P1.5 coalescer (0 unless coal_n >= 2)
    // Squelch visibility (2026-07-07 decode-regression batch) — window
    // counts from fft_burst_tagger_get_squelch_stats():
    uint32_t squelch_events;  // tagger steps where the burst squelch fired
    uint32_t squelch_dropped; // bursts force-closed by the squelch (counted, never dispatched)
    uint32_t noise_resets;    // squelch-driven noise-estimate resets (0.42 s re-prime each)
} dsp_stage_stats_t;

void dsp_processor_get_stage_stats(dsp_processor_t *p, dsp_stage_stats_t *out);

// #127: race-free cumulative FFT-frames count. Use this from any path
// that wants a delta over a wall-clock window (it is NEVER reset).
// dsp_processor_get_stage_stats resets its accumulator on every call,
// so /diag/dsp_health's 2 s window races status_logger's 1 Hz reset
// and reports phantom dsp_ok=false; this getter avoids the race.
uint64_t dsp_processor_get_total_fft_frames(dsp_processor_t *p);

// Scanner (Phase 1) narrowband-density readout. Counts bursts whose spectral
// width (T60) is <= DSP_NARROWBAND_MAX_BINS, i.e. channel-shaped Iridium
// rather than wide broadband RFI.
#define DSP_NARROWBAND_MAX_BINS 48

typedef struct {
    uint32_t narrowband_bursts;
    uint32_t all_bursts;
    float    mean_snr_db;
} dsp_density_t;

void dsp_processor_read_reset_density(dsp_processor_t *p, dsp_density_t *out);
void dsp_processor_reset_tagger_baseline(dsp_processor_t *p);

#endif
