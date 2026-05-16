// channelizer_detector — burst detector running on top of the polyphase
// channelizer. Replaces the single-FFT spectral detector in
// p4-usb-host/main/dsp_processor.c with M=64 per-channel envelope
// detectors, one per 40 kHz channel.
//
// The motivation (D7): the single-FFT detector picks the strongest bin
// across the whole 2.56 MHz subband. When two concurrent bursts on
// nearby channels overlap in time, the FFT bin-resolution average can
// land between them, putting the residual carrier offset outside the
// PLL's capture range. Per-channel detection eliminates this — each
// channel sees only its own burst, so the centre-frequency error after
// a hit is bounded by ½ channel = 20 kHz, well inside the PLL.
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "polyphase_channelizer.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int      channel;             // 0..POLYCHAN_M-1
    int32_t  rel_freq_hz;         // signed offset from LO (channel × fs/M
                                  //  with k>M/2 mapped to negative)
    float    snr_db;              // peak power / baseline at burst peak
    uint32_t start_sample_idx;    // input-sample index where channel
                                  //  first crossed threshold (×fs_in_hz
                                  //  to convert to time)
    uint32_t length_samples;      // input samples between crossing and
                                  //  burst end
} channelizer_burst_t;

typedef void (*channelizer_burst_cb_t)(const channelizer_burst_t *burst,
                                       void *user);

typedef struct channelizer_detector channelizer_detector_t;

// fs_in_hz: input sample rate (e.g. 2_560_000)
// threshold_db: ratio of channel power over baseline EMA before a
//   burst is declared (16 dB matches the legacy FFT detector setpoint)
// cb / user: invoked once per burst end with the burst record. The
//   callback runs from feed() context; keep work bounded.
channelizer_detector_t *channelizer_detector_create(uint32_t fs_in_hz,
                                                     float threshold_db,
                                                     channelizer_burst_cb_t cb,
                                                     void *user);
void channelizer_detector_destroy(channelizer_detector_t *d);

// Feed int16 IQ samples (interleaved I,Q at fs_in_hz, full-scale int16
// matching what dsp_processor.c receives from the smoke test / SDR).
void channelizer_detector_feed_int16(channelizer_detector_t *d,
                                      const int16_t *iq,
                                      size_t n_complex);

// Flush any pending bursts that are currently in the post-burst
// cooldown state. Production code never needs to call this — the
// cooldown self-resolves as more samples arrive. Use it in tests /
// at end-of-stream / before destroy to make sure no burst is left
// pending in the detector's state machine.
void channelizer_detector_flush(channelizer_detector_t *d);

// Stats since last call (resets accumulators).
typedef struct {
    uint32_t input_samples_seen;
    uint32_t cycles_processed;
    uint32_t bursts_detected;
    uint32_t channels_active_peak;   // max # channels above threshold
                                     //  in any one cycle
} channelizer_detector_stats_t;

void channelizer_detector_get_stats(channelizer_detector_t *d,
                                     channelizer_detector_stats_t *out);

#ifdef __cplusplus
}
#endif
