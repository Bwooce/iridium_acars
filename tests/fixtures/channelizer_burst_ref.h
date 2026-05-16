// Shared types for channelizer fixture references. Each fixture_albq_raw*.h
// declares an array of these describing the gr-iridium-detected bursts in
// its 2.56 MHz subband; the channelizer cross-check test verifies our
// channelizer routes each burst's energy into its expected_channel.
#pragma once
#include <stdint.h>

typedef struct {
    int32_t  rel_hz;             // burst freq relative to fixture's LO
    uint8_t  expected_channel;   // round(rel_hz / 40000) mod 64
    uint8_t  conf_pct;            // gr-iridium confidence 0..100
    float    snr_db;             // SNR gr-iridium reported
    float    time_ms;            // burst time in the source recording
} channelizer_burst_ref_t;
