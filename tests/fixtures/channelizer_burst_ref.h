// Burst-reference fixture struct, historically named after the
// channelizer that was the consumer of these tables. The channelizer
// itself was removed when the wideband fft_burst_tagger took over as
// the active front end in Phase 3.6.M, but the struct lives on as
// the canonical shape of "burst metadata" inside the ALBQ fixture
// tables (fixture_albq_raw.h, fixture_albq_stripe_*.h, etc.) and a
// few host tests that still want to know what gri found at each
// position in the raw recording. Renaming the type would touch every
// fixture header for no functional gain.
//
// Each entry: a single burst gr-iridium detected in the raw fixture's
// subband, recorded once at fixture-build time.
#pragma once
#include <stdint.h>

typedef struct {
    int32_t  rel_hz;          // signed offset from the fixture's LO
    uint32_t channel;         // historical channelizer channel (0..M-1)
    uint32_t confidence_pct;  // detection confidence (0..100)
    float    snr_db;          // SNR gri reported for this burst
    float    timestamp_ms;    // burst start, relative to fixture t=0
} channelizer_burst_ref_t;
