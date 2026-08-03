// poa_frontend — POA channelizer, ported from acarsdec rtl.c in_callback
// (Thierry Leconte, GPLv2). Takes the wideband 2.5 MSPS complex IQ and, per
// channel, mixes to baseband + integrate-dumps by rtlMult (= fs/12500 = 200)
// + takes the |.| AM envelope, driving one poa_decoder channel each. This is
// POA's CHANNELIZED front end — the tagger-bypass (plan §1). Because the
// output is an AM envelope, the per-block oscillator reset is harmless (plan).
//
// Pure C11 + libm — links into tests/host directly. Input scale is irrelevant
// (the demod normalizes per bit), so feed int16 IQ of any scale (the device's
// resampled s16 stream, or s16 read straight from a capture in tests).

#pragma once

#include <stdint.h>
#include "poa_decoder.h"

#define POA_INTRATE 12500

typedef struct poa_frontend poa_frontend_t;

// fs_hz must be an integer multiple of 12500 (rtlMult = fs_hz/12500 <= 320).
// chan_hz[] are the absolute channel frequencies; lo_hz is the capture center.
poa_frontend_t *poa_frontend_create(uint32_t fs_hz, uint32_t lo_hz,
                                    const uint32_t *chan_hz, int nch,
                                    poa_block_cb cb, void *user);

// Feed nsamp COMPLEX samples as interleaved int16 I/Q (length 2*nsamp).
// The channelizer reads whole rtlMult-blocks straight from this buffer (no
// intermediate float copy) and converts to float inline in the hot loop.
void poa_frontend_feed(poa_frontend_t *fe, const int16_t *iq, int nsamp);

// Per-channel POA telemetry: envelope level ("signal strength") + the decoder's
// demod-activity counters. Read-and-reset (call periodically). This is the
// live-reception readout the channelized POA path otherwise has none of (no
// tagger/SNR): env_mean/peak show whether a channel hears energy above the
// noise floor; sync/blk_start/delivered/crc_fail show how far the demod gets.
typedef struct {
    int      nch;
    float    env_mean[POA_MAX_CHANNELS];  // mean envelope |D| since last read
    float    env_peak[POA_MAX_CHANNELS];  // peak envelope since last read
    uint32_t sync[POA_MAX_CHANNELS];      // SYN sync locks
    uint32_t blk_start[POA_MAX_CHANNELS]; // SOH block starts
    uint32_t delivered[POA_MAX_CHANNELS]; // ACARS blocks emitted
    uint32_t crc_fail[POA_MAX_CHANNELS];  // blocks reaching CRC/parity but dropped
} poa_stats_t;

void poa_frontend_get_stats(poa_frontend_t *fe, poa_stats_t *out);

void poa_frontend_destroy(poa_frontend_t *fe);
