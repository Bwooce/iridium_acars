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

void poa_frontend_destroy(poa_frontend_t *fe);
