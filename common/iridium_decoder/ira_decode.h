#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "iridium_frame.h"

// Decoded fields from an Iridium Ring Alert (IRA) frame.
//
// Reference: iridium-toolkit/bitsparser.py IridiumRAMessage (line 1553).
// IRA layout (no header — straight to BCH blocks):
//   - First 96 bits = 3 × 32-bit codewords, 3-way interleaved
//     (de_interleave3). Each codeword: BCH(31, 21) with poly=1207
//     (ringalert) + 1 trailing parity bit → 21 data bits.
//   - Three codewords × 21 = 63 data bits of fixed "header":
//       bits  0- 6  sv_id       satellite vehicle id
//       bits  7-12  beam_id     cell / beam id
//       bit  13     pos_x sign (12-bit signed expanded across 13)
//       bits 14-24  pos_x       11-bit unsigned mantissa of X coordinate
//       bit  25     pos_y sign
//       bits 26-36  pos_y       11-bit unsigned mantissa of Y
//       bit  37     pos_z sign
//       bits 38-48  pos_z       11-bit unsigned mantissa of Z
//       bits 49-55  ra_int      90 ms ring-alert interval
//       bit  56     ra_ts       broadcast slot (1 or 4)
//       bit  57     ra_eip      EPI flag
//       bits 58-62  ra_bc_sb    downlink sub-band for BC channel
//
// Position units: each (x, y, z) is a 12-bit two's-complement value
// scaled by a factor of 4 km per LSB (per iridium-toolkit's
// IridiumRAMessage: ra_alt = sqrt(x²+y²+z²)*4).

typedef struct {
    bool      bch_ok;          // all 3 BCH codewords decoded cleanly
    int       sv_id;           // satellite vehicle id (1..66 nominal)
    int       beam_id;         // cell id (1..48 nominal)
    int       pos_x;           // 12-bit signed satellite X in 4-km units
    int       pos_y;           // 12-bit signed Y
    int       pos_z;           // 12-bit signed Z
    int       ra_int;          // 90 ms ring-alert interval
    int       ra_ts;           // slot
    int       ra_eip;          // EPI flag
    int       ra_bc_sb;        // BC sub-band
    float     lat_deg;         // geocentric latitude derived from x,y,z
    float     lon_deg;         // longitude
    float     alt_km;          // distance from origin in km (= 4 × |v|)
} ira_decoded_t;

// Decode an Iridium Ring Alert frame's fixed header.
// `frame` must have been classified as IR_FRAME_RA.
// Returns 0 on a parse attempt (inspect ira.bch_ok for cleanliness),
// -1 on too-short input.
int ira_decode(const iridium_frame_t *frame, ira_decoded_t *out);
