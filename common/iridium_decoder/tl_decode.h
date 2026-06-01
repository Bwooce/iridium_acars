// TL (Time/Location, "ITL") frame body parser.
//
// Partial port of iridium-toolkit's IridiumSTLMessage (bitsparser.py:767).
// We extract the ITL version (V0 / V1 / V2) and the satellite plane number,
// which together identify which Iridium plane sent this beacon. The full
// PRS-list message decode (sat XYZ position, UTC time) is deferred — it
// requires ~30+ KB of PRS lookup tables and is rarely the operationally
// relevant bit.
//
// Input: a classified IR_FRAME_TL frame. Output: version + plane (or -1
// for "unknown"). Caller logs / surfaces the fields.

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "iridium_frame.h"

typedef struct {
    int8_t  version;   // 0..2; -1 if PRS header doesn't match any known version
    int8_t  plane;     // 0..5 within the active version's MAP_PLANE; -1 if no match
    uint8_t header_ok; // 1 iff the 96-bit "11+94 zeros" header matched (always 1
                       // here since iridium_frame.c already gated on it; included
                       // for symmetry with ibc_decode_t)
} tl_decoded_t;

// Decode a TL frame's version + plane. Sets out->version = -1 / out->plane
// = -1 on no-match. Safe to call with an unclassified frame (returns all
// -1 / 0 if frame->type != IR_FRAME_TL).
void tl_decode(const iridium_frame_t *frame, tl_decoded_t *out);
