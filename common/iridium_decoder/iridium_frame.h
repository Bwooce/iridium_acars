#ifndef IRIDIUM_FRAME_H
#define IRIDIUM_FRAME_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Top-level frame classification. Mirrors iridium-toolkit/bitsparser.py's
// dispatch path before the LCW sub-classification. Sub-types of LCW
// (IDA, VOC, IIU, ...) are deferred to Phase B (ida_lcw.c).
typedef enum {
    IR_FRAME_UNKNOWN, // Could not classify — invalid header / unknown type
    IR_FRAME_MS,      // MS  — Messaging frame
                      //       (carries pager-style messages on simplex channels)
    IR_FRAME_TL,      // TL  — Time/Location broadcast
                      //       (carries sat ephemeris + UTC reference)
    IR_FRAME_BC,      // BC  — Broadcast (IBC) — sat/cell config beacon
    IR_FRAME_LW,      // LW  — Link Control Word frame
                      //       superclass of IDA / VOC / IIU / DA / etc.
                      //       sub-discrimination happens in Phase B
    IR_FRAME_RA,      // RA  — Ring Alert (paging broadcast on simplex)
                      //       carries sat sv_id + 3D position + TMSI pages
} ir_frame_type_t;

typedef enum {
    IR_FRM_DIR_DOWNLINK = 0,
    IR_FRM_DIR_UPLINK   = 1,
} ir_frame_direction_t;

// Sub-classification of IR_FRAME_LW (Link Control Word) frames. Mirrors
// the `ft` (frame type) field in iridium-toolkit's IridiumLCWMessage —
// the 3-bit message in the first BCH-protected lcw1 word.
typedef enum {
    IR_LW_VO   = 0,  // Voice — mission data
    IR_LW_IP   = 1,  // IP via PPP — mission data
    IR_LW_DA   = 2,  // DAta (SBD)  ← carries the IDA / SBD / ACARS payload
    IR_LW_U3   = 3,  // Mission control inband signalling
    IR_LW_U4   = 4,  // Reserved / unknown
    IR_LW_U5   = 5,  // Reserved / unknown
    IR_LW_U6   = 6,  // "PT=,"
    IR_LW_SY   = 7,  // Synchronisation
    IR_LW_NONE = -1, // Frame is not LW (or LW classification deferred)
} ir_lw_subtype_t;

typedef struct {
    ir_frame_type_t      type;
    ir_frame_direction_t direction;
    ir_lw_subtype_t      lw_subtype; // valid iff type == IR_FRAME_LW

    // Where in `bits` the post-UW payload starts. For standard Iridium
    // bursts this is 24 (12 UW symbols × 2 bits/symbol).
    size_t payload_off;

    // Pointer into the caller's bit array (NOT owned). Caller must keep
    // the bits alive while iridium_frame_t is in use.
    const uint8_t *bits;
    size_t         n_bits;
} iridium_frame_t;

// Classify a demodulated bit stream emitted by qpsk_demod_process.
// `bits` is a 0/1-per-byte array starting with the 12-symbol UW (24 bits).
// `direction` is what qpsk_demod determined (DL or UL UW match).
//
// Returns 0 on success and fills *out — out->type may still be
// IR_FRAME_UNKNOWN if no header pattern matched.
// Returns -1 on argument error (NULL pointer / n_bits below the minimum
// 24-bit UW length).
int iridium_frame_classify(const uint8_t *bits, size_t n_bits,
                           ir_frame_direction_t direction,
                           iridium_frame_t     *out);

// Convenience: human-readable type name. Returns a 2- to 3-char ASCII
// string ("MS", "TL", "BC", "LW", "??"). Always non-NULL.
const char *iridium_frame_type_name(ir_frame_type_t type);

// Human-readable LW subtype ("VO", "IP", "DA", "SY", "U3"...).
// Returns "??" when subtype is IR_LW_NONE.
const char *iridium_lw_subtype_name(ir_lw_subtype_t subtype);

#ifdef __cplusplus
}
#endif

#endif // IRIDIUM_FRAME_H
