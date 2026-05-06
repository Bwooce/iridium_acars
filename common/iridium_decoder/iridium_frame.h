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
    IR_FRAME_UNKNOWN,    // Could not classify — invalid header / unknown type
    IR_FRAME_MS,         // MS  — Messaging frame
                         //       (carries pager-style messages on simplex channels)
    IR_FRAME_TL,         // TL  — Time/Location broadcast
                         //       (carries sat ephemeris + UTC reference)
    IR_FRAME_BC,         // BC  — Broadcast (IBC) — sat/cell config beacon
    IR_FRAME_LW,         // LW  — Link Control Word frame
                         //       superclass of IDA / VOC / IIU / DA / etc.
                         //       sub-discrimination happens in Phase B
} ir_frame_type_t;

typedef enum {
    IR_FRM_DIR_DOWNLINK = 0,
    IR_FRM_DIR_UPLINK   = 1,
} ir_frame_direction_t;

typedef struct {
    ir_frame_type_t       type;
    ir_frame_direction_t  direction;

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
                           iridium_frame_t *out);

// Convenience: human-readable type name. Returns a 2- to 3-char ASCII
// string ("MS", "TL", "BC", "LW", "??"). Always non-NULL.
const char *iridium_frame_type_name(ir_frame_type_t type);

#ifdef __cplusplus
}
#endif

#endif // IRIDIUM_FRAME_H
