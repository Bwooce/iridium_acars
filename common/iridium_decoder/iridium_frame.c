// Iridium frame top-level classifier — port of the dispatch logic from
// iridium-toolkit/bitsparser.py (Message.upgrade -> IridiumMessage).
//
// The bit-stream format we receive is the output of qpsk_demod_process:
// a 0/1-per-byte array where bits[0..23] are the 12-symbol UW and bits[24..]
// is the descrambled payload. Caller has already verified the UW
// matches IR_UW_DL or IR_UW_UL (qpsk_demod returns 1 only when it does)
// and reports `direction`.
//
// We classify by matching fixed header bit patterns at the start of the
// payload. Sub-classification of LW frames (LCW → IDA/VOC/IIU/...) is
// deferred to Phase B.

#include "iridium_frame.h"
#include <string.h>

// 32-bit "messaging" header (BPSK 0x9669). Frames whose post-UW prefix
// matches this are MS-type.
static const uint8_t HEADER_MESSAGING[32] = {
    0,0,1,1, 0,0,1,1, 1,1,1,1, 0,0,1,1,
    0,0,1,1, 0,0,1,1, 1,1,1,1, 0,0,1,1,
};

// 96-bit "time/location" header: bits "11" + 94 zeros.
#define HEADER_TIME_LOCATION_LEN 96

// Minimum bit count for any classification. UW (24) + smallest header
// pattern we look at (32 = MS).
#define MIN_PAYLOAD_BITS 32
#define UW_BITS          24

static int bits_equal(const uint8_t *a, const uint8_t *b, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        if ((a[i] & 1) != (b[i] & 1)) return 0;
    }
    return 1;
}

// Returns 1 iff bits[0]=1, bits[1]=1, bits[2..95]=0.
static int looks_like_time_location(const uint8_t *p, size_t n)
{
    if (n < HEADER_TIME_LOCATION_LEN) return 0;
    if ((p[0] & 1) != 1 || (p[1] & 1) != 1) return 0;
    for (size_t i = 2; i < HEADER_TIME_LOCATION_LEN; i++) {
        if (p[i] & 1) return 0;
    }
    return 1;
}

int iridium_frame_classify(const uint8_t *bits, size_t n_bits,
                           ir_frame_direction_t direction,
                           iridium_frame_t *out)
{
    if (!bits || !out) return -1;
    if (n_bits < UW_BITS) return -1;

    out->type        = IR_FRAME_UNKNOWN;
    out->direction   = direction;
    out->bits        = bits;
    out->n_bits      = n_bits;
    out->payload_off = UW_BITS;

    const uint8_t *p = bits + UW_BITS;
    size_t avail = n_bits - UW_BITS;

    if (avail < MIN_PAYLOAD_BITS) {
        // Too short to classify — leave UNKNOWN, return success.
        return 0;
    }

    // Order of dispatch matches iridium-toolkit's bitsparser.py:
    //   1. MS  — header_messaging at offset 0 (32 bits)
    //   2. TL  — header_time_location at offset 0 (96 bits)
    //   3. BC  — 6-bit BCH header + 64-bit double-BCH block (Phase B)
    //   4. LW  — interleaved 29+41+465 LCW (Phase B)
    //
    // Steps 3 and 4 require BCH polynomial division which we haven't
    // ported yet. For now a frame that doesn't match MS or TL is left
    // as UNKNOWN; Phase B will add the BCH-based BC/LW discrimination.

    if (bits_equal(p, HEADER_MESSAGING, sizeof(HEADER_MESSAGING))) {
        out->type = IR_FRAME_MS;
        return 0;
    }

    if (looks_like_time_location(p, avail)) {
        out->type = IR_FRAME_TL;
        return 0;
    }

    // No top-level header match. Could be BC or LW (or noise / corrupt).
    // Leaving UNKNOWN — Phase B will refine.
    return 0;
}

const char *iridium_frame_type_name(ir_frame_type_t type)
{
    switch (type) {
    case IR_FRAME_MS:      return "MS";
    case IR_FRAME_TL:      return "TL";
    case IR_FRAME_BC:      return "BC";
    case IR_FRAME_LW:      return "LW";
    case IR_FRAME_UNKNOWN: return "??";
    }
    return "??";
}
