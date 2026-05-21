// IMS (Iridium Messaging Service) header decoder. See ims_decode.h.
//
// Direct port of the first-block parsing in iridium-toolkit/
// bitsparser.py:IridiumMSMessage (line 1667). Only the 21-bit header
// is extracted; body alphanumeric / paging parsing is out of scope.

#include "ims_decode.h"
#include "iridium_bch.h"
#include <string.h>

#define UW_BITS         24
#define MS_HDR_BITS     32   // HEADER_MESSAGING (32-bit literal)
#define MS_BLOCK_BITS   64   // pre-BCH block (= 2 × 32-bit codewords)
#define MS_BLOCK_POLY   1897u  // messaging_bch_poly

// Same de-interleave-pair as IBC. The 64-bit input arranges into 32
// symbols (sym[i] = in[2i+1]||in[2i]), and odd/even codewords read
// alternating symbols high-to-low.
static void de_interleave_pair(const uint8_t *in, uint8_t *odd_out, uint8_t *even_out)
{
    int n_sym = MS_BLOCK_BITS / 2; // 32
    int odd_idx = 0, even_idx = 0;
    for (int s = n_sym - 1; s >= 0; s -= 2) {
        odd_out[odd_idx++] = in[2 * s + 1] & 1;
        odd_out[odd_idx++] = in[2 * s + 0] & 1;
    }
    for (int s = n_sym - 2; s >= 0; s -= 2) {
        even_out[even_idx++] = in[2 * s + 1] & 1;
        even_out[even_idx++] = in[2 * s + 0] & 1;
    }
}

static uint32_t pick_bits(const uint8_t *bits, int start, int n)
{
    uint32_t v = 0;
    for (int i = 0; i < n; i++) {
        v = (v << 1) | (bits[start + i] & 1u);
    }
    return v;
}

int ims_decode(const iridium_frame_t *frame, ims_decoded_t *out)
{
    if (!frame || !out) return -1;
    memset(out, 0, sizeof(*out));
    if (frame->type != IR_FRAME_MS) return -1;
    if (frame->n_bits < UW_BITS + MS_HDR_BITS + MS_BLOCK_BITS) return -1;

    // Skip UW + HEADER_MESSAGING. iridium-toolkit also applies the
    // adjacent-pair swap during ingest; the classifier already verified
    // the header against the swapped form. The data body uses the
    // SAME swap, so apply it here too before de-interleaving.
    const uint8_t *raw = frame->bits + UW_BITS + MS_HDR_BITS;
    uint8_t swapped[MS_BLOCK_BITS];
    for (int i = 0; i + 1 < MS_BLOCK_BITS; i += 2) {
        swapped[i + 0] = raw[i + 1] & 1;
        swapped[i + 1] = raw[i + 0] & 1;
    }

    // De-interleave the first 64-bit block and decode each 31-bit
    // BCH(31, 21) codeword. We only need the FIRST codeword's 21 data
    // bits for the header (the even one carries bch_blocks/group too,
    // but iridium-toolkit's IridiumMSMessage reads them all from the
    // first 21-bit slice anyway).
    uint8_t odd[32], even[32];
    de_interleave_pair(swapped, odd, even);
    int e_odd = iridium_bch_repair2(MS_BLOCK_POLY, odd, 31);
    if (e_odd < 0) {
        out->bch_ok = false;
        return 0;
    }
    out->bch_ok = true;

    // Header layout (21 data bits from the first codeword):
    out->ms_type    = (int)pick_bits(odd,  0, 1);
    int zero1       = (int)pick_bits(odd,  1, 4);
    (void)zero1;     // for now we don't reject on zero1 != 0
    out->block      = (int)pick_bits(odd,  5, 4);
    out->frame      = (int)pick_bits(odd,  9, 6);
    out->bch_blocks = (int)pick_bits(odd, 15, 4);
    if (out->ms_type == 1) {
        out->group  = -1;    // 'A' marker for Acq group; logged as -1
    } else {
        out->group  = (int)pick_bits(odd, 19, 2);
    }
    return 0;
}
