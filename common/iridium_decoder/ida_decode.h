#ifndef IDA_DECODE_H
#define IDA_DECODE_H

// Iridium DA (Data) frame post-LCW decoder. Takes a frame classified by
// iridium_frame_classify as IR_FRAME_LW with subtype IR_LW_DA and runs
// the data section through the second-level BCH chain (de-interleave +
// BCH(31,21) per block) to produce the descrambled byte stream that
// the SBD reassembler will eat.
//
// Mirrors iridium-toolkit/bitsparser.py IridiumDAMessage post-LCW
// processing (lines ~1339-1430). The 312-bit data section after the
// 46-bit LCW splits into:
//   - 2 × 124-bit chunks (each de-interleaved into 4 BCH(31,21) blocks)
//   - 1 × 64-bit tail   (de-interleaved into 2 BCH(31,21) blocks)
// = 10 BCH blocks × 21 message bits = 210 bits = ~26 bytes.

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "iridium_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

// Maximum decoded message size: 10 BCH(31,20) blocks under ACCH
// poly=3545 → 20 message bits each → 200 bits total. Real-world IDA
// frames carry ≤23 bytes of SBD payload.
#define IDA_DECODE_MAX_BITS 200
#define IDA_DECODE_MAX_BYTES 25

typedef struct {
    bool     ok;                       // true iff all BCH blocks decoded
    int      n_blocks;                 // BCH blocks attempted (always 10 for full IDA)
    int      blocks_ok;                // BCH blocks that decoded successfully
    int      total_errors;             // total bit errors corrected across blocks
    uint16_t n_bits;                   // count of valid bits (= blocks_ok * 20);
                                       // NOT a contiguous prefix length when
                                       // blocks_ok < n_blocks — see `bits[]`.
    uint8_t bits[IDA_DECODE_MAX_BITS]; // 0/1-per-byte descrambled stream.
                                       // Positionally addressed by BCH block:
                                       // block i's 20 message bits live at
                                       // bits[i*20 .. i*20+20). A block that
                                       // failed BCH repair is zero-filled at
                                       // its own position (never shifted or
                                       // backfilled with another block's
                                       // bits). Header/payload/CRC fields
                                       // below are only parsed — and bits[]
                                       // is only fully valid — when `ok` is
                                       // true (blocks_ok == n_blocks); a
                                       // partial decode's zero-filled gaps
                                       // are NOT meaningful data.

    // Header fields parsed from bits[0..19] (per
    // iridium-toolkit/bitsparser.py:IridiumDAMessage):
    //   bits[ 0.. 2]  flags1
    //   bits[ 3]      cont (continuation flag — bitsparser.py:1388)
    //   bits[ 4]      spacer (unlabeled/unused — bitsparser.py:1389)
    //   bits[ 5.. 7]  da_ctr (3-bit message counter)
    //   bits[ 8..10]  flags2
    //   bits[11..15]  da_len (5-bit payload byte count, 0..24)
    //   bits[16]      flags3
    //   bits[17..19]  zero1 (sanity-check, must be 0)
    uint8_t da_flags1; // 3 bits (bits 0..2; bit 4 is an unused spacer)
    uint8_t da_cont;   // 1 bit (bit 3)
    uint8_t da_ctr;    // 3 bits
    uint8_t da_flags2; // 3 bits
    uint8_t da_len;    // 5 bits — number of valid payload bytes
    uint8_t da_flags3; // 1 bit
    bool    header_ok; // zero1 == 0 (frame structure valid)

    // Payload bytes from bits[20..(20+da_len*8)], packed MSB-first.
    uint8_t payload[24];
    uint8_t payload_len; // = da_len, capped at 24

    // CRC field at bits[180..195] (= 9*20..9*20+16).
    uint16_t da_crc_reported;
    // CRC validation (D12). crc_computed is the CRC-16/CCITT-FALSE
    // computed over (header + 12 zeros + bits[20..195]), matching
    // iridium-toolkit's bitsparser.py:IridiumDAMessage. Because the
    // computation includes the received CRC, a valid frame yields
    // crc_computed == 0 (CCITT-FALSE residual property).
    uint16_t da_crc_computed;
    bool     crc_ok;
} ida_decoded_t;

// Run the IDA-specific decode chain on `frame`. Caller must have already
// classified the frame and confirmed type == IR_FRAME_LW &&
// lw_subtype == IR_LW_DA before calling — this function asserts the
// classification.
//
// Returns:
//    0  on success (out->ok set; out->blocks_ok and out->total_errors
//                   describe how many BCH blocks survived ECC repair)
//   -1  invalid input (NULL pointers, frame type wrong, n_bits too short)
int ida_decode(const iridium_frame_t *frame, ida_decoded_t *out);

#ifdef __cplusplus
}
#endif

#endif // IDA_DECODE_H
