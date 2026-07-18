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

// Data-section geometry shared with the Chase-2 soft fallback
// (ida_chase.c): 10 BCH(31,20) codewords of 31 bits each, built from
// the 312-bit post-LCW data section.
#define IDA_DECODE_N_CW 10
#define IDA_DECODE_CW_BITS 31
#define IDA_DECODE_N_CW_BITS (IDA_DECODE_N_CW * IDA_DECODE_CW_BITS)
#define IDA_DECODE_DATA_BITS 312
// Frame-bit offset of the data section: UW(24) + LCW(46).
#define IDA_DECODE_DATA_OFF 70

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
    //   bits[ 4]      flag1b — NOT a spacer (see da_flag1b below)
    //   bits[ 5.. 7]  da_ctr (3-bit message counter)
    //   bits[ 8..10]  flags2
    //   bits[11..15]  da_len (5-bit payload byte count, 0..24)
    //   bits[16]      flags3
    //   bits[17..19]  zero1 (sanity-check, must be 0)
    uint8_t da_flags1; // 3 bits (bits 0..2)
    uint8_t da_cont;   // 1 bit (bit 3) — continuation flag; governs reassembly
    uint8_t da_flag1b; // 1 bit (bit 4) — NOT a spacer. Set on ~13% of frames, almost
                       // exclusively single-burst 0x7605 SBD Ring-Alert/paging messages.
                       // 0x7605 is GSM-04.08-derived L3 (0x76 = RRM protocol discriminator,
                       // 0x05 = paging); payload = 76 05 00 4b | TMSI(4B) | 50 | trailer(2B),
                       // corroborated across 755 corpus frames (TMSI field ~55% unique).
                       // bit 4 is an L2 fast-path/control flag ("immediate signaling — skip
                       // the SBD reassembler"). Safe to ignore for reassembly (keys only on
                       // da_cont); captured for a possible future Ring-Alert / TMSI tracker.
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

    // Chase-2 soft-decision fallback provenance (task #16, ida_chase.c).
    // Both stay zero on the pure hard-decision path; set only when
    // ida_chase_decode() recovered this frame. A chase-recovered frame is
    // still CRC-16-arbitrated (crc_ok true by construction on recovery),
    // but carries this marker so counters/logs can A/B the two paths.
    bool     chase_used;   // true iff this decode came from the chase fallback
    uint16_t chase_checks; // CRC-16 arbiter checks spent (<= configured cap)
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

// --- shared internals (used by ida_decode itself and ida_chase.c) ----

// Build the 10 x 31-bit BCH codewords (0/1-per-byte, concatenated) from
// the RAW 312-bit post-LCW data section (`frame->bits + 70`, qpsk_demod
// orientation — pair-swap is applied internally). This is the exact
// received-codeword transform ida_decode() runs before BCH repair;
// exposed so the Chase-2 fallback enumerates flips of the SAME received
// words the hard path saw.
void ida_decode_build_codewords(const uint8_t *data_section,
                                uint8_t out_codewords[IDA_DECODE_N_CW_BITS]);

// Parse header/payload/CRC fields from out->bits[0..n_bits). Factored
// out of ida_decode() (behaviour identical) so the Chase-2 fallback can
// re-run the same field parse + CRC arbitration after it rewrites
// out->bits with a recovered message. Requires out->n_bits already set;
// no-op (fields left zero) when n_bits < 196.
void ida_decode_parse_fields(ida_decoded_t *out);

// (crc16_ccitt_false moved to the shared crc16.h/crc16.c — single tree-wide
// table-based implementation, still pinned by test_crc16_ccitt.)

#ifdef __cplusplus
}
#endif

#endif // IDA_DECODE_H
