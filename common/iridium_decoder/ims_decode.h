#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "iridium_frame.h"

// Decoded fields from an Iridium Messaging Service (IMS) frame header.
//
// Reference: iridium-toolkit/bitsparser.py IridiumMSMessage line 1667.
// MS frame layout after the 24-bit UW + 32-bit HEADER_MESSAGING:
//   - N × 64-bit blocks. Each block de-interleaves into TWO BCH(31,21)
//     codewords using messaging_bch_poly = 1897 → 42 data bits per
//     block (21 from each codeword).
//   - First 21 data bits = header:
//       bit  0     ms_type     (1 = Acq group, 0 = regular)
//       bits 1- 4  zero1       (must be 0)
//       bits 5- 8  block       block number in the super frame (0..15)
//       bits 9-14  frame       current frame / cell number (0..63)
//       bits 15-18 bch_blocks  total 42-bit blocks in this MS message
//       For ms_type=1: bit 19 = unknown1, bit 20 = secondary
//       For ms_type=0: bits 19-20 = group (0..3)
//
// Body parsing (alphanumeric messages, paging payloads, BCD) is NOT
// implemented -- that's iridium-toolkit IridiumMSMessageBody and its
// alphanumeric/BCD subclasses, which require ASCII / BCD decoding
// tables and aren't on the ACARS critical path. The header alone
// answers "what does our stream contain when MS arrives?".

typedef struct {
    bool      bch_ok;            // first block BCH decoded cleanly
    int       ms_type;           // 0 = normal, 1 = Acq group
    int       block;             // 0..15
    int       frame;             // 0..63
    int       bch_blocks;        // total blocks (length field)
    int       group;             // 0..3 (only if ms_type=0); 'A' marker if ms_type=1
} ims_decoded_t;

// Decode the first block of an Iridium Messaging frame's header.
// `frame` must have been classified as IR_FRAME_MS by
// iridium_frame_classify. Returns 0 on a parse attempt (inspect
// out->bch_ok for cleanliness), -1 on too-short input or wrong type.
int ims_decode(const iridium_frame_t *frame, ims_decoded_t *out);
