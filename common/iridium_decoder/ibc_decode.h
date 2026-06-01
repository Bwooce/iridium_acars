#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "iridium_frame.h"

// Decoded fields from an Iridium Broadcast (IBC) frame body.
//
// Reference: iridium-toolkit/bitsparser.py IridiumBCMessage (line 1432)
// and IridiumECCMessage (line 1189). An IBC frame layout:
//   6-bit header  (BCH-encoded against poly=29 → 4-bit bc_type)
//   4 × 64-bit blocks (each de-interleaves to two BCH(31,21) codewords
//                       with poly=1207 → 21 data bits each = 42/block)
//   = 4 × 42 = 168 data bits of broadcast payload
//
// Block 0 (always present for bc_type=0):
//   bits  0- 6  sv_id          satellite vehicle id (1..66 nominally)
//   bits  7-12  beam_id        cell/beam id (1..48)
//   bit  13     unknown01
//   bit  14     slot           broadcast slot (0/1)
//   bit  15     sv_blocking    "Acq" flag
//   bits 16-31  acqu_classes   16-bit class bitmap
//   bits 32-36  acqu_subband
//   bits 37-39  acqu_channels
//   bits 40-41  unknown02
//
// Block 1 (when bc_type=0): a sub-type field at bits 0..5:
//   sub=0  uplink_pwr block
//   sub=1  iri_time (32-bit L-Band Frame Counter at bits 10..41)
//   sub=2  tmsi_expiry (32-bit at bits 10..41)
//
// Blocks 2-3 hold paging assignments (not parsed here -- ACARS path
// doesn't need them, and the assignment record format is variable).

typedef struct {
    bool header_ok;   // BCH(6,2) header CRC passed
    int  bc_type;     // 4-bit type from header (0..15)
    bool block0_ok;   // both BCH(31,21) codewords passed for block 0
    bool block1_ok;   // ... for block 1
    int  n_blocks_ok; // count of blocks 0..3 that passed BCH

    // Block 0 fields (valid iff bc_type==0 && block0_ok)
    int sv_id;       // satellite ID
    int beam_id;     // cell/beam ID
    int slot;        // 0/1
    int sv_blocking; // 0/1

    // Block 1 fields (valid iff bc_type==0 && block1_ok)
    int      block1_subtype; // 0=power, 1=time, 2=tmsi
    uint32_t iri_time;       // valid iff block1_subtype==1
    uint32_t tmsi_expiry;    // valid iff block1_subtype==2
} ibc_decoded_t;

// Decode an Iridium Broadcast frame. `frame` must have been classified
// as IR_FRAME_BC by iridium_frame_classify. Returns 0 on a parse
// attempt (whether or not all blocks decoded cleanly -- inspect the
// flags), -1 on too-short input.
int ibc_decode(const iridium_frame_t *frame, ibc_decoded_t *out);

// Convert the 32-bit iri_time (L-Band Frame Counter) to a Unix-epoch
// seconds value. Mirrors iridium-toolkit's fmt_iritime: the LBFC ticks
// at the Iridium L-band frame rate, anchored to 2014-05-11T14:23:55 UTC
// per the toolkit's epoch reference. Returns 0 if iri_time looks
// unset.
uint64_t ibc_iri_time_to_unix(uint32_t iri_time);
