#ifndef IDA_ENCODE_H
#define IDA_ENCODE_H

// Synthetic LW.DA downlink frame encoder — the mathematical inverse of
// iridium_frame_classify() + ida_decode()'s LW.DA path.
//
// Why this exists: tests/fixtures/fixture_acars_frames.h carries REAL,
// captured, verified-CRC-OK ACARS payload bytes (post-BCH, post-LCW),
// but genuine raw pre-BCH over-the-air bits for those specific bursts
// were never retained (see that fixture's header comment) — the
// capture that produced them has since rotated past this time window.
// The device FRAME_DECODER smoke corpus, however, only has one
// injection point (frame_decoder_push()) and it consumes raw
// post-qpsk_demod bits that go through the FULL production
// classify -> BCH decode chain, not pre-decoded fields.
//
// This module re-encodes a known-good da_cont/da_ctr/payload triple
// into the exact wire bitstream iridium_frame_classify()/ida_decode()
// expect: UW(24) + LCW(46, ft=2/DA) + DATA(312, BCH(31,20) poly=3545
// per block + CRC-16/CCITT-FALSE), so the resulting corpus entry
// exercises the SAME production classify+BCH code path as any other
// FRAME_DECODER corpus entry — it is a content-accurate re-encoding,
// not a decode bypass. See ida_decode.c for the forward transform this
// inverts (chunk_124_to_codewords / tail_64_to_codewords / de_interleave
// / pair_swap / the header+CRC layout).

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Total bit length of an encoded LW.DA downlink frame: UW(24) +
// LCW(46) + DATA(312). Matches every other FRAME_DECODER corpus
// entry's n_bits (see tests/fixtures/fixture_albq_frames_corpus.h).
#define IDA_ENCODE_FRAME_BITS 382

// Build a content-accurate over-the-air LW.DA DOWNLINK frame bitstream
// that decodes back, via the real production
// iridium_frame_classify()/ida_decode(), to:
//   type=IR_FRAME_LW, lw_subtype=IR_LW_DA, ok=true, header_ok=true,
//   crc_ok=true (payload_len>0), blocks_ok==n_blocks==10,
//   da_cont/da_ctr/payload/payload_len exactly as given.
//
// out_bits must hold IDA_ENCODE_FRAME_BITS (382) bytes, 0/1-per-byte,
// in the same orientation qpsk_demod_process emits (the orientation
// every frame_decoder_push() caller uses).
//
// payload_len must be 1..24 (da_len is a 5-bit header field, and
// ida_decode()'s payload[] buffer caps at 24 bytes); da_ctr must be
// 0..7 (3-bit header field). Returns 0 on success, -1 on invalid args.
int ida_encode_da_frame(uint8_t da_cont, uint8_t da_ctr,
                        const uint8_t *payload, uint8_t payload_len,
                        uint8_t out_bits[IDA_ENCODE_FRAME_BITS]);

#ifdef __cplusplus
}
#endif

#endif // IDA_ENCODE_H
