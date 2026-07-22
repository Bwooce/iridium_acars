// rs_vdl2 — Reed-Solomon RS(255,249) codec for VHF VDL Mode 2 (ARINC 631 /
// ICAO VDL2 SARPs FEC). Pure C, integer-only, no ESP dependencies; buildable
// on host and target alike.
//
// Field / code parameters (confirmed against dumpvdl2's rs init,
// src/dumpvdl2.c: init_rs_char(8, 0x187, 120, 1, RS_N - RS_K, 0) with
// RS_N = 255, RS_K = 249):
//   GF(2^8), primitive polynomial 0x187 = x^8 + x^7 + x^2 + x + 1
//   nroots = 6 parity symbols, t = 3 correctable symbol errors
//   first consecutive root fcr = 120, primitive element prim = 1
//   generator g(x) = prod_{i=0..5} (x - alpha^(120+i)), no pad.
//
// Codeword layout: block[0] is the highest-order coefficient (x^254),
// block[254] the lowest (x^0). Data occupies block[0..248], parity
// block[249..254] — the natural over-the-air byte order.

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RS_VDL2_N      255 // codeword length (symbols/bytes)
#define RS_VDL2_K      249 // data length (symbols/bytes)
#define RS_VDL2_NROOTS 6   // parity symbols
#define RS_VDL2_T      3   // max correctable symbol errors

// Compute the 6 parity bytes over block[0..248] into block[249..254].
// Provided for host-test round trips (and any future TX path); the
// receive path only needs rs_vdl2_decode.
void rs_vdl2_encode(uint8_t block[RS_VDL2_N]);

// Correct up to RS_VDL2_T symbol errors in place. Returns 0 on success
// (block is now a valid codeword; *n_corrected = number of symbols fixed,
// 0 if it was already clean) or -1 if uncorrectable (block contents are
// then unspecified — treat the frame as lost). n_corrected may be NULL.
//
// Bounded-distance caveat: >t errors are usually *detected* (-1), but as
// with any RS decoder a small fraction of >t patterns alias into a
// different valid codeword; downstream CRC remains the final arbiter.
//
// This is the 0-erasure fast path; equivalent to rs_vdl2_decode_erasures
// with n_erasures == 0.
int rs_vdl2_decode(uint8_t block[RS_VDL2_N], int *n_corrected);

// Errata (errors + erasures) decode. With the 6 parity symbols and f
// erasures at known positions, corrects e errors where 2*e + f <= 6.
// erasure_pos lists the f known-bad block indices (0..254), typically the
// untransmitted parity positions of a shortened block; each such position
// should be zero-filled by the caller before the call. Returns 0 on
// success (*n_corrected = symbols whose value actually changed, counting
// both corrected errors and recovered erasure symbols) or -1 if
// uncorrectable / the constraint 2*e + f <= 6 is exceeded. n_corrected may
// be NULL. Passing n_erasures == 0 is exactly rs_vdl2_decode.
int rs_vdl2_decode_erasures(uint8_t block[RS_VDL2_N],
                            const uint8_t *erasure_pos, int n_erasures,
                            int *n_corrected);

// Convenience wrapper for VDL2 shortened last blocks. Given the number of
// DATA octets present (data_len), derives dumpvdl2's FEC scheme and erasure
// set and decodes:
//   data_len < 3  -> 0 parity octets: the block is uncoded, so this is a
//                    pass-through (returns 0, *n_corrected = 0; the frame
//                    CRC is the only integrity check).
//   data_len < 31 -> 2 parity octets (4 erasures, corrects up to 1 error)
//   data_len < 68 -> 4 parity octets (2 erasures, corrects up to 2 errors)
//   else          -> 6 parity octets (0 erasures, corrects up to 3 errors)
// The caller must have laid the block out per the scheme: data octets at
// [0 .. data_len-1], zero-fill through [248], the transmitted parity octets
// at [249 .. 249+fec-1], and zeros at the remaining (erased) parity
// positions [249+fec .. 254]. Returns as rs_vdl2_decode_erasures.
int rs_vdl2_decode_shortened(uint8_t block[RS_VDL2_N], int data_len,
                             int *n_corrected);

#ifdef __cplusplus
}
#endif
