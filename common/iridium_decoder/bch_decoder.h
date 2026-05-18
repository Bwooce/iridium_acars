#ifndef BCH_DECODER_H
#define BCH_DECODER_H

#include <stdint.h>
#include <stdbool.h>

#define BCH_POLY_RA 1207   // BCH(31,21) t=2
#define BCH_RA_DATA 21

void bch_decoder_init();
int bch_decode_block(const uint8_t *block31, uint8_t *out_data);

// Chase-2 soft-decision BCH decoder. soft_in is 31 signed values
// where sign carries the hard bit (≥0 → 0, <0 → 1) and magnitude is
// reliability (larger = more confident). K is the number of least-
// reliable bits to try-flip — typically 3 or 4, giving 2^K trial
// decodes. Returns the number of bit errors corrected (≥0) or -1
// if no trial converged. Used out_data identically to bch_decode_block.
//
// Gains ~1-2 dB SNR margin on the BCH stage vs hard-decision; matches
// gr-iridium's soft-decision experimental path (not in their main
// pipeline but documented in burst_downmix discussion).
int bch_decode_block_soft(const int16_t *soft_in31, uint8_t *out_data, int K);

void iridium_deinterleave(const uint8_t *in, uint8_t *out1, uint8_t *out2);

#endif
