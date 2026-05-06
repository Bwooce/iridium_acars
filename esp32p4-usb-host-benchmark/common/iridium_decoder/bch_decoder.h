#ifndef BCH_DECODER_H
#define BCH_DECODER_H

#include <stdint.h>
#include <stdbool.h>

#define BCH_POLY_RA 1207   // BCH(31,21) t=2
#define BCH_RA_DATA 21

void bch_decoder_init();
int bch_decode_block(const uint8_t *block31, uint8_t *out_data);
void iridium_deinterleave(const uint8_t *in, uint8_t *out1, uint8_t *out2);

#endif
