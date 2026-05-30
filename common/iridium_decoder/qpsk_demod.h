#ifndef QPSK_DEMOD_H
#define QPSK_DEMOD_H

#include <stdint.h>
#include <stdbool.h>
#include <complex.h>

#define IR_UW_LENGTH 12

typedef enum {
    DIR_DOWNLINK,
    DIR_UPLINK,
    DIR_UNKNOWN
} ir_direction_t;

typedef struct {
    uint8_t *bits;
    // Per-bit soft metric (sign carries hard decision: ≥0 → bit=0, <0 → bit=1;
    // magnitude = reliability). Same length as bits[]. NULL if qpsk_demod
    // didn't run (or older callers). Used by Chase-2 BCH (#112). Caller
    // must free() alongside bits.
    int16_t *soft_bits;
    int n_bits;
    ir_direction_t direction;
    float snr_db;
    uint32_t timestamp;
} decoded_frame_t;

int qpsk_demod_process(const int16_t *samples_2sps, int n_samples, decoded_frame_t *out);

#endif
