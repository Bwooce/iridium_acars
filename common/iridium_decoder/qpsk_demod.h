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
    int n_bits;
    ir_direction_t direction;
    float snr_db;
    uint32_t timestamp;
} decoded_frame_t;

// Process a burst at 2 samples/symbol and produce decoded bits +
// direction in `out`. `expected_direction` is the UW direction hint
// from the upstream uw_correlator: DIR_DOWNLINK or DIR_UPLINK lets
// the PLL run data-aided over the 12-symbol UW (faster convergence
// → fewer error bits in the early data symbols). DIR_UNKNOWN falls
// back to decision-directed PLL throughout (legacy behaviour).
//
// Returns 1 on UW match + bit production, 0 otherwise.
int qpsk_demod_process(const int16_t *samples_2sps, int n_samples,
                        ir_direction_t expected_direction,
                        decoded_frame_t *out);

#endif
