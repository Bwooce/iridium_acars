// See firmr_s16.h. Port of dsps_firmr_s16_ansi.c — runs the same
// math on host so the polyphase resampler step is identical between
// the host pipeline test and the P4 worker.

#include "firmr_s16.h"

#include <stdint.h>

void firmr_s16_init(firmr_s16_t *fir,
                    int16_t *coeffs, int16_t *delay,
                    int16_t delay_size, int16_t interp, int16_t decim,
                    int16_t start_pos, int16_t shift)
{
    fir->coeffs       = coeffs;
    fir->delay        = delay;
    fir->delay_size   = delay_size;
    fir->interp       = interp;
    fir->decim        = decim;
    fir->shift        = shift;
    fir->rounding_val = (int16_t)0x7fff;
    fir->pos          = 0;
    fir->start_pos    = start_pos;
    for (int i = 0; i < delay_size; i++) {
        fir->delay[i] = 0;
    }
}

int32_t firmr_s16_process(firmr_s16_t *fir,
                          const int16_t *input, int16_t *output,
                          int32_t input_len)
{
    int32_t result = 0;
    long long rounding = (long long)fir->rounding_val;
    const int32_t final_shift = fir->shift - 15;

    // Pre-scale the rounding term to match the final shift direction
    // (mirrors esp-dsp's logic exactly).
    if (fir->shift >= 0) {
        rounding = (rounding >> fir->shift) & 0xFFFFFFFFFFLL;
    } else {
        rounding = (rounding << (-fir->shift)) & 0xFFFFFFFFFFLL;
    }

    int32_t m = fir->start_pos;

    for (int32_t i = 0; i < input_len; i++) {
        fir->delay[fir->pos] = input[i];

        for (m = fir->start_pos; m < fir->interp; m += fir->decim) {
            long long acc = rounding;
            int coeff_pos = 0;
            // Walk the delay line as a circular buffer starting at
            // the newest position. For each tap we use the
            // transposed-polyphase coefficient index
            // coeffs[tap_pos * interp + phase].
            for (int n = fir->pos; n < fir->delay_size; n++) {
                acc += (int32_t)fir->delay[n]
                     * (int32_t)fir->coeffs[coeff_pos++ * fir->interp + m];
            }
            for (int n = 0; n < fir->pos; n++) {
                acc += (int32_t)fir->delay[n]
                     * (int32_t)fir->coeffs[coeff_pos++ * fir->interp + m];
            }

            int16_t out;
            if (final_shift > 0) {
                out = (int16_t)(acc << final_shift);
            } else {
                out = (int16_t)(acc >> (-final_shift));
            }
            output[result++] = out;
        }
        fir->start_pos = (int16_t)(m - fir->interp);

        fir->pos--;
        if (fir->pos < 0) {
            fir->pos = fir->delay_size - 1;
        }
    }

    return result;
}
