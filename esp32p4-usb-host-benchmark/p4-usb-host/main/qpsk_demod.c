#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "qpsk_demod.h"

static const char *TAG = "QPSK";

static const int IR_UW_DL[] = { 0, 2, 2, 2, 2, 0, 0, 0, 2, 0, 0, 2 };
static const int IR_UW_UL[] = { 2, 2, 0, 0, 0, 2, 0, 0, 2, 0, 2, 2 };
static const int DQPSK_MAP[] = { 0, 2, 3, 1 };

#define PLL_ALPHA 0.2f
#define M_SQRT1_2f 0.70710678f

int qpsk_demod_process(const int16_t *samples_2sps, int n_samples, decoded_frame_t *out)
{
    int n_symbols = n_samples / 2;
    if (n_symbols < IR_UW_LENGTH) return 0;

    float complex *symbols = malloc(n_symbols * sizeof(float complex));
    float complex *pll_out = malloc(n_symbols * sizeof(float complex));
    int *hard_decisions = malloc(n_symbols * sizeof(int));
    
    // 1. Simple decimation to 1 sps
    for (int i = 0; i < n_symbols; i++) {
        symbols[i] = (float)samples_2sps[i * 4 + 0] + (float)samples_2sps[i * 4 + 1] * I;
    }

    // 2. PLL Phase Tracking
    float complex phi_hat = 1.0f + 0.0f * I;
    for (int i = 0; i < n_symbols; i++) {
        pll_out[i] = symbols[i] * phi_hat;
        
        float re = crealf(pll_out[i]);
        float im = cimagf(pll_out[i]);
        
        // Hard decision
        float complex x_hat;
        if (re >= 0 && im >= 0) { x_hat = M_SQRT1_2f + M_SQRT1_2f * I; hard_decisions[i] = 0; }
        else if (re < 0 && im >= 0) { x_hat = -M_SQRT1_2f + M_SQRT1_2f * I; hard_decisions[i] = 1; }
        else if (re < 0 && im < 0) { x_hat = -M_SQRT1_2f - M_SQRT1_2f * I; hard_decisions[i] = 2; }
        else { x_hat = M_SQRT1_2f - M_SQRT1_2f * I; hard_decisions[i] = 3; }

        float complex er = conjf(x_hat) * pll_out[i];
        float er_mag = cabsf(er);
        if (er_mag > 1e-10f) {
            float angle = cargf(er / er_mag);
            float scaled_angle = PLL_ALPHA * angle;
            float complex correction = cosf(scaled_angle) + sinf(scaled_angle) * I;
            phi_hat = conjf(correction) * phi_hat;
            phi_hat /= cabsf(phi_hat);
        }
    }

    // 3. UW Check
    int dl_diffs = 0, ul_diffs = 0;
    for (int i = 0; i < IR_UW_LENGTH; i++) {
        dl_diffs += (hard_decisions[i] != IR_UW_DL[i]);
        ul_diffs += (hard_decisions[i] != IR_UW_UL[i]);
    }
    
    if (dl_diffs <= 2) out->direction = DIR_DOWNLINK;
    else if (ul_diffs <= 2) out->direction = DIR_UPLINK;
    else out->direction = DIR_UNKNOWN;

    if (out->direction == DIR_UNKNOWN) {
        free(symbols); free(pll_out); free(hard_decisions);
        return 0;
    }

    // 4. DQPSK Decode
    int old_sym = 0;
    out->bits = malloc(n_symbols * 2);
    out->n_bits = n_symbols * 2;
    for (int i = 0; i < n_symbols; i++) {
        int diff = (hard_decisions[i] - old_sym + 4) % 4;
        old_sym = hard_decisions[i];
        int decoded = DQPSK_MAP[diff];
        out->bits[2 * i + 0] = (decoded >> 1) & 1;
        out->bits[2 * i + 1] = decoded & 1;
    }

    ESP_LOGI(TAG, "Successfully demodulated %s frame, %d bits", 
             (out->direction == DIR_DOWNLINK) ? "DL" : "UL", out->n_bits);

    free(symbols); free(pll_out); free(hard_decisions);
    return 1;
}
