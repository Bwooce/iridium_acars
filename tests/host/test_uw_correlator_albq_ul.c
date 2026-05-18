// Real-RF UL-direction cross-check. Companion to test_uw_correlator_albq
// which only exercises the DL sync pattern; this one feeds the (only)
// UL burst from the Albuquerque corpus through uw_correlator and asserts
// the direction discriminator picks UL.
//
// The DL test passed at 6.2 dB matched-filter SNR which is right at our
// production gate; the UL burst is at gr-iridium-reported 18.8 dB SNR so
// it should be a comfortable margin.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "uw_correlator.h"
#include "fixture_albq_ul_2sps.h"

static int s_passed = 0;
static int s_failed = 0;

#define CHECK(cond, ...) do {                              \
    if (!(cond)) {                                         \
        fprintf(stderr, "  FAIL line %d: ", __LINE__);     \
        fprintf(stderr, __VA_ARGS__);                      \
        fprintf(stderr, "\n");                             \
        s_failed++;                                        \
    } else {                                               \
        s_passed++;                                        \
    }                                                      \
} while (0)

int main(void)
{
    // Upsample 2 sps -> 10 sps via linear interp (uw_correlator now
    // expects 10 sps internally, matches gr-iridium burst_downmix).
    int n_complex_2sps = (int)(ALBQ_UL_2SPS_LEN / 2);
    int n_complex = n_complex_2sps * 5;
    int16_t *burst = malloc(n_complex * 2 * sizeof(int16_t));
    for (int n = 0; n < n_complex; n++) {
        int n2 = n / 5;
        int n2p = (n2 + 1 < n_complex_2sps) ? n2 + 1 : n2;
        float frac = (n % 5) / 5.0f;
        float re = (1.0f - frac) * (float)ALBQ_UL_2SPS[2 * n2]
                 +         frac  * (float)ALBQ_UL_2SPS[2 * n2p];
        float im = (1.0f - frac) * (float)ALBQ_UL_2SPS[2 * n2 + 1]
                 +         frac  * (float)ALBQ_UL_2SPS[2 * n2p + 1];
        burst[2 * n + 0] = (int16_t)re;
        burst[2 * n + 1] = (int16_t)im;
    }

    printf("Albuquerque UL burst (ISY): upsampled 2->10 sps, %d complex samples\n",
           n_complex);
    printf("Truth direction: %s\n", ALBQ_UL_2SPS_DIRECTION);

    int burst_start = uw_correlator_find_burst_start(burst, n_complex,
                                                      /*search_max=*/n_complex);
    printf("D13 burst start: %d (of %d samples)\n", burst_start, n_complex);
    CHECK(burst_start >= 0 && burst_start < n_complex - 64,
          "D13 burst start in plausible range (got %d)", burst_start);

    int16_t *adj_burst = burst + burst_start * 2;
    int adj_n = n_complex - burst_start;

    uw_correlator_apply_rrc(adj_burst, adj_burst, adj_n);

    uw_corr_result_t res;
    uw_correlator_find(adj_burst, adj_n, adj_n - 120, &res);

    printf("UW corr: dir=%s offset=%d corr=%.3f SNR=%.1f dB peak=%.2e omega=%.3f\n",
           res.direction == UW_DIR_DOWNLINK ? "DL"
           : res.direction == UW_DIR_UPLINK ? "UL" : "UNKNOWN",
           res.uw_offset, (double)res.correction,
           (double)res.snr_estimate_db, (double)res.peak_value,
           (double)res.omega_per_sym);

    // The whole point of this test: direction discrimination must pick
    // UL on a real UL burst.
    CHECK(res.direction == UW_DIR_UPLINK,
          "direction = UL (got %d)", (int)res.direction);

    // UL ISY at 18.8 dB should give a comfortable matched-filter SNR.
    CHECK(res.snr_estimate_db >= 6.0f,
          "matched-filter SNR >= 6 dB production gate (got %.1f)",
          (double)res.snr_estimate_db);

    // CFO bounded (sanity).
    float abs_omega = res.omega_per_sym < 0 ? -res.omega_per_sym : res.omega_per_sym;
    CHECK(abs_omega <= 1.5f,
          "CFO magnitude <= 1.5 rad/sym (got %.3f)",
          (double)res.omega_per_sym);

    free(burst);
    printf("\n=== %d passed, %d failed ===\n", s_passed, s_failed);
    return s_failed > 0 ? 1 : 0;
}
