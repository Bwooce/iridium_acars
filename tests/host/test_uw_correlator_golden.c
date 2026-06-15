// Characterization (golden-master) test for uw_correlator — task #133,
// the safety net for the #120 context-threading refactor.
//
// Unlike test_uw_correlator_albq.c (which checks plausibility ranges),
// this PINS the EXACT current output of the sync/CFO pipeline on the
// ALBQ fixture: exact D13 burst start, an FNV-1a checksum of the
// RRC-filtered sample buffer, and exact UW offset/direction plus the
// float CFO/SNR/peak fields to a tight epsilon. A pure-plumbing refactor
// must reproduce all of these bit-for-bit; any drift fails here.
//
// Baseline captured 2026-06-15 from commit 8a41b4f (pre-#120).

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "uw_correlator.h"
#include "fixture_albq_2sps.h" // ALBQ_2SPS, ALBQ_2SPS_LEN

// ---- GOLDEN BASELINE (filled from a capture run; see CAPTURE block) ----
#define G_BURST_START 2304
#define G_RRC_FNV 0x55bc316eu
#define G_UW_OFFSET 196
#define G_DIRECTION UW_DIR_DOWNLINK // == 1
#define G_CORRECTION 0.000000f
#define G_SNR_DB 27.047224f
#define G_OMEGA 0.003292f
#define G_PEAK_RE 393440320.000000f
#define G_PEAK_IM 1375337344.000000f
#define FEPS 1e-3f // relative epsilon for the float fields
// -----------------------------------------------------------------------

static int s_passed = 0, s_failed = 0;
#define CHECK(cond, ...)                                   \
    do {                                                   \
        if (!(cond)) {                                     \
            fprintf(stderr, "  FAIL line %d: ", __LINE__); \
            fprintf(stderr, __VA_ARGS__);                  \
            fprintf(stderr, "\n");                         \
            s_failed++;                                    \
        } else {                                           \
            s_passed++;                                    \
        }                                                  \
    } while (0)

static int feq(float got, float want)
{
    float d = fabsf(got - want);
    float a = fabsf(want);
    return d <= FEPS * (a > 1.0f ? a : 1.0f);
}

static uint32_t fnv1a_i16(const int16_t *p, int n)
{
    uint32_t h = 2166136261u;
    for (int i = 0; i < n; i++) {
        uint16_t v = (uint16_t)p[i];
        h          = (h ^ (v & 0xff)) * 16777619u;
        h          = (h ^ (v >> 8)) * 16777619u;
    }
    return h;
}

int main(void)
{
    // Build the 10-sps burst exactly as test_uw_correlator_albq.c does.
    int      n2 = (int)(ALBQ_2SPS_LEN / 2);
    int      nc = n2 * 5;
    int16_t *b  = malloc(nc * 2 * sizeof(int16_t));
    for (int n = 0; n < nc; n++) {
        int   a      = n / 5;
        int   ap     = (a + 1 < n2) ? a + 1 : a;
        float frac   = (n % 5) / 5.0f;
        b[2 * n + 0] = (int16_t)((1.0f - frac) * ALBQ_2SPS[2 * a] + frac * ALBQ_2SPS[2 * ap]);
        b[2 * n + 1] = (int16_t)((1.0f - frac) * ALBQ_2SPS[2 * a + 1] + frac * ALBQ_2SPS[2 * ap + 1]);
    }

    int      burst_start = uw_correlator_find_burst_start(b, nc, nc);
    int16_t *adj         = b + burst_start * 2;
    int      adj_n       = nc - burst_start;
    uw_correlator_apply_rrc(adj, adj, adj_n);
    uint32_t rrc_fnv = fnv1a_i16(adj, adj_n * 2);

    uw_corr_result_t r;
    uw_correlator_find(adj, adj_n, adj_n - 120, &r);

    // Copy-pasteable capture block (update the #defines if the algorithm
    // intentionally changes).
    printf("GOLDEN CAPTURE:\n");
    printf("  #define G_BURST_START %d\n", burst_start);
    printf("  #define G_RRC_FNV 0x%08xu\n", rrc_fnv);
    printf("  #define G_UW_OFFSET %d\n", r.uw_offset);
    printf("  #define G_DIRECTION %d\n", (int)r.direction);
    printf("  #define G_CORRECTION %.6ff\n", (double)r.correction);
    printf("  #define G_SNR_DB %.6ff\n", (double)r.snr_estimate_db);
    printf("  #define G_OMEGA %.6ff\n", (double)r.omega_per_sym);
    printf("  #define G_PEAK_RE %.6ff\n", (double)r.peak_re);
    printf("  #define G_PEAK_IM %.6ff\n", (double)r.peak_im);

    CHECK(burst_start == G_BURST_START, "burst_start %d != golden %d", burst_start, G_BURST_START);
    CHECK(rrc_fnv == G_RRC_FNV, "RRC fnv 0x%08x != golden 0x%08x", rrc_fnv, G_RRC_FNV);
    CHECK(r.uw_offset == G_UW_OFFSET, "uw_offset %d != golden %d", r.uw_offset, G_UW_OFFSET);
    CHECK((int)r.direction == (int)G_DIRECTION, "direction %d != golden %d", (int)r.direction, (int)G_DIRECTION);
    CHECK(feq(r.correction, G_CORRECTION), "correction %.6f != golden %.6f", (double)r.correction, (double)G_CORRECTION);
    CHECK(feq(r.snr_estimate_db, G_SNR_DB), "snr %.6f != golden %.6f", (double)r.snr_estimate_db, (double)G_SNR_DB);
    CHECK(feq(r.omega_per_sym, G_OMEGA), "omega %.6f != golden %.6f", (double)r.omega_per_sym, (double)G_OMEGA);
    CHECK(feq(r.peak_re, G_PEAK_RE), "peak_re %.6f != golden %.6f", (double)r.peak_re, (double)G_PEAK_RE);
    CHECK(feq(r.peak_im, G_PEAK_IM), "peak_im %.6f != golden %.6f", (double)r.peak_im, (double)G_PEAK_IM);

    free(b);
    printf("\n=== %d passed, %d failed ===\n", s_passed, s_failed);
    return s_failed > 0 ? 1 : 0;
}
