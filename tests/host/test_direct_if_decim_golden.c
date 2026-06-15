// Characterization (golden-master) test for direct_if_decim — task #133,
// safety net for the #120 context-threading refactor.
//
// Self-contained: feeds a deterministic LCG-generated complex int16
// stream through direct_if_decim_process and pins the EXACT output —
// sample count + FNV-1a checksum of the decimated buffer. No external
// /tmp fixtures (unlike test_direct_if_decim.c, which needs the
// iridium-extractor-generated path-C dumps). A pure-plumbing refactor
// must reproduce these exactly.
//
// Baseline captured 2026-06-15 from commit 8a41b4f (pre-#120).

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "direct_if_decim.h"

// ---- GOLDEN BASELINE (from a capture run) ----
#define G_N_POST 5000
#define G_OUT_FNV 0xc60bf47bu
// ----------------------------------------------

#define N_IN_COMPLEX 50000

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
    int16_t *in  = malloc(N_IN_COMPLEX * 2 * sizeof(int16_t));
    int16_t *out = malloc((N_IN_COMPLEX / DIDECIM_DECIM + 16) * 2 * sizeof(int16_t));
    if (!in || !out) {
        fprintf(stderr, "alloc failed\n");
        return 2;
    }

    // Deterministic input: a fixed LCG, scaled into a realistic Q15-ish
    // amplitude band. Identical every run + platform (integer-only).
    uint32_t st = 0x12345678u;
    for (int i = 0; i < N_IN_COMPLEX * 2; i++) {
        st    = st * 1664525u + 1013904223u;
        in[i] = (int16_t)((int32_t)(st >> 16) % 8000); // [-7999, 7999]
    }

    direct_if_decim_t dec;
    direct_if_decim_init(&dec);
    int n_post = direct_if_decim_process(&dec, in, N_IN_COMPLEX, out);

    uint32_t out_fnv = fnv1a_i16(out, n_post * 2);

    printf("GOLDEN CAPTURE:\n");
    printf("  #define G_N_POST %d\n", n_post);
    printf("  #define G_OUT_FNV 0x%08xu\n", out_fnv);

    CHECK(n_post == G_N_POST, "n_post %d != golden %d", n_post, G_N_POST);
    CHECK(out_fnv == G_OUT_FNV, "out fnv 0x%08x != golden 0x%08x", out_fnv, G_OUT_FNV);

    free(in);
    free(out);
    printf("\n=== %d passed, %d failed ===\n", s_passed, s_failed);
    return s_failed > 0 ? 1 : 0;
}
