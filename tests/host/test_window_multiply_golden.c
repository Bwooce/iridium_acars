// test_window_multiply_golden.c — bit-exact golden-fixture harness for
// the tagger's window-multiply kernel (T50 chain-C prerequisite, design
// doc docs/perf-decoupling-design-2026-07-04.md §T50).
//
// Why this exists: PIE `dsps_*_arp4` swaps have repeatedly regressed
// decode silently (58 → 0) because the SIMD routine's rounding or tail
// diverges from the scalar by a few LSBs and no bench gate catches it.
// This harness freezes the CURRENT scalar `fbt_window_multiply_q15`
// (the exact symbol the device calls — single source of truth, see
// fft_burst_tagger.c) as a golden baseline over deterministic vectors,
// and requires any candidate kernel to reproduce it with EXACT-ZERO
// difference. Not an epsilon: the tagger threshold compare downstream
// is integer, so 1 LSB is a real divergence.
//
// Two gates, both exact:
//   1. Reference-freeze: FNV-1a of the scalar output per case must
//      equal the embedded golden (catches accidental drift of the
//      reference itself — same idiom as test_direct_if_decim_golden.c).
//   2. Candidate-vs-reference: elementwise max |diff| must be 0.
//      Today candidate == the production scalar (positive control).
//      A host-runnable C model of a future PIE kernel plugs in here.
//
// Built-in negative controls (run every time, must be DETECTED):
//   - rounding variant ((a*w + 16384) >> 15 instead of truncation —
//     the exact divergence esp.vmul.s32.s16xs16 might have)
//   - single-output-LSB variant
// If either perturbation ever goes UNdetected, the vector set has gone
// toothless and the test fails.
//
// Vector design notes:
//   - All generation is integer xorshift32 with fixed seeds — bit-for-
//     bit reproducible across runs and machines, no floats, no time.
//   - Inputs span the full int16 range INCLUDING the -32768 rail.
//   - Window values are constrained to [0, 32767]: the device window
//     is a Q15 Blackman which is never negative, and window = -32768
//     with input = -32768 is the one pair where (a*w)>>15 == +32768
//     wraps in the int16 cast — pinning that unreachable-on-device
//     corner would reject kernels for behaviour that can never occur.
//
// Modes:
//   (no args)      run the gate (positive + negative controls)
//   --emit-golden  print the golden #define lines (paste into this
//                  file after a DELIBERATE vector/reference change)
//   --demo-fail    run the gate with the rounding variant installed as
//                  the candidate — demonstrates the red path, exits 1.
//
// Baseline captured 2026-07-04 from the scalar kernel at the commit
// introducing this test (pure refactor of the fft_burst_tagger.c:364
// window_multiply loop; arithmetic unchanged since Phase 3.6.M).

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "fft_burst_tagger.h"

// ---- GOLDEN BASELINE (regenerate with --emit-golden) ----
#define G_FNV_RAILS 0x37447dc5u
#define G_FNV_TRIANGLE 0x4f10c657u
#define G_FNV_RANDOM 0x0990753bu
#define G_FNV_NEARZERO 0xc37d991bu
#define G_FNV_ODD_N 0x1282126eu
// ---------------------------------------------------------

#define MAX_N 2048

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

// Deterministic PRNG — xorshift32, fixed seeds only.
static uint32_t xs32(uint32_t *s)
{
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return x;
}

static void fill_i16_fullrange(int16_t *dst, int n, uint32_t seed)
{
    for (int i = 0; i < n; i++)
        dst[i] = (int16_t)(uint16_t)(xs32(&seed) & 0xffffu); // incl. -32768
}

static void fill_window_q15(int16_t *dst, int n, uint32_t seed)
{
    for (int i = 0; i < n; i++)
        dst[i] = (int16_t)(xs32(&seed) & 0x7fffu); // [0, 32767]
}

static void fill_i16_nearzero(int16_t *dst, int n, uint32_t seed)
{
    for (int i = 0; i < n; i++)
        dst[i] = (int16_t)((int)(xs32(&seed) % 9u) - 4); // [-4, +4]
}

// ---- test cases ------------------------------------------------------

typedef struct {
    const char *name;
    int         n_complex;
    uint32_t    golden_fnv;
    void (*gen)(int16_t *in_iq, int16_t *window, int n_complex);
} wm_case_t;

// Case 1: rails + structured — full-scale, ±32768/±32767 rails, near-
// zero, sign changes; window sweeps rail values incl. 0 and 32767.
static void gen_rails(int16_t *in_iq, int16_t *window, int n)
{
    static const int16_t pat[16] = {
        32767, -32768, 32767, -32767, 1, -1, 0, 0,
        2, -2, 32766, -32766, 12345, -12345, -32768, 3};
    static const int16_t wpat[8] = {32767, 0, 1, 16384, 32766, 2, 23170, 32767};
    for (int i = 0; i < 2 * n; i++)
        in_iq[i] = pat[i & 15];
    for (int i = 0; i < n; i++)
        window[i] = (i < n / 2) ? (int16_t)32767 : wpat[i & 7];
}

// Case 2: triangle window (integer-exact Blackman stand-in covering
// the full [0, 32767] window range) × full-range random input.
static void gen_triangle(int16_t *in_iq, int16_t *window, int n)
{
    fill_i16_fullrange(in_iq, 2 * n, 0xA5A5A5A5u);
    for (int i = 0; i < n; i++) {
        int32_t up = (i <= (n - 1) / 2) ? i : (n - 1 - i);
        window[i]  = (int16_t)((up * 32767) / ((n - 1) / 2));
    }
}

// Case 3: random everything (input full-range, window Q15 [0,32767]).
static void gen_random(int16_t *in_iq, int16_t *window, int n)
{
    fill_i16_fullrange(in_iq, 2 * n, 0x12345678u);
    fill_window_q15(window, n, 0x9E3779B9u);
}

// Case 4: near-zero input — truncation-toward-minus-infinity of tiny
// negatives ((-1 * w) >> 15 == -1 for any w > 0) is the LSB behaviour
// a round-to-nearest kernel gets wrong everywhere.
static void gen_nearzero(int16_t *in_iq, int16_t *window, int n)
{
    fill_i16_nearzero(in_iq, 2 * n, 0xDEADBEEFu);
    fill_window_q15(window, n, 0x0BADF00Du);
}

// Case 5: odd n (1003 complex = 2006 int16, not a multiple of 8 SIMD
// lanes) — catches tail mishandling if the helper is ever called with
// n != 2048.
static void gen_odd(int16_t *in_iq, int16_t *window, int n)
{
    fill_i16_fullrange(in_iq, 2 * n, 0x600DCAFEu);
    fill_window_q15(window, n, 0x5EED5EEDu);
}

static const wm_case_t CASES[] = {
    {"rails", 2048, G_FNV_RAILS, gen_rails},
    {"triangle", 2048, G_FNV_TRIANGLE, gen_triangle},
    {"random", 2048, G_FNV_RANDOM, gen_random},
    {"nearzero", 2048, G_FNV_NEARZERO, gen_nearzero},
    {"odd_n", 1003, G_FNV_ODD_N, gen_odd},
};
#define N_CASES ((int)(sizeof(CASES) / sizeof(CASES[0])))

// ---- kernels under test ---------------------------------------------

typedef void (*wm_kernel_fn)(const int16_t *in_iq, const int16_t *window,
                             int16_t *out_iq, int n_complex);

// Negative control A: round-to-nearest instead of truncation — the
// exact way a PIE vmul.s32.s16xs16 swap could silently diverge.
static void perturbed_rounding(const int16_t *in_iq, const int16_t *window,
                               int16_t *out_iq, int n)
{
    for (int i = 0; i < n; i++) {
        int32_t re        = (int32_t)in_iq[i * 2 + 0] * (int32_t)window[i];
        int32_t im        = (int32_t)in_iq[i * 2 + 1] * (int32_t)window[i];
        out_iq[i * 2 + 0] = (int16_t)((re + 16384) >> 15);
        out_iq[i * 2 + 1] = (int16_t)((im + 16384) >> 15);
    }
}

// Negative control B: correct arithmetic, one output LSB flipped.
static void perturbed_one_lsb(const int16_t *in_iq, const int16_t *window,
                              int16_t *out_iq, int n)
{
    fbt_window_multiply_q15(in_iq, window, out_iq, n);
    out_iq[7] ^= 1;
}

// Elementwise exact compare; reports first divergence and max |diff|.
// Returns the number of differing int16 elements (0 == bit-exact).
static int diff_report(const char *tag, const int16_t *ref,
                       const int16_t *cand, int n_i16, int verbose)
{
    int     n_diff    = 0;
    int     first_idx = -1;
    int32_t max_diff  = 0;
    for (int k = 0; k < n_i16; k++) {
        if (ref[k] != cand[k]) {
            if (first_idx < 0) first_idx = k;
            int32_t d = (int32_t)cand[k] - (int32_t)ref[k];
            if (d < 0) d = -d;
            if (d > max_diff) max_diff = d;
            n_diff++;
        }
    }
    if (n_diff && verbose)
        fprintf(stderr,
                "  %s: %d/%d elements differ, first at [%d] ref=%d cand=%d, "
                "max|diff|=%d\n",
                tag, n_diff, n_i16, first_idx, (int)ref[first_idx],
                (int)cand[first_idx], (int)max_diff);
    return n_diff;
}

int main(int argc, char **argv)
{
    int emit_golden = (argc > 1 && strcmp(argv[1], "--emit-golden") == 0);
    int demo_fail   = (argc > 1 && strcmp(argv[1], "--demo-fail") == 0);

    // The candidate slot: point this at a host-runnable C model of a
    // replacement kernel to gate it. Default = the production scalar.
    wm_kernel_fn candidate = fbt_window_multiply_q15;
    if (demo_fail) {
        candidate = perturbed_rounding;
        printf("--demo-fail: candidate = perturbed_rounding (MUST go red)\n");
    }

    static int16_t in_iq[2 * MAX_N];
    static int16_t window[MAX_N];
    static int16_t ref_out[2 * MAX_N];
    static int16_t cand_out[2 * MAX_N];

    for (int c = 0; c < N_CASES; c++) {
        const wm_case_t *tc = &CASES[c];
        int              n  = tc->n_complex;
        tc->gen(in_iq, window, n);

        // Gate 1: reference freeze — the production scalar must still
        // produce the checked-in golden (positive control half 1).
        fbt_window_multiply_q15(in_iq, window, ref_out, n);
        uint32_t fnv = fnv1a_i16(ref_out, 2 * n);
        if (emit_golden) {
            char up[32];
            int  j;
            for (j = 0; tc->name[j] && j < 31; j++)
                up[j] = (char)((tc->name[j] >= 'a' && tc->name[j] <= 'z')
                                   ? tc->name[j] - 32
                                   : tc->name[j]);
            up[j] = '\0';
            printf("#define G_FNV_%-10s 0x%08xu\n", up, fnv);
            continue;
        }
        printf("case %-8s n=%d ref_fnv=0x%08x golden=0x%08x\n",
               tc->name, n, fnv, tc->golden_fnv);
        CHECK(fnv == tc->golden_fnv,
              "%s: scalar reference drifted from golden fixture "
              "(0x%08x != 0x%08x)",
              tc->name, fnv, tc->golden_fnv);

        // Gate 2: candidate vs reference, EXACT-ZERO criterion.
        memset(cand_out, 0, sizeof(cand_out));
        candidate(in_iq, window, cand_out, n);
        int nd = diff_report(tc->name, ref_out, cand_out, 2 * n, 1);
        CHECK(nd == 0, "%s: candidate kernel diverges from scalar "
                       "reference (%d elements) — swap REJECTED, no tuning",
              tc->name, nd);

        if (demo_fail) continue; // negative-control checks are for the
                                 // real gate run, not the red demo

        // Negative controls: both perturbations MUST be detected on
        // every case where they can express (rounding: any case with
        // half-bit products; LSB flip: always). Undetected = the
        // vector set has gone toothless.
        memset(cand_out, 0, sizeof(cand_out));
        perturbed_rounding(in_iq, window, cand_out, n);
        int      nd_round  = diff_report("neg_rounding", ref_out, cand_out, 2 * n, 0);
        uint32_t fnv_round = fnv1a_i16(cand_out, 2 * n);
        CHECK(nd_round > 0 && fnv_round != tc->golden_fnv,
              "%s: rounding perturbation NOT detected (nd=%d fnv=0x%08x) — "
              "harness is toothless",
              tc->name, nd_round, fnv_round);

        memset(cand_out, 0, sizeof(cand_out));
        perturbed_one_lsb(in_iq, window, cand_out, n);
        int      nd_lsb  = diff_report("neg_one_lsb", ref_out, cand_out, 2 * n, 0);
        uint32_t fnv_lsb = fnv1a_i16(cand_out, 2 * n);
        CHECK(nd_lsb == 1 && fnv_lsb != tc->golden_fnv,
              "%s: single-LSB perturbation NOT detected (nd=%d) — "
              "harness is toothless",
              tc->name, nd_lsb);
        printf("  negative controls detected: rounding %d elems, "
               "one-lsb %d elem\n",
               nd_round, nd_lsb);
    }

    if (emit_golden) return 0;

    printf("window_multiply golden: %d passed, %d failed\n",
           s_passed, s_failed);
    return s_failed ? 1 : 0;
}
