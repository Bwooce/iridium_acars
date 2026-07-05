// test_resample_tile.c — bit-exact validation of the T49b convert+resample
// tile fusion used by p4-usb-host/main/ingest_core1.c's ingest_task
// (docs/perf-decoupling-design-2026-07-04.md §T49b).
//
// T49b replaces two full-buffer passes (convert uint8->int16 into PSRAM
// s_conv[], then resample reads it all back) with ONE loop that converts
// + resamples <= TILE_COMPLEX-sample chunks through a single small
// internal-SRAM tile, calling resample_256_to_250_process_explicit
// repeatedly with the SAME persistent (delay, wpos, start_pos) state
// threaded across chunks -- the same mechanism test_resample_split.c
// already proves bit-exact for the Worker A/B split. This test proves
// the analogous thing for SEQUENTIAL tiling of the convert step, driven
// from raw uint8 bytes (the actual USB-ingest input), not pre-converted
// int16 arrays.
//
// Reference: convert the WHOLE buffer, then a single _process_explicit
// call over n_in_complex = n_bytes/2 (== production behaviour before
// T49b, and still what ingest_task's dead split-path branch does).
// Candidate: run_tiled() -- the same algorithm ingest_task's hot path
// uses: loop over <= TILE_COMPLEX chunks, convert each into a small
// tile, resample immediately, thread state across chunks AND across
// simulated dispatches (case 2).
//
// Modes:
//   (no args)      run the gate (positive + negative controls)
//   --demo-fail    run with the broken (state-resetting) tiling
//                  installed as "the candidate" -- demonstrates the red
//                  path, exits 1 (mirrors test_window_multiply_golden.c's
//                  idiom).

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "resample_256_to_250.h"

// Must match INGEST_TILE_COMPLEX in p4-usb-host/main/ingest_core1.h.
// Kept as an independent literal (not a shared header -- ingest_core1.h
// is ESP-IDF/FreeRTOS-only and doesn't build on host) so a future change
// to one is caught by this test's own arithmetic staying self-consistent;
// the production value is pinned by ingest_core1.c's INGEST_TILE_COMPLEX
// comment referencing this file. (Bit-exactness is independent of tile
// size, so this test proves it for whatever value is set here.)
#define TILE_COMPLEX 1024

#define MAX_BYTES (32 * 1024)
#define MAX_OUT (MAX_BYTES / 2 + 16)

static uint8_t g_raw[MAX_BYTES];
static int16_t g_out_ref[MAX_OUT * 2];
static int16_t g_out_cand[MAX_OUT * 2];

static int s_passed = 0, s_failed = 0;

// Exact convert formula from ingest_core1.c's ingest_task convert step
// (out[i] = (int16_t)((b[i] << 8) ^ 0x8000)). Duplicated here rather
// than shared because ingest_core1.c is ESP-IDF/FreeRTOS-only and
// doesn't build on a host toolchain.
static void convert_u8_to_i16(const uint8_t *src, int16_t *dst, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        dst[i] = (int16_t)(((src[i] << 8) ^ 0x8000));
    }
}

// Reference: convert the WHOLE dispatch, then one _process_explicit
// call over n_in_complex = n_bytes/2. Matches production pre-T49b (and
// the still-live dead split-path branch in ingest_core1.c).
static int run_reference(int16_t *delay_i, int16_t *delay_q,
                         int *wpos, int *start_pos,
                         const uint8_t *raw, size_t n_bytes,
                         int16_t *out_iq)
{
    static int16_t conv[MAX_BYTES];
    int            n_in_complex = (int)(n_bytes / 2);
    convert_u8_to_i16(raw, conv, (size_t)n_in_complex * 2);
    return resample_256_to_250_process_explicit(delay_i, delay_q, wpos, start_pos,
                                                conv, n_in_complex,
                                                out_iq, n_in_complex,
                                                /*batch_scratch=*/NULL);
}

// Candidate: T49b tiling -- convert+resample <= TILE_COMPLEX chunks
// through a small tile, looping, with state threaded across chunks.
//
// `broken_reset_state` (negative control): if nonzero, resets the
// persistent delay/wpos/start_pos to a fresh zero state at the START of
// every tile after the first, simulating a tiling bug that drops the
// resampler's continuity across chunk boundaries (e.g. a copy-paste
// that re-inits state per call instead of threading it through). MUST
// be caught by the comparison in main() whenever more than one tile
// runs.
static int run_tiled(int16_t *delay_i, int16_t *delay_q,
                     int *wpos, int *start_pos,
                     const uint8_t *raw, size_t n_bytes,
                     int16_t *out_iq, int broken_reset_state)
{
    int     n_in_complex_total = (int)(n_bytes / 2);
    int     complex_done       = 0;
    int     n_out              = 0;
    int16_t tile[TILE_COMPLEX * 2];

    while (complex_done < n_in_complex_total) {
        int tile_complex = n_in_complex_total - complex_done;
        if (tile_complex > TILE_COMPLEX) tile_complex = TILE_COMPLEX;

        convert_u8_to_i16(raw + (size_t)complex_done * 2, tile,
                          (size_t)tile_complex * 2);

        if (broken_reset_state && complex_done > 0) {
            // NEGATIVE CONTROL: drop continuity between tiles.
            memset(delay_i, 0, 32 * sizeof(int16_t));
            memset(delay_q, 0, 32 * sizeof(int16_t));
            *wpos      = 0;
            *start_pos = 0;
        }

        int16_t batch_scratch[RS25_BATCH_COMPLEX * 2] __attribute__((aligned(16)));
        int     n_out_tile = resample_256_to_250_process_explicit(
            delay_i, delay_q, wpos, start_pos,
            tile, tile_complex,
            out_iq + 2 * n_out, tile_complex,
            batch_scratch);
        n_out += n_out_tile;
        complex_done += tile_complex;
    }
    return n_out;
}

static void fill_random_bytes(uint32_t seed)
{
    uint32_t s = seed;
    for (int i = 0; i < MAX_BYTES; i++) {
        s        = s * 1103515245u + 12345u;
        g_raw[i] = (uint8_t)(s >> 16);
    }
}

// Full-scale saturation pattern: sweeps every uint8 value 0..255
// repeating, so both convert-formula rails (0x00 -> -32768, 0xFF ->
// 32512) and every value between are exercised at tile boundaries.
static void fill_fullrange_bytes(void)
{
    for (int i = 0; i < MAX_BYTES; i++) {
        g_raw[i] = (uint8_t)(i & 0xff);
    }
}

static int compare_outputs(int n_complex, const char *label, int verbose)
{
    int n_int16   = n_complex * 2;
    int errs      = 0;
    int first_err = -1;
    for (int i = 0; i < n_int16; i++) {
        if (g_out_ref[i] != g_out_cand[i]) {
            errs++;
            if (first_err < 0) first_err = i;
        }
    }
    if (errs) {
        if (verbose)
            fprintf(stderr,
                    "  [%s]: %d/%d int16 outputs differ; first at idx %d "
                    "(ref=%d cand=%d)\n",
                    label, errs, n_int16, first_err,
                    g_out_ref[first_err], g_out_cand[first_err]);
        return errs;
    }
    if (verbose) printf("OK   [%s]: %d complex outputs bit-exact\n", label, n_complex);
    return 0;
}

int main(int argc, char **argv)
{
    int demo_fail = (argc > 1 && strcmp(argv[1], "--demo-fail") == 0);
    if (demo_fail) {
        printf("--demo-fail: candidate = broken_reset_state tiling (MUST go red)\n");
    }

    // Trigger one init pass to populate s_coeffs_pp.
    {
        resample_256_to_250_t r0;
        resample_256_to_250_init(&r0);
    }

    int total_fails = 0;

    // --- Case 1: single-dispatch lengths, clean state each time ---
    // Includes: non-multiples of TILE_COMPLEX*2 (4096) bytes, exact tile
    // boundary and boundary+1, the AGC-prefix region (<=256 bytes, and
    // lengths straddling it), and multi-tile lengths.
    {
        int lens[] = {
            1, 2, 3,                 // degenerate: n_in_complex 0 or 1
            255, 256, 300, 511, 512, // AGC-prefix region (<=256B scan window)
            4095, 4096, 4097,        // tile boundary (TILE_COMPLEX*2 bytes)
            8191, 8192, 8193,        // 2-tile boundary
            12288,                   // 3 tiles exactly
            16383, 16384,            // typical 16 KB USB transfer, +/- 1
        };
        int n_lens = (int)(sizeof(lens) / sizeof(lens[0]));
        for (int li = 0; li < n_lens; li++) {
            size_t n_bytes = (size_t)lens[li];
            fill_random_bytes(0xC0FFEE00u + (uint32_t)li);

            int16_t da_i[32] = {0}, da_q[32] = {0};
            int16_t db_i[32] = {0}, db_q[32] = {0};
            int     wp_r = 0, wp_c = 0, sp_r = 0, sp_c = 0;

            int n_ref  = run_reference(da_i, da_q, &wp_r, &sp_r, g_raw, n_bytes, g_out_ref);
            int n_cand = run_tiled(db_i, db_q, &wp_c, &sp_c, g_raw, n_bytes, g_out_cand,
                                   /*broken_reset_state=*/0);

            char label[64];
            snprintf(label, sizeof(label), "case1 n_bytes=%zu", n_bytes);
            if (n_ref != n_cand) {
                fprintf(stderr, "FAIL [%s]: n_ref=%d n_cand=%d\n", label, n_ref, n_cand);
                total_fails++;
                continue;
            }
            int errs = compare_outputs(n_ref, label, !demo_fail);
            if (!demo_fail) total_fails += errs ? 1 : 0;
        }
    }

    // --- Case 2: full-scale saturation input, single large dispatch ---
    {
        fill_fullrange_bytes();
        size_t  n_bytes  = 16384;
        int16_t da_i[32] = {0}, da_q[32] = {0};
        int16_t db_i[32] = {0}, db_q[32] = {0};
        int     wp_r = 0, wp_c = 0, sp_r = 0, sp_c = 0;

        int n_ref  = run_reference(da_i, da_q, &wp_r, &sp_r, g_raw, n_bytes, g_out_ref);
        int n_cand = run_tiled(db_i, db_q, &wp_c, &sp_c, g_raw, n_bytes, g_out_cand,
                               /*broken_reset_state=*/0);
        if (n_ref != n_cand) {
            fprintf(stderr, "FAIL [case2 fullrange]: n_ref=%d n_cand=%d\n", n_ref, n_cand);
            total_fails++;
        } else {
            int errs = compare_outputs(n_ref, "case2 fullrange saturation", !demo_fail);
            if (!demo_fail) total_fails += errs ? 1 : 0;
        }
    }

    // --- Case 3: multi-dispatch (cross-dispatch state continuity) ---
    // Models ingest_task processing several consecutive USB dispatches:
    // persistent (delay, wpos, start_pos) carries across BOTH tile
    // boundaries within a dispatch AND dispatch boundaries.
    {
        int16_t da_i[32] = {0}, da_q[32] = {0};
        int16_t db_i[32] = {0}, db_q[32] = {0};
        int     wp_r = 0, wp_c = 0, sp_r = 0, sp_c = 0;
        int     dispatch_lens[] = {4096, 16384, 300, 8192, 16383};
        int     n_dispatches    = (int)(sizeof(dispatch_lens) / sizeof(dispatch_lens[0]));
        int     chunk_fails     = 0;

        for (int d = 0; d < n_dispatches; d++) {
            size_t n_bytes = (size_t)dispatch_lens[d];
            fill_random_bytes(0xD15C0000u + (uint32_t)d);

            int n_ref  = run_reference(da_i, da_q, &wp_r, &sp_r, g_raw, n_bytes, g_out_ref);
            int n_cand = run_tiled(db_i, db_q, &wp_c, &sp_c, g_raw, n_bytes, g_out_cand,
                                   /*broken_reset_state=*/0);
            if (n_ref != n_cand) {
                fprintf(stderr, "FAIL [case3.d%d]: n_ref=%d n_cand=%d\n", d, n_ref, n_cand);
                chunk_fails++;
                continue;
            }
            char label[64];
            snprintf(label, sizeof(label), "case3 dispatch=%d n_bytes=%zu", d, n_bytes);
            chunk_fails += compare_outputs(n_ref, label, !demo_fail) ? 1 : 0;
        }
        if (chunk_fails == 0 && !demo_fail) {
            printf("OK   [case3 multi-dispatch]: %d dispatches bit-exact, state threaded across both tile and dispatch boundaries\n",
                   n_dispatches);
        }
        if (!demo_fail) total_fails += chunk_fails;
    }

    if (demo_fail) {
        // Red-path demo: install the broken (state-resetting) tiling as
        // "the candidate" and show the gate rejects it.
        fill_random_bytes(0xBADC0DE1u);
        size_t  n_bytes  = 16384; // > 2 tiles, so the reset actually fires
        int16_t da_i[32] = {0}, da_q[32] = {0};
        int16_t db_i[32] = {0}, db_q[32] = {0};
        int     wp_r = 0, wp_c = 0, sp_r = 0, sp_c = 0;

        int n_ref  = run_reference(da_i, da_q, &wp_r, &sp_r, g_raw, n_bytes, g_out_ref);
        int n_cand = run_tiled(db_i, db_q, &wp_c, &sp_c, g_raw, n_bytes, g_out_cand,
                               /*broken_reset_state=*/1);
        int nd     = (n_ref == n_cand) ? compare_outputs(n_ref, "demo-fail broken tiling", 1) : 1;
        if (n_ref != n_cand) {
            fprintf(stderr, "demo-fail: n_ref=%d n_cand=%d (also diverges, as expected)\n",
                    n_ref, n_cand);
        }
        printf("--demo-fail: %s (%d int16 elements differ) -- red path CONFIRMED\n",
               nd > 0 ? "DETECTED divergence" : "NOT detected (unexpected)", nd);
        return nd > 0 ? 1 : 0;
    }

    // --- Negative control: broken (state-resetting) tiling MUST diverge
    // from the reference whenever more than one tile runs. Run on a
    // representative multi-tile length; asserting nd > 0 here is what
    // proves this test harness isn't toothless (an un-caught state reset
    // would mean the positive-control comparison above is too weak to
    // matter).
    {
        size_t n_bytes = 16384; // 4 tiles @ TILE_COMPLEX=2048 complex
        fill_random_bytes(0x5EEDBEEFu);
        int16_t da_i[32] = {0}, da_q[32] = {0};
        int16_t db_i[32] = {0}, db_q[32] = {0};
        int     wp_r = 0, wp_c = 0, sp_r = 0, sp_c = 0;

        int n_ref = run_reference(da_i, da_q, &wp_r, &sp_r, g_raw, n_bytes, g_out_ref);
        int n_brk = run_tiled(db_i, db_q, &wp_c, &sp_c, g_raw, n_bytes, g_out_cand,
                              /*broken_reset_state=*/1);
        int nd    = (n_ref == n_brk) ? compare_outputs(n_ref, "neg_control broken_reset", 0) : 1;
        if (nd > 0) {
            s_passed++;
            printf("OK   [negative control]: broken state-reset tiling correctly DETECTED "
                   "(%d elements diverge / count mismatch=%d)\n",
                   nd, n_ref != n_brk);
        } else {
            s_failed++;
            fprintf(stderr,
                    "FAIL [negative control]: broken state-reset tiling was NOT detected -- "
                    "this harness is toothless\n");
        }
    }

    if (total_fails == 0 && s_failed == 0) {
        printf("\nALL OK: tiled convert+resample is bit-exact vs whole-buffer reference "
               "(negative control confirmed detectable)\n");
        return 0;
    }
    printf("\nFAILED: %d positive-control divergences, %d negative-control failures\n",
           total_fails, s_failed);
    return 1;
}
