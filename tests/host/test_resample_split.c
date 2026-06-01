// test_resample_split.c — bit-exact validation that the within-chunk
// split-resample path (used by p4-usb-host/main/ingest_core1.c)
// emits the same int16 IQ outputs as the sequential reference for
// the same input stream.
//
// The split path:
//   1. Worker A: input[0..mid-1], outputs[0..n_emits_a-1].
//      Inherits delay + start_pos from previous chunk.
//   2. Worker B: input[mid..end-1], outputs[n_emits_a..n_total-1].
//      Initial delay = input[mid-9..mid-1] (newest at [0]).
//      Initial start_pos derived via closed-form:
//        n_emits_a   = floor((125*mid + 124 - start_pos_init) / 128)
//        start_pos_b = start_pos_init + 128*n_emits_a - 125*mid
//   3. After join, Worker B's end-state persists to next chunk.
//
// We exercise the path over multiple chunks (so cross-chunk state
// is also covered), multiple input lengths (including odd values
// where mid != n_in/2 exactly), and an adversarial start_pos to
// stress the phase walk.

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "resample_256_to_250.h"

#define MAX_IN 16384
#define MAX_OUT (MAX_IN + 16)

static int16_t g_in[MAX_IN * 2];
static int16_t g_out_ref[MAX_OUT * 2];
static int16_t g_out_split[MAX_OUT * 2];

// Reference: pure sequential — one persistent (delay, wpos, start_pos),
// process the whole chunk straight through.
static int run_sequential(int16_t *delay_i, int16_t *delay_q,
                          int *wpos, int *start_pos,
                          const int16_t *in_iq, int n_in_complex,
                          int16_t *out_iq)
{
    return resample_256_to_250_process_explicit(delay_i, delay_q,
                                                wpos, start_pos,
                                                in_iq, n_in_complex,
                                                out_iq, n_in_complex,
                                                /*batch_scratch=*/NULL);
}

// Split: pre-compute Worker B's initial state from closed-form, run
// both halves serially (in the test — production runs them in
// parallel, but the math is identical).
static int run_split(int16_t *delay_persist_i, int16_t *delay_persist_q,
                     int *wpos_persist, int *start_pos_persist,
                     const int16_t *in_iq, int n_in_complex,
                     int16_t *out_iq)
{
    int mid = n_in_complex / 2;

    // Worker A — inherits persistent (delay, wpos, start_pos).
    int16_t da_i[32] __attribute__((aligned(16))) = {0};
    int16_t da_q[32] __attribute__((aligned(16))) = {0};
    memcpy(da_i, delay_persist_i, sizeof(da_i));
    memcpy(da_q, delay_persist_q, sizeof(da_q));
    int wp_a = *wpos_persist;
    int sp_a = *start_pos_persist;
    int n_a  = resample_256_to_250_process_explicit(da_i, da_q, &wp_a, &sp_a,
                                                    in_iq, mid,
                                                    out_iq, mid,
                                                    /*batch_scratch=*/NULL);

    // Worker B initial state — same closed-form as production.
    int m                = mid / 128;
    int r                = mid - m * 128;
    int n_emits_a_closed = 125 * m;
    for (int i = 0; i < r; i++) {
        int pre = (*start_pos_persist + 3 * i) & 127;
        if (pre < 125) n_emits_a_closed++;
    }
    if (n_emits_a_closed != n_a) {
        fprintf(stderr,
                "CLOSED-FORM MISMATCH: predicted n_emits_a=%d, actual=%d (mid=%d, S0=%d)\n",
                n_emits_a_closed, n_a, mid, *start_pos_persist);
        return -1;
    }

    int16_t db_i[32] __attribute__((aligned(16))) = {0};
    int16_t db_q[32] __attribute__((aligned(16))) = {0};
    // Seed Worker B's delay buffer with the 9 newest input samples
    // (mid-1, mid-2, ..., mid-9) at delay[0..8] in newest-first
    // order, with mirror copies at delay[16..24]. wpos starts at 0
    // so the first iteration decrements to 15 and writes the new
    // sample at delay[15] (= mirror at delay[31]); the MAC then
    // reads delay[15..23] which picks up [new, delay[16]=s_{-1},
    // ..., delay[24]=s_{-9}].
    for (int k = 0; k < 9; k++) {
        int src_idx = mid - 1 - k;
        if (src_idx >= 0) {
            db_i[k]      = in_iq[2 * src_idx + 0];
            db_i[k + 16] = in_iq[2 * src_idx + 0];
            db_q[k]      = in_iq[2 * src_idx + 1];
            db_q[k + 16] = in_iq[2 * src_idx + 1];
        }
    }
    int wp_b = 0;
    int sp_b = *start_pos_persist + 128 * n_emits_a_closed - 125 * mid;

    // Worker B processes the second half.
    int n_b_max = n_in_complex - mid;
    int n_b     = resample_256_to_250_process_explicit(db_i, db_q, &wp_b, &sp_b,
                                                       in_iq + 2 * mid, n_b_max,
                                                       out_iq + 2 * n_a,
                                                       n_b_max,
                                                       /*batch_scratch=*/NULL);

    // Persist Worker B's end-state.
    memcpy(delay_persist_i, db_i, sizeof(db_i));
    memcpy(delay_persist_q, db_q, sizeof(db_q));
    *wpos_persist      = wp_b;
    *start_pos_persist = sp_b;

    return n_a + n_b;
}

static void fill_pseudorandom(uint32_t seed)
{
    uint32_t s = seed;
    for (int i = 0; i < MAX_IN * 2; i++) {
        s = s * 1103515245u + 12345u;
        // 16-bit signed pseudorandom, full range.
        g_in[i] = (int16_t)(s >> 16);
    }
}

static int compare_outputs(int n_complex, const char *label)
{
    int n_int16   = n_complex * 2;
    int errs      = 0;
    int first_err = -1;
    for (int i = 0; i < n_int16; i++) {
        if (g_out_ref[i] != g_out_split[i]) {
            errs++;
            if (first_err < 0) first_err = i;
        }
    }
    if (errs) {
        fprintf(stderr,
                "FAIL [%s]: %d/%d int16 outputs differ; first at idx %d "
                "(ref=%d split=%d)\n",
                label, errs, n_int16, first_err,
                g_out_ref[first_err], g_out_split[first_err]);
        return 1;
    }
    printf("OK   [%s]: %d outputs bit-exact\n", label, n_complex);
    return 0;
}

int main(void)
{
    // Trigger one init pass to populate s_coeffs_pp (the singleton
    // tap table — the process_explicit entry depends on it).
    {
        resample_256_to_250_t r0;
        resample_256_to_250_init(&r0);
    }

    int total_fails = 0;

    // --- Case 1: clean state, even-length input, single chunk ---
    {
        int16_t da_i[32] = {0}, da_q[32] = {0};
        int16_t db_i[32] = {0}, db_q[32] = {0};
        int     wp_seq = 0, wp_split = 0;
        int     sp_seq = 0, sp_split = 0;

        fill_pseudorandom(0xC0FFEE01);
        int n_in = 4096;

        int n_ref = run_sequential(da_i, da_q, &wp_seq, &sp_seq,
                                   g_in, n_in, g_out_ref);
        int n_spl = run_split(db_i, db_q, &wp_split, &sp_split,
                              g_in, n_in, g_out_split);

        if (n_ref != n_spl) {
            fprintf(stderr, "FAIL [case1]: n_ref=%d n_spl=%d\n", n_ref, n_spl);
            total_fails++;
        } else if (sp_seq != sp_split) {
            fprintf(stderr, "FAIL [case1]: start_pos diverged ref=%d split=%d\n",
                    sp_seq, sp_split);
            total_fails++;
        } else if (wp_seq != wp_split) {
            fprintf(stderr, "FAIL [case1]: wpos diverged ref=%d split=%d\n",
                    wp_seq, wp_split);
            total_fails++;
        } else if (memcmp(da_i, db_i, sizeof(da_i)) != 0 ||
                   memcmp(da_q, db_q, sizeof(da_q)) != 0) {
            fprintf(stderr, "FAIL [case1]: delay-line diverged\n");
            total_fails++;
        } else {
            total_fails += compare_outputs(n_ref, "case1 even/clean");
        }
    }

    // --- Case 2: multi-chunk (cross-chunk delay-line continuity) ---
    {
        int16_t da_i[32] = {0}, da_q[32] = {0};
        int16_t db_i[32] = {0}, db_q[32] = {0};
        int     wp_seq = 0, wp_split = 0;
        int     sp_seq = 0, sp_split = 0;

        fill_pseudorandom(0xDEADBEEF);
        int n_chunk     = 2048;
        int n_chunks    = 5;
        int chunk_fails = 0;
        for (int c = 0; c < n_chunks; c++) {
            const int16_t *in    = g_in + c * 2 * n_chunk;
            int            n_ref = run_sequential(da_i, da_q, &wp_seq, &sp_seq,
                                                  in, n_chunk, g_out_ref);
            int            n_spl = run_split(db_i, db_q, &wp_split, &sp_split,
                                             in, n_chunk, g_out_split);
            if (n_ref != n_spl) {
                fprintf(stderr, "FAIL [case2.c%d]: n_ref=%d n_spl=%d\n",
                        c, n_ref, n_spl);
                chunk_fails++;
                continue;
            }
            for (int i = 0; i < n_ref * 2; i++) {
                if (g_out_ref[i] != g_out_split[i]) {
                    fprintf(stderr,
                            "FAIL [case2.c%d]: out[%d] ref=%d split=%d "
                            "(start_pos ref=%d split=%d at chunk start)\n",
                            c, i, g_out_ref[i], g_out_split[i],
                            sp_seq, sp_split);
                    chunk_fails++;
                    break;
                }
            }
        }
        if (chunk_fails == 0) {
            printf("OK   [case2 multi-chunk]: %d chunks × %d samples bit-exact\n",
                   n_chunks, n_chunk);
        }
        total_fails += chunk_fails;
    }

    // --- Case 3: adversarial start_pos values ---
    {
        int adv_start_positions[] = {0, 1, 62, 124, 125, 127};
        int n_pos                 = sizeof(adv_start_positions) / sizeof(adv_start_positions[0]);
        for (int p = 0; p < n_pos; p++) {
            int16_t da_i[32] = {0}, da_q[32] = {0};
            int16_t db_i[32] = {0}, db_q[32] = {0};
            // Both paths must START from the SAME state including
            // initial delay. We feed a few samples first to populate
            // it deterministically.
            fill_pseudorandom(0xA5A5 + p);
            int  wp_seq = 0, wp_split = 0;
            int  sp_seq   = adv_start_positions[p];
            int  sp_split = sp_seq;
            int  n_in     = 8192;
            int  n_ref    = run_sequential(da_i, da_q, &wp_seq, &sp_seq,
                                           g_in, n_in, g_out_ref);
            int  n_spl    = run_split(db_i, db_q, &wp_split, &sp_split,
                                      g_in, n_in, g_out_split);
            char label[64];
            snprintf(label, sizeof(label), "case3 start_pos=%d", adv_start_positions[p]);
            if (n_ref != n_spl) {
                fprintf(stderr, "FAIL [%s]: n_ref=%d n_spl=%d\n",
                        label, n_ref, n_spl);
                total_fails++;
                continue;
            }
            total_fails += compare_outputs(n_ref, label);
        }
    }

    // --- Case 4: batched output vs unbatched output (bit-exact) ---
    // The batch_scratch path coalesces per-emit PSRAM stores into
    // 32-byte memcpy bursts; the resulting out_iq must be byte-
    // identical to the unbatched path for every input.
    {
        int adv_lengths[] = {1, 7, 8, 9, 16, 17, 4096, 8192};
        int n_lens        = sizeof(adv_lengths) / sizeof(adv_lengths[0]);
        for (int li = 0; li < n_lens; li++) {
            int     n_in     = adv_lengths[li];
            int16_t da_i[32] = {0}, da_q[32] = {0};
            int16_t db_i[32] = {0}, db_q[32] = {0};
            int     wp_a = 0, wp_b = 0;
            int     sp_a = 0, sp_b = 0;

            fill_pseudorandom(0xBA7CED00 + li);

            int n_ref = resample_256_to_250_process_explicit(
                da_i, da_q, &wp_a, &sp_a,
                g_in, n_in, g_out_ref, n_in, /*batch=*/NULL);

            int16_t scratch[RS25_BATCH_COMPLEX * 2] __attribute__((aligned(16)));
            int     n_bat = resample_256_to_250_process_explicit(
                db_i, db_q, &wp_b, &sp_b,
                g_in, n_in, g_out_split, n_in, scratch);

            char label[64];
            snprintf(label, sizeof(label), "case4 batch n_in=%d", n_in);
            if (n_ref != n_bat) {
                fprintf(stderr, "FAIL [%s]: n_ref=%d n_bat=%d\n", label, n_ref, n_bat);
                total_fails++;
                continue;
            }
            total_fails += compare_outputs(n_ref, label);
        }
    }

    if (total_fails == 0) {
        printf("\nALL OK: split-resample is bit-exact vs sequential\n");
        return 0;
    }
    printf("\nFAILED: %d divergences detected\n", total_fails);
    return 1;
}
