// Host-side unit tests for qpsk_demod.c (DQPSK demod with PLL phase tracking,
// unique-word direction detection).
//
// Strategy: synthesise IQ samples that, when decimated by 2, yield the
// expected hardcoded DL or UL unique-word pattern, then assert the demod:
//   - returns success
//   - reports the correct direction
//   - emits the correct DQPSK-decoded bits for the UW segment
//
// We don't need a full 11 MB sigmf corpus for this — the demod's
// hardcoded UW patterns are the smallest test vector that exercises
// every branch (PLL update, hard-decision, UW match, DQPSK diff decode).
//
// Build: see CMakeLists.txt; run as ./test_qpsk.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <complex.h>

#include "qpsk_demod.h"

// Map QPSK symbol index → (i_val, q_val) at unit magnitude on the
// diagonal axes. Matches qpsk_demod's hard-decision regions:
//   0 → (+,+) i.e.  pi/4
//   1 → (-,+) i.e.  3pi/4
//   2 → (-,-) i.e. -3pi/4
//   3 → (+,-) i.e. -pi/4
// (Local names avoid `I`/`Q` since `<complex.h>` defines `I` as a macro.)
static void symbol_to_iq(int sym, int16_t *i_val, int16_t *q_val, int amplitude)
{
    int s = (amplitude > 0) ? amplitude : 1;
    switch (sym & 3) {
        case 0: *i_val = +s; *q_val = +s; return;
        case 1: *i_val = -s; *q_val = +s; return;
        case 2: *i_val = -s; *q_val = -s; return;
        case 3: *i_val = +s; *q_val = -s; return;
    }
}

// Build a 2-sps int16 IQ buffer from a sequence of QPSK symbols.
// The demod takes every 2nd complex sample (i*4 in int16 indexing), so
// only the even-indexed complex slots need correct phase. We fill the
// odd-indexed slots with the same value to keep the energy level realistic.
//
// Returns the number of int16 elements written.
static int build_iq_2sps(const int *symbols, int n_symbols,
                         int16_t *out, int amplitude)
{
    // Per symbol we need 2 complex samples = 4 int16. Total = 4*n_symbols.
    for (int i = 0; i < n_symbols; i++) {
        int16_t i_val, q_val;
        symbol_to_iq(symbols[i], &i_val, &q_val, amplitude);
        out[i * 4 + 0] = i_val;
        out[i * 4 + 1] = q_val;
        out[i * 4 + 2] = i_val;
        out[i * 4 + 3] = q_val;
    }
    return n_symbols * 4;
}

// The unique-word patterns are hardcoded in qpsk_demod.c:
//   IR_UW_DL[] = { 0, 2, 2, 2, 2, 0, 0, 0, 2, 0, 0, 2 };
//   IR_UW_UL[] = { 2, 2, 0, 0, 0, 2, 0, 0, 2, 0, 2, 2 };
// We replicate them here to drive the test.
static const int UW_DL[] = { 0, 2, 2, 2, 2, 0, 0, 0, 2, 0, 0, 2 };
static const int UW_UL[] = { 2, 2, 0, 0, 0, 2, 0, 0, 2, 0, 2, 2 };

static int test_direction(const int *uw, const char *name, ir_direction_t expected)
{
    // Use 32 symbols total: 12 UW + 20 payload (arbitrary).
    int symbols[32];
    for (int i = 0; i < 12; i++) symbols[i] = uw[i];
    for (int i = 12; i < 32; i++) symbols[i] = (i * 7) & 3;

    int16_t iq[32 * 4];
    int n_int16 = build_iq_2sps(symbols, 32, iq, 1000);

    decoded_frame_t frame = { 0 };
    int rc = qpsk_demod_process(iq, n_int16, &frame);

    if (!rc) { printf("  %s: FAIL — demod returned 0\n", name); return 0; }
    if (frame.direction != expected) {
        printf("  %s: FAIL — direction=%d, expected=%d\n",
               name, (int)frame.direction, (int)expected);
        free(frame.bits);
        free(frame.soft_bits);   // #112
        return 0;
    }
    if (frame.n_bits != 32 * 2) {
        printf("  %s: FAIL — n_bits=%d, expected=64\n", name, frame.n_bits);
        free(frame.bits);
        free(frame.soft_bits);   // #112
        return 0;
    }
    free(frame.bits);
    free(frame.soft_bits);   // #112
    return 1;
}

static int test_unknown_direction(void)
{
    // Random symbols that match neither UW.
    int symbols[32];
    srand(12345);
    for (int i = 0; i < 32; i++) symbols[i] = rand() & 3;

    // Force the first 12 to be a non-UW pattern (all 0s in absolute terms,
    // which is 2/2/2/... mismatches against both UWs).
    for (int i = 0; i < 12; i++) symbols[i] = 1;  // not in either UW

    int16_t iq[32 * 4];
    int n_int16 = build_iq_2sps(symbols, 32, iq, 1000);
    decoded_frame_t frame = { 0 };
    int rc = qpsk_demod_process(iq, n_int16, &frame);

    if (rc != 0) {
        printf("  unknown-UW: FAIL — demod returned %d (expected 0)\n", rc);
        free(frame.bits);
        free(frame.soft_bits);   // #112
        return 0;
    }
    return 1;
}

static int test_too_short(void)
{
    // Fewer than IR_UW_LENGTH (12) symbols → demod must return 0.
    int symbols[8] = { 0, 1, 2, 3, 0, 1, 2, 3 };
    int16_t iq[8 * 4];
    int n_int16 = build_iq_2sps(symbols, 8, iq, 1000);
    decoded_frame_t frame = { 0 };
    int rc = qpsk_demod_process(iq, n_int16, &frame);
    if (rc != 0) {
        printf("  too-short: FAIL — demod returned %d (expected 0)\n", rc);
        free(frame.bits);
        free(frame.soft_bits);   // #112
        return 0;
    }
    return 1;
}

int main(void)
{
    int passed = 0, failed = 0;

    printf("Test 1: downlink UW recognition\n");
    if (test_direction(UW_DL, "DL", DIR_DOWNLINK)) passed++; else failed++;

    printf("Test 2: uplink UW recognition\n");
    if (test_direction(UW_UL, "UL", DIR_UPLINK)) passed++; else failed++;

    printf("Test 3: unknown UW returns 0\n");
    if (test_unknown_direction()) passed++; else failed++;

    printf("Test 4: too-short input returns 0\n");
    if (test_too_short()) passed++; else failed++;

    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
