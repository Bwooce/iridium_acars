// Unit test for autotune_gainset.h — the pure R828D gain-step snapping,
// coarse gain-set generation, and inverted-U peak-pick selection used by the
// autotune RF-recalibration routine. Host-testable because the arithmetic is
// factored out of the device autotune.c (no class_driver / FreeRTOS deps).
#include <stdio.h>
#include "autotune_gainset.h"

static int  fails = 0;
static void expect_eq(const char *what, int got, int want)
{
    if (got != want) {
        printf("  FAIL %s: got %d want %d\n", what, got, want);
        fails++;
    }
}

int main(void)
{
    // --- snap_gain: nearest real step -----------------------------------
    expect_eq("snap 0", autotune_snap_gain(0), 0);
    expect_eq("snap exact 254", autotune_snap_gain(254), 254);
    expect_eq("snap 80 -> 77 (closer than 87)", autotune_snap_gain(80), 77);
    expect_eq("snap 83 -> 87 (closer than 77)", autotune_snap_gain(83), 87);
    expect_eq("snap 250 -> 254", autotune_snap_gain(250), 254);
    expect_eq("snap above max -> 496", autotune_snap_gain(600), 496);
    expect_eq("snap negative -> 0", autotune_snap_gain(-50), 0);

    // --- build_gain_set --------------------------------------------------
    int out[64];

    // Full table, stride 1 -> all 29 steps, ascending, first=0 last=496.
    int n = autotune_build_gain_set(0, 496, 1, out, 64);
    expect_eq("full stride1 count", n, AUTOTUNE_R828D_N);
    expect_eq("full first", out[0], 0);
    expect_eq("full last", out[n - 1], 496);

    // Stride 3 over full table: 0,3,6,...,27 (10 samples), then top (28=496)
    // appended because index 27 (480) != 496.
    n = autotune_build_gain_set(0, 496, 3, out, 64);
    expect_eq("stride3 first", out[0], AUTOTUNE_R828D_GAINS[0]);   // 0
    expect_eq("stride3 [1]", out[1], AUTOTUNE_R828D_GAINS[3]);     // 27
    expect_eq("stride3 top appended", out[n - 1], 496);
    // ascending + within range
    int ascending = 1;
    for (int i = 1; i < n; i++)
        if (out[i] <= out[i - 1]) ascending = 0;
    expect_eq("stride3 strictly ascending", ascending, 1);

    // Range clamps to in-band steps only.
    n = autotune_build_gain_set(80, 460, 3, out, 64);
    expect_eq("range first >= 80", out[0], 87);   // 77 excluded, 87 first in-range
    expect_eq("range last <= 460", out[n - 1], 445); // 480/496 excluded; 445 top in-range

    // Out of range -> 0.
    expect_eq("empty range", autotune_build_gain_set(500, 600, 1, out, 64), 0);
    expect_eq("inverted range", autotune_build_gain_set(400, 100, 1, out, 64), 0);

    // cap honoured, no overflow.
    n = autotune_build_gain_set(0, 496, 1, out, 5);
    if (n > 5) { printf("  FAIL cap exceeded: %d\n", n); fails++; }

    // stride < 1 coerced to 1.
    n = autotune_build_gain_set(0, 496, 0, out, 64);
    expect_eq("stride0 -> full", n, AUTOTUNE_R828D_N);

    // --- pick_best: inverted-U argmax, tie -> lower gain -----------------
    int curve1[] = {0, 2, 6, 0, 1}; // matches the 2026-07-08 measured shape
    expect_eq("pick peak idx", autotune_pick_best(curve1, 5), 2);

    int curve2[] = {3, 3, 1}; // tie between idx0/idx1 -> lower gain (idx0)
    expect_eq("pick tie -> lower gain", autotune_pick_best(curve2, 3), 0);

    int curve3[] = {5}; // single
    expect_eq("pick single", autotune_pick_best(curve3, 1), 0);

    expect_eq("pick empty -> -1", autotune_pick_best(curve1, 0), -1);
    expect_eq("pick null -> -1", autotune_pick_best(NULL, 5), -1);

    if (fails == 0) {
        printf("test_autotune_gainset: ALL PASS\n");
        return 0;
    }
    printf("test_autotune_gainset: %d FAILURE(S)\n", fails);
    return 1;
}
