// Unit test for worker_dcfine_index() — the pure bin-index math for the
// fine near-DC diagnostic. Kept in worker_core1.h as a static inline so it
// is testable on host without pulling in the device worker.
#include <stdio.h>
#include "worker_dcfine.h"

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
    // DC (0 Hz offset) → centre bucket = WORKER_DCFINE_HALF.
    expect_eq("rel=0", worker_dcfine_index(0.0f), WORKER_DCFINE_HALF);
    // +1 bin (≈+1220.7 Hz) → centre+1.
    expect_eq("rel=+1220.7", worker_dcfine_index(1220.703f), WORKER_DCFINE_HALF + 1);
    // -1 bin → centre-1.
    expect_eq("rel=-1220.7", worker_dcfine_index(-1220.703f), WORKER_DCFINE_HALF - 1);
    // The observed artifact region ~ -100 kHz → -82 bins → centre-82.
    expect_eq("rel=-100k", worker_dcfine_index(-100000.0f), WORKER_DCFINE_HALF - 82);
    // Just inside the low edge: -191 bins is valid (index 1).
    expect_eq("rel=-191bins", worker_dcfine_index(-191.0f * 1220.703f), 1);
    // Outside the window (beyond -192 bins) → -1 (ignored).
    expect_eq("rel=-300k", worker_dcfine_index(-300000.0f), -1);
    // Outside the window (beyond +192 bins) → -1 (ignored).
    expect_eq("rel=+300k", worker_dcfine_index(300000.0f), -1);

    if (fails) {
        printf("test_dcfine_index: %d FAILURES\n", fails);
        return 1;
    }
    printf("test_dcfine_index: PASS\n");
    return 0;
}
