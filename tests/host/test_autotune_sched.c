// Unit test for autotune_sched.h's autotune_is_due() -- the pure "has this
// interval elapsed" arithmetic behind the boot-time + periodic autotune
// auto-run (2026-07-08 boot/periodic extension). Host-testable because the
// due-check is factored out of autotune_sched.c (no FreeRTOS/esp_timer
// deps), mirroring autotune_gainset.h.
#include <stdio.h>
#include <stdbool.h>
#include "autotune_sched.h"

static int  fails = 0;
static void expect(const char *what, bool got, bool want)
{
    if (got != want) {
        printf("  FAIL %s: got %d want %d\n", what, (int)got, (int)want);
        fails++;
    }
}

int main(void)
{
    // interval==0 -> disabled, never due, regardless of elapsed time.
    expect("disabled at t=0", autotune_is_due(0, 0, 0), false);
    expect("disabled at huge elapsed", autotune_is_due(0, 1000000, 0), false);

    // Not yet due: elapsed < interval.
    expect("not due, just started", autotune_is_due(1000, 1000, 3600), false);
    expect("not due, partway", autotune_is_due(1000, 1000 + 3599, 3600), false);

    // Exactly due at the boundary (>=, not >).
    expect("due at exact boundary", autotune_is_due(1000, 1000 + 3600, 3600), true);

    // Overdue (elapsed well past interval) is still due.
    expect("overdue", autotune_is_due(1000, 1000 + 7200, 3600), true);

    // Negative elapsed (clock skew / bad inputs) must not underflow into an
    // immediate fire -- treated as not due.
    expect("negative elapsed -> not due", autotune_is_due(2000, 1000, 3600), false);

    // Small interval sanity check (e.g. a test/dev override).
    expect("small interval due", autotune_is_due(0, 5, 5), true);
    expect("small interval not due", autotune_is_due(0, 4, 5), false);

    if (fails == 0) {
        printf("test_autotune_sched: ALL PASS\n");
        return 0;
    }
    printf("test_autotune_sched: %d FAILURE(S)\n", fails);
    return 1;
}
