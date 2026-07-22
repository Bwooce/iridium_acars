// Host test: band-health re-survey trigger decision core (band_health_core.h)
// — the median and the below-baseline staleness level. The stateful hourly
// ring / NVS plumbing lives in band_health.c (device-only).
#include <assert.h>
#include <stdint.h>
#include "band_health_core.h"

int main(void)
{
    // Median: odd n = true median, unsorted input.
    uint16_t odd[5] = {40, 10, 30, 50, 20};
    assert(bh_median_u16(odd, 5) == 30);

    // Even n = upper median (documented choice).
    uint16_t even[4] = {10, 20, 30, 40};
    assert(bh_median_u16(even, 4) == 30);

    // Degenerate inputs.
    assert(bh_median_u16(odd, 0) == 0);
    assert(bh_median_u16((const uint16_t *)0, 5) == 0);
    uint16_t one[1] = {7};
    assert(bh_median_u16(one, 1) == 7);

    // Robustness: a single wild hour must not move the median — the reason
    // the trigger judges the 7-day median, not any single bucket.
    uint16_t spiky[7] = {50, 52, 48, 51, 49, 0, 65535};
    assert(bh_median_u16(spiky, 7) == 50);

    // Staleness level: strict "< 60% of baseline" boundary.
    assert(bh_below_baseline(59, 100) == true);   // 59% — stale
    assert(bh_below_baseline(60, 100) == false);  // exactly 60% — not stale
    assert(bh_below_baseline(100, 100) == false); // healthy
    assert(bh_below_baseline(0, 100) == true);    // dead band — stale
    assert(bh_below_baseline(0, 0) == false);     // no baseline yet — never stale
    assert(bh_below_baseline(65535, 0) == false);

    // No uint16 overflow in the percentage compare at extreme values.
    assert(bh_below_baseline(39320, 65535) == true);  // 59.99%
    assert(bh_below_baseline(39321, 65535) == false); // 60.00%

    return 0;
}
