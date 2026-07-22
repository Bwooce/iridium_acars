#include <assert.h>
#include <stdint.h>
#include "scanner_map.h"

int main(void)
{
    uint32_t c[SCANNER_MAX_POSITIONS];

    // 1616..1626 MHz step 2.5 MHz -> 5 centers (1616,1618.5,1621,1623.5,1626).
    int n = scanner_enumerate_centers(1616000000u, 1626000000u, 2500000u, c, SCANNER_MAX_POSITIONS);
    assert(n == 5);
    assert(c[0] == 1616000000u);
    assert(c[4] == 1626000000u);

    // Degenerate inputs.
    assert(scanner_enumerate_centers(1626000000u, 1616000000u, 2500000u, c, 16) == 0);
    assert(scanner_enumerate_centers(1616000000u, 1626000000u, 0u, c, 16) == 0);

    // Rate + ranking.
    scanner_pos_t p[3] = {
        {1616000000u, 4, 10, 12.0f, 2000}, // 2.0/s
        {1618500000u, 9, 20, 14.0f, 2000}, // 4.5/s  <-- hottest
        {1621000000u, 0, 30, 0.0f, 2000},  // 0/s
    };
    assert(scanner_pos_narrowband_rate(&p[0]) == 2.0f);
    assert(scanner_pos_narrowband_rate(&p[1]) == 4.5f);
    assert(scanner_rank_hottest(p, 3) == 1);
    assert(scanner_rank_hottest(p, 0) == -1);

    // Edge case: dwell_ms == 0 must yield rate 0.0 (div-by-zero guard).
    scanner_pos_t zdw = {1616000000u, 5, 10, 12.0f, 0};
    assert(scanner_pos_narrowband_rate(&zdw) == 0.0f);

    // Edge case: tie on narrowband rate must return the LOWEST index.
    scanner_pos_t tie[2] = {
        {1616000000u, 4, 10, 12.0f, 2000}, // 2.0/s
        {1618500000u, 4, 10, 12.0f, 2000}, // 2.0/s (tie)
    };
    assert(scanner_rank_hottest(tie, 2) == 0);

    // IRA-region penalty scoping: a dense center in the ring-alert simplex
    // sub-band (>= SCANNER_IRA_REGION_LO_HZ) vs a less-dense IDA-band center.
    // The PENALISED ranker (density-only paths) must prefer the IDA-band center
    // (the IRA flood is de-prioritised so the park can't wander to 1626 MHz).
    // The RAW ranker (decode-survey shortlist) must prefer the truly densest —
    // decode arbitrates, so a legitimate upper-IDA-tail center must not be
    // heuristically excluded before it is measured.
    scanner_pos_t ira[2] = {
        {1620500000u, 6, 10, 12.0f, 2000},           // IDA band, 3.0 nb/s
        {SCANNER_IRA_REGION_LO_HZ, 16, 40, 8.0f, 2000}, // IRA region, 8.0 nb/s raw
    };
    // Penalised: 8.0 * 0.25 = 2.0 < 3.0 -> IDA-band center (index 0) wins.
    assert(scanner_rank_hottest(ira, 2) == 0);
    // Raw: 8.0 > 3.0 -> densest (index 1) wins, unpenalised.
    assert(scanner_rank_hottest_raw(ira, 2) == 1);

    // Edge case: scanner_enumerate_centers must clamp output to max when natural count exceeds it.
    uint32_t clamp[3];
    assert(scanner_enumerate_centers(0u, 100u, 10u, clamp, 3) == 3); // natural count 11, clamped to 3

    // Multi-sweep accumulation (integrated survey): counts and dwell sum, so
    // the rate becomes the mean over all accumulated dwell.
    scanner_pos_t acc = {1620500000u, 0, 0, 0.0f, 0};
    scanner_pos_t s1  = {1620500000u, 10, 20, 12.0f, 2000}; // 5.0 nb/s
    scanner_pos_t s2  = {1620500000u, 2, 60, 18.0f, 2000};  // 1.0 nb/s
    scanner_pos_accumulate(&acc, &s1);
    assert(acc.narrowband_bursts == 10 && acc.all_bursts == 20 && acc.dwell_ms == 2000);
    assert(acc.mean_snr_db == 12.0f); // first sweep dominates an empty accumulator
    scanner_pos_accumulate(&acc, &s2);
    assert(acc.narrowband_bursts == 12 && acc.all_bursts == 80 && acc.dwell_ms == 4000);
    assert(scanner_pos_narrowband_rate(&acc) == 3.0f); // 12 bursts / 4 s = mean of 5 and 1
    // Burst-weighted SNR: (12*20 + 18*60) / 80 = 16.5 dB.
    assert(acc.mean_snr_db > 16.49f && acc.mean_snr_db < 16.51f);

    // A failed-hop sweep entry (all zeros, dwell 0) must not perturb the accumulator.
    scanner_pos_t failed = {1620500000u, 0, 0, 0.0f, 0};
    scanner_pos_accumulate(&acc, &failed);
    assert(acc.narrowband_bursts == 12 && acc.all_bursts == 80 && acc.dwell_ms == 4000);
    assert(acc.mean_snr_db > 16.49f && acc.mean_snr_db < 16.51f);

    return 0;
}
