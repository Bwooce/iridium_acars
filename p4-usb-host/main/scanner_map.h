#pragma once
#include <stdint.h>

#define SCANNER_MAX_POSITIONS 16

// Iridium ring-alert (IRA) simplex sub-band (~1626 MHz) is IDA-barren but
// burst-DENSE: an all-burst density rank once wandered the parked LO up there
// (IRA ≈ dense, IDA ≈ 0), which is why the old hourly rescan was judged
// harmful. Down-weight the RANK of any center whose 2.5 MHz window sits in the
// simplex region so raw density can't pick it over a real IDA center. This is a
// rank-only bias — the recorded measurements are unchanged, and a center that
// genuinely out-densities the IDA hump by more than 1/SCANNER_IRA_RANK_WEIGHT
// still wins (so a decode-bearing simplex-edge center is not vetoed outright).
// A center at 1624 MHz still hears down to 1622.75 MHz via its −1.25 MHz window,
// so the real IDA hump (~1620.6 MHz) is nowhere near this cutoff.
#define SCANNER_IRA_REGION_LO_HZ 1624000000u
#define SCANNER_IRA_RANK_WEIGHT 0.25f

typedef struct {
    uint32_t center_hz;
    uint32_t narrowband_bursts;
    uint32_t all_bursts;
    float    mean_snr_db;
    uint32_t dwell_ms;
} scanner_pos_t;

int   scanner_enumerate_centers(uint32_t start_hz, uint32_t stop_hz,
                                uint32_t step_hz, uint32_t *out, int max);
float scanner_pos_narrowband_rate(const scanner_pos_t *p);
int   scanner_rank_hottest(const scanner_pos_t *pos, int n);

// Fold one sweep's measurement at a center into a running accumulator for the
// same center: burst counts and dwell_ms sum, so scanner_pos_narrowband_rate()
// on the accumulator is the mean rate over ALL accumulated dwell (a failed hop
// contributes 0 bursts / 0 ms and doesn't skew it). mean_snr_db becomes the
// burst-weighted mean. center_hz is left untouched (caller keys by center).
void  scanner_pos_accumulate(scanner_pos_t *acc, const scanner_pos_t *p);
