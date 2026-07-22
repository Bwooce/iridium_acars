#pragma once
// Pure decision helpers for the band-health re-survey trigger (band_health.c).
// No ESP dependencies so the thresholds and the median/fire logic are
// host-testable (tests/host/test_band_health_core.c).
//
// Model (40 h HydraSDR band-strategy analysis): the hourly IDA (LW.DA) frame
// rate inside the parked ±1.25 MHz window swings naturally with satellite
// geometry / diurnal cycle (SD ~12.7 frames/h observed), so a single bad hour
// must never trigger anything. Staleness is judged on the 7-day TRAILING
// MEDIAN against a frozen COMMISSIONING baseline (median of the first 7 days
// of tracked hours) — a sustained-decline event trigger, deliberately NOT a
// schedule and NOT instantaneous-hotspot chasing.

#include <stdbool.h>
#include <stdint.h>

#define BH_HIST_HOURS 168     // trailing window: 7 days of tracked uptime hours
#define BH_BASELINE_HOURS 168 // commissioning period: first 7 days of tracked hours
#define BH_MIN_EVAL_HOURS 48  // don't judge staleness on < 2 days of history
#define BH_TRIGGER_PCT 60     // fire when trailing median < 60% of baseline
#define BH_COOLDOWN_HOURS 24  // minimum spacing between fires (and after any
                              // integrated survey, however it was launched)

// Median of v[0..n) without modifying v (insertion sort of a local copy —
// n <= BH_HIST_HOURS, runs once per hour, cost is irrelevant). n odd gives
// the true median; n even gives the upper median (fine at this granularity).
static inline uint16_t bh_median_u16(const uint16_t *v, int n)
{
    uint16_t tmp[BH_HIST_HOURS];
    if (!v || n <= 0) return 0;
    if (n > BH_HIST_HOURS) n = BH_HIST_HOURS;
    for (int i = 0; i < n; i++) {
        uint16_t x = v[i];
        int      j = i;
        while (j > 0 && tmp[j - 1] > x) {
            tmp[j] = tmp[j - 1];
            j--;
        }
        tmp[j] = x;
    }
    return tmp[n / 2];
}

// The staleness LEVEL: trailing median below BH_TRIGGER_PCT of the frozen
// baseline. baseline == 0 means "still commissioning" — never stale.
static inline bool bh_below_baseline(uint16_t trailing_median, uint16_t baseline)
{
    if (baseline == 0) return false;
    return (uint32_t)trailing_median * 100u < (uint32_t)baseline * BH_TRIGGER_PCT;
}
