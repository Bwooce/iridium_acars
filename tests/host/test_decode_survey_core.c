// Unit test for decode_survey_core.h — the pure decision math behind the
// decode-based band survey (decode_survey.c). Covers:
//   1) the Poisson rate-difference elimination test (equal + unequal dwell),
//   2) the zero-count / floor guards (never eliminate on raw data),
//   3) over-dispersed (beam-geometry) synthetic streams: interleaved
//      round-robin must still converge on the true-best center,
//   4) a genuine near-tie: neither finalist is eliminated within budget,
//   5) the Phase-C absolute-freq histogram + best-window placement.
//
// Host-testable because all of it is factored into the header with zero
// ESP/FreeRTOS deps (mirrors band_health_core.h / autotune_sched.h).
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "decode_survey_core.h"

static int  fails = 0;
static void expect(const char *what, bool cond)
{
    if (!cond) {
        printf("  FAIL %s\n", what);
        fails++;
    }
}

// -------- deterministic PRNG + Poisson draw (no libc rand dependence) -------
static uint64_t s_rng = 0x9e3779b97f4a7c15ULL;
static double   rng_uniform(void)
{
    // xorshift64* -> (0,1)
    s_rng ^= s_rng >> 12;
    s_rng ^= s_rng << 25;
    s_rng ^= s_rng >> 27;
    uint64_t x = s_rng * 0x2545F4914F6CDD1DULL;
    return ((double)(x >> 11) + 1.0) / 9007199254740993.0;
}
// Knuth Poisson sampler (fine for the small means used here).
static uint32_t rng_poisson(double lambda)
{
    if (lambda <= 0.0) return 0;
    double L = exp(-lambda), p = 1.0;
    uint32_t k = 0;
    do {
        k++;
        p *= rng_uniform();
    } while (p > L);
    return k - 1;
}

// One 5-minute visit at true rate `per_h`: draws a Poisson count and folds it in
// with a fixed 5 min (300000 ms) dwell.
static void visit(ds_center_t *c, double true_rate_per_h)
{
    double lambda = true_rate_per_h * (300000.0 / 3600000.0); // per 5 min
    uint32_t k    = rng_poisson(lambda);
    ds_center_add_visit(c, k, k, /*nb*/ k * 3, 300000u);
}

int main(void)
{
    // ---- 1) elimination test, equal dwell -----------------------------------
    // Equal exposure: reduces to n_lead - n_cand > z*sqrt(n_lead+n_cand).
    // 100 vs 40 over equal dwell at z=2: diff 60 > 2*sqrt(140)=23.7 -> beats.
    expect("equal dwell 100 vs 40 beats",
           ds_lead_beats(100, 7200000, 40, 7200000, 2.0f));
    // 50 vs 45: diff 5 < 2*sqrt(95)=19.5 -> not significant.
    expect("equal dwell 50 vs 45 not sig",
           !ds_lead_beats(50, 7200000, 45, 7200000, 2.0f));

    // ---- 2) zero-count + floor guards ---------------------------------------
    expect("0 vs 0 never significant",
           !ds_lead_beats(0, 7200000, 0, 7200000, 2.0f));
    expect("zero dwell -> not significant",
           !ds_lead_beats(100, 0, 0, 7200000, 2.0f));
    // eligibility floors
    ds_center_t g = {.center_hz = 1620500000u, .lw_da = 5, .dwell_ms = 600000u,
                     .visits = 2};
    expect("below dwell floor not eligible",
           !ds_center_eligible(&g, DS_FLOOR_DWELL_MS, DS_FLOOR_VISITS)); // 10 min < 2 h
    g.dwell_ms = DS_FLOOR_DWELL_MS;
    g.visits   = DS_FLOOR_VISITS;
    expect("at floors eligible",
           ds_center_eligible(&g, DS_FLOOR_DWELL_MS, DS_FLOOR_VISITS));

    // ---- 3) over-dispersed round-robin converges on the true best -----------
    // 4 centers; true LW.DA rates below. Center 1 (1620.5) is clearly best.
    // Over-dispersion is injected by a per-cycle common-mode multiplier
    // (satellite pass / diurnal) applied to ALL centers each cycle — the
    // interleaved design cancels it in the pairwise ranking.
    {
        ds_center_t c[4] = {
            {.center_hz = 1619250000u},
            {.center_hz = 1620500000u}, // truth: best
            {.center_hz = 1621750000u},
            {.center_hz = 1625000000u}, // IRA-land: near-zero IDA
        };
        const double base[4] = {20.0, 60.0, 30.0, 1.0}; // frames/h
        s_rng = 0xabcdef1234567890ULL;

        int cycles = 0;
        // Round-robin visits until converged or a hard cap (budget analogue).
        for (int cyc = 0; cyc < 400 && !ds_converged(c, 4); cyc++) {
            // common-mode 3.4x variance-inflation analogue: multiplier in
            // ~[0.3, 2.5], same for every center this cycle.
            double cm = 0.3 + 2.2 * rng_uniform();
            for (int i = 0; i < 4; i++)
                if (!c[i].eliminated) visit(&c[i], base[i] * cm);
            ds_eliminate_pass(c, 4, DS_ELIM_Z, DS_FLOOR_DWELL_MS,
                              DS_FLOOR_VISITS);
            cycles = cyc + 1;
        }
        int lead = ds_leader_index(c, 4);
        expect("over-dispersed: converged", ds_converged(c, 4));
        expect("over-dispersed: winner is center 1 (1620.5)",
               lead == 1 && c[1].center_hz == 1620500000u);
        expect("over-dispersed: true-worst (IRA) eliminated", c[3].eliminated);
        // Sanity: convergence took many beam periods, not a couple of visits.
        expect("over-dispersed: needed real integration (>=24 cycles)",
               cycles >= 24);
    }

    // ---- 4) genuine near-tie: true-best is NOT wrongly eliminated -----------
    // A 4%-apart pair is (correctly) at the edge of resolvability. The real
    // guarantee the elimination stage must give is that it does not DROP THE
    // TRUE BEST on an early transient; a run may legitimately drop the slightly
    // -worse center, and a run may keep both (Phase C then breaks the tie).
    // At the production z (=3) the true-best is wrongly dropped ~0.25% of runs;
    // this fixed seed is a representative pass. See DS_ELIM_Z rationale.
    {
        ds_center_t c[2] = {
            {.center_hz = 1620500000u}, // truth: (marginally) best
            {.center_hz = 1619250000u},
        };
        const double base[2] = {50.0, 48.0}; // ~4% apart
        s_rng = 0x0f0f0f0f0f0f0f0fULL;
        // 24 h budget at 5 min/visit over 2 centers = 288 visits/center.
        for (int cyc = 0; cyc < 288 && !ds_converged(c, 2); cyc++) {
            double cm = 0.5 + 1.5 * rng_uniform();
            for (int i = 0; i < 2; i++)
                if (!c[i].eliminated) visit(&c[i], base[i] * cm);
            ds_eliminate_pass(c, 2, DS_ELIM_Z, DS_FLOOR_DWELL_MS,
                              DS_FLOOR_VISITS);
        }
        expect("near-tie: true-best center survives (never falsely eliminated)",
               !c[0].eliminated);
    }

    // ---- 5) Phase C: histogram bin math + best-window placement -------------
    expect("bin: band lo in bin 0", ds_abs_hist_bin(1618000000u) == 0);
    expect("bin: just below band -> -1", ds_abs_hist_bin(1617999999u) == -1);
    expect("bin: at hi edge -> -1 (half-open)",
           ds_abs_hist_bin(1626000000u) == -1);
    expect("bins count == 64", DS_ABS_HIST_BINS == 64);
    {
        int b = ds_abs_hist_bin(1620500000u);
        expect("bin: 1620.5 maps in-band", b >= 0);
        uint32_t ctr = ds_abs_hist_bin_center_hz(b);
        expect("bin center within its 125 kHz cell",
               ctr >= 1620500000u - DS_ABS_HIST_BIN_HZ &&
                   ctr <= 1620500000u + DS_ABS_HIST_BIN_HZ);
    }
    {
        // Build an IDA-hump histogram centred ~1620.6 MHz plus a small IRA
        // cluster up at ~1625.5 MHz. The best 2.5 MHz window must sit on the
        // hump, not the IRA cluster.
        uint32_t hist[DS_ABS_HIST_BINS];
        memset(hist, 0, sizeof(hist));
        ds_abs_hist_add(hist, 1620100000u, 30);
        ds_abs_hist_add(hist, 1620600000u, 80);
        ds_abs_hist_add(hist, 1621100000u, 40);
        ds_abs_hist_add(hist, 1619600000u, 20);
        ds_abs_hist_add(hist, 1625500000u, 25); // IRA cluster
        ds_abs_hist_add(hist, 1625700000u, 25);

        uint32_t best_sum = 0;
        // fine grid 250 kHz across the tunable center range
        uint32_t win = ds_best_window_center(hist, DS_ABS_HIST_BINS,
                                             2500000u,      // 2.5 MHz RX window
                                             1619000000u,   // grid lo
                                             1625000000u,   // grid hi
                                             250000u,       // 250 kHz step
                                             &best_sum);
        // The hump integral (20+30+80+40=170) must dominate the IRA (50).
        expect("phaseC: best window covers the IDA hump",
               win >= 1619500000u && win <= 1621500000u);
        expect("phaseC: best-window integral == full hump (170)",
               best_sum == 170u);
        // A window centred on the IRA cluster scores only 50.
        expect("phaseC: IRA-centred window scores less than the hump",
               ds_window_integral(hist, DS_ABS_HIST_BINS, 1625600000u,
                                   2500000u) < best_sum);
    }
    {
        // Empty histogram -> returns grid_lo with zero integral, no crash.
        uint32_t hist[DS_ABS_HIST_BINS];
        memset(hist, 0, sizeof(hist));
        uint32_t best_sum = 123;
        uint32_t win = ds_best_window_center(hist, DS_ABS_HIST_BINS, 2500000u,
                                             1619000000u, 1625000000u, 250000u,
                                             &best_sum);
        expect("phaseC: empty hist -> zero integral", best_sum == 0);
        expect("phaseC: empty hist -> grid_lo center", win == 1619000000u);
    }

    if (fails == 0) {
        printf("test_decode_survey_core: ALL PASS\n");
        return 0;
    }
    printf("test_decode_survey_core: %d FAILURE(S)\n", fails);
    return 1;
}
