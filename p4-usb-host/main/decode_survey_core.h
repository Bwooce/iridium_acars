#pragma once
// Pure decision math for the decode-based band survey (decode_survey.c).
// No ESP / FreeRTOS deps so the elimination test, the scheduler bookkeeping
// and the Phase-C histogram-window integral are host-testable
// (tests/host/test_decode_survey_core.c). Mirrors band_health_core.h's
// "pure helpers in a header, task glue in the .c" split.
//
// Model (40 h HydraSDR band-strategy analysis, memory reference_freq_coverage
// _analysis): the instantaneous LW.DA (IDA/ACARS-bearing) decode hotspot rides
// Iridium's ~1 min spot beams; only the HOURS-average is stationary. So a
// decode survey must (a) integrate each candidate center over MANY beam periods
// and (b) revisit centers round-robin so beam/diurnal common-mode cancels in the
// pairwise ranking. This header owns the arithmetic that decides, from
// Poisson-sparse per-center counts, when one center is provably worse than the
// leader and may be dropped, and where inside the winning region to place the
// final 2.5 MHz window.

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

// Decision defaults, shared by the device task (decode_survey.c) and the host
// test so both reference the same numbers.
//
// DS_ELIM_Z = 3.0, not the textbook 2.0, ON PURPOSE: the round-robin re-tests
// every center against the leader on every cycle and elimination is PERMANENT,
// so we "peek" hundreds of times — classic multiple-comparisons inflation. A
// Monte-Carlo sweep (2000 trials, a 4% true near-tie, 288 visits ≈ 24 h budget)
// showed the true-best center is wrongly eliminated 4.0% of the time at z=2 but
// only 0.25% at z=3, while genuine gaps (e.g. 60 vs 30 frames/h) still resolve
// 100% of the time. z=3 buys peeking-robustness at negligible cost to real
// discrimination. A 4%-apart pair mostly runs to budget (Phase C breaks it) or
// drops the genuinely-worse member — both acceptable.
#define DS_ELIM_Z 3.0f
// A center may only be eliminated after clearing BOTH floors — enough dwell for
// the Poisson SE to mean something and enough wall-clock revisits to average
// beam geometry (~1 min beam period; 24 visits × 5 min ≈ 2 h spans many).
#define DS_FLOOR_DWELL_MS (2ULL * 3600ULL * 1000ULL) // 2 h
#define DS_FLOOR_VISITS   24u

// ---------------------------------------------------------------------------
// Per-center accumulator (survey working state, one per shortlisted center).
// ---------------------------------------------------------------------------
typedef struct {
    uint32_t center_hz;
    uint32_t lw_da;       // accumulated LW.DA classifications over the survey
    uint32_t lw_da_valid; // subset that passed ida_decode (diagnostic only)
    uint32_t nb_bursts;   // accumulated narrowband density (diagnostic only)
    uint64_t dwell_ms;    // summed dwell actually spent parked here
    uint32_t visits;      // number of round-robin visits folded in
    bool     eliminated;  // dropped from the round-robin
} ds_center_t;

// Fold one round-robin visit's DELTAS into a center's accumulator. Deltas are
// computed by the caller as (counter_after − counter_before) around the dwell,
// with the ~2-3 s attribution guard applied before the "before" snapshot so a
// frame captured pre-hop but classified post-hop isn't mis-credited.
static inline void ds_center_add_visit(ds_center_t *c, uint32_t lw_da_delta,
                                       uint32_t valid_delta, uint32_t nb_delta,
                                       uint64_t dwell_ms_delta)
{
    if (!c) return;
    c->lw_da += lw_da_delta;
    c->lw_da_valid += valid_delta;
    c->nb_bursts += nb_delta;
    c->dwell_ms += dwell_ms_delta;
    c->visits++;
}

// LW.DA decode rate in frames/hour over the accumulated dwell (0 if no dwell).
static inline float ds_center_rate_per_h(const ds_center_t *c)
{
    if (!c || c->dwell_ms == 0) return 0.0f;
    return (float)c->lw_da * 3600000.0f / (float)c->dwell_ms;
}

// ---------------------------------------------------------------------------
// Elimination test (Poisson rate-difference, normal approx).
// ---------------------------------------------------------------------------
// Two centers, each an independent Poisson process observed over its own
// exposure (dwell). rate = n / t; Var(rate) ≈ n / t² (Poisson). The difference
// of two independent rate estimates is ~Normal with SE = sqrt(n1/t1² + n2/t2²).
// `lead` is "significantly better" than `cand` when
//     rate(lead) − rate(cand) > z · SE.
// A +0.5 count floor in the variance keeps SE finite when a center has zero
// decodes yet (so a 0-vs-0 pair is never called significant — SE > 0, diff = 0).
// Interleaved round-robin dwell makes the two exposures nearly equal and their
// beam/diurnal common-mode cancels, so this reduces (for equal t) to the
// familiar n1 − n2 > z·sqrt(n1 + n2) the design quoted.
static inline bool ds_lead_beats(uint32_t lead_cnt, uint64_t lead_dwell_ms,
                                 uint32_t cand_cnt, uint64_t cand_dwell_ms,
                                 float z)
{
    if (lead_dwell_ms == 0 || cand_dwell_ms == 0) return false;
    double tl = (double)lead_dwell_ms / 3600000.0; // hours (units cancel; keeps magnitudes sane)
    double tc = (double)cand_dwell_ms / 3600000.0;
    double rl = (double)lead_cnt / tl;
    double rc = (double)cand_cnt / tc;
    double var = ((double)lead_cnt + 0.5) / (tl * tl) +
                 ((double)cand_cnt + 0.5) / (tc * tc);
    double se = sqrt(var);
    return (rl - rc) > (double)z * se;
}

// Index of the current leader (highest LW.DA rate) among non-eliminated
// centers, or −1 if none remain. Ties resolve to the lowest index (stable).
static inline int ds_leader_index(const ds_center_t *c, int n)
{
    int   best = -1;
    float best_rate = -1.0f;
    for (int i = 0; i < n; i++) {
        if (c[i].eliminated) continue;
        float r = ds_center_rate_per_h(&c[i]);
        if (r > best_rate) {
            best_rate = r;
            best      = i;
        }
    }
    return best;
}

// Count of centers still in the running.
static inline int ds_alive_count(const ds_center_t *c, int n)
{
    int a = 0;
    for (int i = 0; i < n; i++)
        if (!c[i].eliminated) a++;
    return a;
}

// A center is only eligible for elimination once it has cleared BOTH floors
// (enough wall-time revisits to average beam geometry, enough dwell to make the
// Poisson SE meaningful). Below the floors the counts are too raw to trust.
static inline bool ds_center_eligible(const ds_center_t *c,
                                      uint64_t floor_dwell_ms,
                                      uint32_t floor_visits)
{
    return c && !c->eliminated && c->dwell_ms >= floor_dwell_ms &&
           c->visits >= floor_visits;
}

// Run one elimination pass: drop every eligible candidate the leader provably
// beats at z sigma. The leader itself and any center below the floors are kept.
// Returns the number of centers eliminated this pass. Never eliminates the last
// survivor (leader), so ds_alive_count stays >= 1.
static inline int ds_eliminate_pass(ds_center_t *c, int n, float z,
                                    uint64_t floor_dwell_ms,
                                    uint32_t floor_visits)
{
    int lead = ds_leader_index(c, n);
    if (lead < 0) return 0;
    int dropped = 0;
    for (int i = 0; i < n; i++) {
        if (i == lead || c[i].eliminated) continue;
        if (!ds_center_eligible(&c[i], floor_dwell_ms, floor_visits)) continue;
        if (ds_lead_beats(c[lead].lw_da, c[lead].dwell_ms, c[i].lw_da,
                          c[i].dwell_ms, z)) {
            c[i].eliminated = true;
            dropped++;
        }
    }
    return dropped;
}

// The survey has converged when only one candidate remains. (The caller also
// stops on a wall-clock budget, which is what ends a genuine near-tie.)
static inline bool ds_converged(const ds_center_t *c, int n)
{
    return ds_alive_count(c, n) <= 1;
}

// ---------------------------------------------------------------------------
// Phase C: absolute-frequency LW.DA histogram + best 2.5 MHz window placement.
// ---------------------------------------------------------------------------
// The per-visit LW.DA rel-freq histogram (frame_decoder, keyed by detector bin)
// is folded — at the visit's KNOWN LO — into this LO-independent absolute-freq
// histogram spanning the survey band. Final placement maximises the decode
// integral of the 2.5 MHz RX window over a fine candidate grid, so precision
// comes from the histogram centroid, not from center-vs-center dwell wars.
#define DS_ABS_HIST_LO_HZ  1618000000u
#define DS_ABS_HIST_HI_HZ  1626000000u
#define DS_ABS_HIST_BIN_HZ 125000u // 125 kHz resolution
#define DS_ABS_HIST_BINS \
    ((int)((DS_ABS_HIST_HI_HZ - DS_ABS_HIST_LO_HZ) / DS_ABS_HIST_BIN_HZ)) // 64

// Absolute-freq bin for a frequency, or −1 if outside the histogram band.
static inline int ds_abs_hist_bin(uint32_t abs_hz)
{
    if (abs_hz < DS_ABS_HIST_LO_HZ || abs_hz >= DS_ABS_HIST_HI_HZ) return -1;
    return (int)((abs_hz - DS_ABS_HIST_LO_HZ) / DS_ABS_HIST_BIN_HZ);
}

// Center frequency (Hz) of an absolute-freq bin.
static inline uint32_t ds_abs_hist_bin_center_hz(int bin)
{
    return DS_ABS_HIST_LO_HZ + (uint32_t)bin * DS_ABS_HIST_BIN_HZ +
           DS_ABS_HIST_BIN_HZ / 2u;
}

// Add `count` LW.DA decodes observed at absolute frequency abs_hz into hist.
// Out-of-band frequencies are dropped (window overlap can push a detector bin a
// little past the band edge; it simply doesn't count).
static inline void ds_abs_hist_add(uint32_t *hist, uint32_t abs_hz,
                                   uint32_t count)
{
    int b = ds_abs_hist_bin(abs_hz);
    if (b >= 0) hist[b] += count;
}

// Integral of the RX window centred at cand_center_hz over the histogram:
// sum of every bin whose center falls within ±rx_bw_hz/2 of the candidate.
static inline uint32_t ds_window_integral(const uint32_t *hist, int nbins,
                                          uint32_t cand_center_hz,
                                          uint32_t rx_bw_hz)
{
    uint32_t half = rx_bw_hz / 2u;
    uint32_t lo   = (cand_center_hz > half) ? cand_center_hz - half : 0u;
    uint32_t hi   = cand_center_hz + half;
    uint32_t sum  = 0;
    for (int b = 0; b < nbins; b++) {
        uint32_t f = ds_abs_hist_bin_center_hz(b);
        if (f >= lo && f <= hi) sum += hist[b];
    }
    return sum;
}

// Slide a 2.5 MHz window over a fine candidate grid [grid_lo, grid_hi] stepping
// by grid_step_hz; return the center that maximises the LW.DA integral. Writes
// the winning integral to *out_best_sum if non-NULL. Ties keep the lowest
// (leftmost) center. Returns 0 with *out_best_sum=0 if the histogram is empty.
static inline uint32_t ds_best_window_center(const uint32_t *hist, int nbins,
                                             uint32_t rx_bw_hz,
                                             uint32_t grid_lo_hz,
                                             uint32_t grid_hi_hz,
                                             uint32_t grid_step_hz,
                                             uint32_t *out_best_sum)
{
    uint32_t best_center = grid_lo_hz;
    uint32_t best_sum    = 0;
    if (grid_step_hz == 0 || grid_hi_hz < grid_lo_hz) {
        if (out_best_sum) *out_best_sum = 0;
        return grid_lo_hz;
    }
    for (uint32_t c = grid_lo_hz; c <= grid_hi_hz; c += grid_step_hz) {
        uint32_t s = ds_window_integral(hist, nbins, c, rx_bw_hz);
        if (s > best_sum) {
            best_sum    = s;
            best_center = c;
        }
    }
    if (out_best_sum) *out_best_sum = best_sum;
    return best_center;
}
