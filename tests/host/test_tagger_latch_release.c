// test_tagger_latch_release.c — frozen-baseline latch release (P1).
//
// Mechanism under test (all three ported from gr-iridium
// lib/fft_burst_tagger_impl.cc):
//   1. max-burst-len force-close (:263-270): a burst active longer than
//      FBT_MAX_BURST_LEN is force-closed instead of tracked forever.
//   2. forced noise-floor refresh on force-close (:283-285 →
//      update_filters_post(true)): each force-close pushes the CURRENT
//      spectrum (carrier included) into the baseline history even
//      though a burst is active — so over repeated force-close cycles
//      a persistent carrier is absorbed into the baseline and stops
//      re-triggering.
//   3. burst squelch (:327-355): when tracked bursts exceed
//      FBT_SQUELCH_MAX_BURSTS the tracking table is dumped, and
//      repeated squelches reset the noise estimate entirely
//      (observable: step() returns false while re-priming).
//
// Without 1+2, `update_baseline_ema` returns while n_bursts > 0, so a
// persistent carrier keeps the baseline frozen at its pre-carrier
// value FOREVER (the P1 latch): the carrier's burst never goes gone,
// and the detector floods downstream with junk.
//
// The input is synthetic (bin-centered tones + deterministic uniform
// noise) because the test targets a specific mechanism; every pass
// criterion below is a property of the mechanism, not a number fitted
// to bench data:
//   A1 the persistent carrier's burst DOES go gone (latch ⇒ never);
//   A2 every gone burst's length is bounded by the force-close math;
//   A3 the carrier is re-detected a few times and then STOPS being
//      re-detected (baseline absorbed it) — latch ⇒ exactly one
//      detection and no release;
//   B  a fresh burst at another bin is still detected afterwards
//      (detector alive, not squelched dead);
//   C1 a >FBT_SQUELCH_MAX_BURSTS flood drives the squelch's noise
//      reset (step() returns false, then re-primes);
//   C2 after re-priming, the flood tones are IN the baseline and no
//      longer re-detected;
//   C3 a fresh, stronger burst is still detected after the reset.

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fft_burst_tagger.h"

#define N_FFT FBT_FFT_SIZE
#define BURST_PRE_LEN (2 * FBT_FFT_SIZE)
#define BURST_POST_LEN 40000 // gri: 2.5 MSPS * 16e-3
#define BURST_WIDTH 32
#define THRESHOLD_DB 10.0f

// Force-close length bound: last_active - start can reach
// FBT_MAX_BURST_LEN + one FFT step before the check fires; stop is at
// most one post_len later (timeout path); start includes pre_len.
#define GONE_LEN_BOUND \
    (FBT_MAX_BURST_LEN + BURST_POST_LEN + BURST_PRE_LEN + 2 * FBT_FFT_SIZE)

static int32_t s_baseline_history[FBT_HISTORY_SIZE * FBT_FFT_SIZE];

// Deterministic PRNG (xorshift32) — reproducible noise. Amplitude
// matters: the int16 FFT truncates per stage, so the noise floor must
// sit well ABOVE the FFT's quantization floor or the per-bin baseline
// is dominated by truncation artifacts (a strong tone then throws
// correlated quantization spurs across the whole band and the test
// squelches on its own arithmetic, not the mechanism under test).
// Uniform ±8192 keeps the per-bin noise power comfortably above 1 LSB²
// after the FFT's /N scaling.
static uint32_t       s_rng = 0x1234567u;
static inline int16_t noise_sample(void)
{
    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;
    return (int16_t)((int32_t)(s_rng & 0x3FFF) - 8192); // uniform ±8192
}

static inline int16_t sat16(int v)
{
    if (v > 32767) return 32767;
    if (v < -32768) return -32768;
    return (int16_t)v;
}

// Bin-centered complex tone: freq = (bin - N/2) * fs / N advances an
// integer number of cycles per N-sample step, so one precomputed
// N-sample block repeats exactly. amp in int16 units. phase0 decorrelates
// multiple tones' peaks (golden-angle steps keep the flood's crest factor
// sane). Blocks accumulate in int32 — a 58-tone sum wraps int16 — and are
// saturated once at compose time (sat16).
static void add_tone_block(int32_t *block_iq, int bin, int amp, double phase0)
{
    double w = 2.0 * M_PI * (double)(bin - N_FFT / 2) / (double)N_FFT;
    for (int n = 0; n < N_FFT; n++) {
        block_iq[2 * n + 0] += (int32_t)lrint((double)amp * cos(w * n + phase0));
        block_iq[2 * n + 1] += (int32_t)lrint((double)amp * sin(w * n + phase0));
    }
}

static void fill_noise(int16_t *block_iq)
{
    for (int k = 0; k < 2 * N_FFT; k++)
        block_iq[k] = noise_sample();
}

static int near_bin(int bin, int target)
{
    return abs(bin - target) <= 2;
}

int main(void)
{
    int         failures = 0;
    fbt_burst_t nb[FBT_MAX_BURSTS], gb[FBT_MAX_BURSTS];
    int16_t     in[2 * N_FFT];

    // ---------------------------------------------------------------
    // Scenario 1: persistent single carrier (the P1 latch scenario).
    // ---------------------------------------------------------------
    fft_burst_tagger_t *t = fft_burst_tagger_init(
        BURST_PRE_LEN, BURST_POST_LEN, BURST_WIDTH, THRESHOLD_DB,
        s_baseline_history);
    if (!t) {
        fprintf(stderr, "tagger init failed\n");
        return 2;
    }
    fft_burst_tagger_set_start(t, 0);

    // Prime on noise only.
    bool primed = false;
    for (int i = 0; i < FBT_HISTORY_SIZE + 8 && !primed; i++) {
        int n_new = FBT_MAX_BURSTS, n_gone = FBT_MAX_BURSTS;
        fill_noise(in);
        primed = fft_burst_tagger_step(t, in, NULL, nb, &n_new, gb, &n_gone);
    }
    if (!primed) {
        fprintf(stderr, "FAIL: never primed\n");
        return 1;
    }

    // Persistent tone at bin_a, on for PHASE_A_STEPS (≫ absorption time:
    // absorption needs ~HISTORY/threshold ≈ 52 carrier-bearing history
    // slots; the force-close path guarantees ≥1 forced slot-write per
    // ~FBT_MAX_BURST_LEN/FBT_FFT_SIZE ≈ 110-step cycle ⇒ ≲ 5800 steps).
    //
    // Amplitude 4500 keeps the carrier's per-bin mag² below the EMA
    // slot clamp (INT32_MAX / FBT_HISTORY_SIZE, see ema_step_inner) —
    // the regime where full absorption is possible. A carrier much
    // stronger than the clamp can never drop below threshold via
    // absorption (the baseline saturates); the force-close still
    // bounds its bursts and the forced refresh still unfreezes the
    // REST of the band, but "stops re-triggering" would no longer be
    // the right assertion there.
    enum { BIN_A         = 700,
           PHASE_A_STEPS = 8000,
           LATE_WIN      = 1200 };
    static int32_t tone_a[2 * N_FFT]; // block template
    memset(tone_a, 0, sizeof(tone_a));
    add_tone_block(tone_a, BIN_A, 4500, 0.0);

    int      gone_at_a = 0, new_at_a = 0, new_at_a_late = 0;
    uint64_t worst_gone_len = 0;
    for (int s = 0; s < PHASE_A_STEPS; s++) {
        int n_new = FBT_MAX_BURSTS, n_gone = FBT_MAX_BURSTS;
        fill_noise(in);
        for (int k = 0; k < 2 * N_FFT; k++)
            in[k] = sat16(in[k] + tone_a[k]);
        fft_burst_tagger_step(t, in, NULL, nb, &n_new, gb, &n_gone);
        for (int i = 0; i < n_new; i++) {
            if (near_bin(nb[i].center_bin, BIN_A)) {
                new_at_a++;
                if (s >= PHASE_A_STEPS - LATE_WIN) new_at_a_late++;
            }
        }
        for (int i = 0; i < n_gone; i++) {
            uint64_t len = gb[i].stop - gb[i].start;
            if (len > worst_gone_len) worst_gone_len = len;
            if (near_bin(gb[i].center_bin, BIN_A)) gone_at_a++;
        }
    }

    printf("Scenario 1 (persistent carrier at bin %d):\n", BIN_A);
    printf("  new bursts at carrier bin:  %d (late window: %d)\n",
           new_at_a, new_at_a_late);
    printf("  gone events at carrier bin: %d\n", gone_at_a);
    printf("  worst gone length:          %llu (bound %d)\n",
           (unsigned long long)worst_gone_len, GONE_LEN_BOUND);

    if (gone_at_a < 1) {
        printf("FAIL A1: carrier burst never closed — latch not released\n");
        failures++;
    }
    if (worst_gone_len > (uint64_t)GONE_LEN_BOUND) {
        printf("FAIL A2: gone length %llu exceeds force-close bound %d\n",
               (unsigned long long)worst_gone_len, GONE_LEN_BOUND);
        failures++;
    }
    if (new_at_a < 3) {
        printf("FAIL A3a: carrier re-detected only %d times — force-close/"
               "re-detect cycling absent\n",
               new_at_a);
        failures++;
    }
    // Bound derivation: unabsorbed force-close cycling re-detects the
    // carrier every ~FBT_MAX_BURST_LEN/FBT_FFT_SIZE ≈ 112 steps ⇒
    // ≥ LATE_WIN/112 ≈ 10 late hits. Noise false positives (normal
    // detector behavior, ~0.1/step across 2016 bins) land in this ±2
    // window ≈ 0.3 times. ≤ 2 separates the regimes by 5×.
    if (new_at_a_late > 2) {
        printf("FAIL A3b: carrier still re-detected %d times in the last %d "
               "steps — baseline never absorbed it\n",
               new_at_a_late, LATE_WIN);
        failures++;
    }

    // Phase B: detector still alive — fresh strong burst at another bin
    // (carrier still on) must be detected.
    enum { BIN_B         = 1300,
           PHASE_B_STEPS = 60,
           BURST_B_STEPS = 30 };
    static int32_t tone_b[2 * N_FFT];
    memset(tone_b, 0, sizeof(tone_b));
    add_tone_block(tone_b, BIN_B, 6000, 0.0);

    int new_at_b = 0;
    for (int s = 0; s < PHASE_B_STEPS; s++) {
        int n_new = FBT_MAX_BURSTS, n_gone = FBT_MAX_BURSTS;
        fill_noise(in);
        for (int k = 0; k < 2 * N_FFT; k++) {
            int v = in[k] + tone_a[k];
            if (s < BURST_B_STEPS) v += tone_b[k];
            in[k] = sat16(v);
        }
        fft_burst_tagger_step(t, in, NULL, nb, &n_new, gb, &n_gone);
        for (int i = 0; i < n_new; i++)
            if (near_bin(nb[i].center_bin, BIN_B)) new_at_b++;
    }
    printf("Scenario 1 phase B: fresh burst detections at bin %d: %d\n",
           BIN_B, new_at_b);
    if (new_at_b < 1) {
        printf("FAIL B: detector dead after release — fresh burst missed\n");
        failures++;
    }
    fft_burst_tagger_destroy(t);

    // ---------------------------------------------------------------
    // Scenario 2: burst squelch + noise-estimate reset.
    // ---------------------------------------------------------------
    t = fft_burst_tagger_init(BURST_PRE_LEN, BURST_POST_LEN, BURST_WIDTH,
                              THRESHOLD_DB, s_baseline_history);
    if (!t) {
        fprintf(stderr, "tagger init failed (scenario 2)\n");
        return 2;
    }
    fft_burst_tagger_set_start(t, 0);
    primed = false;
    for (int i = 0; i < FBT_HISTORY_SIZE + 8 && !primed; i++) {
        int n_new = FBT_MAX_BURSTS, n_gone = FBT_MAX_BURSTS;
        fill_noise(in);
        primed = fft_burst_tagger_step(t, in, NULL, nb, &n_new, gb, &n_gone);
    }
    if (!primed) {
        fprintf(stderr, "FAIL: scenario 2 never primed\n");
        return 1;
    }

    // Flood: FBT_SQUELCH_MAX_BURSTS + 9 tones, spaced wider than the
    // burst mask (burst_width/2 = 16 bins each side, so spacing 34 keeps
    // every tone in its own tracking slot). Deterministic per-tone
    // phases (golden-angle steps) keep the block's crest factor sane —
    // see add_tone_block.
    enum { N_FLOOD        = FBT_SQUELCH_MAX_BURSTS + 9,
           FLOOD_SPACING  = 34,
           FLOOD_BASE_BIN = 20,
           PHASE_C_STEPS  = 1400,
           LATE_WIN_C     = 300 };
    static int32_t flood[2 * N_FFT];
    memset(flood, 0, sizeof(flood));
    for (int m = 0; m < N_FLOOD; m++)
        add_tone_block(flood, FLOOD_BASE_BIN + m * FLOOD_SPACING, 1500,
                       2.399963 * (double)m);

    int  new_at_flood_late = 0;
    bool saw_unprimed = false, reprimed = false;
    for (int s = 0; s < PHASE_C_STEPS; s++) {
        int n_new = FBT_MAX_BURSTS, n_gone = FBT_MAX_BURSTS;
        fill_noise(in);
        for (int k = 0; k < 2 * N_FFT; k++)
            in[k] = sat16(in[k] + flood[k]);
        bool ok = fft_burst_tagger_step(t, in, NULL, nb, &n_new, gb, &n_gone);
        if (!ok) saw_unprimed = true;
        if (ok && saw_unprimed) reprimed = true;
        if (ok && s >= PHASE_C_STEPS - LATE_WIN_C) {
            for (int i = 0; i < n_new; i++) {
                int b   = nb[i].center_bin;
                int rel = b - FLOOD_BASE_BIN;
                if (rel >= -2 && (rel % FLOOD_SPACING <= 2 || rel % FLOOD_SPACING >= FLOOD_SPACING - 2) &&
                    b <= FLOOD_BASE_BIN + (N_FLOOD - 1) * FLOOD_SPACING + 2)
                    new_at_flood_late++;
            }
        }
    }
    printf("Scenario 2 (flood of %d tones):\n", N_FLOOD);
    printf("  squelch noise reset seen:  %s (re-primed: %s)\n",
           saw_unprimed ? "yes" : "NO", reprimed ? "yes" : "NO");
    printf("  late-window flood-bin re-detections: %d\n", new_at_flood_late);
    if (!saw_unprimed || !reprimed) {
        printf("FAIL C1: flood did not drive the squelch noise reset "
               "(unprimed=%d reprimed=%d)\n",
               (int)saw_unprimed, (int)reprimed);
        failures++;
    }
    // Bound derivation: if the reset did NOT fold the tones into the
    // re-learned floor, all N_FLOOD tones keep force-close cycling ⇒
    // ≥ N_FLOOD × LATE_WIN_C/112 ≈ 155 late hits. The flood-grid bin
    // filter spans ~15% of all bins, so ordinary noise false positives
    // contaminate it by ~0.1/step × 15% × LATE_WIN_C ≈ 4. N_FLOOD/2
    // (= 29) sits an order of magnitude from both regimes.
    if (new_at_flood_late >= N_FLOOD / 2) {
        printf("FAIL C2: flood tones still re-detected %d times after the "
               "noise reset re-primed — floor not re-learned\n",
               new_at_flood_late);
        failures++;
    }

    // C3: fresh, much stronger burst on top of the flood is detected.
    // Bin sits midway between two flood tones (20 + 22*34 = 768 and
    // 802), well inside the peak-scan margin [16, 2032).
    enum { BIN_C = 785 };
    static int32_t tone_c[2 * N_FFT];
    memset(tone_c, 0, sizeof(tone_c));
    add_tone_block(tone_c, BIN_C, 9000, 0.0);
    int new_at_c = 0;
    for (int s = 0; s < 40; s++) {
        int n_new = FBT_MAX_BURSTS, n_gone = FBT_MAX_BURSTS;
        fill_noise(in);
        for (int k = 0; k < 2 * N_FFT; k++) {
            int v = in[k] + flood[k];
            if (s < 30) v += tone_c[k];
            in[k] = sat16(v);
        }
        fft_burst_tagger_step(t, in, NULL, nb, &n_new, gb, &n_gone);
        for (int i = 0; i < n_new; i++)
            if (near_bin(nb[i].center_bin, BIN_C)) new_at_c++;
    }
    printf("  fresh strong burst at bin %d detections: %d\n", BIN_C, new_at_c);
    if (new_at_c < 1) {
        printf("FAIL C3: detector dead after squelch reset\n");
        failures++;
    }
    fft_burst_tagger_destroy(t);

    if (failures) {
        printf("\n[FAIL] %d assertion(s) failed\n", failures);
        return 1;
    }
    printf("\n[pass] tagger latch release\n");
    return 0;
}
