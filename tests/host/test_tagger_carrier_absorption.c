// test_tagger_carrier_absorption.c — clamp-saturated strong-carrier
// absorption (2026-07-07 decode-regression batch, fix 2).
//
// Mechanism under test: gr-iridium's float baseline absorbs ANY
// persistent carrier — each max-burst-len force-close pushes a forced
// EMA refresh, and after at most FBT_HISTORY_SIZE refreshes the whole
// history holds the carrier and it stops re-triggering (full turnover
// = 512 × 225000 / 2.5e6 = 46.08 s at pure force-close cadence). Our
// int32 port clamps each history slot at FBT_EMA_SLOT_CLAMP
// (INT32_MAX / FBT_HISTORY_SIZE); a carrier with
//   mag² > threshold_lin × INT32_MAX / HISTORY_SIZE   (~1.05e8 @ 14 dB)
// could therefore NEVER be retired by the threshold math — it
// force-close-cycled forever, feeding the squelch storm observed live
// (test_tagger_latch_release deliberately keeps its carrier BELOW the
// clamp for exactly this reason; this test covers the regime above it).
// The fix tracks consecutive clamped EMA writes per bin and treats a
// bin whose ENTIRE history is pinned at the clamp as absorbed.
//
// Assertions (properties of the mechanism, no fitted numbers):
//   V1 (validity) the carrier's per-bin mag² measured through the
//      EXACT device kernels (fbt_window_multiply_q15 + fft_sc16_2048)
//      exceeds the int32-retirement bound — i.e. without the fix this
//      carrier can never stop re-triggering;
//   V2 (validity) force-close/re-detect cycling is observed first
//      (the pre-absorption storm regime exists);
//   A1 the carrier bin stops producing new bursts within the derived
//      bound of FBT_HISTORY_SIZE force-close cycles (>= 1 forced EMA
//      write per cycle => absorption in <= 512 × ~112 steps; the
//      post-close EMA step roughly halves that in practice);
//   A2 once absorbed it STAYS absorbed while the carrier persists
//      (no new bursts at the bin over a sustained window);
//   A3 the detector is still alive: a fresh burst at another bin is
//      detected after absorption, carrier still on.

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fft_burst_tagger.h"
#include "fft_sc16_2048.h"

#define N_FFT FBT_FFT_SIZE
#define BURST_PRE_LEN (2 * FBT_FFT_SIZE)
#define BURST_POST_LEN 40000 // gri: 2.5 MSPS * 16e-3
#define BURST_WIDTH 32
#define THRESHOLD_DB 14.0f // device default (dsp_processor.c FBT_THRESHOLD_DB)

// One force-close cycle: (last_active - start) must EXCEED
// FBT_MAX_BURST_LEN (~110 steps), plus the close/re-detect steps.
#define CYCLE_STEPS_MAX 114
// Absorption bound: >= 1 forced EMA write per cycle, FBT_HISTORY_SIZE
// writes needed => at most HISTORY × CYCLE_STEPS_MAX steps.
#define ABSORB_STEP_BOUND (FBT_HISTORY_SIZE * CYCLE_STEPS_MAX)
// "Absorbed" detector: no new burst at the carrier bin for 3 cycles'
// worth of steps while the carrier is still applied.
#define QUIET_STEPS (3 * CYCLE_STEPS_MAX)

static int32_t s_baseline_history[FBT_HISTORY_SIZE * FBT_FFT_SIZE];

// Deterministic PRNG (xorshift32) — same rationale and amplitude as
// test_tagger_latch_release: the noise floor must sit above the int16
// FFT's quantization floor.
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

// Bin-centered complex tone block (same construction as the other
// tagger tests): integer cycles per N-sample step, so one block
// repeats exactly.
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

// Measure the carrier's per-bin magnitude² through the EXACT kernels
// the tagger runs (Q15 Blackman window-multiply + fft_sc16_2048 +
// FFT-shifted re²+im²). Returns the max mag² within ±2 bins of `bin`.
static int64_t measure_mag2(const int16_t *iq, int bin)
{
    // Rebuild the tagger's window (build_blackman_q15 is static there;
    // same formula, Q15 Blackman a0=0.42 a1=0.5 a2=0.08).
    static int16_t win[N_FFT];
    const double   PI = 3.14159265358979323846;
    for (int i = 0; i < N_FFT; i++) {
        double t = (double)i / (double)(N_FFT - 1);
        double v = 0.42 - 0.5 * cos(2.0 * PI * t) + 0.08 * cos(4.0 * PI * t);
        double q = v * 32767.0;
        if (q > 32767.0) q = 32767.0;
        if (q < 0) q = 0;
        win[i] = (int16_t)(q + 0.5);
    }
    static int16_t fb[2 * N_FFT];
    fbt_window_multiply_q15(iq, win, fb, N_FFT);
    fft_sc16_2048(fb);
    int64_t best = 0;
    for (int d = -2; d <= 2; d++) {
        int     shifted = bin + d;                       // DC-centred bin index
        int     raw     = (shifted + N_FFT / 2) % N_FFT; // undo FFT-shift
        int64_t re = fb[2 * raw + 0], im = fb[2 * raw + 1];
        int64_t m2 = re * re + im * im;
        if (m2 > best) best = m2;
    }
    return best;
}

int main(void)
{
    int         failures = 0;
    fbt_burst_t nb[FBT_MAX_BURSTS], gb[FBT_MAX_BURSTS];
    int16_t     in[2 * N_FFT];

    fft_sc16_2048_init();

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

    // Carrier amplitude: 30000 puts the windowed FFT bin at
    // ~0.42 (Blackman coherent gain) × 30000 ≈ 12600 => mag² ≈ 1.6e8,
    // above the int32-retirement bound (~1.05e8 at 14 dB). V1 verifies
    // the actual value through the device kernels rather than trusting
    // this estimate. Noise ±8192 on top clips <0.5% of samples (sat16),
    // same order as the live-conditions overlay tolerates.
    enum { BIN_CW = 700,
           CW_AMP = 30000 };
    static int32_t tone_cw[2 * N_FFT];
    memset(tone_cw, 0, sizeof(tone_cw));
    add_tone_block(tone_cw, BIN_CW, CW_AMP, 0.0);

    // V1: measured mag² must exceed the bound above which the clamped
    // int32 baseline can never retire the carrier:
    //   trigger iff mag² × HISTORY > (baseline_sum × thr_q15) >> 15
    //   with baseline_sum at its clamped max HISTORY × (INT32_MAX/HISTORY).
    {
        static int16_t probe[2 * N_FFT];
        fill_noise(probe);
        for (int k = 0; k < 2 * N_FFT; k++)
            probe[k] = sat16((int)probe[k] + (int)tone_cw[k]);
        int64_t mag2 = measure_mag2(probe, BIN_CW);

        int64_t thr_q15    = (int64_t)(pow(10.0, THRESHOLD_DB / 10.0) * 32768.0 + 0.5);
        int64_t bsum_max   = (int64_t)FBT_HISTORY_SIZE * (INT32_MAX / FBT_HISTORY_SIZE);
        int64_t mag2_bound = ((bsum_max * thr_q15) >> 15) / FBT_HISTORY_SIZE;
        printf("V1: carrier mag2 = %lld, int32-retirement bound = %lld (x%.2f)\n",
               (long long)mag2, (long long)mag2_bound,
               (double)mag2 / (double)mag2_bound);
        if (mag2 <= mag2_bound) {
            printf("FAIL V1: carrier mag2 %lld <= bound %lld — not in the "
                   "clamp-saturated regime, test invalid (raise CW_AMP)\n",
                   (long long)mag2, (long long)mag2_bound);
            failures++;
        }
    }

    // Phase A: carrier on, run until absorbed or the derived bound.
    int new_at_cw = 0, gone_at_cw = 0, forcecloses = 0;
    int last_new_step = -1, absorbed_step = -1;
    int s;
    for (s = 0; s < ABSORB_STEP_BOUND; s++) {
        int n_new = FBT_MAX_BURSTS, n_gone = FBT_MAX_BURSTS;
        fill_noise(in);
        for (int k = 0; k < 2 * N_FFT; k++)
            in[k] = sat16((int)in[k] + (int)tone_cw[k]);
        fft_burst_tagger_step(t, in, NULL, nb, &n_new, gb, &n_gone);
        for (int i = 0; i < n_new; i++) {
            if (near_bin(nb[i].center_bin, BIN_CW)) {
                new_at_cw++;
                last_new_step = s;
            }
        }
        for (int i = 0; i < n_gone; i++) {
            if (near_bin(gb[i].center_bin, BIN_CW)) {
                gone_at_cw++;
                if (gb[i].stop - gb[i].start >= (uint64_t)FBT_MAX_BURST_LEN)
                    forcecloses++;
            }
        }
        if (last_new_step >= 0 && s - last_new_step >= QUIET_STEPS) {
            absorbed_step = last_new_step;
            break;
        }
    }
    printf("Phase A: new@cw=%d gone@cw=%d forcecloses=%d "
           "last_new_step=%d (bound %d)\n",
           new_at_cw, gone_at_cw, forcecloses, last_new_step,
           ABSORB_STEP_BOUND);

    // V2: the pre-absorption cycling regime must have existed.
    if (forcecloses < 2 || new_at_cw < 3) {
        printf("FAIL V2: force-close cycling absent (forcecloses=%d, "
               "new@cw=%d) — carrier never entered the storm regime\n",
               forcecloses, new_at_cw);
        failures++;
    }
    // A1: absorbed within the derived bound.
    if (absorbed_step < 0) {
        printf("FAIL A1: carrier still re-triggering after %d steps "
               "(bound %d) — clamp-saturated carrier never absorbed\n",
               s, ABSORB_STEP_BOUND);
        failures++;
    } else {
        printf("A1: absorbed at step %d (%.1f s equivalent; gri full-"
               "turnover bound 46.08 s)\n",
               absorbed_step,
               (double)absorbed_step * N_FFT / 2.5e6);
    }

    // A2: stays absorbed over a further sustained window, carrier on.
    if (absorbed_step >= 0) {
        int late_new = 0;
        for (int s2 = 0; s2 < 6 * CYCLE_STEPS_MAX; s2++) {
            int n_new = FBT_MAX_BURSTS, n_gone = FBT_MAX_BURSTS;
            fill_noise(in);
            for (int k = 0; k < 2 * N_FFT; k++)
                in[k] = sat16((int)in[k] + (int)tone_cw[k]);
            fft_burst_tagger_step(t, in, NULL, nb, &n_new, gb, &n_gone);
            for (int i = 0; i < n_new; i++)
                if (near_bin(nb[i].center_bin, BIN_CW)) late_new++;
        }
        printf("A2: post-absorption re-detections over %d steps: %d\n",
               6 * CYCLE_STEPS_MAX, late_new);
        if (late_new > 0) {
            printf("FAIL A2: carrier re-detected %d times after absorption "
                   "— cutoff not holding\n",
                   late_new);
            failures++;
        }
    }

    // A3: detector alive — fresh burst at another bin, carrier still on.
    {
        enum { BIN_B         = 1300,
               PHASE_B_STEPS = 60,
               BURST_B_STEPS = 30 };
        static int32_t tone_b[2 * N_FFT];
        memset(tone_b, 0, sizeof(tone_b));
        add_tone_block(tone_b, BIN_B, 6000, 0.0);
        int new_at_b = 0;
        for (int s2 = 0; s2 < PHASE_B_STEPS; s2++) {
            int n_new = FBT_MAX_BURSTS, n_gone = FBT_MAX_BURSTS;
            fill_noise(in);
            for (int k = 0; k < 2 * N_FFT; k++) {
                int v = in[k] + tone_cw[k];
                if (s2 < BURST_B_STEPS) v += tone_b[k];
                in[k] = sat16(v);
            }
            fft_burst_tagger_step(t, in, NULL, nb, &n_new, gb, &n_gone);
            for (int i = 0; i < n_new; i++)
                if (near_bin(nb[i].center_bin, BIN_B)) new_at_b++;
        }
        printf("A3: fresh burst detections at bin %d: %d\n", BIN_B, new_at_b);
        if (new_at_b < 1) {
            printf("FAIL A3: detector dead after absorption\n");
            failures++;
        }
    }
    fft_burst_tagger_destroy(t);

    if (failures) {
        printf("\n[FAIL] %d assertion(s) failed\n", failures);
        return 1;
    }
    printf("\n[pass] tagger carrier absorption (absorbed at step %d of "
           "bound %d, %d force-close cycles)\n",
           absorbed_step, ABSORB_STEP_BOUND, forcecloses);
    return 0;
}
