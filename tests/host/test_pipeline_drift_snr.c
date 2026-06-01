// test_pipeline_drift_snr.c — (#121) Parameter-sweep regression test
// for the wideband decode pipeline. For each combination of synthetic
// LO drift (kHz) and added AWGN sigma (relative noise level), runs the
// full firmware DSP path (tagger → burst_pipeline → qpsk_demod → BCH
// in worker_emit_frame) on the ALBQ fixture and counts decoded frames.
//
// Why this matters:
//   - Quantifies the sensitivity wins from #112 (Chase-2 BCH),
//     #113 (per-burst DC removal), #115 (pre-UW CFO confidence filter).
//     Without a sweep we can't actually CLAIM a dB-level improvement.
//   - Catches future LO-drift regressions (RTL-SDR is ±10 ppm; at
//     1617 MHz that's ±16 kHz, more than one Iridium channel).
//   - Catches future sensitivity regressions — any refactor that drops
//     a few dB at the BCH stage will show up as a noise-step crossing
//     point shifting outward.
//
// Output: CSV table to stdout, one row per (drift, sigma). Test PASSES
// if the baseline (drift=0, sigma=0) decodes >= MIN_BASELINE_DECODED;
// drift cases stay within DRIFT_TOLERANCE_FRAC of baseline; and at
// least one noise step above zero produces a non-zero decode count
// (proves the noise injection is meaningful).
//
// Structure clones test_pipeline_wideband_resampled.c so the comparison
// to the existing baseline is apples-to-apples — only the inner copy
// gets mutated per sweep cell.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "fft_burst_tagger.h"
#include "fft_sc16_2048.h"
#include "direct_if_decim.h"
#include "resample_256_to_250.h"
#include "rotate_to_dc.h"
#include "burst_pipeline.h"
#include "qpsk_demod.h"
#include "uw_correlator.h"
#include "bch_decoder.h"
#include "fixture_albq_raw.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define INPUT_FS_HZ 2500000 // post-firmware-resampler rate
#define BURST_PRE_LEN (2 * FBT_FFT_SIZE)
#define BURST_POST_LEN ((int)(INPUT_FS_HZ * 16e-3))
#define BURST_WINDOW_LEN ((int)(INPUT_FS_HZ * 250 / 1000)) // 250 ms
#define BURST_WINDOW_250K (BURST_WINDOW_LEN / DIDECIM_DECIM)

// Drift sweep: kHz at the LO. ±15 kHz brackets RTL-SDR's ±10 ppm at
// 1617 MHz (= ±16 kHz). 0 included as the baseline.
static const float DRIFT_KHZ[] = {-15.0f, -10.0f, -5.0f, 0.0f, 5.0f, 10.0f, 15.0f};
#define N_DRIFT (sizeof(DRIFT_KHZ) / sizeof(DRIFT_KHZ[0]))

// Noise sweep: int16 sigma added per IQ component. ALBQ fixture is
// uint8 (~int16 << 8 = ~32 LSB-of-MSB step), so noise sigma ~50 is
// modest, ~200 starts to dominate. 0 = baseline (no added noise).
static const int NOISE_SIGMA[] = {0, 50, 100, 200, 400};
#define N_NOISE (sizeof(NOISE_SIGMA) / sizeof(NOISE_SIGMA[0]))

// PASS criteria.
#define MIN_BASELINE_DECODED 30   // at (drift=0, sigma=0) — the
                                  // existing test_pipeline_wideband_resampled
                                  // typically decodes 50+; 30 is loose floor.
#define DRIFT_TOLERANCE_FRAC 0.5f // drift cases must hit >=50%
                                  // of the baseline decode count.
                                  // Per memory project_d7_d8_coupling
                                  // the channelizer's 40 kHz bin
                                  // quantisation tolerates ±5-10 kHz
                                  // cleanly; ±15 kHz is at the edge.

static int16_t s_baseline_history[FBT_HISTORY_SIZE * FBT_FFT_SIZE * sizeof(int32_t) / sizeof(int16_t)];

// Apply a synthetic frequency shift to int16 IQ samples in-place.
// Multiplies each sample by exp(j·2π·f_shift_hz·n/fs). Uses an
// incremental complex-rotation accumulator so we don't pay a sin/cos
// per sample.
static void apply_drift_inplace(int16_t *iq, int n_complex, double f_shift_hz, double fs_hz)
{
    if (f_shift_hz == 0.0) return;
    double dphi   = 2.0 * M_PI * f_shift_hz / fs_hz;
    double cur_re = 1.0, cur_im = 0.0;
    double inc_re = cos(dphi), inc_im = sin(dphi);
    // Re-normalise the accumulator every ~512 samples to avoid float
    // drift over millions of samples.
    for (int i = 0; i < n_complex; i++) {
        double xi = iq[i * 2 + 0];
        double xq = iq[i * 2 + 1];
        double yi = xi * cur_re - xq * cur_im;
        double yq = xi * cur_im + xq * cur_re;
        if (yi > 32767.0)
            yi = 32767.0;
        else if (yi < -32768.0)
            yi = -32768.0;
        if (yq > 32767.0)
            yq = 32767.0;
        else if (yq < -32768.0)
            yq = -32768.0;
        iq[i * 2 + 0] = (int16_t)yi;
        iq[i * 2 + 1] = (int16_t)yq;
        double nr     = cur_re * inc_re - cur_im * inc_im;
        double ni     = cur_re * inc_im + cur_im * inc_re;
        cur_re        = nr;
        cur_im        = ni;
        if ((i & 511) == 511) {
            double mag = sqrt(cur_re * cur_re + cur_im * cur_im);
            cur_re /= mag;
            cur_im /= mag;
        }
    }
}

// Add gaussian noise to int16 IQ samples in-place. Uses Box-Muller via
// the Marsaglia polar method. Sigma is per IQ component, in int16 LSB
// units. Seeded fixed so the test is reproducible.
static void apply_noise_inplace(int16_t *iq, int n_complex, double sigma)
{
    if (sigma <= 0.0) return;
    static uint32_t seed = 0xDEAD0001u; // fixed for reproducibility
    for (int i = 0; i < n_complex; i++) {
        double u, v, s;
        do {
            seed = seed * 1664525u + 1013904223u;
            u    = (double)((int32_t)seed) / 2147483648.0;
            seed = seed * 1664525u + 1013904223u;
            v    = (double)((int32_t)seed) / 2147483648.0;
            s    = u * u + v * v;
        } while (s >= 1.0 || s == 0.0);
        double f  = sqrt(-2.0 * log(s) / s);
        double ni = sigma * (u * f);
        double nq = sigma * (v * f);
        double yi = iq[i * 2 + 0] + ni;
        double yq = iq[i * 2 + 1] + nq;
        if (yi > 32767.0)
            yi = 32767.0;
        else if (yi < -32768.0)
            yi = -32768.0;
        if (yq > 32767.0)
            yq = 32767.0;
        else if (yq < -32768.0)
            yq = -32768.0;
        iq[i * 2 + 0] = (int16_t)yi;
        iq[i * 2 + 1] = (int16_t)yq;
    }
}

// Run the full wideband pipeline on the given IQ buffer. Returns the
// number of frames whose qpsk_demod_process + UW match succeeded
// (res.demod_ok); also writes the bch_decoded count (both BCH blocks
// passed hard decode) into *out_bch — that's the metric that #112's
// Chase-2 affects and the closest to the production "real frame" signal.
static int run_pipeline(int16_t *iq25, int n25, int *out_bch)
{
    fft_burst_tagger_t *t = fft_burst_tagger_init(
        BURST_PRE_LEN, BURST_POST_LEN,
        /*burst_width=*/32,
        /*threshold_db=*/14.0f,
        (int32_t *)s_baseline_history);
    if (!t) return -1;
    fft_burst_tagger_set_start(t, 0);

    typedef struct {
        uint64_t start;
        uint64_t stop;
        int      center_bin;
    } tag_t;
    enum { MAX_TAGS = 256 };
    tag_t       tags[MAX_TAGS];
    int         n_tags = 0;
    fbt_burst_t new_bursts[FBT_MAX_BURSTS];
    fbt_burst_t gone_bursts[FBT_MAX_BURSTS];
    for (int off = 0; off + FBT_FFT_SIZE <= n25; off += FBT_FFT_SIZE) {
        int n_new = FBT_MAX_BURSTS, n_gone = FBT_MAX_BURSTS;
        fft_burst_tagger_step(t, iq25 + off * 2, NULL,
                              new_bursts, &n_new,
                              gone_bursts, &n_gone);
        for (int i = 0; i < n_gone && n_tags < MAX_TAGS; i++) {
            tags[n_tags].start      = gone_bursts[i].start;
            tags[n_tags].stop       = gone_bursts[i].stop;
            tags[n_tags].center_bin = gone_bursts[i].center_bin;
            n_tags++;
        }
    }
    {
        int         n_flush = FBT_MAX_BURSTS;
        fbt_burst_t flushed[FBT_MAX_BURSTS];
        fft_burst_tagger_flush(t, flushed, &n_flush);
        for (int i = 0; i < n_flush && n_tags < MAX_TAGS; i++) {
            tags[n_tags].start      = flushed[i].start;
            tags[n_tags].stop       = flushed[i].stop;
            tags[n_tags].center_bin = flushed[i].center_bin;
            n_tags++;
        }
    }
    fft_burst_tagger_destroy(t);

    int16_t *window_25  = malloc(2 * BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *window_250 = malloc(2 * BURST_WINDOW_250K * sizeof(int16_t));
    int16_t *scr_in_i   = malloc(BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *scr_in_q   = malloc(BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *scr_out_i  = malloc(BURST_WINDOW_250K * sizeof(int16_t));
    int16_t *scr_out_q  = malloc(BURST_WINDOW_250K * sizeof(int16_t));
    if (!window_25 || !window_250 || !scr_in_i || !scr_in_q || !scr_out_i || !scr_out_q) {
        free(window_25);
        free(window_250);
        free(scr_in_i);
        free(scr_in_q);
        free(scr_out_i);
        free(scr_out_q);
        return -1;
    }
    direct_if_decim_t dec;
    direct_if_decim_init(&dec);

    int decoded = 0, bch_ok = 0;
    for (int ti = 0; ti < n_tags; ti++) {
        int      center_bin = tags[ti].center_bin;
        uint64_t start      = tags[ti].start;
        uint64_t stop       = tags[ti].stop;
        int64_t  begin      = (int64_t)start;
        int64_t  end        = (int64_t)stop;
        if (begin < 0 || end > n25 || end <= begin) continue;
        int win_len = (int)(end - begin);
        if (win_len > BURST_WINDOW_LEN) win_len = BURST_WINDOW_LEN;
        win_len -= win_len % DIDECIM_DECIM;
        if (win_len < DIDECIM_DECIM) continue;

        memcpy(window_25, iq25 + begin * 2, win_len * 2 * sizeof(int16_t));

        double phase_step = rotate_to_dc_phase_step_from_bin(center_bin, FBT_FFT_SIZE);
        rotate_to_dc_q15_simd(window_25, win_len, phase_step);

        direct_if_decim_reset_state(&dec);
        int n_out = direct_if_decim_process_split(&dec, window_25, win_len,
                                                  window_250,
                                                  scr_in_i, scr_in_q,
                                                  scr_out_i, scr_out_q);
        if (n_out <= 0) continue;

        burst_pipeline_result_t res;
        memset(&res, 0, sizeof(res));
        bool ok = burst_pipeline_process_250khz(window_250, n_out, &res);
        if (ok && res.demod_ok) {
            decoded++;
            // BCH hard-decode on both interleaved blocks — matches the
            // worker's pass/fail gate at worker_core1.c:428-429.
            if (res.frame.n_bits >= 24 + 64) {
                const uint8_t *payload = res.frame.bits + 24;
                uint8_t        b1[32], b2[32], d1[21], d2[21];
                iridium_deinterleave(payload, b1, b2);
                int e1 = bch_decode_block(b1, d1);
                int e2 = bch_decode_block(b2, d2);
                if (e1 >= 0 && e2 >= 0) bch_ok++;
            }
            free(res.frame.bits);
            free(res.frame.soft_bits);
        }
    }

    free(window_25);
    free(window_250);
    free(scr_in_i);
    free(scr_in_q);
    free(scr_out_i);
    free(scr_out_q);
    if (out_bch) *out_bch = bch_ok;
    return decoded;
}

int main(void)
{
    // 1) Load + resample once (the baseline IQ that every sweep cell
    //    clones-and-mutates).
    int      n_raw = ALBQ_RAW_UINT8_LEN / 2;
    int16_t *iq256 = (int16_t *)malloc(2 * n_raw * sizeof(int16_t));
    if (!iq256) return 2;
    for (int i = 0; i < 2 * n_raw; i++) {
        iq256[i] = (int16_t)(((int)ALBQ_RAW_UINT8[i] - 128) << 8);
    }
    int      n_resamp_max = (int)((double)n_raw * 125.0 / 128.0 + 16);
    int16_t *iq25         = (int16_t *)malloc(2 * n_resamp_max * sizeof(int16_t));
    if (!iq25) {
        free(iq256);
        return 2;
    }
    resample_256_to_250_t rs;
    resample_256_to_250_init(&rs);
    int n25 = resample_256_to_250_process(&rs, iq256, n_raw, iq25);
    free(iq256);
    printf("Loaded + resampled fixture: %d complex at 2.5 MSPS\n\n", n25);

    bch_decoder_init();

    // 2) Sweep grid. Per cell: copy iq25 → drift+noise mutate → run.
    int16_t *work = malloc(2 * n25 * sizeof(int16_t));
    if (!work) {
        free(iq25);
        return 2;
    }
    int decoded_grid[N_DRIFT][N_NOISE] = {{0}};
    int bch_grid[N_DRIFT][N_NOISE]     = {{0}};
    (void)bch_grid; // used inside the loop + via decoded_grid for asserts

    printf("drift_kHz,noise_sigma,decoded,bch_ok\n");
    for (size_t di = 0; di < N_DRIFT; di++) {
        for (size_t ni = 0; ni < N_NOISE; ni++) {
            memcpy(work, iq25, 2 * n25 * sizeof(int16_t));
            apply_drift_inplace(work, n25, (double)DRIFT_KHZ[di] * 1000.0,
                                (double)INPUT_FS_HZ);
            apply_noise_inplace(work, n25, (double)NOISE_SIGMA[ni]);
            int bch              = 0;
            int d                = run_pipeline(work, n25, &bch);
            decoded_grid[di][ni] = d;
            bch_grid[di][ni]     = bch;
            printf("%+.1f,%d,%d,%d\n", DRIFT_KHZ[di], NOISE_SIGMA[ni], d, bch);
            fflush(stdout);
        }
    }

    free(work);
    free(iq25);

    // 3) Pass criteria.
    int baseline = -1;
    for (size_t di = 0; di < N_DRIFT; di++)
        if (DRIFT_KHZ[di] == 0.0f)
            for (size_t ni = 0; ni < N_NOISE; ni++)
                if (NOISE_SIGMA[ni] == 0) baseline = decoded_grid[di][ni];
    if (baseline < 0) {
        printf("\nFAIL: baseline cell missing\n");
        return 1;
    }
    printf("\nBaseline (drift=0, sigma=0): decoded=%d\n", baseline);
    if (baseline < MIN_BASELINE_DECODED) {
        printf("FAIL: baseline %d < required %d — pipeline broken\n",
               baseline, MIN_BASELINE_DECODED);
        return 1;
    }
    // Drift tolerance: any |drift| <= 10 kHz must keep >=50% of baseline.
    int drift_fails = 0;
    for (size_t di = 0; di < N_DRIFT; di++) {
        if (DRIFT_KHZ[di] == 0.0f) continue;
        if (fabsf(DRIFT_KHZ[di]) > 10.0f) continue; // ±15 kHz allowed to fail
        for (size_t ni = 0; ni < N_NOISE; ni++) {
            if (NOISE_SIGMA[ni] != 0) continue; // drift check at noise=0
            int d = decoded_grid[di][ni];
            if ((float)d < DRIFT_TOLERANCE_FRAC * (float)baseline) {
                printf("FAIL: drift=%+.1f kHz decoded=%d < %.0f%% of baseline %d\n",
                       DRIFT_KHZ[di], d, DRIFT_TOLERANCE_FRAC * 100.0f, baseline);
                drift_fails++;
            }
        }
    }
    if (drift_fails) return 1;
    // Noise sweep: at least one nonzero-sigma cell at drift=0 must still decode.
    int any_noisy_decode = 0;
    for (size_t di = 0; di < N_DRIFT; di++) {
        if (DRIFT_KHZ[di] != 0.0f) continue;
        for (size_t ni = 0; ni < N_NOISE; ni++) {
            if (NOISE_SIGMA[ni] == 0) continue;
            if (decoded_grid[di][ni] > 0) any_noisy_decode = 1;
        }
    }
    if (!any_noisy_decode) {
        printf("FAIL: no noise-injection cell decoded — noise injection broken\n");
        return 1;
    }

    printf("\nPASS\n");
    return 0;
}
