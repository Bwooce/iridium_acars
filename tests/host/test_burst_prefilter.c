// test_burst_prefilter.c — P1.5b redesign validation: the cheap,
// feature-based burst pre-discriminator (burst_prefilter) that replaces
// the P1.5a UW/CFO "fast-pass" triage. Same self-contained ALBQ harness
// as test_pipeline_wideband_resampled (fixture → firmware resampler →
// tagger → rotate → decim → per-burst pipeline).
//
// Gates (all must hold):
//  1. POSITIVE CONTROL (recall): EVERY burst the full pipeline decodes —
//     including retry-loop recoveries, not just first-attempt — must be
//     ACCEPTED by the pre-filter. The P1.5a triage could only accept
//     first-attempt decodes (it WAS a single-attempt decode); this
//     feature filter runs before correlation and must not reject any
//     real, decodable burst. Zero false-rejects required.
//  2. END-TO-END floor: with the pre-filter gating escalation
//     (accept → full pipeline, reject → drop), the decode count must
//     stay at/above the test_pipeline_wideband_resampled floor (55).
//  3. JUNK VERDICTS: a synthetic noise burst and a synthetic impulse
//     burst must both be REJECTED, and at least one real fixture burst
//     ACCEPTED. Junk-rejection rate on the tagged-but-undecoded
//     population is reported.
//
// The pre-filter runs on the SAME decimated 250 ksps window the full
// pipeline decodes (it does not mutate the buffer), giving a clean A/B.

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
#include "burst_prefilter.h"
#include "qpsk_demod.h"
#include "uw_correlator.h"
#include "fixture_albq_raw.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define INPUT_FS_HZ 2500000
#define BURST_PRE_LEN (2 * FBT_FFT_SIZE)
#define BURST_POST_LEN ((int)(INPUT_FS_HZ * 16e-3))
#define BURST_WINDOW_LEN ((int)(INPUT_FS_HZ * 250 / 1000)) // = 625000
#define BURST_WINDOW_250K (BURST_WINDOW_LEN / DIDECIM_DECIM)

static int32_t s_baseline_history[FBT_HISTORY_SIZE * FBT_FFT_SIZE];

// Deterministic LCG (numerical recipes constants).
static uint32_t s_lcg = 0x12345678u;
static int16_t  lcg_i16(int amp)
{
    s_lcg = s_lcg * 1664525u + 1013904223u;
    return (int16_t)((int32_t)(s_lcg >> 16) % (2 * amp + 1) - amp);
}

// Synthesize a complex-tone burst at frequency f_hz (relative to DC, the
// working rate is 250 ksps), `dur` samples long, starting at `off`, over a
// uniform-noise floor of amplitude `nfloor`, tone amplitude `amp`. Fills
// the whole `n`-complex buffer. Used to build junk (short) and
// signal-like (long) synthetic populations.
static void make_tone_burst(int16_t *buf, int n, int off, int dur,
                            double f_hz, int amp, int nfloor)
{
    for (int k = 0; k < n * 2; k++)
        buf[k] = lcg_i16(nfloor);
    double w = 2.0 * M_PI * f_hz / 250000.0;
    for (int k = 0; k < dur && (off + k) < n; k++) {
        int     i      = off + k;
        double  re     = amp * cos(w * k);
        double  im     = amp * sin(w * k);
        int32_t vr     = (int32_t)buf[i * 2 + 0] + (int32_t)lrint(re);
        int32_t vi     = (int32_t)buf[i * 2 + 1] + (int32_t)lrint(im);
        buf[i * 2 + 0] = (int16_t)(vr > 32767 ? 32767 : vr < -32768 ? -32768
                                                                    : vr);
        buf[i * 2 + 1] = (int16_t)(vi > 32767 ? 32767 : vi < -32768 ? -32768
                                                                    : vi);
    }
}

int main(void)
{
    // ---- Harness identical to test_pipeline_wideband_resampled ----
    int      n_raw = ALBQ_RAW_UINT8_LEN / 2;
    int16_t *iq256 = (int16_t *)malloc(2 * n_raw * sizeof(int16_t));
    if (!iq256) return 2;
    for (int i = 0; i < 2 * n_raw; i++) {
        iq256[i] = (int16_t)(((int)ALBQ_RAW_UINT8[i] - 128) << 8);
    }

    int      n_resamp_max = (int)((double)n_raw * 125.0 / 128.0 + 16);
    int16_t *iq25         = (int16_t *)malloc(2 * n_resamp_max * sizeof(int16_t));
    if (!iq25) return 2;
    resample_256_to_250_t rs;
    resample_256_to_250_init(&rs);
    int n25 = resample_256_to_250_process(&rs, iq256, n_raw, iq25);

    fft_burst_tagger_t *t = fft_burst_tagger_init(
        BURST_PRE_LEN, BURST_POST_LEN,
        /*burst_width=*/32,
        /*threshold_db=*/14.0f,
        s_baseline_history);
    if (!t) {
        fprintf(stderr, "tagger init\n");
        return 2;
    }
    fft_burst_tagger_set_start(t, 0);

    typedef struct {
        uint64_t start;
        uint64_t stop;
        int      center_bin;
        int      width_bins;
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
            tags[n_tags].width_bins = gone_bursts[i].width_bins;
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
            tags[n_tags].width_bins = flushed[i].width_bins;
            n_tags++;
        }
    }
    printf("Tagger emitted %d bursts\n", n_tags);

    direct_if_decim_t dec;
    direct_if_decim_init(&dec);

    int16_t *window_25  = malloc(2 * BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *window_250 = malloc(2 * BURST_WINDOW_250K * sizeof(int16_t));
    int16_t *scr_in_i   = malloc(BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *scr_in_q   = malloc(BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *scr_out_i  = malloc(BURST_WINDOW_250K * sizeof(int16_t));
    int16_t *scr_out_q  = malloc(BURST_WINDOW_250K * sizeof(int16_t));
    if (!window_25 || !window_250 ||
        !scr_in_i || !scr_in_q || !scr_out_i || !scr_out_q) {
        fprintf(stderr, "alloc\n");
        return 2;
    }

    int decoded_total = 0; // full pipeline decodes (pre-filter OFF baseline)
    int decoded_first = 0;
    int decoded_retry = 0;
    int prefilter_acc = 0; // pre-filter accepts (any tagged burst)
    int decoded_on    = 0; // decodes surviving pre-filter gating
    int pos_ctl_fail  = 0; // decodes REJECTED by the pre-filter
    int junk_rejected = 0; // tagged-but-undecoded bursts rejected
    int junk_total    = 0;
    int fails         = 0;

    for (int i = 0; i < n_tags; i++) {
        uint64_t start      = tags[i].start;
        uint64_t stop       = tags[i].stop;
        int      center_bin = tags[i].center_bin;
        int64_t  begin      = (int64_t)start;
        int64_t  end        = (int64_t)stop;
        if (begin < 0 || end > n25 || end <= begin) continue;
        int win_len = (int)(end - begin);
        if (win_len > BURST_WINDOW_LEN) win_len = BURST_WINDOW_LEN;
        win_len -= win_len % DIDECIM_DECIM;
        if (win_len < DIDECIM_DECIM) continue;

        memcpy(window_25, iq25 + begin * 2,
               win_len * 2 * sizeof(int16_t));

        double phase_step =
            rotate_to_dc_phase_step_from_bin(center_bin, FBT_FFT_SIZE);
        rotate_to_dc_q15_simd(window_25, win_len, phase_step);

        direct_if_decim_reset_state(&dec);
        int n_out = direct_if_decim_process_split(&dec, window_25,
                                                  win_len,
                                                  window_250,
                                                  scr_in_i, scr_in_q,
                                                  scr_out_i, scr_out_q);
        if (n_out <= 0) continue;

        // Pre-filter verdict on the intact window (does not mutate it).
        // Real tagger width plumbed through so the width gate sees exactly
        // what the device worker would (BURST_WIDTH_BINS).
        burst_prefilter_result_t pf;
        bool                     accept =
            burst_prefilter(window_250, n_out, tags[i].width_bins, &pf);
        if (accept) prefilter_acc++;

        // Full pipeline, exactly as test_pipeline_wideband_resampled.
        burst_pipeline_result_t res;
        memset(&res, 0, sizeof(res));
        (void)burst_pipeline_process_250khz(window_250, n_out, &res);
        int first_ss = burst_pipeline_last_first_search_start();
        if (res.demod_ok) {
            decoded_total++;
            if (first_ss == 0)
                decoded_first++;
            else
                decoded_retry++;
            if (accept) decoded_on++;
            if (!accept) {
                pos_ctl_fail++;
                printf("POSITIVE-CONTROL VIOLATION: burst %d (start=%llu "
                       "bin=%d first_ss=%d) DECODED but pre-filter REJECTED "
                       "[dur_ok=%d active=%d snr_ok=%d snr=%.1f dB]\n",
                       i, (unsigned long long)start, center_bin, first_ss,
                       pf.dur_ok, pf.active_len, pf.snr_ok, pf.channel_snr_db);
            }
            free(res.frame.bits);
            free(res.frame.soft_bits); // #112
        } else {
            // Tagged but did not decode → junk / undecodable population.
            junk_total++;
            if (!accept) junk_rejected++;
        }
    }

    printf("\nBurst pre-filter A/B on the ALBQ fixture:\n");
    printf("  Bursts tagged:              %d\n", n_tags);
    printf("  Decoded (pre-filter OFF):   %d\n", decoded_total);
    printf("    at first attempt:         %d\n", decoded_first);
    printf("    via retry loop:           %d\n", decoded_retry);
    printf("  Pre-filter accepts (all):   %d\n", prefilter_acc);
    printf("  Decoded (pre-filter ON):    %d\n", decoded_on);
    printf("  Tagged-but-undecoded:       %d\n", junk_total);
    printf("    rejected by pre-filter:   %d  (%.0f%% junk rejection)\n",
           junk_rejected,
           junk_total ? 100.0 * junk_rejected / junk_total : 0.0);

    // Gate 1: positive control — no real decode rejected.
    if (pos_ctl_fail > 0) {
        printf("FAIL: positive control — %d decode(s) rejected by pre-filter\n",
               pos_ctl_fail);
        fails++;
    } else {
        printf("PASS: positive control — all %d decodes accepted "
               "(0 false-rejects)\n",
               decoded_total);
    }

    // Gate 2: end-to-end floor with pre-filter ON.
    const int DECODE_FLOOR = 55;
    if (decoded_on < DECODE_FLOOR) {
        printf("FAIL: pre-filter-ON decode %d < floor %d\n",
               decoded_on, DECODE_FLOOR);
        fails++;
    } else {
        printf("PASS: pre-filter-ON decode %d (floor %d)\n",
               decoded_on, DECODE_FLOOR);
    }

    // Gate 3a: at least one real burst accepted.
    if (prefilter_acc < 1) {
        printf("FAIL: pre-filter accepted no fixture burst\n");
        fails++;
    }

    // ---- Gate 3b/3c: synthetic junk verdicts ----
    int      junk_n = BURST_WINDOW_250K;
    int16_t *junk   = malloc(2 * junk_n * sizeof(int16_t));
    if (!junk) return 2;

    // Noise burst: uniform noise filling a full window. Flat spectrum,
    // long duration → must be rejected by the spectral (channel-SNR)
    // gate.
    {
        for (int k = 0; k < junk_n * 2; k++)
            junk[k] = lcg_i16(6000);
        burst_prefilter_result_t pf;
        bool                     accept = burst_prefilter(junk, junk_n, 0, &pf);
        if (accept) {
            printf("FAIL: pre-filter ACCEPTED synthetic noise "
                   "[dur_ok=%d snr=%.1f dB]\n",
                   pf.dur_ok, pf.channel_snr_db);
            fails++;
        } else {
            printf("PASS: synthetic noise rejected "
                   "[dur_ok=%d active=%d snr_ok=%d snr=%.1f dB]\n",
                   pf.dur_ok, pf.active_len, pf.snr_ok, pf.channel_snr_db);
        }
    }
    // Impulse burst: near-silent floor with a short full-scale impulse —
    // the ≤2.5 ms bench junk signature. Short → must be rejected by the
    // duration gate.
    {
        for (int k = 0; k < junk_n * 2; k++)
            junk[k] = lcg_i16(60);
        for (int k = 2000; k < 2200 && k < junk_n; k++) {
            junk[k * 2 + 0] = (k & 1) ? 30000 : -30000;
            junk[k * 2 + 1] = (k & 2) ? 30000 : -30000;
        }
        burst_prefilter_result_t pf;
        bool                     accept = burst_prefilter(junk, junk_n, 0, &pf);
        if (accept) {
            printf("FAIL: pre-filter ACCEPTED synthetic impulse "
                   "[active=%d snr=%.1f dB]\n",
                   pf.active_len, pf.channel_snr_db);
            fails++;
        } else {
            printf("PASS: synthetic impulse rejected "
                   "[dur_ok=%d active=%d snr_ok=%d snr=%.1f dB]\n",
                   pf.dur_ok, pf.active_len, pf.snr_ok, pf.channel_snr_db);
        }
    }

    // ---- Gate 3d: synthetic junk / signal POPULATION sweep ----
    // Positive control FIRST (per project discipline): a population of
    // long, in-band, narrowband tone bursts (the shape of a real
    // narrowband signal) must be ACCEPTED — proving the gates don't
    // simply reject everything. THEN measure junk rejection on two
    // populations matching the documented bench signatures:
    //   (i) short narrowband impulses (1-bin tone, dur < one frame) —
    //       the "flat-spectrum impulses of 1-3 FFT bins, short duration"
    //       flood; must fail the duration gate.
    //  (ii) flat wideband noise windows; must fail the channel-SNR gate.
    // Amplitudes/durations are drawn well inside each regime (not at the
    // 800-sample / 14 dB knife-edge) — the thresholds themselves stay
    // gr-grounded and fixed.
    {
        enum { POP = 60 };
        int      junk_n2 = BURST_WINDOW_250K;
        int16_t *b       = malloc(2 * junk_n2 * sizeof(int16_t));
        if (!b) return 2;

        // (a) POSITIVE CONTROL: long narrowband tone bursts (accept).
        int sig_acc = 0;
        for (int j = 0; j < POP; j++) {
            double f   = -18000.0 + (36000.0 * j) / (POP - 1); // in ±18 kHz
            int    dur = 1400 + (j % 5) * 800;                 // ≥ one frame
            int    off = 500 + (j * 37) % 2000;
            make_tone_burst(b, junk_n2, off, dur, f, /*amp=*/6000, /*nfloor=*/250);
            if (burst_prefilter(b, junk_n2, 0, NULL)) sig_acc++;
        }
        printf("\nSynthetic population sweep (%d each):\n", POP);
        printf("  long narrowband tone accepted: %d/%d  (positive control)\n",
               sig_acc, POP);
        if (sig_acc < POP) {
            printf("FAIL: pre-filter rejected %d long narrowband signal(s)\n",
                   POP - sig_acc);
            fails++;
        }

        // (b) short narrowband impulses (reject via duration gate).
        int imp_rej = 0;
        for (int j = 0; j < POP; j++) {
            double f   = -15000.0 + (30000.0 * j) / (POP - 1);
            int    dur = 80 + (j * 11) % 620; // 80..699 < 800 (one frame)
            int    off = 300 + (j * 53) % 3000;
            make_tone_burst(b, junk_n2, off, dur, f, /*amp=*/12000, /*nfloor=*/200);
            if (!burst_prefilter(b, junk_n2, 0, NULL)) imp_rej++;
        }
        printf("  short narrowband impulse rejected: %d/%d\n", imp_rej, POP);
        if (imp_rej < POP) {
            printf("FAIL: pre-filter accepted %d short impulse(s)\n",
                   POP - imp_rej);
            fails++;
        }

        // (c) flat wideband noise (reject via channel-SNR gate).
        int noise_rej = 0;
        for (int j = 0; j < POP; j++) {
            int amp = 2000 + j * 200;
            for (int k = 0; k < junk_n2 * 2; k++)
                b[k] = lcg_i16(amp);
            if (!burst_prefilter(b, junk_n2, 0, NULL)) noise_rej++;
        }
        printf("  flat wideband noise rejected: %d/%d\n", noise_rej, POP);
        if (noise_rej < POP) {
            printf("FAIL: pre-filter accepted %d flat-noise window(s)\n",
                   POP - noise_rej);
            fails++;
        }
        printf("  overall synthetic junk rejection: %d/%d (%.0f%%)\n",
               imp_rej + noise_rej, 2 * POP,
               100.0 * (imp_rej + noise_rej) / (2 * POP));
        free(b);
    }

    // ---- Gate 0: spectral WIDTH ----
    // Build ONE signal-like burst (long, in-band, narrowband tone) that
    // otherwise passes gates 1+2, then feed it with different tagger
    // width_bins values. The width gate is metadata-only and must:
    //   - ACCEPT it at ~one Iridium channel (34 bins) — never reject a
    //     real, channel-wide burst (the recall-critical direction);
    //   - ACCEPT it when width is unmeasured (width_bins <= 0);
    //   - REJECT it only when width exceeds the broadband threshold
    //     (>100 bins ≈ 3 channels), leaving the SAME samples otherwise
    //     acceptable — proving the rejection is the WIDTH, not the signal.
    {
        int      wn = BURST_WINDOW_250K;
        int16_t *w  = malloc(2 * wn * sizeof(int16_t));
        if (!w) return 2;
        make_tone_burst(w, wn, /*off=*/500, /*dur=*/3000, /*f_hz=*/5000.0,
                        /*amp=*/6000, /*nfloor=*/250);

        burst_prefilter_result_t pf_n, pf_u, pf_w;
        bool                     acc_narrow = burst_prefilter(w, wn, /*width_bins=*/34, &pf_n);
        bool                     acc_unmeas = burst_prefilter(w, wn, /*width_bins=*/0, &pf_u);
        bool                     acc_wide   = burst_prefilter(w, wn, /*width_bins=*/150, &pf_w);

        printf("\nWidth gate (same signal-like burst, varying width):\n");
        if (!acc_narrow) {
            printf("FAIL: width gate REJECTED a ~1-channel burst "
                   "[width_ok=%d dur_ok=%d snr_ok=%d snr=%.1f dB]\n",
                   pf_n.width_ok, pf_n.dur_ok, pf_n.snr_ok,
                   pf_n.channel_snr_db);
            fails++;
        } else {
            printf("  1-channel (34 bins) accepted\n");
        }
        if (!acc_unmeas) {
            printf("FAIL: width gate REJECTED an unmeasured-width burst\n");
            fails++;
        } else {
            printf("  unmeasured width (0) accepted\n");
        }
        if (acc_wide) {
            printf("FAIL: width gate ACCEPTED a broadband burst "
                   "[width_ok=%d width_bins=%d]\n",
                   pf_w.width_ok, pf_w.width_bins);
            fails++;
        } else {
            printf("  broadband (150 bins) rejected [width_ok=%d]\n",
                   pf_w.width_ok);
        }
        free(w);
    }

    fft_burst_tagger_destroy(t);
    free(scr_in_i);
    free(scr_in_q);
    free(scr_out_i);
    free(scr_out_q);
    free(window_250);
    free(window_25);
    free(iq25);
    free(iq256);

    if (fails > 0) {
        printf("\nFAIL: %d pre-filter gate(s) failed\n", fails);
        return 1;
    }
    printf("\nPASS: all pre-filter gates\n");
    return 0;
}
