// test_triage_fast_pass.c — P1.5a Design A validation: the fast-pass
// demod-reject triage (burst_pipeline_triage) against the full
// pipeline, on the same self-contained ALBQ harness as
// test_pipeline_wideband_resampled (fixture → firmware resampler →
// tagger → rotate → decim → per-burst pipeline).
//
// Gates (all must hold):
//  1. POSITIVE CONTROL: every burst the full pipeline decodes on its
//     FIRST try_decode_frame attempt (burst_pipeline_last_first_
//     search_start() == 0) must be ACCEPTED by the triage pass run on
//     the truncated window. Triage runs the identical head + the
//     identical single-attempt criterion, so a violation means the
//     truncation (DC mean / RRC tail) changed a verdict — a recall
//     regression the design forbids.
//  2. END-TO-END A/B: with triage gating escalation (accept → full
//     pipeline, reject → drop), the decode count must stay at or
//     above the same floor test_pipeline_wideband_resampled enforces
//     (55). Retry-recovered decodes (first_search_start > 0) are the
//     known, counted recall-risk population — reported explicitly.
//  3. UNIT VERDICTS: a synthetic noise burst and a synthetic
//     wideband impulse burst must both be REJECTED (that's the junk
//     population triage exists to shed), and at least one real
//     fixture burst must be ACCEPTED.
//
// The triage input is a COPY of the first BURST_PIPELINE_TRIAGE_LEN_
// 250K complex samples of the full 250 ksps window — matching the
// device flow, where the worker decimates a truncated extraction
// (identical streaming-FIR prefix) into the same scratch and, on
// accept, re-extracts the full window from the ring so the escalated
// pass is bit-identical to today's.

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

// Deterministic LCG (numerical recipes constants) so the synthetic
// noise/impulse verdict cases are reproducible run-to-run.
static uint32_t s_lcg = 0x12345678u;
static int16_t  lcg_i16(int amp)
{
    s_lcg = s_lcg * 1664525u + 1013904223u;
    return (int16_t)((int32_t)(s_lcg >> 16) % (2 * amp + 1) - amp);
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
    printf("Tagger emitted %d bursts\n", n_tags);

    direct_if_decim_t dec;
    direct_if_decim_init(&dec);

    int16_t *window_25  = malloc(2 * BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *window_250 = malloc(2 * BURST_WINDOW_250K * sizeof(int16_t));
    int16_t *triage_buf = malloc(2 * BURST_PIPELINE_TRIAGE_LEN_250K * sizeof(int16_t));
    int16_t *scr_in_i   = malloc(BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *scr_in_q   = malloc(BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *scr_out_i  = malloc(BURST_WINDOW_250K * sizeof(int16_t));
    int16_t *scr_out_q  = malloc(BURST_WINDOW_250K * sizeof(int16_t));
    if (!window_25 || !window_250 || !triage_buf ||
        !scr_in_i || !scr_in_q || !scr_out_i || !scr_out_q) {
        fprintf(stderr, "alloc\n");
        return 2;
    }

    // ---- Per-burst: triage on the truncated copy, full pipeline on
    //      the intact window, cross-classification ----
    int decoded_total  = 0; // full pipeline decodes (triage OFF baseline)
    int decoded_first  = 0; //   ... whose first frame decoded at search_start 0
    int decoded_retry  = 0; //   ... whose first frame needed the retry loop
    int triage_accepts = 0;
    int decoded_on     = 0; // decodes surviving triage gating (triage ON count)
    int pos_ctl_fail   = 0; // first-attempt decodes REJECTED by triage
    int fails          = 0;

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

        // Triage pass on a truncated COPY (triage mutates in place;
        // the full pass must see the pristine window).
        int n_tri = n_out;
        if (n_tri > BURST_PIPELINE_TRIAGE_LEN_250K)
            n_tri = BURST_PIPELINE_TRIAGE_LEN_250K;
        memcpy(triage_buf, window_250, n_tri * 2 * sizeof(int16_t));
        bool accept = burst_pipeline_triage(triage_buf, n_tri);
        if (accept) triage_accepts++;

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
            if (first_ss == 0 && !accept) {
                pos_ctl_fail++;
                printf("POSITIVE-CONTROL VIOLATION: burst %d (start=%llu "
                       "bin=%d) decoded at first attempt but triage "
                       "REJECTED it\n",
                       i, (unsigned long long)start, center_bin);
            }
            free(res.frame.bits);
            free(res.frame.soft_bits); // #112
        }
    }

    printf("\nTriage fast-pass A/B on the ALBQ fixture:\n");
    printf("  Bursts tagged:            %d\n", n_tags);
    printf("  Decoded (triage OFF):     %d\n", decoded_total);
    printf("    at first attempt:       %d\n", decoded_first);
    printf("    via retry loop:         %d  (recall-risk population)\n",
           decoded_retry);
    printf("  Triage accepts:           %d\n", triage_accepts);
    printf("  Decoded (triage ON):      %d\n", decoded_on);

    // Gate 1: positive control.
    if (pos_ctl_fail > 0) {
        printf("FAIL: positive control — %d first-attempt decode(s) "
               "rejected by triage\n",
               pos_ctl_fail);
        fails++;
    } else {
        printf("PASS: positive control — all %d first-attempt decodes "
               "accepted by triage\n",
               decoded_first);
    }

    // Gate 2: end-to-end floor with triage ON. Same floor as
    // test_pipeline_wideband_resampled (55; steady state 63 with
    // triage OFF).
    const int DECODE_FLOOR = 55;
    if (decoded_on < DECODE_FLOOR) {
        printf("FAIL: triage-ON decode %d < floor %d\n",
               decoded_on, DECODE_FLOOR);
        fails++;
    } else {
        printf("PASS: triage-ON decode %d (floor %d)\n",
               decoded_on, DECODE_FLOOR);
    }

    // Gate 3a: at least one real burst accepted (positive verdict).
    if (triage_accepts < 1) {
        printf("FAIL: triage accepted no fixture burst\n");
        fails++;
    }

    // ---- Gate 3b/3c: synthetic junk verdicts ----
    // Noise burst: uniform noise at a bench-plausible level, one full
    // triage window long. Must REJECT (qpsk_demod's UW diffs<=2 check
    // shouldn't pass on noise).
    {
        int n = BURST_PIPELINE_TRIAGE_LEN_250K;
        for (int k = 0; k < n * 2; k++)
            triage_buf[k] = lcg_i16(6000);
        bool accept = burst_pipeline_triage(triage_buf, n);
        if (accept) {
            printf("FAIL: triage ACCEPTED synthetic noise\n");
            fails++;
        } else {
            printf("PASS: synthetic noise rejected\n");
        }
    }
    // Impulse burst: near-silent floor with a short full-scale
    // impulse — the band-wide junk signature from the bench soak
    // (P1.5). Must REJECT.
    {
        int n = BURST_PIPELINE_TRIAGE_LEN_250K;
        for (int k = 0; k < n * 2; k++)
            triage_buf[k] = lcg_i16(60);
        for (int k = 2000; k < 2200 && k < n; k++) {
            triage_buf[k * 2 + 0] = (k & 1) ? 30000 : -30000;
            triage_buf[k * 2 + 1] = (k & 2) ? 30000 : -30000;
        }
        bool accept = burst_pipeline_triage(triage_buf, n);
        if (accept) {
            printf("FAIL: triage ACCEPTED synthetic impulse\n");
            fails++;
        } else {
            printf("PASS: synthetic impulse rejected\n");
        }
    }

    fft_burst_tagger_destroy(t);
    free(scr_in_i);
    free(scr_in_q);
    free(scr_out_i);
    free(scr_out_q);
    free(triage_buf);
    free(window_250);
    free(window_25);
    free(iq25);
    free(iq256);

    if (fails > 0) {
        printf("\nFAIL: %d triage gate(s) failed\n", fails);
        return 1;
    }
    printf("\nPASS: all triage gates\n");
    return 0;
}
