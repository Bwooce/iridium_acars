// test_pipeline_wideband_resampled.c — same end-to-end measurement as
// test_pipeline_wideband_albq, but feeds the wideband chain via the
// firmware's ingest resampler (resample_256_to_250) instead of reading
// scipy's pre-computed cf32. The decode-count delta between the two
// tests measures the quality cost of the firmware's lower-tap-count
// polyphase resample vs scipy's wider Kaiser filter.
//
// Why this matters: the firmware classifies ~12% of processed bursts;
// the cf32-fed host test classifies ~80% on the same logical input.
// If the resample is the cause, this test will show the regression on
// host (where everything else is identical), proving the hypothesis
// before we invest in a wider firmware resampler.

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

#define INPUT_FS_HZ      2500000
#define BURST_PRE_LEN    (2 * FBT_FFT_SIZE)
#define BURST_POST_LEN   ((int)(INPUT_FS_HZ * 16e-3))
// gri-style variable-length window: see test_pipeline_wideband_albq.c
// for rationale. 250 ms cap matches gri's max_burst_len default.
#define BURST_WINDOW_LEN  ((int)(INPUT_FS_HZ * 250 / 1000))      // = 625000
#define BURST_WINDOW_250K (BURST_WINDOW_LEN / DIDECIM_DECIM)

static int32_t s_baseline_history[FBT_HISTORY_SIZE * FBT_FFT_SIZE];

int main(void) {
    // 1) Load the 2.56 MSPS uint8 fixture and convert to int16 Q15.
    int n_raw = ALBQ_RAW_UINT8_LEN / 2;   // complex sample count
    int16_t *iq256 = (int16_t *)malloc(2 * n_raw * sizeof(int16_t));
    if (!iq256) return 2;
    for (int i = 0; i < 2 * n_raw; i++) {
        iq256[i] = (int16_t)(((int)ALBQ_RAW_UINT8[i] - 128) << 8);
    }
    printf("Loaded fixture: %d complex at 2.56 MSPS (%.2f ms)\n",
           n_raw, (double)n_raw / 2.56e3);

    // 2) Resample 2.56 → 2.5 MSPS via the firmware's resampler.
    int n_resamp_max = (int)((double)n_raw * 125.0 / 128.0 + 16);
    int16_t *iq25 = (int16_t *)malloc(2 * n_resamp_max * sizeof(int16_t));
    if (!iq25) { free(iq256); return 2; }
    resample_256_to_250_t rs;
    resample_256_to_250_init(&rs);
    int n25 = resample_256_to_250_process(&rs, iq256, n_raw, iq25);
    printf("Resampled: %d complex at 2.5 MSPS (firmware's "
           "resample_256_to_250)\n", n25);

    // 3) Init tagger with the same params as test_pipeline_wideband_albq.
    fft_burst_tagger_t *t = fft_burst_tagger_init(
        BURST_PRE_LEN, BURST_POST_LEN,
        /*burst_width=*/ 32,
        /*threshold_db=*/ 14.0f,
        s_baseline_history);
    if (!t) { fprintf(stderr, "tagger init\n"); return 2; }
    fft_burst_tagger_set_start(t, 0);

    // 4) Collect gone burst tags (variable window — see
    // test_pipeline_wideband_albq.c for the gri-alignment rationale).
    typedef struct { uint64_t start; uint64_t stop; int center_bin; } tag_t;
    enum { MAX_TAGS = 256 };
    tag_t tags[MAX_TAGS];
    int n_tags = 0;
    fbt_burst_t new_bursts[FBT_MAX_BURSTS];
    fbt_burst_t gone_bursts[FBT_MAX_BURSTS];
    for (int off = 0; off + FBT_FFT_SIZE <= n25; off += FBT_FFT_SIZE) {
        int n_new = FBT_MAX_BURSTS, n_gone = FBT_MAX_BURSTS;
        fft_burst_tagger_step(t, iq25 + off * 2, NULL,
                               new_bursts, &n_new,
                               gone_bursts, &n_gone);
        for (int i = 0; i < n_gone && n_tags < MAX_TAGS; i++) {
            tags[n_tags].start = gone_bursts[i].start;
            tags[n_tags].stop  = gone_bursts[i].stop;
            tags[n_tags].center_bin = gone_bursts[i].center_bin;
            n_tags++;
        }
    }
    {
        int n_flush = FBT_MAX_BURSTS;
        fbt_burst_t flushed[FBT_MAX_BURSTS];
        fft_burst_tagger_flush(t, flushed, &n_flush);
        for (int i = 0; i < n_flush && n_tags < MAX_TAGS; i++) {
            tags[n_tags].start = flushed[i].start;
            tags[n_tags].stop  = flushed[i].stop;
            tags[n_tags].center_bin = flushed[i].center_bin;
            n_tags++;
        }
    }
    printf("Tagger emitted %d bursts (gone+flush; cf32 reference gets ~133)\n", n_tags);

    // 5) Per-burst processing — same chain as test_pipeline_wideband_albq.
    direct_if_decim_t dec;
    direct_if_decim_init(&dec);

    int16_t *window_25 = malloc(2 * BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *window_250 = malloc(2 * BURST_WINDOW_250K * sizeof(int16_t));
    int16_t *scr_in_i  = malloc(BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *scr_in_q  = malloc(BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *scr_out_i = malloc(BURST_WINDOW_250K * sizeof(int16_t));
    int16_t *scr_out_q = malloc(BURST_WINDOW_250K * sizeof(int16_t));
    if (!window_25 || !window_250
        || !scr_in_i || !scr_in_q || !scr_out_i || !scr_out_q) {
        fprintf(stderr, "alloc\n"); return 2;
    }

    int decoded = 0;
    int pipeline_ok = 0;
    int uw_found = 0;
    for (int i = 0; i < n_tags; i++) {
        uint64_t start = tags[i].start;
        uint64_t stop  = tags[i].stop;
        int center_bin = tags[i].center_bin;
        int64_t begin = (int64_t)start;
        int64_t end   = (int64_t)stop;
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

        burst_pipeline_result_t res;
        memset(&res, 0, sizeof(res));
        bool ok = burst_pipeline_process_250khz(window_250, n_out, &res);
        if (ok) pipeline_ok++;
        if (res.uw_res.direction != UW_DIR_UNKNOWN) uw_found++;
        if (res.demod_ok) {
            decoded++;
            free(res.frame.bits);
            free(res.frame.soft_bits);   // #112
        }
    }

    printf("\nSummary (firmware-style ingest resample):\n");
    printf("  Bursts tagged:    %d\n", n_tags);
    printf("  Pipeline ran:     %d\n", pipeline_ok);
    printf("  UW found:         %d\n", uw_found);
    printf("  DECODED:          %d\n", decoded);
    printf("\nCompare to test_pipeline_wideband_albq (scipy-resampled cf32 input):\n");
    printf("  expected DECODED ≈ 56 if resample quality is the only firmware/host gap\n");

    fft_burst_tagger_destroy(t);
    free(scr_in_i); free(scr_in_q); free(scr_out_i); free(scr_out_q);
    free(window_250); free(window_25);
    free(iq25);
    free(iq256);
    return 0;
}
