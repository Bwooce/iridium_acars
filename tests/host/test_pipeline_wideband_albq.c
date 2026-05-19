// test_pipeline_wideband_albq.c — Phase 3.6.M step 4: full
// wideband-path-A replacement test.
//
// Pipeline:
//   1. Load ALBQ raw fixture (uint8 IQ at 2.56 MSPS).
//   2. Convert + (optional) resample to 2.5 MSPS as int16 IQ.
//   3. Feed FBT_FFT_SIZE-sized chunks to fft_burst_tagger; collect
//      (start_sample, center_bin) tags for new bursts.
//   4. For each detected burst:
//        a. Slice a window from the cached 2.5 MSPS stream around the
//           burst (pre + burst_pre_len, post + burst_post_len padded).
//        b. Rotate by -relative_freq via q15_freq_shift_inplace.
//        c. Decimate 10× via direct_if_decim → 250 ksps int16.
//        d. Run through burst_pipeline_process_250khz.
//        e. Count decoded.
//   5. Report total decoded count vs gr-iridium's known 65/65 on
//      this fixture.
//
// What "success" looks like:
//   - At least matches path A's current 17/99 (i.e. the new front
//     end isn't worse than what we have)
//   - Ideally approaches path C's 64/65 (which used Python scipy
//     for the front end with a manifest of gri-tagged positions)
//   - Anything in between is informative: tells us the front-end
//     parity gap.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "fft_burst_tagger.h"
#include "fft_sc16_2048.h"
#include "direct_if_decim.h"
#include "resample_256_to_250.h"
#include "burst_pipeline.h"
#include "qpsk_demod.h"
#include "uw_correlator.h"

#include "fixture_albq_raw.h"

// Match gr-iridium: 2.5 MSPS, 2048-pt FFT every 2048 samples.
// Our fixture is 2.56 MSPS — close enough for the wideband detector
// (Iridium channel spacing 41.67 kHz; 4 kHz rate-error is small).
// Burst windows are sized at 2.5 MSPS to match the burst_pipeline's
// 250 ksps target after 10× decim.
#define INPUT_FS_HZ      2500000
#define BURST_PRE_LEN    (2 * FBT_FFT_SIZE)            // = 4096
#define BURST_POST_LEN   ((int)(INPUT_FS_HZ * 16e-3))  // = 40000
#define BURST_WINDOW_LEN (BURST_PRE_LEN + BURST_POST_LEN)
#define BURST_WINDOW_250K (BURST_WINDOW_LEN / DIDECIM_DECIM)

static int32_t s_baseline_history[FBT_HISTORY_SIZE * FBT_FFT_SIZE];

// Resample 2.56 MSPS uint8 IQ → 2.5 MSPS int16 IQ via the gri-aligned
// 125/128 polyphase Kaiser resampler (resample_256_to_250 / firmr_s16).
//
// We first convert u8 → int16 with the scale gri uses (cu8 - 128) × 256,
// then run the resampler. Output length ≈ n_in × 125 / 128.
static int resample_2_56_to_2_5(const uint8_t *in_u8, int n_in_complex,
                                  int16_t *out_iq) {
    int16_t *in_s16 = (int16_t *)malloc(2 * n_in_complex * sizeof(int16_t));
    if (!in_s16) return 0;
    for (int k = 0; k < 2 * n_in_complex; k++) {
        in_s16[k] = ((int16_t)in_u8[k] - 128) << 8;     // ±127 → ±32512
    }
    static resample_256_to_250_t r;
    resample_256_to_250_init(&r);
    int n_out = resample_256_to_250_process(&r, in_s16, n_in_complex, out_iq);
    free(in_s16);
    return n_out;
}

// Compute Q15 rotation parameters for a tagged burst.
// relative_frequency = (center_bin - N/2) / N (cycles/sample at input fs)
// Phase increment per sample = -2π × relative_frequency rad.
static void compute_rotation(int center_bin, int n_total_samples,
                              int16_t *cs_q, int16_t *ss_q) {
    double rel_f = ((double)center_bin - (double)FBT_FFT_SIZE / 2.0)
                   / (double)FBT_FFT_SIZE;
    double dphi  = -2.0 * 3.14159265358979323846 * rel_f;
    *cs_q = (int16_t)lrint(cos(dphi) * 32767.0);
    *ss_q = (int16_t)lrint(sin(dphi) * 32767.0);
    (void)n_total_samples;
}

// q15_freq_shift_inplace is defined static inside burst_pipeline.c;
// we replicate the same math here so this test doesn't link to
// burst_pipeline's private symbols. Multiply iq[k] by phasor
// p_init * exp(j·dphi)^k.
static void q15_rotate(int16_t *iq, int n,
                        int16_t cs_q, int16_t ss_q) {
    int16_t pr = 32767, pi = 0;     // phase starts at exp(j·0)
    for (int k = 0; k < n; k++) {
        int32_t r = iq[2 * k + 0];
        int32_t v = iq[2 * k + 1];
        int32_t nr = (r * pr - v * pi) >> 15;
        int32_t ni = (r * pi + v * pr) >> 15;
        iq[2 * k + 0] = (int16_t)(nr > 32767 ? 32767 : nr < -32768 ? -32768 : nr);
        iq[2 * k + 1] = (int16_t)(ni > 32767 ? 32767 : ni < -32768 ? -32768 : ni);
        // Advance phasor by cs_q + j·ss_q
        int32_t npr = (pr * cs_q - pi * ss_q) >> 15;
        int32_t npi = (pr * ss_q + pi * cs_q) >> 15;
        pr = (int16_t)(npr > 32767 ? 32767 : npr < -32768 ? -32768 : npr);
        pi = (int16_t)(npi > 32767 ? 32767 : npi < -32768 ? -32768 : npi);
    }
}

// When DUMP_TARGET_BIN env var is set, the burst whose center_bin is
// closest to that value gets its burst_pipeline stages dumped to
// /tmp/host_signals/ (via burst_pipeline_set_dump_once). Used to feed
// the dsp_compare.py stagewise tool against gri's debug dumps for the
// same physical burst. Default: gri id=30 lands near bin 1209
// (freq +225 kHz / bin_width 1220 Hz + N/2).
static int parse_dump_target_bin(void) {
    const char *s = getenv("DUMP_TARGET_BIN");
    if (!s) return -1;
    return atoi(s);
}

int main(void) {
    // 1) Load raw fixture, resample to 2.5 MSPS int16 IQ.
    size_t n_in_u8 = ALBQ_RAW_UINT8_LEN / 2;
    int16_t *iq25 = malloc(2 * (n_in_u8 * 125 / 128 + 1) * sizeof(int16_t));
    if (!iq25) { fprintf(stderr, "malloc failed\n"); return 2; }
    int n25 = resample_2_56_to_2_5(ALBQ_RAW_UINT8, (int)n_in_u8, iq25);
    printf("Loaded fixture: %d complex samples at 2.5 MSPS (%.2f ms)\n",
           n25, (double)n25 / 2.5e3);

    // 2) Init the burst tagger.
    fft_burst_tagger_t *t = fft_burst_tagger_init(
        BURST_PRE_LEN, BURST_POST_LEN,
        /*burst_width=*/ 32,
        /*threshold_db=*/ 10.0f,
        s_baseline_history);
    if (!t) { fprintf(stderr, "tagger init\n"); free(iq25); return 2; }
    fft_burst_tagger_set_start(t, 0);

    // 3) Step through input + collect burst tags.
    //    We collect them ALL first, then process each in step 4.
    //    On P4 this would be streaming with a PSRAM ring buffer.
    typedef struct {
        uint64_t start_sample;
        int      center_bin;
    } tag_t;
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
        for (int i = 0; i < n_new && n_tags < MAX_TAGS; i++) {
            tags[n_tags].start_sample = new_bursts[i].start;
            tags[n_tags].center_bin   = new_bursts[i].center_bin;
            n_tags++;
        }
    }
    printf("Tagger emitted %d bursts\n", n_tags);

    // 4) For each tag, build a window, rotate, decim, run pipeline.
    direct_if_decim_t dec;
    direct_if_decim_init(&dec);

    int16_t *window_25 = malloc(2 * BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *window_250 = malloc(2 * BURST_WINDOW_250K * sizeof(int16_t));
    if (!window_25 || !window_250) {
        fprintf(stderr, "alloc\n"); return 2;
    }

    int decoded = 0;
    int pipeline_ok = 0;
    int uw_found = 0;
    // Stage-dump targeting (env-driven).
    int target_bin = parse_dump_target_bin();
    int best_dump_match_idx = -1;
    int best_dump_match_dist = 1 << 30;
    if (target_bin >= 0) {
        for (int i = 0; i < n_tags; i++) {
            int dist = abs(tags[i].center_bin - target_bin);
            if (dist < best_dump_match_dist) {
                best_dump_match_dist = dist;
                best_dump_match_idx = i;
            }
        }
        if (best_dump_match_idx >= 0) {
            printf("Will dump stages for burst %d (bin %d, target was %d)\n",
                   best_dump_match_idx,
                   tags[best_dump_match_idx].center_bin, target_bin);
        }
    }
    for (int i = 0; i < n_tags; i++) {
        uint64_t start = tags[i].start_sample;
        int center_bin = tags[i].center_bin;
        // Window centred on start. start_sample is already
        // burst_pre_len before the actual envelope.
        int64_t begin = (int64_t)start;
        int64_t end   = begin + BURST_WINDOW_LEN;
        if (begin < 0 || end > n25) continue;

        memcpy(window_25, iq25 + begin * 2,
               BURST_WINDOW_LEN * 2 * sizeof(int16_t));

        // Rotate by -relative_frequency
        int16_t cs_q, ss_q;
        compute_rotation(center_bin, BURST_WINDOW_LEN, &cs_q, &ss_q);
        q15_rotate(window_25, BURST_WINDOW_LEN, cs_q, ss_q);

        // Decim 10×
        int n_out = direct_if_decim_process(&dec, window_25,
                                              BURST_WINDOW_LEN, window_250);
        if (n_out <= 0) continue;

        // Pipeline
        burst_pipeline_result_t res;
        memset(&res, 0, sizeof(res));
        if (i == best_dump_match_idx) {
            int sysrc = system("mkdir -p /tmp/host_signals");
            (void)sysrc;
            burst_pipeline_set_dump_once("/tmp/host_signals");
            // Also dump the pre-burst-pipeline 250 ksps input so the
            // stagewise compare can see our equivalent of gri's
            // signal-filtered-deci.
            FILE *f = fopen("/tmp/host_signals/03_resamp_250k.cf32", "wb");
            if (f) {
                for (int k = 0; k < n_out; k++) {
                    float fr = (float)window_250[2*k+0] / 32768.0f;
                    float fi = (float)window_250[2*k+1] / 32768.0f;
                    fwrite(&fr, 4, 1, f);
                    fwrite(&fi, 4, 1, f);
                }
                fclose(f);
                printf("dumped pre-pipeline 03_resamp_250k.cf32 (%d cplx)\n",
                       n_out);
            }
        }
        bool ok = burst_pipeline_process_250khz(window_250, n_out, &res);
        if (ok) pipeline_ok++;
        if (res.uw_res.direction != UW_DIR_UNKNOWN) uw_found++;
        if (res.demod_ok) {
            decoded++;
            free(res.frame.bits);
        }
    }

    printf("\nSummary:\n");
    printf("  Bursts tagged:    %d\n", n_tags);
    printf("  Pipeline ran:     %d\n", pipeline_ok);
    printf("  UW found:         %d\n", uw_found);
    printf("  DECODED:          %d\n", decoded);
    printf("\nReference: gr-iridium decodes 65 on this fixture; path A\n");
    printf("(polyphase channelizer) currently at 17/99; path C (Python\n");
    printf("scipy front end + manifest tagging) at 64/65.\n");

    fft_burst_tagger_destroy(t);
    free(window_250);
    free(window_25);
    free(iq25);

    return 0;       // always pass — this is a measurement, not a gate
}
