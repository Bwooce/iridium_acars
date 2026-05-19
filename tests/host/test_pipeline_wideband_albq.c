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
#include "rotate_to_dc.h"
#include "burst_pipeline.h"
#include "qpsk_demod.h"
#include "uw_correlator.h"

// We do NOT pull in fixture_albq_raw.h (the 2.56 MSPS u8 fixture) any
// more — instead we read /tmp/host_direct_if/fixture_albq_raw_2500k.cf32
// which is the SAME 2.5 MSPS cf32 stream that gr-iridium consumes
// in this fixture's reference flow. That input file is produced by
// `python3 tests/scripts/direct_if_dump.py` (which runs
// scipy.signal.resample_poly(u8_data, 125, 128) once and writes the
// result for both gri-iridium-extractor and our C wideband test to
// share).
//
// Why: the wideband-tagger comparison was confounded by us doing an
// independent 2.56→2.5 MSPS resample in C while gri saw scipy's
// resample output. Different filters → different input to the
// tagger → guaranteed amplitude/shape divergence we'd then chase as
// downstream bugs. Reading the same cf32 makes "be the same as
// gr-iridium" trivially true at the input boundary.

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

// Read the 2.5 MSPS cf32 (gri's input format) and convert to int16
// with the SAME scale gri's volk path implicitly uses on cf32 → mag²
// (= multiply by 32768, clamp). Returns number of complex samples.
static int load_25msps_cf32(const char *path, int16_t **out_iq) {
    FILE *fh = fopen(path, "rb");
    if (!fh) {
        fprintf(stderr,
                "Could not open %s\n"
                "Run: python3 tests/scripts/direct_if_dump.py\n", path);
        return -1;
    }
    fseek(fh, 0, SEEK_END);
    long bytes = ftell(fh);
    fseek(fh, 0, SEEK_SET);
    int n = (int)(bytes / 8);   // 2 × float32 per complex sample
    float *cf32 = malloc(2 * n * sizeof(float));
    int16_t *s16 = malloc(2 * n * sizeof(int16_t));
    if (!cf32 || !s16) { free(cf32); free(s16); fclose(fh); return -1; }
    size_t got = fread(cf32, sizeof(float), 2 * n, fh);
    fclose(fh);
    if ((int)got != 2 * n) { free(cf32); free(s16); return -1; }
    for (int k = 0; k < 2 * n; k++) {
        float v = cf32[k] * 32768.0f;
        if (v >  32767.0f) v =  32767.0f;
        if (v < -32768.0f) v = -32768.0f;
        s16[k] = (int16_t)lrintf(v);
    }
    free(cf32);
    *out_iq = s16;
    return n;
}

// (Absolute-phase rotation moved to common/iridium_decoder/rotate_to_dc.{h,c}
// — shared with worker_core1.c on the firmware side. See the header
// there for the Q15-magnitude-decay trap that makes this contract
// load-bearing.)

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
    // 1) Load the 2.5 MSPS cf32 — same stream gr-iridium consumes via
    //    iridium-extractor on this fixture. Generated by
    //    tests/scripts/direct_if_dump.py via scipy.signal.resample_poly.
    int16_t *iq25 = NULL;
    int n25 = load_25msps_cf32(
        "/tmp/host_direct_if/fixture_albq_raw_2500k.cf32", &iq25);
    if (n25 <= 0) return 2;
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

        // Rotate by -relative_frequency (shared helper). Using the
        // Q15-incremental variant — same one the firmware worker
        // uses post-task-#58. Validated against the cosf/sinf
        // reference at NMSE ≤ -40 dB by test_rotate_to_dc.
        double phase_step = rotate_to_dc_phase_step_from_bin(center_bin,
                                                              FBT_FFT_SIZE);
        rotate_to_dc_q15_inc(window_25, BURST_WINDOW_LEN, phase_step);

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
