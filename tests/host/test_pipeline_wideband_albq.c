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
#include "fixture_albq_golden_bits.h"

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
// Upper bound on the per-burst window. gri publishes variable-length
// PDUs from `start` (= d_index - burst_pre_len) to `stop` (= last_active
// + burst_post_len). Single-frame bursts span ~16 ms = 40000 samples
// at 2.5 MSPS; multi-frame bursts (gri's handle_multiple_frames_per_burst)
// keep last_active advancing as long as the signal stays above
// threshold, often 50-100 ms total. We bound at 250 ms which exceeds
// any Iridium burst type and matches gri's max_burst_len default
// (sample_rate * 0.09 = 225 ms). Allocations use this max; the
// per-burst slice is the actual gone.stop - gone.start.
#define BURST_WINDOW_MAX_MS  250
#define BURST_WINDOW_LEN ((int)(INPUT_FS_HZ * BURST_WINDOW_MAX_MS / 1000))  // = 625000
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
//
// DUMP_START_SAMPLE is the unambiguous selector: target the burst
// whose start_sample (in 2.5 MSPS units) is closest to the given value.
// Use when multiple bursts share a bin -- e.g. gri_id=0 is at
// start_sample=1045958 and bin 1038, but other bursts also hit bin 1038.
static int parse_dump_target_bin(void) {
    const char *s = getenv("DUMP_TARGET_BIN");
    if (!s) return -1;
    return atoi(s);
}
static long long parse_dump_start_sample(void) {
    const char *s = getenv("DUMP_START_SAMPLE");
    if (!s) return -1;
    return atoll(s);
}

// =========================================================================
// Host-side golden-bit comparison (mirrors the device path in
// worker_core1.c). Same tolerances, same buckets, same printout format
// so device vs host runs read identically. The split:
//   - device path computes BCH error counts (e1, e2) per block; this
//     host test does not (the host wideband pipeline doesn't run BCH),
//     so the "bch=" column always prints "skip" here.
//   - device dumps the first 3 GOLDEN-BITDUMP rows so an offline aligner
//     can search for the bit-permutation that fits; host does the same.
// =========================================================================
#define HG_TIME_TOL_2500K   125000
#define HG_FREQ_TOL_HZ      20000

static uint32_t hg_decoded         = 0;
static uint32_t hg_matched         = 0;
static uint32_t hg_unmatched       = 0;
static uint32_t hg_exact           = 0;
static uint32_t hg_close           = 0;
static uint32_t hg_partial         = 0;
static uint32_t hg_divergent       = 0;
static uint32_t hg_total_bits      = 0;
static uint32_t hg_total_errors    = 0;
static uint32_t hg_len_eq          = 0;
static uint32_t hg_len_short       = 0;
static uint8_t  hg_claimed[128];

typedef struct {
    int gri_id;
    int host_n_bits;
    int gri_n_bits;
    int compared_bits;
    int errors;
    int bucket;       // 0=exact 1=close 2=partial 3=divergent 4=unmatched
    int gri_conf;
} hg_row_t;
static hg_row_t hg_rows[128];
static int      hg_n_rows = 0;

static int hg_dump_count = 0;

static void host_golden_compare(uint32_t start_sample, float rel_freq_hz,
                                 const uint8_t *bits, int n_bits)
{
    hg_decoded++;
    int best = -1;
    int64_t best_score = INT64_MAX;
    for (int g = 0; g < FIXTURE_ALBQ_RAW_GOLDEN_COUNT; g++) {
        if (hg_claimed[g]) continue;
        const golden_burst_t *e = &FIXTURE_ALBQ_RAW_GOLDEN_BURSTS[g];
        int64_t ds = (int64_t)start_sample - (int64_t)e->start_sample_2500k;
        if (ds < 0) ds = -ds;
        if (ds > HG_TIME_TOL_2500K) continue;
        int64_t df = (int64_t)rel_freq_hz - (int64_t)e->freq_offset_hz;
        if (df < 0) df = -df;
        if (df > HG_FREQ_TOL_HZ) continue;
        int64_t score = (ds * 100 / HG_TIME_TOL_2500K)
                       + (df * 100 / HG_FREQ_TOL_HZ);
        if (score < best_score) { best_score = score; best = g; }
    }
    if (best < 0) {
        hg_unmatched++;
        if (hg_n_rows < (int)(sizeof(hg_rows)/sizeof(hg_rows[0]))) {
            hg_rows[hg_n_rows++] = (hg_row_t){
                .gri_id = -1, .host_n_bits = n_bits,
                .gri_n_bits = 0, .compared_bits = 0, .errors = 0,
                .bucket = 4, .gri_conf = -1,
            };
        }
        return;
    }
    hg_claimed[best] = 1;
    const golden_burst_t *e = &FIXTURE_ALBQ_RAW_GOLDEN_BURSTS[best];
    int cmp_n = n_bits < e->gri_n_bits ? n_bits : e->gri_n_bits;
    int errors = 0;
    for (int k = 0; k < cmp_n; k++) {
        if (bits[k] != e->gri_bits[k]) errors++;
    }
    int ber_pct_for_dump = cmp_n > 0 ? errors * 100 / cmp_n : 0;
    if (hg_dump_count < 3 || ber_pct_for_dump >= 5) {
        char dev_str[400], gri_str[400];
        int nd = cmp_n < 384 ? cmp_n : 384;
        for (int k = 0; k < nd; k++) {
            dev_str[k] = bits[k] ? '1' : '0';
            gri_str[k] = e->gri_bits[k] ? '1' : '0';
        }
        dev_str[nd] = 0; gri_str[nd] = 0;
        printf("GOLDEN-BITDUMP gri_id=%d host_n=%d gri_n=%d errors=%d/%d\n",
               e->gri_id, n_bits, e->gri_n_bits, errors, cmp_n);
        printf("GOLDEN-BITDUMP   dev=%s\n", dev_str);
        printf("GOLDEN-BITDUMP   gri=%s\n", gri_str);
        // Error-position histogram: count errors per 32-bit window.
        if (errors > 0 && cmp_n > 0) {
            printf("GOLDEN-BITDUMP   err_per_32bits:");
            for (int w = 0; w < cmp_n; w += 32) {
                int we = 0;
                int we_end = w + 32 < cmp_n ? w + 32 : cmp_n;
                for (int k = w; k < we_end; k++) {
                    if (bits[k] != e->gri_bits[k]) we++;
                }
                printf(" [%d-%d]=%d", w, we_end - 1, we);
            }
            printf("\n");
        }
        hg_dump_count++;
    }
    hg_matched++;
    hg_total_bits   += (uint32_t)cmp_n;
    hg_total_errors += (uint32_t)errors;
    if (n_bits == e->gri_n_bits) hg_len_eq++;
    else if (n_bits < e->gri_n_bits) hg_len_short++;

    int bucket;
    if (cmp_n == 0) bucket = 4;
    else {
        int ber_pct = errors * 100 / cmp_n;
        if (errors == 0)      { hg_exact++;     bucket = 0; }
        else if (ber_pct < 5) { hg_close++;     bucket = 1; }
        else if (ber_pct < 25){ hg_partial++;   bucket = 2; }
        else                  { hg_divergent++; bucket = 3; }
    }
    if (hg_n_rows < (int)(sizeof(hg_rows)/sizeof(hg_rows[0]))) {
        hg_rows[hg_n_rows++] = (hg_row_t){
            .gri_id = e->gri_id, .host_n_bits = n_bits,
            .gri_n_bits = e->gri_n_bits, .compared_bits = cmp_n,
            .errors = errors, .bucket = bucket, .gri_conf = e->conf_pct,
        };
    }
}

static void host_golden_print_summary(void)
{
    int n_gri = FIXTURE_ALBQ_RAW_GOLDEN_COUNT;
    int n_missed = 0;
    for (int g = 0; g < n_gri; g++) if (!hg_claimed[g]) n_missed++;
    double recall    = n_gri > 0 ? 100.0 * (double)hg_matched / (double)n_gri : 0.0;
    double precision = hg_decoded > 0 ? 100.0 * (double)hg_matched / (double)hg_decoded : 0.0;
    printf("\n");
    printf("GOLDEN: gri_total=%d host_decoded=%u matched=%u "
           "missed_by_host=%d unmatched_host=%u "
           "(recall=%.1f%% precision=%.1f%%)\n",
           n_gri, hg_decoded, hg_matched, n_missed, hg_unmatched,
           recall, precision);
    printf("GOLDEN: histogram exact=%u close(BER<5%%)=%u partial(<25%%)=%u "
           "divergent(>=25%%)=%u\n",
           hg_exact, hg_close, hg_partial, hg_divergent);
    if (hg_total_bits > 0) {
        printf("GOLDEN: overall BER %u/%u = %.2f%% "
               "(length_eq=%u length_short=%u)\n",
               hg_total_errors, hg_total_bits,
               100.0 * (double)hg_total_errors / (double)hg_total_bits,
               hg_len_eq, hg_len_short);
    }
    for (int i = 0; i < hg_n_rows; i++) {
        const hg_row_t *r = &hg_rows[i];
        if (r->bucket == 4 && r->gri_id < 0) {
            printf("GOLDEN[%d]: UNMATCHED host_n=%d\n", i, r->host_n_bits);
        } else {
            const char *tag = r->bucket == 0 ? "EXACT"
                            : r->bucket == 1 ? "CLOSE"
                            : r->bucket == 2 ? "PARTIAL"
                            :                  "DIVERG";
            printf("GOLDEN[%d]: gri_id=%d conf=%d%% %s "
                   "raw_err=%d/%d (host_n=%d gri_n=%d)\n",
                   i, r->gri_id, r->gri_conf, tag, r->errors,
                   r->compared_bits, r->host_n_bits, r->gri_n_bits);
        }
    }
    for (int g = 0; g < n_gri; g++) {
        if (hg_claimed[g]) continue;
        const golden_burst_t *e = &FIXTURE_ALBQ_RAW_GOLDEN_BURSTS[g];
        printf("GOLDEN-MISSED: gri_id=%d start=%llu freq_off=%ld "
               "conf=%d gri_n_bits=%d\n",
               e->gri_id, (unsigned long long)e->start_sample_2500k,
               (long)e->freq_offset_hz, e->conf_pct, e->gri_n_bits);
    }
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

    // 2) Init the burst tagger. Threshold 10 dB. This LOOKS 3 dB tighter
    // than gr-iridium's 7 dB default, but our tagger doesn't fold the
    // window ENBW into the threshold while gri does (see
    // fft_burst_tagger.c:131-141). Net effective threshold:
    //   gri:  7.0 dB - 10*log10(ENBW) ≈ 4.6 dB  (after ENBW absorbed)
    //   ours: 10.0 dB - 10*log10(ENBW) ≈ 7.6 dB (no ENBW absorption)
    // So our 10 dB is ~0.6 dB tighter than gri's 7 dB on the same
    // mag²/baseline comparison. Defensible. (Tested 7 dB: matched drops
    // 54→4 -- our downstream pipeline doesn't handle gri's false-positive
    // tag rate gracefully yet.)
    fft_burst_tagger_t *t = fft_burst_tagger_init(
        BURST_PRE_LEN, BURST_POST_LEN,
        /*burst_width=*/ 32,
        /*threshold_db=*/ 10.0f,
        s_baseline_history);
    if (!t) { fprintf(stderr, "tagger init\n"); free(iq25); return 2; }
    fft_burst_tagger_set_start(t, 0);

    // 3) Step through input + collect GONE burst tags (matches gri's
    //    publish-on-gone behaviour). Each gone event carries (start,
    //    stop, center_bin) — the variable-length PDU gri's
    //    tagged_burst_to_pdu would emit. Using `gone` (not `new`) is
    //    important: multi-frame bursts stay active across many FFT
    //    steps, and gri's handle_multiple_frames_per_burst expects
    //    the whole PDU window. Using `new` with a fixed BURST_POST_LEN
    //    truncates those to ~16 ms and loses the trailing frames.
    typedef struct {
        uint64_t start_sample;
        uint64_t stop_sample;
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
        for (int i = 0; i < n_gone && n_tags < MAX_TAGS; i++) {
            tags[n_tags].start_sample = gone_bursts[i].start;
            tags[n_tags].stop_sample  = gone_bursts[i].stop;
            tags[n_tags].center_bin   = gone_bursts[i].center_bin;
            n_tags++;
        }
    }
    // Drain any still-active bursts at the end of the input: force-emit
    // their `gone` records so we don't miss bursts that didn't naturally
    // time out before the fixture ended.
    {
        int n_flush = FBT_MAX_BURSTS;
        fbt_burst_t flushed[FBT_MAX_BURSTS];
        fft_burst_tagger_flush(t, flushed, &n_flush);
        for (int i = 0; i < n_flush && n_tags < MAX_TAGS; i++) {
            tags[n_tags].start_sample = flushed[i].start;
            tags[n_tags].stop_sample  = flushed[i].stop;
            tags[n_tags].center_bin   = flushed[i].center_bin;
            n_tags++;
        }
        printf("Tagger flushed %d still-active bursts at end-of-stream\n",
               n_flush);
    }
    printf("Tagger emitted %d gone bursts total\n", n_tags);

    // 4) For each tag, build a window, rotate, decim, run pipeline.
    direct_if_decim_t dec;
    direct_if_decim_init(&dec);

    int16_t *window_25 = malloc(2 * BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *window_250 = malloc(2 * BURST_WINDOW_250K * sizeof(int16_t));
    // Scratch buffers for direct_if_decim_process_split. Caller-provided
    // so the same buffers can be reused across bursts (PSRAM on P4).
    int16_t *scr_in_i  = malloc(BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *scr_in_q  = malloc(BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *scr_out_i = malloc(BURST_WINDOW_250K * sizeof(int16_t));
    int16_t *scr_out_q = malloc(BURST_WINDOW_250K * sizeof(int16_t));
    if (!window_25 || !window_250 || !scr_in_i || !scr_in_q
        || !scr_out_i || !scr_out_q) {
        fprintf(stderr, "alloc\n"); return 2;
    }

    int decoded = 0;
    int pipeline_ok = 0;
    int uw_found = 0;
    // Stage-dump targeting (env-driven). DUMP_START_SAMPLE takes priority
    // over DUMP_TARGET_BIN when both are set.
    int target_bin = parse_dump_target_bin();
    long long target_start = parse_dump_start_sample();
    // CLIP_WINDOW_N: hypothesis-test knob. Caps every burst's 250 ksps
    // window length at the given sample count before the pipeline runs.
    // Used to test task #70: gri's gone-event windows for clean bursts
    // average ~4000-4500 samples at 250 ksps; ours come in at ~9000+ for
    // the same bursts. Capping at e.g. 5000 simulates gri's tighter window
    // boundary -- if gri_id=0 / 550 then decode cleanly, the tagger
    // window-length divergence is confirmed as the residual BER lever.
    const char *clip_env = getenv("CLIP_WINDOW_N");
    int clip_window_n = clip_env ? atoi(clip_env) : 0;
    if (clip_window_n > 0) {
        printf("CLIP_WINDOW_N=%d (post-decim 250 ksps samples)\n", clip_window_n);
    }
    int best_dump_match_idx = -1;
    long long best_dump_match_dist = (long long)1 << 60;
    if (target_start >= 0) {
        // When both DUMP_START_SAMPLE and DUMP_TARGET_BIN are given,
        // restrict to tags within ±4 bins of target_bin (~5 kHz) so a
        // burst at the same time but different frequency doesn't win.
        for (int i = 0; i < n_tags; i++) {
            if (target_bin >= 0 && abs(tags[i].center_bin - target_bin) > 4) continue;
            long long dist = (long long)tags[i].start_sample - target_start;
            if (dist < 0) dist = -dist;
            if (dist < best_dump_match_dist) {
                best_dump_match_dist = dist;
                best_dump_match_idx = i;
            }
        }
        if (best_dump_match_idx >= 0) {
            printf("Will dump stages for burst %d (start_sample=%llu bin=%d, "
                   "target start=%lld bin=%d)\n",
                   best_dump_match_idx,
                   (unsigned long long)tags[best_dump_match_idx].start_sample,
                   tags[best_dump_match_idx].center_bin, target_start, target_bin);
        } else {
            printf("DUMP_START_SAMPLE=%lld bin=%d: no matching tag found\n",
                   target_start, target_bin);
        }
    } else if (target_bin >= 0) {
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
        uint64_t stop  = tags[i].stop_sample;
        int center_bin = tags[i].center_bin;
        // Variable-length window from gri's gone-event (start, stop).
        // start_sample is already burst_pre_len before the actual
        // envelope. stop = last_active + burst_post_len so it overshoots
        // the burst by burst_post_len — that's the gri-equivalent
        // padding for the matched filter's tail.
        int64_t begin = (int64_t)start;
        int64_t end   = (int64_t)stop;
        if (begin < 0 || end > n25 || end <= begin) continue;
        int win_len = (int)(end - begin);
        if (win_len > BURST_WINDOW_LEN) win_len = BURST_WINDOW_LEN;
        // Round to multiple of DIDECIM_DECIM so direct_if_decim has
        // a clean integer output count.
        win_len -= win_len % DIDECIM_DECIM;
        if (win_len < DIDECIM_DECIM) continue;

        memcpy(window_25, iq25 + begin * 2,
               win_len * 2 * sizeof(int16_t));

        // Rotate by -relative_frequency (shared helper). Using the
        // platform-best dispatcher — on host this resolves to the
        // chunked scalar reference (rotate_to_dc_q15_simd_ref); on
        // P4 firmware it will resolve to the PIE asm once that
        // lands. Either path is validated by test_rotate_to_dc to
        // match the q15_inc baseline that the original 59-decodes
        // run used.
        double phase_step = rotate_to_dc_phase_step_from_bin(center_bin,
                                                              FBT_FFT_SIZE);
        rotate_to_dc_q15_simd(window_25, win_len, phase_step);

        // Decim 10× via the split (deinterleave + real-FIR ×2) path —
        // same I/O as direct_if_decim_process but uses streaming
        // delay-line FIR that maps directly to dsps_fird_s16_arp4 on
        // P4. Reset state between bursts so leftover history from a
        // previous burst doesn't leak in.
        direct_if_decim_reset_state(&dec);
        int n_out = direct_if_decim_process_split(&dec, window_25,
                                                    win_len,
                                                    window_250,
                                                    scr_in_i, scr_in_q,
                                                    scr_out_i, scr_out_q);
        if (n_out <= 0) continue;
        if (clip_window_n > 0 && n_out > clip_window_n) {
            n_out = clip_window_n;
        }

        // Pipeline
        burst_pipeline_result_t res;
        memset(&res, 0, sizeof(res));
        // Two ways to select the dump target:
        //  - DUMP_TARGET_BIN env var: dump the burst whose center_bin is
        //    closest to the requested bin (default mode).
        //  - DUMP_FIRST_UW env var: dump the FIRST burst whose
        //    burst_pipeline succeeds and UW direction != UNKNOWN. This
        //    matches QPSK_DUMP's trigger (it also fires on first
        //    UW-lock), so the stage dumps + QPSK CSV cover the same
        //    physical burst -- needed to compare apples-to-apples
        //    against gri's debug-id dumps for the burst that
        //    golden-matches gri_id N.
        static bool dump_first_uw = false;
        static bool dump_first_uw_inited = false;
        static bool dump_first_uw_fired = false;
        if (!dump_first_uw_inited) {
            const char *e = getenv("DUMP_FIRST_UW");
            dump_first_uw = (e != NULL && e[0] != 0);
            dump_first_uw_inited = true;
        }
        bool do_dump = false;
        if (!dump_first_uw && i == best_dump_match_idx) {
            do_dump = true;
            printf("dump-mode: bin-target (burst %d at bin %d)\n",
                   i, tags[i].center_bin);
        }
        if (do_dump) {
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
            if (dump_first_uw && !dump_first_uw_fired) {
                printf("FIRST-UW-LOCK burst tag #%d bin=%d -- set DUMP_TARGET_BIN=%d "
                       "and re-run to dump this burst's stages.\n",
                       i, tags[i].center_bin, tags[i].center_bin);
                dump_first_uw_fired = true;
            }
            // Device-equivalent compare: same tolerance window, same
            // bucketing as worker_core1.c's golden_compare_burst().
            // rel_freq_hz derived from center_bin via the same formula
            // dsp_processor uses.
            int signed_bin = tags[i].center_bin - FBT_FFT_SIZE / 2;
            float rel_freq_hz = (float)signed_bin * (float)INPUT_FS_HZ
                                 / (float)FBT_FFT_SIZE;
            host_golden_compare((uint32_t)begin, rel_freq_hz,
                                 res.frame.bits, res.frame.n_bits);
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

    host_golden_print_summary();

    fft_burst_tagger_destroy(t);
    free(window_250);
    free(window_25);
    free(scr_in_i);  free(scr_in_q);
    free(scr_out_i); free(scr_out_q);
    free(iq25);

    return 0;       // always pass — this is a measurement, not a gate
}
