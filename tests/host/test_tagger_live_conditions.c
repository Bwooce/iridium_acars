// test_tagger_live_conditions.c — live-RF-conditions regression gate for
// the P1 tagger paths (force-close / burst squelch / noise reset).
//
// WHY THIS TEST EXISTS (2026-07-07): firmware with the P1 tagger changes
// (2ee1d6f on) decodes ZERO bursts over the air across ~16 h while every
// host test passes — including test_pipeline_wideband_resampled (62-63
// decodes on ALBQ) and test_tagger_latch_release. The difference: on live
// RF the burst squelch fires ~15/s and the noise estimate resets every
// 1-3 s; on the clean ALBQ fixture neither path EVER executes. This test
// closes that coverage hole: it feeds the exact ALBQ fixture through the
// exact end-to-end chain of test_pipeline_wideband_resampled, but with a
// deterministic overlay that forces all three P1 paths to fire at live-
// like rates, then gates on the decode count of the ORIGINAL ALBQ bursts.
//
// Overlay design (all bin-centered tones so 2048-sample blocks repeat
// exactly; everything deterministic, no PRNG):
//   1. CW carrier at a quiet bin (away from every ALBQ burst bin),
//      switched ON after the EMA primes and strong enough that its
//      per-bin mag-squared exceeds the EMA slot clamp
//      (INT32_MAX / FBT_HISTORY_SIZE, see ema_step_inner) — the regime
//      where baseline absorption can never retire it, so max-burst-len
//      force-close cycling engages for the rest of the run
//      (gone-burst lengths ~FBT_MAX_BURST_LEN = 225000).
//   2. A broadband tone-flood "impulse" (two interleaved grids of
//      bin-centered tones confined to quiet bins, 17-bin spacing so each
//      tone escapes its neighbour's burst mask) applied for single FFT
//      steps every ~65 ms — each event creates >FBT_SQUELCH_MAX_BURSTS
//      tracked bursts in one step and fires the burst squelch (~15/s,
//      matching the live logs).
//   3. One 4-step flood near the end of the run — four consecutive
//      squelch steps drive squelch_count 3,6,9,12 >= 10 and trigger the
//      noise-estimate reset (fft_burst_tagger_reset_baseline), placed
//      late so its 512-step re-prime blindness only covers the fixture
//      tail (~90 ms) instead of masking the measurement.
//
// The flood grids and the CW avoid every ALBQ burst bin by >= 26 bins
// (>31 kHz), so the original bursts keep their fixture SNR at their own
// bins; the only legitimate losses the overlay can cause are (a) squelch
// force-close truncating an in-flight burst's window and (b) the ~90 ms
// re-prime tail. Both are bounded and small on this fixture, hence the
// PASS floor below.
//
// PASS criteria:
//   V1 (validity) squelch fires >= MIN_SQUELCH_EVENTS times and the
//      noise reset fires >= 1 time — otherwise the overlay failed to
//      exercise the live-only paths and the test is meaningless;
//   V2 (validity) CW force-close cycling observed (>= 2 gone bursts at
//      the CW bin with length >= ~FBT_MAX_BURST_LEN);
//   G1 (gate) decode count on the overlaid input >= DECODE_FLOOR_OVERLAY.
//      A collapse toward 0 while V1/V2 hold reproduces the live regression
//      (squelch/reset/force-close corrupting dispatched burst windows);
//      proportional degradation instead means the live-only failure is
//      NOT a host-reproducible content bug.
//
// NOTE: if this test FAILS, treat it as a reproduction of the live decode
// regression class — it must PASS again after the responsible fix lands.
// Do not raise the overlay amplitudes or move the events to make it pass
// (no test-fitting; see feedback_no_test_fitting).
//
// FINDING (2026-07-07, first run of this test on 2ee1d6f..be3408d): the
// content-corruption hypothesis did NOT reproduce on host. With squelch
// firing 10-16 steps, 1 noise reset, and 2 max-burst-len force-close
// cycles (gone len 225280):
//   PASS 2 decoded 49/63 (proportional; 20 lost to squelch truncation +
//          terminal blindness, only 1 tagged-but-undecodable);
//   PASS 3 (device parity: 16-slot gone buffer + P1.5a triage) decoded
//          48, triage accepting 48/69 — not the live "0 accepts";
//   PASS 4 post-reset re-primed epoch decoded 67/71 (94%).
// The pipelined d_index path (fft_burst_tagger.c tagger_pipe_post_fft)
// is compile-disabled on host AND device (s_pipe_helper_task = NULL at
// init), so it cannot be the live culprit either. The live zero-decode
// regression therefore lives in device-only territory (what the tagger
// is fed, signal_buffer staleness/lap under real load, worker PQ
// starvation at real junk-burst rates, retune interactions) — not in
// the tagger's dispatch content or window geometry.

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fcntl.h>
#include <unistd.h>

#include "burst_pipeline.h"
#include "direct_if_decim.h"
#include "fft_burst_tagger.h"
#include "fft_sc16_2048.h"
#include "fixture_albq_raw.h"
#include "resample_256_to_250.h"
#include "rotate_to_dc.h"
#include "uw_correlator.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define INPUT_FS_HZ 2500000
#define BURST_PRE_LEN (2 * FBT_FFT_SIZE)
#define BURST_POST_LEN ((int)(INPUT_FS_HZ * 16e-3))
#define BURST_WINDOW_LEN ((int)(INPUT_FS_HZ * 250 / 1000)) // 625000
#define THRESHOLD_DB 14.0f
#define BURST_WIDTH 32

// ---- overlay schedule (units: FFT steps of 2048 samples = 0.8192 ms) ----
// EMA primes after FBT_HISTORY_SIZE = 512 steps; fixture has ~1218 steps.
#define CW_ON_STEP 520 // CW appears just after priming
#define CW_BIN 400     // quiet region, 263 bins from nearest ALBQ burst
#define CW_AMP 6000    // per-bin mag2 ~6.3e6 > clamp 4.19e6 (see header)

// Flood events: single-step broadband tone bursts. Spacing alternates
// ~65 ms / ~107 ms (~12/s average, close to the live ~15/s squelch
// rate) — the two >110-step gaps are deliberate: a squelch force-closes
// every tracked burst, so the CW can only reach FBT_MAX_BURST_LEN
// (110 steps) and exercise the max-burst-len force-close during a gap
// longer than that. Events past the fixture length only apply to the
// doubled-fixture pass (PASS 4, post-reset epoch) and keep the same
// cadence there.
static const int FLOOD_EVENT_STEPS[] = {560, 625, 755, 820, 950, 1015,
                                        1900, 1965, 2095, 2160, 2290, 2355};
#define N_FLOOD_EVENTS ((int)(sizeof(FLOOD_EVENT_STEPS) / sizeof(FLOOD_EVENT_STEPS[0])))
// Reset event: 4 consecutive flood steps => squelch_count reaches 12 >= 10.
#define RESET_EVENT_START 1105
#define RESET_EVENT_LEN 4
#define FLOOD_AMP 900 // per tone; grids sized below keep composite < sat16

#define MIN_SQUELCH_EVENTS 5
#define MIN_FORCECLOSE_CYCLES 2
// Expected legitimate losses vs the 63 baseline: squelch truncation of
// in-flight bursts (~events x concurrent bursts) + the ~90 ms unprimed
// tail. Generously that is ~20 decodes; a healthy tagger keeps >= 40.
// The live regression is a collapse to ~0.
#define DECODE_FLOOR_OVERLAY 40
#define DECODE_FLOOR_BASELINE 55 // same floor as pipeline_wideband_resampled

static int32_t s_baseline_history[FBT_HISTORY_SIZE * FBT_FFT_SIZE];

static inline int16_t sat16(int32_t v)
{
    if (v > 32767) return 32767;
    if (v < -32768) return -32768;
    return (int16_t)v;
}

// Bin-centered complex tone accumulated into an int32 2048-sample block
// (same construction as test_tagger_latch_release.c).
static void add_tone_block(int32_t *block_iq, int bin, int amp, double phase0)
{
    double w = 2.0 * M_PI * (double)(bin - FBT_FFT_SIZE / 2) / (double)FBT_FFT_SIZE;
    for (int n = 0; n < FBT_FFT_SIZE; n++) {
        block_iq[2 * n + 0] += (int32_t)lrint((double)amp * cos(w * n + phase0));
        block_iq[2 * n + 1] += (int32_t)lrint((double)amp * sin(w * n + phase0));
    }
}

// ---- quiet-bin bookkeeping -------------------------------------------

static int s_truth_bins[64];
static int s_n_truth_bins;

static void collect_truth_bins(void)
{
    s_n_truth_bins = 0;
    int n          = (int)(sizeof(ALBQ_RAW_BURSTS) / sizeof(ALBQ_RAW_BURSTS[0]));
    for (int i = 0; i < n && s_n_truth_bins < 64; i++) {
        int bin = FBT_FFT_SIZE / 2 +
                  (int)lrint((double)ALBQ_RAW_BURSTS[i].rel_hz *
                             (double)FBT_FFT_SIZE / (double)INPUT_FS_HZ);
        s_truth_bins[s_n_truth_bins++] = bin;
    }
}

static int bin_is_quiet(int bin)
{
    if (bin < 20 || bin > 2028) return 0;
    if (abs(bin - CW_BIN) < 26) return 0;
    if (abs(bin - FBT_FFT_SIZE / 2) < 26) return 0; // stay off DC
    for (int i = 0; i < s_n_truth_bins; i++)
        if (abs(bin - s_truth_bins[i]) < 26) return 0;
    return 1;
}

// ---- overlay application ---------------------------------------------

typedef struct {
    long clipped; // samples saturated while adding the overlay
    int  n_flood_tones;
} overlay_stats_t;

// Apply CW + flood schedule onto iq25 in place. Block-aligned: FFT step s
// covers samples [s*2048, (s+1)*2048).
static void apply_overlay(int16_t *iq25, int n25, overlay_stats_t *st)
{
    static int32_t cw_block[2 * FBT_FFT_SIZE];
    static int32_t flood_block[2 * FBT_FFT_SIZE];
    memset(cw_block, 0, sizeof(cw_block));
    memset(flood_block, 0, sizeof(flood_block));

    add_tone_block(cw_block, CW_BIN, CW_AMP, 0.0);

    // Two interleaved grids, 34-bin pitch, 17-bin offset — every tone is
    // outside every other tone's burst mask (half-width 16), so one flood
    // step can create up to FBT_MAX_BURSTS tracked bursts at once.
    int n_tones = 0;
    for (int bin = 20; bin <= 2028; bin += 17) {
        if (!bin_is_quiet(bin)) continue;
        add_tone_block(flood_block, bin, FLOOD_AMP,
                       2.399963 * (double)n_tones); // golden-angle phases
        n_tones++;
    }
    st->n_flood_tones = n_tones;

    int n_steps = n25 / FBT_FFT_SIZE;
    st->clipped = 0;
    for (int s = 0; s < n_steps; s++) {
        int cw_on    = (s >= CW_ON_STEP);
        int flood_on = 0;
        for (int e = 0; e < N_FLOOD_EVENTS; e++)
            if (s == FLOOD_EVENT_STEPS[e]) flood_on = 1;
        if (s >= RESET_EVENT_START && s < RESET_EVENT_START + RESET_EVENT_LEN)
            flood_on = 1;
        if (!cw_on && !flood_on) continue;

        int16_t *blk = iq25 + (size_t)s * 2 * FBT_FFT_SIZE;
        for (int k = 0; k < 2 * FBT_FFT_SIZE; k++) {
            int32_t v = (int32_t)blk[k];
            if (cw_on) v += cw_block[k];
            if (flood_on) v += flood_block[k];
            int16_t sv = sat16(v);
            if ((int32_t)sv != v) st->clipped++;
            blk[k] = sv;
        }
    }
}

// ---- tagger + decode chain -------------------------------------------

typedef struct {
    uint64_t start;
    uint64_t stop;
    int      center_bin;
    int      decoded; // filled by the decode loop
} tag_t;

enum { MAX_TAGS = 2048 };

typedef struct {
    tag_t    tags[MAX_TAGS];
    int      n_tags;
    int      n_tags_dropped; // tags beyond MAX_TAGS (should stay 0)
    int      pipeline_ok;
    int      uw_found;
    int      decoded;
    int      triage_accepts; // device-parity pass only
    int      triage_rejects; // device-parity pass only
    int      noise_resets;   // primed->unprimed transitions of step()
    int      squelch_steps;  // counted from the tagger's stderr prints
    int      cw_forcecloses; // gone bursts at CW bin with force-close length
    int      cw_tags_any;    // all gone bursts at CW bin (any length)
    uint64_t cw_max_len;     // longest CW gone burst seen
    long     clipped;
} run_result_t;

// Count occurrences of `needle` in file `path`.
static int count_lines_with(const char *path, const char *needle)
{
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    char line[256];
    int  n = 0;
    while (fgets(line, sizeof(line), f))
        if (strstr(line, needle)) n++;
    fclose(f);
    return n;
}

// gone_cap: out_gone capacity per step. FBT_MAX_BURSTS = ideal harness;
// 16 = device parity (dsp_processor.c FBT_GONE_BUF_SIZE — a squelch step
// closes up to 63 bursts at once and the device silently drops the rest).
// with_triage: gate decode behind burst_pipeline_triage on a truncated
// head window, device parity with worker_core1's P1.5a fast pass.
static int run_chain(const int16_t *iq25_in, int n25, int with_overlay,
                     int gone_cap, int with_triage, run_result_t *r)
{
    memset(r, 0, sizeof(*r));

    int16_t *iq25 = (int16_t *)malloc((size_t)2 * n25 * sizeof(int16_t));
    if (!iq25) return -1;
    memcpy(iq25, iq25_in, (size_t)2 * n25 * sizeof(int16_t));

    if (with_overlay) {
        overlay_stats_t ost;
        apply_overlay(iq25, n25, &ost);
        r->clipped = ost.clipped;
        printf("  overlay: %d flood tones, CW at bin %d amp %d, "
               "%ld samples clipped\n",
               ost.n_flood_tones, CW_BIN, CW_AMP, ost.clipped);
    }

    fft_burst_tagger_t *t = fft_burst_tagger_init(
        BURST_PRE_LEN, BURST_POST_LEN, BURST_WIDTH, THRESHOLD_DB,
        s_baseline_history);
    if (!t) {
        free(iq25);
        return -1;
    }
    fft_burst_tagger_set_start(t, 0);

    // Capture stderr around the tagger loop: the squelch and noise-reset
    // paths announce themselves there (fft_burst_tagger.c host branch)
    // and expose no counter API. Host-test-only trick; POSIX dup/dup2.
    const char *errlog = !with_overlay ? "tlc_stderr_baseline.tmp"
                         : with_triage ? "tlc_stderr_parity.tmp"
                                       : "tlc_stderr_overlay.tmp";
    fflush(stderr);
    int saved_err = dup(STDERR_FILENO);
    int logfd     = open(errlog, O_CREAT | O_TRUNC | O_WRONLY, 0644);
    if (logfd >= 0) dup2(logfd, STDERR_FILENO);

    fbt_burst_t nb[FBT_MAX_BURSTS], gb[FBT_MAX_BURSTS];
    bool        prev_primed = false;
    for (int off = 0; off + FBT_FFT_SIZE <= n25; off += FBT_FFT_SIZE) {
        int  n_new = FBT_MAX_BURSTS, n_gone = gone_cap;
        bool primed = fft_burst_tagger_step(t, iq25 + (size_t)off * 2, NULL,
                                            nb, &n_new, gb, &n_gone);
        if (prev_primed && !primed) r->noise_resets++;
        prev_primed = primed;
        for (int i = 0; i < n_gone; i++) {
            uint64_t len = gb[i].stop - gb[i].start;
            if (abs(gb[i].center_bin - CW_BIN) <= 2) {
                r->cw_tags_any++;
                if (len > r->cw_max_len) r->cw_max_len = len;
                if (len >= (uint64_t)FBT_MAX_BURST_LEN) r->cw_forcecloses++;
            }
            if (r->n_tags < MAX_TAGS) {
                r->tags[r->n_tags].start      = gb[i].start;
                r->tags[r->n_tags].stop       = gb[i].stop;
                r->tags[r->n_tags].center_bin = gb[i].center_bin;
                r->n_tags++;
            } else {
                r->n_tags_dropped++;
            }
        }
    }
    {
        int         n_flush = FBT_MAX_BURSTS;
        fbt_burst_t flushed[FBT_MAX_BURSTS];
        fft_burst_tagger_flush(t, flushed, &n_flush);
        for (int i = 0; i < n_flush && r->n_tags < MAX_TAGS; i++) {
            r->tags[r->n_tags].start      = flushed[i].start;
            r->tags[r->n_tags].stop       = flushed[i].stop;
            r->tags[r->n_tags].center_bin = flushed[i].center_bin;
            r->n_tags++;
        }
    }
    fft_burst_tagger_destroy(t);

    // Restore stderr, count the squelch announcements.
    fflush(stderr);
    if (logfd >= 0) {
        dup2(saved_err, STDERR_FILENO);
        close(logfd);
    }
    close(saved_err);
    r->squelch_steps = count_lines_with(errlog, "burst squelch");

    // Per-burst decode: identical chain to test_pipeline_wideband_resampled.
    direct_if_decim_t dec;
    direct_if_decim_init(&dec);

    int      win250_max = BURST_WINDOW_LEN / DIDECIM_DECIM;
    int16_t *triage_buf = malloc((size_t)2 * BURST_PIPELINE_TRIAGE_LEN_250K *
                                 sizeof(int16_t));
    int16_t *window_25  = malloc((size_t)2 * BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *window_250 = malloc((size_t)2 * win250_max * sizeof(int16_t));
    int16_t *scr_in_i   = malloc((size_t)BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *scr_in_q   = malloc((size_t)BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *scr_out_i  = malloc((size_t)win250_max * sizeof(int16_t));
    int16_t *scr_out_q  = malloc((size_t)win250_max * sizeof(int16_t));
    if (!triage_buf || !window_25 || !window_250 || !scr_in_i || !scr_in_q ||
        !scr_out_i || !scr_out_q) {
        free(iq25);
        return -1;
    }

    for (int i = 0; i < r->n_tags; i++) {
        tag_t  *tg    = &r->tags[i];
        int64_t begin = (int64_t)tg->start;
        int64_t end   = (int64_t)tg->stop;
        tg->decoded   = 0;
        if (begin < 0 || end > n25 || end <= begin) continue;
        int win_len = (int)(end - begin);
        if (win_len > BURST_WINDOW_LEN) win_len = BURST_WINDOW_LEN;
        win_len -= win_len % DIDECIM_DECIM;
        if (win_len < DIDECIM_DECIM) continue;

        memcpy(window_25, iq25 + begin * 2, (size_t)win_len * 2 * sizeof(int16_t));
        double phase_step =
            rotate_to_dc_phase_step_from_bin(tg->center_bin, FBT_FFT_SIZE);
        rotate_to_dc_q15_simd(window_25, win_len, phase_step);

        direct_if_decim_reset_state(&dec);
        int n_out = direct_if_decim_process_split(&dec, window_25, win_len,
                                                  window_250, scr_in_i,
                                                  scr_in_q, scr_out_i,
                                                  scr_out_q);
        if (n_out <= 0) continue;

        if (with_triage) {
            // Device parity: worker_core1's P1.5a fast pass triages a
            // truncated head window (same start, same decim phase) with
            // ONE UW attempt at slot 0 before escalating to the full
            // pipeline. Same modelling as test_triage_fast_pass.c —
            // triage mutates its buffer, so run it on a copy.
            int n_tri = n_out;
            if (n_tri > BURST_PIPELINE_TRIAGE_LEN_250K)
                n_tri = BURST_PIPELINE_TRIAGE_LEN_250K;
            memcpy(triage_buf, window_250,
                   (size_t)n_tri * 2 * sizeof(int16_t));
            if (!burst_pipeline_triage(triage_buf, n_tri)) {
                r->triage_rejects++;
                continue; // device sheds the burst here
            }
            r->triage_accepts++;
        }

        burst_pipeline_result_t res;
        memset(&res, 0, sizeof(res));
        bool ok = burst_pipeline_process_250khz(window_250, n_out, &res);
        if (ok) r->pipeline_ok++;
        if (res.uw_res.direction != UW_DIR_UNKNOWN) r->uw_found++;
        if (res.demod_ok) {
            tg->decoded = 1;
            r->decoded++;
            free(res.frame.bits);
            free(res.frame.soft_bits);
        }
    }

    free(scr_out_q);
    free(scr_out_i);
    free(scr_in_q);
    free(scr_in_i);
    free(window_250);
    free(window_25);
    free(triage_buf);
    free(iq25);
    return 0;
}

// Attribute overlay losses: for every DECODED baseline tag, was there an
// overlapping overlay tag at the same bin, and did it decode?
static void attribute_losses(const run_result_t *base, const run_result_t *ovl)
{
    int lost_detection = 0, tagged_not_decoded = 0, still_decoded = 0;
    for (int i = 0; i < base->n_tags; i++) {
        if (!base->tags[i].decoded) continue;
        int best = -1;
        for (int j = 0; j < ovl->n_tags; j++) {
            if (abs(ovl->tags[j].center_bin - base->tags[i].center_bin) > 2)
                continue;
            if (ovl->tags[j].stop <= base->tags[i].start ||
                ovl->tags[j].start >= base->tags[i].stop)
                continue; // no time overlap
            if (best < 0 || ovl->tags[j].decoded) best = j;
        }
        if (best < 0) {
            lost_detection++;
        } else if (ovl->tags[best].decoded) {
            still_decoded++;
        } else {
            tagged_not_decoded++;
        }
    }
    printf("  loss attribution (of %d baseline decodes):\n", base->decoded);
    printf("    still decoded under overlay:      %d\n", still_decoded);
    printf("    tagged but NOT decoded:           %d  <- content/geometry loss\n",
           tagged_not_decoded);
    printf("    detection lost entirely:          %d  <- squelch/blindness loss\n",
           lost_detection);
}

int main(void)
{
    collect_truth_bins();

    // Load fixture and resample, exactly as test_pipeline_wideband_resampled.
    int      n_raw = ALBQ_RAW_UINT8_LEN / 2;
    int16_t *iq256 = (int16_t *)malloc((size_t)2 * n_raw * sizeof(int16_t));
    if (!iq256) return 2;
    for (int i = 0; i < 2 * n_raw; i++)
        iq256[i] = (int16_t)(((int)ALBQ_RAW_UINT8[i] - 128) << 8);

    int      n_resamp_max = (int)((double)n_raw * 125.0 / 128.0 + 16);
    int16_t *iq25         = (int16_t *)malloc((size_t)2 * n_resamp_max * sizeof(int16_t));
    if (!iq25) return 2;
    resample_256_to_250_t rs;
    resample_256_to_250_init(&rs);
    int n25 = resample_256_to_250_process(&rs, iq256, n_raw, iq25);
    free(iq256);
    printf("Fixture: %d complex at 2.5 MSPS (%.1f ms, %d FFT steps)\n", n25,
           (double)n25 / 2.5e3, n25 / FBT_FFT_SIZE);

    run_result_t base, ovl, par;

    printf("\n=== PASS 1: baseline (no overlay) ===\n");
    if (run_chain(iq25, n25, 0, FBT_MAX_BURSTS, 0, &base) != 0) return 2;
    printf("  tags=%d pipeline_ok=%d uw=%d DECODED=%d "
           "(squelch_steps=%d resets=%d)\n",
           base.n_tags, base.pipeline_ok, base.uw_found, base.decoded,
           base.squelch_steps, base.noise_resets);

    printf("\n=== PASS 2: live-conditions overlay ===\n");
    if (run_chain(iq25, n25, 1, FBT_MAX_BURSTS, 0, &ovl) != 0) return 2;
    printf("  tags=%d (dropped=%d) pipeline_ok=%d uw=%d DECODED=%d\n",
           ovl.n_tags, ovl.n_tags_dropped, ovl.pipeline_ok, ovl.uw_found,
           ovl.decoded);
    printf("  squelch_steps=%d noise_resets=%d cw_forcecloses=%d "
           "(cw gone tags: %d, max len %llu)\n",
           ovl.squelch_steps, ovl.noise_resets, ovl.cw_forcecloses,
           ovl.cw_tags_any, (unsigned long long)ovl.cw_max_len);

    attribute_losses(&base, &ovl);

    // PASS 3 — device parity: gone-buffer capped at 16 per step
    // (dsp_processor.c FBT_GONE_BUF_SIZE) and decode gated behind the
    // P1.5a triage fast pass (worker_core1.c), both exactly as the
    // firmware runs them. Compares against PASS 2 to expose losses the
    // DEVICE-side plumbing adds on top of the tagger itself.
    printf("\n=== PASS 3: overlay + device parity (gone cap 16, triage) ===\n");
    if (run_chain(iq25, n25, 1, 16, 1, &par) != 0) return 2;
    printf("  tags=%d pipeline_ok=%d uw=%d DECODED=%d\n",
           par.n_tags, par.pipeline_ok, par.uw_found, par.decoded);
    printf("  triage: accepts=%d rejects=%d\n",
           par.triage_accepts, par.triage_rejects);
    printf("  squelch_steps=%d noise_resets=%d cw_forcecloses=%d\n",
           par.squelch_steps, par.noise_resets, par.cw_forcecloses);
    attribute_losses(&base, &par);

    // PASS 4 — post-reset epoch. Live RF resets the noise estimate every
    // 1-3 s, so the device runs almost entirely in the POST-reset,
    // re-primed state — which passes 1-3 never exercise for decode (the
    // reset sits at the fixture tail). Feed the fixture TWICE through one
    // continuous tagger: the overlay run resets during loop 1, re-primes
    // (512 steps), then loop 2 decode is measured strictly AFTER the
    // re-prime completes and compared against a no-overlay doubled run
    // over the same sample range. A corrupted post-reset state (priming,
    // history indexing, window geometry) would collapse the loop-2 count.
    printf("\n=== PASS 4: doubled fixture — decode in the post-reset epoch ===\n");
    int      n_ext  = 2 * n25;
    int16_t *iq_ext = (int16_t *)malloc((size_t)2 * n_ext * sizeof(int16_t));
    if (!iq_ext) return 2;
    memcpy(iq_ext, iq25, (size_t)2 * n25 * sizeof(int16_t));
    memcpy(iq_ext + (size_t)2 * n25, iq25, (size_t)2 * n25 * sizeof(int16_t));
    free(iq25);

    // Everything from this sample on is decoded by a fully re-primed
    // post-reset tagger (reset ends at RESET_EVENT_START+LEN, re-prime
    // takes FBT_HISTORY_SIZE steps, +2 slack for boundary effects).
    uint64_t post_cut = (uint64_t)(RESET_EVENT_START + RESET_EVENT_LEN +
                                   FBT_HISTORY_SIZE + 2) *
                        FBT_FFT_SIZE;

    run_result_t dbase, dovl;
    if (run_chain(iq_ext, n_ext, 0, FBT_MAX_BURSTS, 0, &dbase) != 0) return 2;
    if (run_chain(iq_ext, n_ext, 1, FBT_MAX_BURSTS, 0, &dovl) != 0) return 2;
    free(iq_ext);

    int l2_base = 0, l2_ovl = 0;
    for (int i = 0; i < dbase.n_tags; i++)
        if (dbase.tags[i].decoded && dbase.tags[i].start >= post_cut) l2_base++;
    for (int i = 0; i < dovl.n_tags; i++)
        if (dovl.tags[i].decoded && dovl.tags[i].start >= post_cut) l2_ovl++;
    printf("  baseline x2: tags=%d decoded=%d (post-reprime window: %d)\n",
           dbase.n_tags, dbase.decoded, l2_base);
    printf("  overlay  x2: tags=%d decoded=%d (post-reprime window: %d) "
           "squelch=%d resets=%d\n",
           dovl.n_tags, dovl.decoded, l2_ovl, dovl.squelch_steps,
           dovl.noise_resets);

    int failures = 0;

    // Baseline sanity: the harness must reproduce the resampled test.
    if (base.decoded < DECODE_FLOOR_BASELINE) {
        printf("FAIL: baseline decode %d < %d — harness broken, not a "
               "tagger result\n",
               base.decoded, DECODE_FLOOR_BASELINE);
        failures++;
    }
    if (base.squelch_steps != 0 || base.noise_resets != 0) {
        printf("FAIL: baseline run fired squelch(%d)/reset(%d) — overlay "
               "leaked into baseline\n",
               base.squelch_steps, base.noise_resets);
        failures++;
    }

    // V1/V2: the overlay must actually exercise the live-only paths.
    if (ovl.squelch_steps < MIN_SQUELCH_EVENTS) {
        printf("FAIL V1: only %d squelch steps (need >= %d) — overlay too "
               "weak, repro invalid\n",
               ovl.squelch_steps, MIN_SQUELCH_EVENTS);
        failures++;
    }
    if (ovl.noise_resets < 1) {
        printf("FAIL V1: noise reset never fired — repro invalid\n");
        failures++;
    }
    if (ovl.cw_forcecloses < MIN_FORCECLOSE_CYCLES) {
        printf("FAIL V2: CW force-close cycling absent (%d gone bursts >= "
               "max-burst-len at bin %d, need >= %d)\n",
               ovl.cw_forcecloses, CW_BIN, MIN_FORCECLOSE_CYCLES);
        failures++;
    }

    // G1: the decode gate. See file header — a collapse here while V1/V2
    // hold is the live regression reproduced on host.
    if (ovl.decoded < DECODE_FLOOR_OVERLAY) {
        printf("FAIL G1: decoded %d under live-conditions overlay "
               "(floor %d, baseline %d) — squelch/force-close/reset paths "
               "are destroying decodable bursts\n",
               ovl.decoded, DECODE_FLOOR_OVERLAY, base.decoded);
        failures++;
    }

    // G2: device-parity pass (gone-cap 16 + triage) must not collapse
    // either — this is the closest host model of the worker's view.
    if (par.decoded < DECODE_FLOOR_OVERLAY) {
        printf("FAIL G2: device-parity decode %d < %d (triage accepts %d) — "
               "16-slot gone buffer or triage is destroying bursts under "
               "live conditions\n",
               par.decoded, DECODE_FLOOR_OVERLAY, par.triage_accepts);
        failures++;
    }

    // G3: post-reset epoch. Validity first: the doubled overlay run must
    // have reset in loop 1, and the doubled baseline must decode a full
    // fixture's worth in the post-cut window.
    if (dovl.noise_resets < 1) {
        printf("FAIL G3v: doubled overlay run never reset — post-reset "
               "epoch not exercised\n");
        failures++;
    } else if (l2_base < DECODE_FLOOR_BASELINE) {
        printf("FAIL G3v: doubled baseline post-cut decode %d < %d — "
               "harness/window accounting broken\n",
               l2_base, DECODE_FLOOR_BASELINE);
        failures++;
    } else if (l2_ovl < DECODE_FLOOR_OVERLAY) {
        printf("FAIL G3: post-reset decode %d (baseline window %d, floor "
               "%d) — the re-primed tagger state after a noise reset is "
               "corrupting detection or window geometry\n",
               l2_ovl, l2_base, DECODE_FLOOR_OVERLAY);
        failures++;
    }

    if (failures) {
        printf("\n[FAIL] %d assertion(s) failed\n", failures);
        return 1;
    }
    printf("\n[pass] tagger live conditions: baseline %d, overlay %d, "
           "device-parity %d, post-reset %d/%d "
           "(squelch=%d resets=%d forcecloses=%d)\n",
           base.decoded, ovl.decoded, par.decoded, l2_ovl, l2_base,
           ovl.squelch_steps, ovl.noise_resets, ovl.cw_forcecloses);
    return 0;
}
