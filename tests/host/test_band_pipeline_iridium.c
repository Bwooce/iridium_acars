// test_band_pipeline_iridium — bit-identity gate for the band_pipeline
// abstraction (VHF/VDL2 foundation).
//
// The worker (p4-usb-host/main/worker_core1.c) now dispatches its two
// per-burst calls through band_pipeline_t instead of calling
// burst_prefilter / burst_pipeline_process_burst directly. Iridium
// regression is the #1 risk of that change, so this test proves the
// iridium_band_pipeline adapter is a ZERO-processing shim: it runs the
// ALBQ fixture end-to-end (same harness as
// test_pipeline_wideband_resampled) and, for EVERY tagged burst, runs
// the decimated window through BOTH paths on identical buffer copies:
//
//   path A (legacy):  burst_prefilter() + burst_pipeline_process_burst()
//   path B (adapter): iridium_band_pipeline()->prefilter / ->process_burst
//
// asserting identical prefilter verdicts (incl. per-gate booleans the
// worker's continuation-rescue reads), identical frame counts, and
// byte-identical bits / soft_bits / direction / demod_ok per frame.
// Any divergence == the adapter is NOT transparent == fail.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "fft_burst_tagger.h"
#include "direct_if_decim.h"
#include "resample_256_to_250.h"
#include "rotate_to_dc.h"
#include "burst_pipeline.h"
#include "burst_prefilter.h"
#include "band_pipeline.h"
#include "iridium_band_pipeline.h"
#include "qpsk_demod.h"
#include "fixture_albq_raw.h"

#define INPUT_FS_HZ 2500000
#define BURST_PRE_LEN (2 * FBT_FFT_SIZE)
#define BURST_POST_LEN ((int)(INPUT_FS_HZ * 16e-3))
#define BURST_WINDOW_LEN ((int)(INPUT_FS_HZ * 250 / 1000))
#define BURST_WINDOW_250K (BURST_WINDOW_LEN / DIDECIM_DECIM)

static int32_t s_baseline_history[FBT_HISTORY_SIZE * FBT_FFT_SIZE];

// Collected per-frame record (both paths use the same shape).
typedef struct {
    uint8_t *bits;      // owned copy
    int16_t *soft;      // owned copy or NULL
    int      n_bits;
    int      direction;
    int      demod_ok;
} rec_t;

#define MAX_RECS 16
typedef struct {
    rec_t recs[MAX_RECS];
    int   n;
} rec_list_t;

static void rec_push(rec_list_t *l, const uint8_t *bits, const int16_t *soft,
                     int n_bits, int direction, int demod_ok)
{
    if (l->n >= MAX_RECS) {
        fprintf(stderr, "FAIL: more than %d frames in one burst\n", MAX_RECS);
        exit(1);
    }
    rec_t *r = &l->recs[l->n++];
    r->n_bits    = n_bits;
    r->direction = direction;
    r->demod_ok  = demod_ok;
    r->bits      = NULL;
    r->soft      = NULL;
    if (bits && n_bits > 0) {
        r->bits = malloc((size_t)n_bits);
        memcpy(r->bits, bits, (size_t)n_bits);
    }
    if (soft && n_bits > 0) {
        r->soft = malloc((size_t)n_bits * sizeof(int16_t));
        memcpy(r->soft, soft, (size_t)n_bits * sizeof(int16_t));
    }
}

static void rec_list_free(rec_list_t *l)
{
    for (int i = 0; i < l->n; i++) {
        free(l->recs[i].bits);
        free(l->recs[i].soft);
    }
    l->n = 0;
}

// Path A callback: legacy burst_pipeline signature.
static void cb_legacy(burst_pipeline_result_t *res, void *ctx)
{
    rec_list_t *l = (rec_list_t *)ctx;
    rec_push(l, res->frame.bits, res->frame.soft_bits, res->frame.n_bits,
             (int)res->frame.direction, res->demod_ok ? 1 : 0);
    free(res->frame.bits); // callback owns (burst_pipeline.h contract)
    free(res->frame.soft_bits);
}

// Path B callback: generic band_pipeline signature.
static void cb_band(band_frame_t *f, void *ctx)
{
    rec_list_t *l = (rec_list_t *)ctx;
    rec_push(l, f->bits, f->soft_bits, f->n_bits, f->direction,
             f->demod_ok ? 1 : 0);
    free(f->bits); // callback owns (band_pipeline.h contract)
    free(f->soft_bits);
}

static int rec_lists_equal(const rec_list_t *a, const rec_list_t *b)
{
    if (a->n != b->n) return 0;
    for (int i = 0; i < a->n; i++) {
        const rec_t *ra = &a->recs[i], *rb = &b->recs[i];
        if (ra->n_bits != rb->n_bits) return 0;
        if (ra->direction != rb->direction) return 0;
        if (ra->demod_ok != rb->demod_ok) return 0;
        if ((ra->bits == NULL) != (rb->bits == NULL)) return 0;
        if (ra->bits && memcmp(ra->bits, rb->bits, (size_t)ra->n_bits) != 0) return 0;
        if ((ra->soft == NULL) != (rb->soft == NULL)) return 0;
        if (ra->soft &&
            memcmp(ra->soft, rb->soft, (size_t)ra->n_bits * sizeof(int16_t)) != 0)
            return 0;
    }
    return 1;
}

int main(void)
{
    // 1) Fixture -> int16 -> firmware resample to 2.5 MSPS (same front
    // half as test_pipeline_wideband_resampled.c).
    int      n_raw = ALBQ_RAW_UINT8_LEN / 2;
    int16_t *iq256 = malloc(2 * (size_t)n_raw * sizeof(int16_t));
    if (!iq256) return 2;
    for (int i = 0; i < 2 * n_raw; i++) {
        iq256[i] = (int16_t)(((int)ALBQ_RAW_UINT8[i] - 128) << 8);
    }
    int      n_resamp_max = (int)((double)n_raw * 125.0 / 128.0 + 16);
    int16_t *iq25         = malloc(2 * (size_t)n_resamp_max * sizeof(int16_t));
    if (!iq25) return 2;
    resample_256_to_250_t rs;
    resample_256_to_250_init(&rs);
    int n25 = resample_256_to_250_process(&rs, iq256, n_raw, iq25);

    // 2) Tag bursts (iridium profile parameters, as dsp_processor.c uses).
    fft_burst_tagger_t *t = fft_burst_tagger_init(BURST_PRE_LEN, BURST_POST_LEN,
                                                  /*burst_width=*/32,
                                                  /*threshold_db=*/14.0f,
                                                  s_baseline_history);
    if (!t) return 2;
    fft_burst_tagger_set_start(t, 0);

    typedef struct {
        uint64_t start, stop;
        int      center_bin, width_bins;
    } tag_t;
    enum { MAX_TAGS = 256 };
    static tag_t tags[MAX_TAGS];
    int          n_tags = 0;
    fbt_burst_t  new_bursts[FBT_MAX_BURSTS], gone_bursts[FBT_MAX_BURSTS];
    for (int off = 0; off + FBT_FFT_SIZE <= n25; off += FBT_FFT_SIZE) {
        int n_new = FBT_MAX_BURSTS, n_gone = FBT_MAX_BURSTS;
        fft_burst_tagger_step(t, iq25 + (size_t)off * 2, NULL,
                              new_bursts, &n_new, gone_bursts, &n_gone);
        for (int i = 0; i < n_gone && n_tags < MAX_TAGS; i++) {
            tags[n_tags++] = (tag_t){gone_bursts[i].start, gone_bursts[i].stop,
                                     gone_bursts[i].center_bin,
                                     gone_bursts[i].width_bins};
        }
    }
    {
        int         n_flush = FBT_MAX_BURSTS;
        fbt_burst_t flushed[FBT_MAX_BURSTS];
        fft_burst_tagger_flush(t, flushed, &n_flush);
        for (int i = 0; i < n_flush && n_tags < MAX_TAGS; i++) {
            tags[n_tags++] = (tag_t){flushed[i].start, flushed[i].stop,
                                     flushed[i].center_bin, flushed[i].width_bins};
        }
    }
    printf("Tagger emitted %d bursts\n", n_tags);

    // 3) Per burst: decimate once, then run BOTH paths on copies.
    direct_if_decim_t dec;
    direct_if_decim_init(&dec);
    int16_t *window_25   = malloc(2 * (size_t)BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *window_250  = malloc(2 * (size_t)BURST_WINDOW_250K * sizeof(int16_t));
    int16_t *buf_a       = malloc(2 * (size_t)BURST_WINDOW_250K * sizeof(int16_t));
    int16_t *buf_b       = malloc(2 * (size_t)BURST_WINDOW_250K * sizeof(int16_t));
    int16_t *scr_in_i    = malloc((size_t)BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *scr_in_q    = malloc((size_t)BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *scr_out_i   = malloc((size_t)BURST_WINDOW_250K * sizeof(int16_t));
    int16_t *scr_out_q   = malloc((size_t)BURST_WINDOW_250K * sizeof(int16_t));
    if (!window_25 || !window_250 || !buf_a || !buf_b || !scr_in_i ||
        !scr_in_q || !scr_out_i || !scr_out_q)
        return 2;

    const band_pipeline_t *bp = iridium_band_pipeline();
    if (!bp || !bp->prefilter || !bp->process_burst ||
        strcmp(bp->name, "iridium") != 0) {
        fprintf(stderr, "FAIL: iridium_band_pipeline vtable malformed\n");
        return 1;
    }

    int bursts_compared = 0, frames_ok = 0, prefilter_rejects = 0;
    for (int i = 0; i < n_tags; i++) {
        int64_t begin = (int64_t)tags[i].start;
        int64_t end   = (int64_t)tags[i].stop;
        if (begin < 0 || end > n25 || end <= begin) continue;
        int win_len = (int)(end - begin);
        if (win_len > BURST_WINDOW_LEN) win_len = BURST_WINDOW_LEN;
        win_len -= win_len % DIDECIM_DECIM;
        if (win_len < DIDECIM_DECIM) continue;

        memcpy(window_25, iq25 + begin * 2, (size_t)win_len * 2 * sizeof(int16_t));
        double phase_step =
            rotate_to_dc_phase_step_from_bin(tags[i].center_bin, FBT_FFT_SIZE);
        rotate_to_dc_q15_simd(window_25, win_len, phase_step);
        direct_if_decim_reset_state(&dec);
        int n_out = direct_if_decim_process_split(&dec, window_25, win_len,
                                                  window_250,
                                                  scr_in_i, scr_in_q,
                                                  scr_out_i, scr_out_q);
        if (n_out <= 0) continue;

        memcpy(buf_a, window_250, (size_t)n_out * 2 * sizeof(int16_t));
        memcpy(buf_b, window_250, (size_t)n_out * 2 * sizeof(int16_t));

        // --- prefilter equivalence (verdict + every gate the worker reads)
        burst_prefilter_result_t pfa;
        bool                     acc_a =
            burst_prefilter(buf_a, n_out, tags[i].width_bins, &pfa);
        band_prefilter_verdict_t pfb;
        bool                     acc_b =
            bp->prefilter(buf_b, n_out, tags[i].width_bins, &pfb);
        if (acc_a != acc_b || pfa.accept != pfb.accept ||
            pfa.width_ok != pfb.width_ok || pfa.dur_ok != pfb.dur_ok ||
            pfa.snr_ok != pfb.snr_ok) {
            fprintf(stderr,
                    "FAIL: prefilter divergence on burst %d "
                    "(A: acc=%d w=%d d=%d s=%d  B: acc=%d w=%d d=%d s=%d)\n",
                    i, acc_a, pfa.width_ok, pfa.dur_ok, pfa.snr_ok,
                    acc_b, pfb.width_ok, pfb.dur_ok, pfb.snr_ok);
            return 1;
        }
        if (!acc_a) prefilter_rejects++;
        // (The worker would drop rejected bursts; run the demod anyway —
        // more comparison coverage costs nothing here.)

        // --- process_burst equivalence
        rec_list_t la = {0}, lb = {0};
        int        na = burst_pipeline_process_burst(buf_a, n_out, cb_legacy, &la);
        int        nb = bp->process_burst(buf_b, n_out, cb_band, &lb);
        if (na != nb || !rec_lists_equal(&la, &lb)) {
            fprintf(stderr,
                    "FAIL: process_burst divergence on burst %d "
                    "(A: ret=%d frames=%d  B: ret=%d frames=%d)\n",
                    i, na, la.n, nb, lb.n);
            return 1;
        }
        for (int k = 0; k < la.n; k++) {
            if (la.recs[k].demod_ok) frames_ok++;
        }
        rec_list_free(&la);
        rec_list_free(&lb);
        bursts_compared++;
    }

    printf("\nSummary: bursts_compared=%d prefilter_rejects=%d "
           "demod_ok_frames(per path)=%d\n",
           bursts_compared, prefilter_rejects, frames_ok);

    fft_burst_tagger_destroy(t);
    free(scr_in_i); free(scr_in_q); free(scr_out_i); free(scr_out_q);
    free(buf_a); free(buf_b); free(window_250); free(window_25);
    free(iq25); free(iq256);

    // Vacuity guards: the equivalence proof means nothing if no real
    // decodes flowed. The same fixture decodes ~63 via the single-frame
    // API (test_pipeline_wideband_resampled floor 55); the multi-frame
    // API here can only find MORE frames, so reuse that floor.
    if (bursts_compared < 50 || frames_ok < 55) {
        fprintf(stderr, "FAIL: harness vacuous (bursts=%d frames_ok=%d)\n",
                bursts_compared, frames_ok);
        return 1;
    }
    printf("PASS: iridium band_pipeline adapter is bit-identical "
           "(%d bursts, %d decoded frames compared)\n",
           bursts_compared, frames_ok);
    return 0;
}
