// worker_core1 — per-burst processing chain (Core 1 task).
//
// Phase 3.6.M cutover (commit 1e386b7): replaces the per-channel
// extract → freq-shift → 40k→250k resample chain with the wideband
// direct-IF path. Each burst now arrives from dsp_processor with a
// precise rel_freq_hz tag (from the FFT bin), and the worker:
//   1. extracts a wideband window from the 2.5 MSPS signal_buffer
//   2. rotates the burst to DC using absolute-phase float rotation
//      (cosf/sinf per sample — see memory note about Q15 incremental
//      phasor magnitude decay over long windows)
//   3. 10× decimates 2.5 MSPS → 250 ksps via direct_if_decim's
//      gri-exact 279-tap Kaiser FIR
//   4. hands the 250 ksps burst to burst_pipeline for D13 / CFO /
//      RRC / UW / pre-rotate / demod
//
// Validated on host as gr-iridium per-stage equivalent (NMSE ≤ −17 dB,
// phase coherence ≥ 0.993 on burst id=30) — commit de72f24.

#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "worker_core1.h"
#include "signal_buffer.h"
#include "dsp_processor.h"
#include "qpsk_demod.h"
#include "uw_correlator.h"
#include "sd_capture.h"
#include "burst_pipeline.h"
#include "direct_if_decim.h"
#include "rotate_to_dc.h"
#include "bch_decoder.h"
#include "frame_decoder.h"
#include "iridium_frame.h"
#include "sdkconfig.h"

#if CONFIG_SMOKE_TEST_RAW_IRIDIUM
#include "fixture_albq_golden_bits.h"
#endif

static const char *TAG = "WORKER1";

static QueueHandle_t burst_queue = NULL;

// Diagnostic counters. Read & reset by worker_core1_get_stats().
static volatile uint32_t s_bursts_queued = 0;
static volatile uint32_t s_bursts_dropped = 0;
static volatile uint32_t s_bursts_processed = 0;
static volatile uint32_t s_bursts_skipped = 0;
static volatile uint32_t s_queue_high_water = 0;
static volatile uint64_t s_burst_total_us = 0;
// BCH outcome counters. processed counts qpsk_demod successes (the
// "DEMOD SUCCESS" log). bch_decoded counts the ones that actually
// passed BCH — the real decode rate. bch_failed counts qpsk-demod
// successes that produced an uncorrectable frame (false-positive
// decodes from the application's perspective).
static volatile uint32_t s_bursts_bch_decoded = 0;  // BCH OK AND classify returned a known type
static volatile uint32_t s_bursts_bch_unknown = 0;  // BCH OK but iridium_frame_classify => IR_FRAME_UNKNOWN (BCH false-positive — task #111)
static volatile uint32_t s_bursts_bch_failed  = 0;  // BCH itself uncorrectable
static volatile uint32_t s_bursts_bch_chase_recovered = 0;  // Chase-2 soft decoder rescued a hard-decision BCH failure (#112)

// Per-stage timing accumulators, summed over processed bursts only.
static volatile uint64_t s_t_extract_us = 0;
static volatile uint64_t s_t_rotate_us  = 0;
static volatile uint64_t s_t_decim_us   = 0;
static volatile uint64_t s_t_pipeline_us = 0;
static volatile uint64_t s_t_bch_us     = 0;

#if CONFIG_SMOKE_TEST_RAW_IRIDIUM
// Golden-bits comparison counters (smoke build only). Each decoded
// burst is matched to a gri golden entry by (start_sample_2500k,
// freq_offset_hz) tolerance window. We then compute Hamming
// distance over the overlapping prefix and bucket the result.
//
// Tolerances:
//   ±125000 samples (50 ms at 2.5 MSPS) — gri's reported timestamp_ms
//     is the DECODED-FRAME time (post-UW-correlation offset), not the
//     burst-tagger start. Same window we use for tagger-vs-manifest
//     in test_tagger_vs_manifest.c (the 86% recall test).
//   ±20000 Hz freq offset — about half a channel width (40 kHz)
//     since adjacent-channel bursts shouldn't false-match.
#define GOLDEN_TIME_TOL_2500K   125000
#define GOLDEN_FREQ_TOL_HZ      20000

static volatile uint32_t s_gold_decoded         = 0;  // device decoded a frame
static volatile uint32_t s_gold_matched         = 0;  // and matched a golden entry
static volatile uint32_t s_gold_unmatched       = 0;  // decoded but no golden in window
static volatile uint32_t s_gold_exact           = 0;  // BER = 0 over min length
static volatile uint32_t s_gold_close           = 0;  // BER < 5 %
static volatile uint32_t s_gold_partial         = 0;  // BER < 25 %
static volatile uint32_t s_gold_divergent       = 0;  // BER >= 25 %
static volatile uint32_t s_gold_total_bits      = 0;  // sum of min(device, gri) lengths
static volatile uint32_t s_gold_total_errors    = 0;  // sum of Hamming distances
static volatile uint32_t s_gold_len_eq          = 0;  // device n_bits == gri n_bits
static volatile uint32_t s_gold_len_short       = 0;  // device n_bits < gri n_bits

// Per-golden-entry "claimed" flag. When a device burst matches a
// golden entry, we mark it so a second device burst at the same
// time-freq cell can't double-count it. After all bursts have been
// processed, unclaimed golden entries are "MISSED by device" —
// the recall complement of the matched count.
static uint8_t s_gold_claimed[128];  // sized > FIXTURE_ALBQ_RAW_GOLDEN_COUNT

// Per-burst row for end-of-run reporting (smoke only — 65-ish bursts,
// fits comfortably in RAM).
typedef struct {
    int      gri_id;
    int      device_n_bits;
    int      gri_n_bits;
    int      compared_bits;
    int      errors;
    int      bucket;   // 0=exact 1=close 2=partial 3=divergent 4=unmatched
    int      bch_e1;   // post-BCH block-1 correctable-error count, -1 if no BCH
    int      bch_e2;   // post-BCH block-2 correctable-error count, -1 if no BCH
    int      gri_conf; // gri's reported confidence pct (-1 for UNMATCHED rows)
} golden_row_t;
static golden_row_t s_gold_rows[128];
static int          s_gold_n_rows = 0;

// Aggregated post-BCH stats across matched-AND-BCH-attempted bursts.
// e1/e2 < 0 means BCH failed (more than 3 errors → uncorrectable).
// e1 == 0 && e2 == 0 → clean decode, no errors needed correcting.
static volatile uint32_t s_gold_bch_clean    = 0;  // e1==0 && e2==0
static volatile uint32_t s_gold_bch_corrected = 0; // (e1>0 || e2>0) && both ≥ 0
static volatile uint32_t s_gold_bch_failed   = 0;  // either block uncorrectable
static volatile uint32_t s_gold_bch_skipped  = 0;  // frame too short for BCH

static void golden_compare_burst(const detected_burst_t *burst,
                                  const decoded_frame_t *frame,
                                  int e1_bch, int e2_bch)
{
    s_gold_decoded++;
    // Find golden entry within tolerance window. Pick the one with
    // smallest combined (Δsample, Δfreq) distance.
    int best = -1;
    int64_t best_score = INT64_MAX;
    for (int g = 0; g < FIXTURE_ALBQ_RAW_GOLDEN_COUNT; g++) {
        if (s_gold_claimed[g]) continue;  // already paired with an earlier burst
        const golden_burst_t *e = &FIXTURE_ALBQ_RAW_GOLDEN_BURSTS[g];
        int64_t ds = (int64_t)burst->start_sample_idx
                     - (int64_t)e->start_sample_2500k;
        if (ds < 0) ds = -ds;
        if (ds > GOLDEN_TIME_TOL_2500K) continue;
        int64_t df = (int64_t)burst->rel_freq_hz - (int64_t)e->freq_offset_hz;
        if (df < 0) df = -df;
        if (df > GOLDEN_FREQ_TOL_HZ) continue;
        // Combined score normalised to tolerances.
        int64_t score = (ds * 100 / GOLDEN_TIME_TOL_2500K)
                       + (df * 100 / GOLDEN_FREQ_TOL_HZ);
        if (score < best_score) {
            best_score = score;
            best = g;
        }
    }
    // Track BCH outcome regardless of whether we found a golden match —
    // it's the most direct quality signal we have for the device's
    // own decode (independent of any external reference).
    if (e1_bch < 0 && e2_bch < 0) {
        s_gold_bch_skipped++;
    } else if (e1_bch < 0 || e2_bch < 0) {
        s_gold_bch_failed++;
    } else if (e1_bch == 0 && e2_bch == 0) {
        s_gold_bch_clean++;
    } else {
        s_gold_bch_corrected++;
    }

    if (best < 0) {
        s_gold_unmatched++;
        if (s_gold_n_rows < (int)(sizeof(s_gold_rows) / sizeof(s_gold_rows[0]))) {
            s_gold_rows[s_gold_n_rows++] = (golden_row_t){
                .gri_id = -1, .device_n_bits = frame->n_bits,
                .gri_n_bits = 0, .compared_bits = 0, .errors = 0,
                .bucket = 4,
                .bch_e1 = e1_bch, .bch_e2 = e2_bch, .gri_conf = -1,
            };
        }
        return;
    }
    s_gold_claimed[best] = 1;  // pair this golden entry to the current device burst
    const golden_burst_t *e = &FIXTURE_ALBQ_RAW_GOLDEN_BURSTS[best];
    int cmp_n = frame->n_bits < e->gri_n_bits ? frame->n_bits : e->gri_n_bits;
    int errors = 0;
    for (int k = 0; k < cmp_n; k++) {
        if (frame->bits[k] != e->gri_bits[k]) errors++;
    }
    // Dump the first N matched-bursts' raw bit strings so an offline
    // analyzer (tests/scripts/golden_bits_align.py) can search for
    // the transform that aligns device bits to gri bits (shift, NOT,
    // Gray inversion, etc.). Cleared once we know the mapping.
    if (s_gold_matched < 3) {
        char dev_str[400];
        char gri_str[400];
        int n_to_dump = cmp_n < 384 ? cmp_n : 384;
        for (int k = 0; k < n_to_dump; k++) {
            dev_str[k] = frame->bits[k]   ? '1' : '0';
            gri_str[k] = e->gri_bits[k]   ? '1' : '0';
        }
        dev_str[n_to_dump] = 0;
        gri_str[n_to_dump] = 0;
        ESP_LOGI(TAG, "GOLDEN-BITDUMP gri_id=%d device_n=%d gri_n=%d",
                 e->gri_id, frame->n_bits, e->gri_n_bits);
        ESP_LOGI(TAG, "GOLDEN-BITDUMP   dev=%s", dev_str);
        ESP_LOGI(TAG, "GOLDEN-BITDUMP   gri=%s", gri_str);
    }
    s_gold_matched++;
    s_gold_total_bits += (uint32_t)cmp_n;
    s_gold_total_errors += (uint32_t)errors;
    if (frame->n_bits == e->gri_n_bits) s_gold_len_eq++;
    else if (frame->n_bits < e->gri_n_bits) s_gold_len_short++;

    int bucket;
    if (cmp_n == 0) {
        bucket = 4;  // can't classify with zero bits compared
    } else {
        int ber_pct = errors * 100 / cmp_n;
        if (errors == 0)       { s_gold_exact++;     bucket = 0; }
        else if (ber_pct < 5)  { s_gold_close++;     bucket = 1; }
        else if (ber_pct < 25) { s_gold_partial++;   bucket = 2; }
        else                   { s_gold_divergent++; bucket = 3; }
    }
    if (s_gold_n_rows < (int)(sizeof(s_gold_rows) / sizeof(s_gold_rows[0]))) {
        s_gold_rows[s_gold_n_rows++] = (golden_row_t){
            .gri_id = e->gri_id,
            .device_n_bits = frame->n_bits,
            .gri_n_bits = e->gri_n_bits,
            .compared_bits = cmp_n,
            .errors = errors,
            .bucket = bucket,
            .bch_e1 = e1_bch, .bch_e2 = e2_bch, .gri_conf = e->conf_pct,
        };
    }
}

void worker_core1_golden_print_summary(void)
{
    int n_gri = FIXTURE_ALBQ_RAW_GOLDEN_COUNT;
    int n_missed = 0;
    for (int g = 0; g < n_gri; g++) {
        if (!s_gold_claimed[g]) n_missed++;
    }
    double recall_pct    = n_gri > 0 ? 100.0 * (double)s_gold_matched / (double)n_gri : 0.0;
    double precision_pct = s_gold_decoded > 0
        ? 100.0 * (double)s_gold_matched / (double)s_gold_decoded : 0.0;
    ESP_LOGI(TAG, "GOLDEN: gri_total=%d device_decoded=%u matched=%u "
                  "missed_by_device=%d unmatched_device=%u "
                  "(recall=%.1f%% precision=%.1f%%)",
             n_gri, s_gold_decoded, s_gold_matched, n_missed,
             s_gold_unmatched, recall_pct, precision_pct);
    ESP_LOGI(TAG, "GOLDEN: histogram exact=%u close(BER<5%%)=%u "
                  "partial(<25%%)=%u divergent(>=25%%)=%u",
             s_gold_exact, s_gold_close, s_gold_partial, s_gold_divergent);
    // Post-BCH quality — the most semantic signal we can produce
    // without a known-good gri-side post-BCH oracle (task #61 follow-
    // up). bch_clean means both BCH(31,21) blocks decoded with zero
    // bit errors corrected (= ideal); bch_corrected means BCH had to
    // fix 1-3 errors per block (still a valid frame, but at the edge
    // of the code's capacity); bch_failed means at least one block
    // had >3 errors (BCH gave up; the frame's first 2 BCH words are
    // garbage); bch_skipped means the frame was shorter than 88 bits
    // (UW + 2 blocks) so we didn't try BCH at all.
    ESP_LOGI(TAG, "GOLDEN: BCH outcomes clean(e==0)=%u corrected(1<=e<=3)=%u "
                  "failed(uncorrectable)=%u skipped(short_frame)=%u",
             s_gold_bch_clean, s_gold_bch_corrected,
             s_gold_bch_failed, s_gold_bch_skipped);
    if (s_gold_total_bits > 0) {
        ESP_LOGI(TAG, "GOLDEN: overall BER %u/%u = %.2f%% "
                      "(length_eq=%u length_short=%u)",
                 s_gold_total_errors, s_gold_total_bits,
                 100.0 * (double)s_gold_total_errors / (double)s_gold_total_bits,
                 s_gold_len_eq, s_gold_len_short);
    }
    // Detail dump (one line per row) for offline diff against gri.
    // bch=ee/EE prints the two block correctable-error counts so a
    // pattern of high BCH errors localised to a few bursts can be
    // distinguished from a global precision issue.
    for (int i = 0; i < s_gold_n_rows; i++) {
        const golden_row_t *r = &s_gold_rows[i];
        char bch_str[32];
        if (r->bch_e1 < 0 && r->bch_e2 < 0)      snprintf(bch_str, sizeof(bch_str), "skip");
        else if (r->bch_e1 < 0 || r->bch_e2 < 0) snprintf(bch_str, sizeof(bch_str), "FAIL");
        else                                      snprintf(bch_str, sizeof(bch_str), "%d/%d",
                                                          r->bch_e1, r->bch_e2);
        if (r->bucket == 4 && r->gri_id < 0) {
            ESP_LOGI(TAG, "GOLDEN[%d]: UNMATCHED device_n=%d bch=%s",
                     i, r->device_n_bits, bch_str);
        } else {
            const char *tag = r->bucket == 0 ? "EXACT"
                            : r->bucket == 1 ? "CLOSE"
                            : r->bucket == 2 ? "PARTIAL"
                            : r->bucket == 3 ? "DIVERG"
                            :                 "UNMATCH";
            ESP_LOGI(TAG, "GOLDEN[%d]: gri_id=%d conf=%d%% %s "
                          "raw_err=%d/%d bch=%s (device_n=%d gri_n=%d)",
                     i, r->gri_id, r->gri_conf, tag, r->errors, r->compared_bits,
                     bch_str, r->device_n_bits, r->gri_n_bits);
        }
    }
    // Missed-by-device rows: gri decoded these but no device burst
    // claimed them in the alignment window. The gri burst could be:
    //   (a) below our SNR threshold (we never tagged it),
    //   (b) tagged but with bad start / freq putting it outside our window,
    //   (c) tagged but failed to demod end-to-end (no decoded_frame_t).
    // Identifying which requires cross-referencing with the tagger
    // log — for now just emit the gri_id so an offline script can
    // do the deeper match.
    for (int g = 0; g < n_gri; g++) {
        if (s_gold_claimed[g]) continue;
        const golden_burst_t *e = &FIXTURE_ALBQ_RAW_GOLDEN_BURSTS[g];
        ESP_LOGI(TAG, "GOLDEN-MISSED: gri_id=%d start=%llu freq_off=%ld "
                      "conf=%d gri_n_bits=%d",
                 e->gri_id, (unsigned long long)e->start_sample_2500k,
                 (long)e->freq_offset_hz, e->conf_pct, e->gri_n_bits);
    }
}
#else
// Outside the smoke build, the public summary entry is a no-op so
// smoke_test.c can call it unconditionally.
void worker_core1_golden_print_summary(void) {}
#endif  // CONFIG_SMOKE_TEST_RAW_IRIDIUM

// Buffer sizes for the wideband per-burst window. The tagger
// publishes variable-length bursts via gri's gone-event semantics
// (start → stop = last_active + burst_post_len). Single-frame
// bursts span ~16 ms; multi-frame bursts (gri's
// handle_multiple_frames_per_burst case) often run 50-100 ms, with
// gri capping at max_burst_len = sample_rate * 0.09 = 225 ms.
// We size to 250 ms (multiple of 16 complex = 64-byte aligned) so
// any realistic burst fits without truncation.
//
// Cache alignment: tagger reports burst.start_sample_idx as a
// multiple of FBT_FFT_SIZE (2048), so the address (start × 4 bytes
// per complex) is naturally 64-byte aligned. We subtract
// WB_PRE_PAD_SAMPLES; making that subtraction a multiple of 16
// complex samples (= 64 bytes) keeps the extract address aligned
// for esp_cache_msync. DIDECIM_NTAPS is 144 (gri-aligned 141 + 3
// zero-pad for PIE 8-alignment); 144 already lines up on 16-cplx,
// but we keep 288 as a margin to absorb any future Kaiser-design
// revisions without re-checking alignment.
#define WB_PRE_PAD_SAMPLES    288     // 18 × 16, ≥ DIDECIM_NTAPS - 1
#define WB_MAX_BURST_SAMPLES  ((int)(FS_DETECT_HZ / 4))   // 250 ms = 625000
#define WB_EXTRACT_SAFETY     1024
#define WB_EXTRACT_MAX        (WB_MAX_BURST_SAMPLES + WB_PRE_PAD_SAMPLES \
                                + WB_EXTRACT_SAFETY)

// 250 ksps output is at most WB_EXTRACT_MAX / 10 + 1.
#define WB_DECIM_MAX          ((WB_EXTRACT_MAX / DIDECIM_DECIM) + 8)

// Per-chunk size for the streaming decim. The PIE FIR scratch needs
// to be in INTERNAL SRAM (dsps_fird_s16_arp4's `esp.vld.128.ip` can't
// service PSRAM); chunking keeps that scratch tiny. 4000 input samples
// per chunk = 8 KB per I/Q channel = ~17 KB total internal SRAM
// (vs ~200 KB if we tried full-burst scratch). 4000 is a multiple of
// DIDECIM_DECIM=10 and of 8 (PIE alignment), so each chunk consumes
// exactly N inputs and produces exactly N/10 outputs; no boundary
// math needed inside the loop. The streaming FIR delay-line state
// in `s_decim` is reset once at the start of each burst (so leftover
// history from previous bursts doesn't bleed in), then preserves
// naturally across chunks within a burst.
#define DECIM_CHUNK_IN        4000

static int16_t          *s_extract_buf  = NULL;   // 2.5 MSPS wideband window
static int16_t          *s_decim_buf    = NULL;   // 250 ksps post-decim
static int16_t          *s_chunk_iq     = NULL;   // INTERNAL — per-chunk IQ scratch
                                                  // (rotate runs here, then
                                                  // process_split deinterleaves
                                                  // from here)
// Deinterleave scratch for direct_if_decim_process_split (PSRAM —
// no DMA so plain heap_caps_malloc with MALLOC_CAP_SPIRAM is fine).
static int16_t          *s_decim_scr_in_i  = NULL;
static int16_t          *s_decim_scr_in_q  = NULL;
static int16_t          *s_decim_scr_out_i = NULL;
static int16_t          *s_decim_scr_out_q = NULL;
static direct_if_decim_t s_decim;

// (Absolute-phase rotation lives in common/iridium_decoder/rotate_to_dc.{h,c}
// — shared with the host wideband test. Per-sample cosf/sinf on P4's
// scalar FPU is acceptable for first cutover; task #58 covers a
// cordic/table replacement if profiling shows it dominates.)

// Per-burst context passed to worker_emit_frame. The callback fires
// once per decoded sub-frame within a multi-frame burst (gri's
// handle_multiple_frames_per_burst behaviour).
typedef struct {
    const detected_burst_t *burst;
    uint64_t                t_bch_accum;   // accumulated BCH+log+queue time
} wb_worker_ctx_t;

static void worker_emit_frame(burst_pipeline_result_t *bres, void *ctx)
{
    wb_worker_ctx_t *wctx = (wb_worker_ctx_t *)ctx;
    int64_t t_bch0 = esp_timer_get_time();

    // Verbose D13/UW info -- one line per FRAME now (with multi-frame
    // this fires multiple times per burst). Demoted to ESP_LOGD; the
    // BCH outcome below stays at ESP_LOGI as the per-frame outcome
    // marker.
    ESP_LOGD(TAG, "D13 start=%d  UW dir=%s off=%d corr=%.3f SNR=%.1f omega=%.3f",
             bres->burst_start,
             bres->uw_res.direction == UW_DIR_DOWNLINK ? "DL" :
             bres->uw_res.direction == UW_DIR_UPLINK   ? "UL" : "??",
             bres->uw_res.uw_offset,
             (double)bres->uw_res.correction,
             (double)bres->uw_res.snr_estimate_db,
             (double)bres->uw_res.omega_per_sym);

    if (!bres->demod_ok) {
        // Failed sub-frames still fire the callback for diagnostic
        // logging; just free the bits and return.
        free(bres->frame.bits);
        free(bres->frame.soft_bits);   // #112
        wctx->t_bch_accum += (uint64_t)(esp_timer_get_time() - t_bch0);
        return;
    }

    decoded_frame_t frame = bres->frame;
    ESP_LOGI(TAG, "DEMOD SUCCESS: %s frame (%d bits)",
             frame.direction == DIR_DOWNLINK ? "DL" : "UL",
             frame.n_bits);

    int e1_bch = -1, e2_bch = -1;
    bool chase_used = false;
    if (frame.n_bits >= 24 + 64) {
        const uint8_t *payload = frame.bits + 24;
        uint8_t block1[32], block2[32];
        uint8_t data1[21], data2[21];

        iridium_deinterleave(payload, block1, block2);
        e1_bch = bch_decode_block(block1, data1);
        e2_bch = bch_decode_block(block2, data2);

        // Chase-2 soft decoder rescue path (#112). On hard-BCH failure
        // for either block, retry with K=3 (8 trials) soft Chase-2 using
        // the per-bit soft metrics qpsk_demod populated. Typical gain
        // 1.0-1.5 dB at the BCH stage on Iridium; sub-100 µs of compute
        // per frame on P4 (8 hard decodes × 31-bit poly division).
        if ((e1_bch < 0 || e2_bch < 0) && frame.soft_bits != NULL) {
            int16_t soft1[32], soft2[32];
            iridium_deinterleave_int16(frame.soft_bits + 24, soft1, soft2);
            if (e1_bch < 0) {
                int e = bch_decode_block_soft(soft1, data1, 3);
                if (e >= 0) { e1_bch = e; chase_used = true; }
            }
            if (e2_bch < 0) {
                int e = bch_decode_block_soft(soft2, data2, 3);
                if (e >= 0) { e2_bch = e; chase_used = true; }
            }
        }

        if (e1_bch >= 0 && e2_bch >= 0) {
            if (chase_used) s_bursts_bch_chase_recovered++;
            // BCH passed — but at marginal SNR (~12-13 dB) BCH(31,21)
            // can correct random noise into a "valid" 31-bit codeword
            // that has no Iridium frame structure. Classify before
            // calling this a real decode (task #111): only count
            // bch_decoded when iridium_frame_classify returns a known
            // frame type. The downstream frame_decoder re-classifies
            // independently and feeds the /status frames.{ms,tl,bc,lw,ra}
            // counters; this is the worker-side "real frame" signal.
            iridium_frame_t classified = { 0 };
            ir_frame_direction_t fdir = (frame.direction == DIR_DOWNLINK)
                                        ? IR_FRM_DIR_DOWNLINK
                                        : IR_FRM_DIR_UPLINK;
            int crc = iridium_frame_classify(frame.bits, frame.n_bits,
                                             fdir, &classified);
            if (crc == 0 && classified.type != IR_FRAME_UNKNOWN) {
                ESP_LOGD(TAG, "BCH PASS: errors=%d/%d type=%s (real decode)",
                         e1_bch, e2_bch,
                         iridium_frame_type_name(classified.type));
                s_bursts_bch_decoded++;
            } else {
                ESP_LOGD(TAG, "BCH PASS but UNKNOWN: errors=%d/%d "
                              "(BCH false-positive — noise corrected into "
                              "a valid codeword with no frame structure)",
                         e1_bch, e2_bch);
                s_bursts_bch_unknown++;
            }
        } else {
            ESP_LOGD(TAG, "BCH FAIL: e1=%d e2=%d (false-positive "
                           "qpsk_demod success — bits unusable)",
                     e1_bch, e2_bch);
            s_bursts_bch_failed++;
        }
    }
#if CONFIG_SMOKE_TEST_RAW_IRIDIUM
    golden_compare_burst(wctx->burst, &frame, e1_bch, e2_bch);
#endif
    frame_decoder_push(frame.bits, frame.n_bits,
                       frame.direction, 0u,
                       wctx->burst->peak_bin, wctx->burst->peak_snr_db);
    free(frame.bits);
    free(frame.soft_bits);   // #112
    wctx->t_bch_accum += (uint64_t)(esp_timer_get_time() - t_bch0);
}

void worker_task(void *arg)
{
    ESP_LOGI(TAG, "Worker Task started on Core %d", xPortGetCoreID());
    detected_burst_t burst;

    while (1) {
        if (xQueueReceive(burst_queue, &burst, portMAX_DELAY)) {
            int64_t burst_t0 = esp_timer_get_time();
            ESP_LOGI(TAG, "Worker burst: start=%lu len=%lu rel=%+.0f Hz SNR=%.1f dB",
                     (unsigned long)burst.start_sample_idx,
                     (unsigned long)burst.length_samples,
                     (double)burst.rel_freq_hz,
                     (double)burst.peak_snr_db);

            // Length guard. Tagger emits stop - start as length, which
            // can range from ~30 ms (single frame) to ~250 ms
            // (multi-frame). Clamp to WB_MAX_BURST_SAMPLES so we never
            // exceed the PSRAM scratch capacity, and reject bursts too
            // short to survive FIR transients.
            if (burst.length_samples < 128) {
                s_bursts_skipped++;
                continue;
            }

            // Stale-burst guard (task #60/#65): the producer may have
            // lapped the signal buffer between when this burst was
            // queued and when the worker dequeued it (queue backlog
            // under live-SDR load). Reading the burst's window now
            // would extract garbage from later ingest data. Skip.
            //
            // We use a conservative envelope (length + pre-pad) so the
            // check matches what signal_buffer_invalidate_range +
            // signal_buffer_read_chunk will touch later.
            uint32_t check_start = burst.start_sample_idx - WB_PRE_PAD_SAMPLES;
            uint32_t check_len   = burst.length_samples + WB_PRE_PAD_SAMPLES;
            if (!signal_buffer_burst_valid(check_start, check_len)) {
                ESP_LOGW(TAG, "stale burst: start=%lu len=%lu (head wrapped) — drop",
                         (unsigned long)burst.start_sample_idx,
                         (unsigned long)burst.length_samples);
                s_bursts_skipped++;
                continue;
            }
            uint32_t safe_len = burst.length_samples;
            if (safe_len > (uint32_t)WB_MAX_BURST_SAMPLES) {
                safe_len = (uint32_t)WB_MAX_BURST_SAMPLES;
            }
            // Two alignments to honour simultaneously:
            //   - DIDECIM_DECIM (= 10): so the decim produces an integer
            //     output count with no leftover input
            //   - 16 complex samples (= 64 bytes): so esp_cache_msync on
            //     the extracted PSRAM range doesn't reject the call
            //     with ESP_ERR_INVALID_ARG (cache line = 64 bytes on P4)
            // LCM(10, 16) = 80. Rounding safe_len down to a multiple of
            // 80 satisfies both. WB_PRE_PAD_SAMPLES is already 16-cplx
            // aligned (= 288 = 18 * 16, also a multiple of 80? no — 288
            // mod 80 = 48 — but combined-with rule still works: we need
            // ext_len % 16 == 0, and 288 is 16-aligned, so safe_len need
            // only be 16-aligned; the 80 rounding is the strictest needed
            // for the joint constraint).
            //
            // Before this rounding only 1-in-8 valid safe_len values
            // were cache-aligned; the rest silently failed msync and
            // the worker processed stale cached PSRAM → ~46% bit error
            // rate vs gri's ground truth across most bursts.
            safe_len -= safe_len % 80;
            uint32_t ext_len = safe_len + WB_PRE_PAD_SAMPLES;
            if (ext_len > WB_EXTRACT_MAX) {
                ext_len = WB_EXTRACT_MAX;
            }

            // 1. Prepare wideband window: invalidate L2 cache for the
            // whole range so the per-chunk reads in step 2+3 see
            // fresh DMA-written data. Task #64: skip the PSRAM
            // intermediate `s_extract_buf`; the decim chunk loop
            // reads directly from circular_buf via
            // signal_buffer_read_chunk, saving the extract-stage
            // PSRAM write (~3 ms/burst on the smoke corpus).
            uint32_t ext_start = burst.start_sample_idx
                                  - WB_PRE_PAD_SAMPLES;
            int64_t t_ext0 = esp_timer_get_time();
            signal_buffer_invalidate_range(ext_start, ext_len);
            int64_t t_ext1 = esp_timer_get_time();
            s_t_extract_us += (uint64_t)(t_ext1 - t_ext0);

            // 2 + 3. Fused rotate-to-DC + 10× decim, both running on
            // internal-SRAM chunks. The previous design rotated the
            // whole burst in PSRAM (~25 ms PSRAM round-trip) and then
            // decimated from PSRAM scratch (PIE FIR partially blocked
            // by the same vld.128 constraint as the FFT). This loop
            // reads each DECIM_CHUNK_IN-sample slice from extract_buf
            // (PSRAM, single linear read) into the internal-SRAM
            // chunk buffer, rotates it there, then hands it to
            // process_split (whose scratch is also internal). PSRAM
            // is touched once per burst (read-only) instead of three
            // times (extract-write + rotate-read+write + decim-read).
            //
            // Phase continuity across chunks via
            // rotate_to_dc_q15_inc_at(..., sample_offset = off): each
            // chunk's first absolute-phase renorm uses the burst-global
            // sample index so the rotated output is identical to a
            // single all-burst rotate call (within Q15 saturation).
            //
            // Streaming FIR delay-line state in `s_decim` carries
            // history across chunks within a burst; reset once per
            // burst so leftover history from previous bursts doesn't
            // bleed in.
            int64_t t_rot0 = esp_timer_get_time();
            double phase_step = -2.0 * M_PI * (double)burst.rel_freq_hz
                                 / (double)FS_DETECT_HZ;
            int64_t t_rot1 = esp_timer_get_time();
            // Rotate-stage timer (kept for the per-stage breakdown) is
            // effectively the cosf/sinf phase_step setup now — the
            // per-chunk rotate work counts under the decim timer.
            s_t_rotate_us += (uint64_t)(t_rot1 - t_rot0);

            int64_t t_dec0 = esp_timer_get_time();
            direct_if_decim_reset_state(&s_decim);
            // If burst-mode SD capture is active, emit one record
            // per burst: header + raw 2.5 MSPS IQ samples (pre-
            // rotate, pre-decim — the exact bytes the worker just
            // read from signal_buffer). Chunked via the same
            // signal_buffer_read_chunk loop the decode path uses.
            // sd_capture_record_burst_* are no-ops when not in
            // burst mode, so the hot-path cost is two predicted-
            // false branches.
            sd_capture_record_burst_begin((uint32_t)ext_len,
                                           burst.rel_freq_hz,
                                           burst.peak_snr_db,
                                           burst.magnitude_db,
                                           burst.noise_db);
            int n_250k = 0;
            for (int off = 0; off < (int)ext_len; off += DECIM_CHUNK_IN) {
                int chunk = (int)ext_len - off;
                if (chunk > DECIM_CHUNK_IN) chunk = DECIM_CHUNK_IN;
                // Pull chunk DIRECTLY from circular_buf (PSRAM) into the
                // internal-SRAM chunk buffer. Task #64: skips the
                // s_extract_buf intermediate -- L2 was already
                // invalidated for the whole window above, so this
                // single linear PSRAM read fills internal scratch
                // without a PSRAM intermediate.
                signal_buffer_read_chunk(ext_start + (uint32_t)off,
                                          (uint32_t)chunk, s_chunk_iq);
                // SD burst-capture tap. Writes the raw pre-rotate
                // IQ chunk to the capture stream buffer; host
                // pipeline replay sees exactly what the worker
                // saw, so device-vs-host decode comparisons are
                // apples-to-apples.
                sd_capture_record_burst_chunk(s_chunk_iq, (size_t)chunk);
                // Rotate the chunk in internal SRAM. sample_offset = off
                // keeps the absolute-phase renorm aligned across the
                // burst as if it were a single rotate call.
                // _simd_at dispatches to the PIE asm on target (when
                // ROT_SIMD_ARP4_AVAILABLE is set in the build) and to
                // the chunked-scalar reference on host — same
                // numerical contract either way.
                rotate_to_dc_q15_simd_at(s_chunk_iq, chunk,
                                          phase_step, off);
                int n_chunk_out = direct_if_decim_process_split(&s_decim,
                                       s_chunk_iq, chunk,
                                       s_decim_buf + (size_t)n_250k * 2,
                                       s_decim_scr_in_i, s_decim_scr_in_q,
                                       s_decim_scr_out_i, s_decim_scr_out_q);
                n_250k += n_chunk_out;
            }
            sd_capture_record_burst_end();
            int64_t t_dec1 = esp_timer_get_time();
            s_t_decim_us += (uint64_t)(t_dec1 - t_dec0);

            if (n_250k <= 64) {
                ESP_LOGD(TAG, "direct_if_decim produced %d samples — too short",
                         n_250k);
                s_bursts_skipped++;
                continue;
            }

            // 4. Per-burst pipeline at 250 ksps. Multi-frame callback
            //    fires once per decoded sub-frame so multi-frame bursts
            //    (gri's handle_multiple_frames_per_burst) yield all
            //    their frames instead of just the first one.
            int64_t t_pipe0 = esp_timer_get_time();
            wb_worker_ctx_t worker_ctx = {
                .burst = &burst,
                .t_bch_accum = 0,
            };
            int n_frames = burst_pipeline_process_burst(
                                s_decim_buf, n_250k,
                                worker_emit_frame, &worker_ctx);
            int64_t t_pipe1 = esp_timer_get_time();
            // burst_pipeline includes the per-frame BCH+log+queue cost
            // inside the callback. Subtract that out so s_t_pipeline_us
            // reflects DSP-only time.
            s_t_pipeline_us += (uint64_t)(t_pipe1 - t_pipe0) - worker_ctx.t_bch_accum;
            s_t_bch_us += worker_ctx.t_bch_accum;
            (void)n_frames;

            s_bursts_processed++;
            s_burst_total_us += (uint64_t)(esp_timer_get_time() - burst_t0);
        }

        // Periodic yield: worker is at prio 5; the frame_decoder at
        // prio 4 must get scheduled. One vTaskDelay(1) every 8 bursts
        // gives ~10 ms of frame_decoder CPU per 8 bursts processed.
        static int yield_counter = 0;
        if (++yield_counter >= 8) {
            yield_counter = 0;
            vTaskDelay(1);
        }
    }
}

esp_err_t worker_core1_init(void)
{
    // Burst queue depth 1024, storage in PSRAM. Each detected_burst_t
    // is 28 bytes, so 1024 entries cost ~28 KB of PSRAM (trivial out
    // of 32 MB). At the current 103 ms/burst worker time this is
    // ~100 seconds of buffering -- well past any transient overload.
    //
    // History: we ran 16, then 32, growing as the wideband front end
    // produced multi-frame bursts that take longer to process. 32
    // still hit high_water=10 on the smoke corpus and would queue-
    // overflow under live RF with bursty traffic. Going large is
    // safer than guessing; PSRAM is cheap and the per-enqueue cost
    // (a 28-byte memcpy via L2 cache) is invisible at burst rates.
    //
    // Static queue: control block in internal-SRAM .bss, storage
    // array in PSRAM heap. xQueueCreateStatic binds the two.
    #define BURST_QUEUE_DEPTH 1024
    static StaticQueue_t s_burst_queue_buf;
    static uint8_t *s_burst_queue_storage = NULL;
    size_t storage_bytes = (size_t)BURST_QUEUE_DEPTH * sizeof(detected_burst_t);
    s_burst_queue_storage = (uint8_t *)heap_caps_malloc(storage_bytes,
                                                        MALLOC_CAP_SPIRAM);
    if (!s_burst_queue_storage) {
        ESP_LOGE(TAG, "Burst queue PSRAM alloc failed (%zu B)", storage_bytes);
        return ESP_ERR_NO_MEM;
    }
    burst_queue = xQueueCreateStatic(BURST_QUEUE_DEPTH,
                                      sizeof(detected_burst_t),
                                      s_burst_queue_storage,
                                      &s_burst_queue_buf);
    if (!burst_queue) return ESP_ERR_NO_MEM;
    ESP_LOGI(TAG, "Burst queue: depth=%d × %u B = %zu B PSRAM",
             BURST_QUEUE_DEPTH, (unsigned)sizeof(detected_burst_t),
             storage_bytes);

    // Wideband buffers. Big working surfaces stay in PSRAM (extract +
    // decim output). The PIE FIR scratch must live in INTERNAL SRAM,
    // otherwise `esp.vld.128.ip` inside dsps_fird_s16_arp4 can't
    // service PSRAM access timing and the PIE path silently degrades
    // (we previously measured 22.9 ms/burst on this stage; PIE should
    // do ~1.5 ms). Decim runs in DECIM_CHUNK_IN-sample chunks so the
    // scratch stays tiny (8 KB per I/Q channel) rather than the 100 KB
    // each that a full-burst buffer would need.
    size_t ext_bytes      = (size_t)WB_EXTRACT_MAX * 2 * sizeof(int16_t);
    size_t dec_bytes      = (size_t)WB_DECIM_MAX   * 2 * sizeof(int16_t);
    size_t scr_in_bytes   = (size_t)DECIM_CHUNK_IN      * sizeof(int16_t);
    size_t scr_out_bytes  = (size_t)(DECIM_CHUNK_IN / DIDECIM_DECIM)
                                                        * sizeof(int16_t);
    size_t chunk_iq_bytes = (size_t)DECIM_CHUNK_IN * 2 * sizeof(int16_t);
    // Task #64: s_extract_buf removed -- decim loop reads directly
    // from signal_buffer via signal_buffer_read_chunk(). The static
    // pointer is kept null; init failure check below ignores it.
    // Saves ~2.5 MB of PSRAM and the 7.5 ms/burst extract-stage
    // PSRAM write.
    (void)ext_bytes;
    s_extract_buf      = NULL;
    s_decim_buf        = heap_caps_malloc(dec_bytes,     MALLOC_CAP_SPIRAM);
    s_chunk_iq         = heap_caps_aligned_alloc(16, chunk_iq_bytes, MALLOC_CAP_INTERNAL);
    s_decim_scr_in_i   = heap_caps_aligned_alloc(16, scr_in_bytes,  MALLOC_CAP_INTERNAL);
    s_decim_scr_in_q   = heap_caps_aligned_alloc(16, scr_in_bytes,  MALLOC_CAP_INTERNAL);
    s_decim_scr_out_i  = heap_caps_aligned_alloc(16, scr_out_bytes, MALLOC_CAP_INTERNAL);
    s_decim_scr_out_q  = heap_caps_aligned_alloc(16, scr_out_bytes, MALLOC_CAP_INTERNAL);
    // s_extract_buf intentionally NULL since task #64; don't check it.
    if (!s_decim_buf || !s_chunk_iq
        || !s_decim_scr_in_i || !s_decim_scr_in_q
        || !s_decim_scr_out_i || !s_decim_scr_out_q) {
        ESP_LOGE(TAG, "Worker buffer alloc failed");
        return ESP_ERR_NO_MEM;
    }

    // direct_if_decim is idempotent at init; safe to call here.
    direct_if_decim_init(&s_decim);

    // PSRAM stack — see feedback_task_stacks_in_psram memory note.
    // 16 KB internal stack would fragment the main internal-SRAM
    // pool and silently regress decode (verified empirically with
    // the resample worker tasks). Burst-decode runs ~5–10/s on real
    // RF with ~85 ms compute each → PSRAM stack overhead is < 0.1%.
    // Priority: lower than frame_decoder (4) so a backlog of bursts
    // never starves the WDT-watched decoder task. Worker is the
    // slowest consumer (~85 ms/burst) and bursts queue with
    // graceful drop-on-full at the tagger callback — back-pressure
    // here is correct; CPU starvation of the decoder is not.
    //
    // History: was 5 (above frame_decoder). At task #77's 10 dB
    // tagger threshold, the worker stayed runnable for >5 s
    // straight under bench noise, starved frame_decoder, and the
    // TASK_WDT aborted. Moving below frame_decoder lets the
    // scheduler give the WDT-watched task its cycles even when
    // worker has a backlog.
    // Prio 4 = same as frame_decoder. Empirically (Mon 2026-05-25)
    // prio 3 was getting <1% Core 1 under noise-heavy bench load
    // (tagger emitting 140 bursts/sec); the queue filled within a
    // second and stayed full, worker_dropped=130/s indefinitely.
    // Round-tripping with frame_decoder (same prio, both not
    // WDT-fatal-blocking) shares cycles equitably. Prio is BELOW
    // sd_capture writer (5) so SD writes don't compete with worker
    // for the few cycles ingest/fbt_pipe leave behind.
    //
    // History: was 5 → starved frame_decoder past WDT → moved to 3
    // → starved by ingest (8) + fbt_pipe (9) under load. Prio 4 is
    // the goldilocks slot.
    BaseType_t ok = xTaskCreatePinnedToCoreWithCaps(worker_task, "worker_core1",
                                     16384, NULL, 4, NULL, 1, MALLOC_CAP_SPIRAM);
    if (ok != pdPASS) {
        // Don't report a healthy init with no worker: burst_queue would still
        // exist, so push_burst would enqueue bursts that nothing ever drains —
        // the queue fills and every burst is silently dropped. Tear down so
        // the caller sees the failure.
        ESP_LOGE(TAG, "worker_core1 task create failed — tearing down");
        vQueueDelete(burst_queue);
        burst_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void worker_core1_push_burst(const detected_burst_t *burst)
{
    if (burst_queue) {
        s_bursts_queued++;
        UBaseType_t depth = uxQueueMessagesWaiting(burst_queue);
        if (depth > s_queue_high_water) s_queue_high_water = depth;
        if (xQueueSend(burst_queue, burst, 0) != pdTRUE) {
            s_bursts_dropped++;
        }
    }
}

void worker_core1_get_stats(worker_stats_t *out)
{
    uint32_t n = s_bursts_processed;
    out->bursts_queued      = s_bursts_queued;
    out->bursts_dropped     = s_bursts_dropped;
    out->bursts_processed   = n;
    out->bursts_skipped     = s_bursts_skipped;
    out->bursts_bch_decoded = s_bursts_bch_decoded;
    out->bursts_bch_unknown = s_bursts_bch_unknown;
    out->bursts_bch_failed  = s_bursts_bch_failed;
    out->bursts_bch_chase_recovered = s_bursts_bch_chase_recovered;
    out->queue_high_water   = s_queue_high_water;
    if (n > 0) {
        float fn = (float)n;
        out->avg_burst_us    = (float)s_burst_total_us / fn;
        out->extract_us      = (float)s_t_extract_us   / fn;
        out->freq_center_us  = (float)s_t_rotate_us    / fn;
        out->fir_decim_us    = (float)s_t_decim_us     / fn;
        out->resample_us     = 0.0f;
        out->demod_us        = (float)s_t_pipeline_us  / fn;
        out->bch_us          = (float)s_t_bch_us       / fn;
    } else {
        out->avg_burst_us = out->extract_us = out->freq_center_us =
            out->fir_decim_us = out->resample_us = out->demod_us =
            out->bch_us = 0.0f;
    }
    s_bursts_queued = 0;
    s_bursts_dropped = 0;
    s_bursts_processed = 0;
    s_bursts_skipped = 0;
    s_bursts_bch_decoded = 0;
    s_bursts_bch_unknown = 0;
    s_bursts_bch_failed = 0;
    s_bursts_bch_chase_recovered = 0;
    s_queue_high_water = 0;
    s_burst_total_us = 0;
    s_t_extract_us = s_t_rotate_us = s_t_decim_us = s_t_pipeline_us
        = s_t_bch_us = 0;
}
