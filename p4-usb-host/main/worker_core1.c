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
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "worker_core1.h"
#include "frame_pdu.h"
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

// SNR priority queue (T59). The worker can only fully-decode ~13 bursts/s
// (~76 ms each, dominated by the demod pipeline); the tagger, at the
// gri-aligned threshold, hands it far more under bench noise (~70-130/s, mostly
// false positives). A plain FIFO/LIFO forces the worker to waste its budget on
// whatever arrived, in arrival order. Instead, keep a small bounded buffer
// ordered by the tagger's peak_snr_db (known for free at detection): admit the
// strongest, EVICT the weakest when full, and always decode the strongest
// available first. Noise (low SNR) is shed WITHOUT ever paying the 76 ms. This
// does not touch detection (still gri-aligned) — it only allocates the scarce
// decode budget to the most-likely-real bursts. When the worker is NOT
// overloaded (good antenna, few bursts), nothing is evicted and every burst is
// decoded, just strongest-first.
// Cap sized above the dense-corpus peak backlog (smoke high_water≈36) so the
// GOLDEN corpus — where every burst is real — evicts nothing and still decodes
// all 62. Live, the strongest (real) bursts are never the ones evicted, so a
// larger cap doesn't change the triage; it only widens the candidate pool.
#define BURST_PQ_CAP 64
static detected_burst_t  s_pq[BURST_PQ_CAP];
static int               s_pq_count = 0;
static SemaphoreHandle_t s_pq_lock  = NULL; // guards s_pq / s_pq_count
static SemaphoreHandle_t s_pq_items = NULL; // counts occupied slots (worker waits on it)

// Producer-side insert (called from the tagger callback context). Returns:
//   1  = admitted into an empty slot (caller must give s_pq_items)
//   0  = admitted by EVICTING the weakest (net slot count unchanged, no give)
//  -1  = dropped (buffer full and this burst is weaker than everything in it)
// Caller holds s_pq_lock.
static int pq_insert_locked(const detected_burst_t *b)
{
    if (s_pq_count < BURST_PQ_CAP) {
        s_pq[s_pq_count++] = *b;
        return 1;
    }
    // Full: find the weakest slot; replace it only if the newcomer is stronger.
    int   min_i = 0;
    float min_s = s_pq[0].peak_snr_db;
    for (int i = 1; i < BURST_PQ_CAP; i++) {
        if (s_pq[i].peak_snr_db < min_s) {
            min_s = s_pq[i].peak_snr_db;
            min_i = i;
        }
    }
    if (b->peak_snr_db > min_s) {
        s_pq[min_i] = *b; // evict weakest (its occupied-slot give still stands)
        return 0;
    }
    return -1;
}

// Consumer-side extract of the STRONGEST burst; compacts the array. Caller
// holds s_pq_lock and has already taken s_pq_items (so count > 0 is guaranteed).
static void pq_extract_max_locked(detected_burst_t *out)
{
    int   max_i = 0;
    float max_s = s_pq[0].peak_snr_db;
    for (int i = 1; i < s_pq_count; i++) {
        if (s_pq[i].peak_snr_db > max_s) {
            max_s = s_pq[i].peak_snr_db;
            max_i = i;
        }
    }
    *out        = s_pq[max_i];
    s_pq[max_i] = s_pq[--s_pq_count]; // move last into the hole
}

// Diagnostic counters. Read & reset by worker_core1_get_stats().
// _Atomic, not volatile: push_burst increments s_bursts_queued from the
// producer (Core 0 tagger callback) while get_stats read-and-resets from
// class_driver — a volatile RMW on RV32 loses increments across that
// race, and volatile 64-bit reads can tear. Relaxed ordering is plenty
// for diagnostics; get_stats uses atomic_exchange for exact windows.
static _Atomic uint32_t s_bursts_queued    = 0;
static _Atomic uint32_t s_bursts_dropped   = 0;
static _Atomic uint32_t s_bursts_processed = 0;
static _Atomic uint32_t s_bursts_skipped   = 0;
static _Atomic uint32_t s_queue_high_water = 0;
// T59 lag instrument: throttle for the (spammy) pre-read stale log so we can
// print the actual staleness magnitude ~1/s instead of ~130/s.
static uint32_t         s_stale_log_throttle = 0;
static _Atomic uint64_t s_burst_total_us     = 0;
// BCH outcome counters. processed counts qpsk_demod successes (the
// "DEMOD SUCCESS" log). bch_decoded counts the ones that actually
// passed BCH — the real decode rate. bch_failed counts qpsk-demod
// successes that produced an uncorrectable frame (false-positive
// decodes from the application's perspective).
static _Atomic uint32_t s_bursts_bch_decoded         = 0; // BCH OK AND classify returned a known type
static _Atomic uint32_t s_bursts_bch_unknown         = 0; // BCH OK but iridium_frame_classify => IR_FRAME_UNKNOWN (BCH false-positive — task #111)
static _Atomic uint32_t s_bursts_bch_failed          = 0; // BCH itself uncorrectable
static _Atomic uint32_t s_bursts_bch_chase_recovered = 0; // Chase-2 soft decoder rescued a hard-decision BCH failure (#112)

// Diagnostic histograms (#116). Cumulative since boot — no decay /
// rolling window; clients compute deltas if they want a rate.
// Closes the design-review gap "can't tell antenna-empty from
// demod-broken from /status alone." SNR bins are 1 dB wide [0..32);
// BCH bins are a 4×4 joint of e1 × e2 codes (-1 = failed, 0..2 = corrected).
// T45: _Atomic (not volatile) for the same reason as the counters
// above -- hist_*_record's `array[bin]++` is a read-modify-write;
// worker_core1_get_histograms() reads these concurrently from the
// httpd task. A plain volatile RMW is a data race (UB) even though
// there's a single writer, and matches the exact pattern this file
// already fixed for s_bursts_queued et al.
#define HIST_SNR_BINS 32
static _Atomic uint32_t s_hist_snr[HIST_SNR_BINS]; // bin i = bursts with floor(SNR_dB) == i
#define HIST_BCH_BINS 16                           // (e1+1)*4 + (e2+1), e ∈ {-1..2}
static _Atomic uint32_t s_hist_bch[HIST_BCH_BINS];

static inline void hist_snr_record(float snr_db)
{
    int bin = (int)snr_db;
    if (bin < 0) bin = 0;
    if (bin >= HIST_SNR_BINS) bin = HIST_SNR_BINS - 1;
    s_hist_snr[bin]++;
}
static inline void hist_bch_record(int e1, int e2)
{
    // Clamp to {-1, 0, 1, 2} then offset to {0..3}
    int a = (e1 < -1) ? -1 : (e1 > 2 ? 2 : e1);
    int b = (e2 < -1) ? -1 : (e2 > 2 ? 2 : e2);
    s_hist_bch[(a + 1) * 4 + (b + 1)]++;
}

// Per-stage timing accumulators, summed over processed bursts only.
static _Atomic uint64_t s_t_extract_us  = 0;
static _Atomic uint64_t s_t_rotate_us   = 0;
static _Atomic uint64_t s_t_decim_us    = 0;
static _Atomic uint64_t s_t_pipeline_us = 0;
static _Atomic uint64_t s_t_bch_us      = 0;

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
#define GOLDEN_TIME_TOL_2500K 125000
#define GOLDEN_FREQ_TOL_HZ 20000

static volatile uint32_t s_gold_decoded      = 0; // device decoded a frame
static volatile uint32_t s_gold_matched      = 0; // and matched a golden entry
static volatile uint32_t s_gold_unmatched    = 0; // decoded but no golden in window
static volatile uint32_t s_gold_exact        = 0; // BER = 0 over min length
static volatile uint32_t s_gold_close        = 0; // BER < 5 %
static volatile uint32_t s_gold_partial      = 0; // BER < 25 %
static volatile uint32_t s_gold_divergent    = 0; // BER >= 25 %
static volatile uint32_t s_gold_total_bits   = 0; // sum of min(device, gri) lengths
static volatile uint32_t s_gold_total_errors = 0; // sum of Hamming distances
static volatile uint32_t s_gold_len_eq       = 0; // device n_bits == gri n_bits
static volatile uint32_t s_gold_len_short    = 0; // device n_bits < gri n_bits

// Per-golden-entry "claimed" flag. When a device burst matches a
// golden entry, we mark it so a second device burst at the same
// time-freq cell can't double-count it. After all bursts have been
// processed, unclaimed golden entries are "MISSED by device" —
// the recall complement of the matched count.
static uint8_t s_gold_claimed[128]; // sized > FIXTURE_ALBQ_RAW_GOLDEN_COUNT

// Per-burst row for end-of-run reporting (smoke only — 65-ish bursts,
// fits comfortably in RAM).
typedef struct {
    int gri_id;
    int device_n_bits;
    int gri_n_bits;
    int compared_bits;
    int errors;
    int bucket;   // 0=exact 1=close 2=partial 3=divergent 4=unmatched
    int bch_e1;   // post-BCH block-1 correctable-error count, -1 if no BCH
    int bch_e2;   // post-BCH block-2 correctable-error count, -1 if no BCH
    int gri_conf; // gri's reported confidence pct (-1 for UNMATCHED rows)
} golden_row_t;
static golden_row_t s_gold_rows[128];
static int          s_gold_n_rows = 0;

// Aggregated post-BCH stats across matched-AND-BCH-attempted bursts.
// e1/e2 < 0 means BCH failed (more than 3 errors → uncorrectable).
// e1 == 0 && e2 == 0 → clean decode, no errors needed correcting.
static volatile uint32_t s_gold_bch_clean     = 0; // e1==0 && e2==0
static volatile uint32_t s_gold_bch_corrected = 0; // (e1>0 || e2>0) && both ≥ 0
static volatile uint32_t s_gold_bch_failed    = 0; // either block uncorrectable
static volatile uint32_t s_gold_bch_skipped   = 0; // frame too short for BCH

static void golden_compare_burst(const detected_burst_t *burst,
                                 const decoded_frame_t  *frame,
                                 int e1_bch, int e2_bch)
{
    s_gold_decoded++;
    // Find golden entry within tolerance window. Pick the one with
    // smallest combined (Δsample, Δfreq) distance.
    int     best       = -1;
    int64_t best_score = INT64_MAX;
    for (int g = 0; g < FIXTURE_ALBQ_RAW_GOLDEN_COUNT; g++) {
        if (s_gold_claimed[g]) continue; // already paired with an earlier burst
        const golden_burst_t *e  = &FIXTURE_ALBQ_RAW_GOLDEN_BURSTS[g];
        int64_t               ds = (int64_t)burst->start_sample_idx - (int64_t)e->start_sample_2500k;
        if (ds < 0) ds = -ds;
        if (ds > GOLDEN_TIME_TOL_2500K) continue;
        int64_t df = (int64_t)burst->rel_freq_hz - (int64_t)e->freq_offset_hz;
        if (df < 0) df = -df;
        if (df > GOLDEN_FREQ_TOL_HZ) continue;
        // Combined score normalised to tolerances.
        int64_t score = (ds * 100 / GOLDEN_TIME_TOL_2500K) + (df * 100 / GOLDEN_FREQ_TOL_HZ);
        if (score < best_score) {
            best_score = score;
            best       = g;
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
                .gri_id        = -1,
                .device_n_bits = frame->n_bits,
                .gri_n_bits    = 0,
                .compared_bits = 0,
                .errors        = 0,
                .bucket        = 4,
                .bch_e1        = e1_bch,
                .bch_e2        = e2_bch,
                .gri_conf      = -1,
            };
        }
        return;
    }
    s_gold_claimed[best]         = 1; // pair this golden entry to the current device burst
    const golden_burst_t *e      = &FIXTURE_ALBQ_RAW_GOLDEN_BURSTS[best];
    int                   cmp_n  = frame->n_bits < e->gri_n_bits ? frame->n_bits : e->gri_n_bits;
    int                   errors = 0;
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
        int  n_to_dump = cmp_n < 384 ? cmp_n : 384;
        for (int k = 0; k < n_to_dump; k++) {
            dev_str[k] = frame->bits[k] ? '1' : '0';
            gri_str[k] = e->gri_bits[k] ? '1' : '0';
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
    if (frame->n_bits == e->gri_n_bits)
        s_gold_len_eq++;
    else if (frame->n_bits < e->gri_n_bits)
        s_gold_len_short++;

    int bucket;
    if (cmp_n == 0) {
        bucket = 4; // can't classify with zero bits compared
    } else {
        int ber_pct = errors * 100 / cmp_n;
        if (errors == 0) {
            s_gold_exact++;
            bucket = 0;
        } else if (ber_pct < 5) {
            s_gold_close++;
            bucket = 1;
        } else if (ber_pct < 25) {
            s_gold_partial++;
            bucket = 2;
        } else {
            s_gold_divergent++;
            bucket = 3;
        }
    }
    if (s_gold_n_rows < (int)(sizeof(s_gold_rows) / sizeof(s_gold_rows[0]))) {
        s_gold_rows[s_gold_n_rows++] = (golden_row_t){
            .gri_id        = e->gri_id,
            .device_n_bits = frame->n_bits,
            .gri_n_bits    = e->gri_n_bits,
            .compared_bits = cmp_n,
            .errors        = errors,
            .bucket        = bucket,
            .bch_e1        = e1_bch,
            .bch_e2        = e2_bch,
            .gri_conf      = e->conf_pct,
        };
    }
}

void worker_core1_golden_get(uint32_t *matched, uint32_t *decoded, int *gri_total)
{
    if (matched) *matched = s_gold_matched;
    if (decoded) *decoded = s_gold_decoded;
    if (gri_total) *gri_total = FIXTURE_ALBQ_RAW_GOLDEN_COUNT;
}

void worker_core1_golden_print_summary(void)
{
    int n_gri    = FIXTURE_ALBQ_RAW_GOLDEN_COUNT;
    int n_missed = 0;
    for (int g = 0; g < n_gri; g++) {
        if (!s_gold_claimed[g]) n_missed++;
    }
    double recall_pct    = n_gri > 0 ? 100.0 * (double)s_gold_matched / (double)n_gri : 0.0;
    double precision_pct = s_gold_decoded > 0
                               ? 100.0 * (double)s_gold_matched / (double)s_gold_decoded
                               : 0.0;
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
        char                bch_str[32];
        if (r->bch_e1 < 0 && r->bch_e2 < 0)
            snprintf(bch_str, sizeof(bch_str), "skip");
        else if (r->bch_e1 < 0 || r->bch_e2 < 0)
            snprintf(bch_str, sizeof(bch_str), "FAIL");
        else
            snprintf(bch_str, sizeof(bch_str), "%d/%d",
                     r->bch_e1, r->bch_e2);
        if (r->bucket == 4 && r->gri_id < 0) {
            ESP_LOGI(TAG, "GOLDEN[%d]: UNMATCHED device_n=%d bch=%s",
                     i, r->device_n_bits, bch_str);
        } else {
            const char *tag = r->bucket == 0   ? "EXACT"
                              : r->bucket == 1 ? "CLOSE"
                              : r->bucket == 2 ? "PARTIAL"
                              : r->bucket == 3 ? "DIVERG"
                                               : "UNMATCH";
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
void worker_core1_golden_print_summary(void)
{
}
#endif // CONFIG_SMOKE_TEST_RAW_IRIDIUM

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
// zero-pad for PIE 8-alignment); 144 already lines up on 16-cplx.
//
// T45: was 288 (18 × 16), which is 16-cplx aligned but NOT a multiple
// of DIDECIM_DECIM (=10): 288 mod 10 = 8. Since safe_len below is
// rounded to a multiple of 80 (= LCM(10, 16)), ext_len = safe_len +
// WB_PRE_PAD_SAMPLES inherited that same "8 mod 10" remainder. The
// streaming decim loop's LAST chunk then had a non-multiple-of-10
// tail that direct_if_decim_process_split silently drops (n_out =
// n_in / DIDECIM_DECIM, integer division, and there's no next chunk
// to carry the remainder into) -- up to 9 raw samples lost off the
// very end of the extraction window every burst. That end is deep in
// WB_EXTRACT_SAFETY trailing padding, past the real burst content, so
// it didn't affect decode -- but it's a real dropped-sample bug, not
// just a latent one. 320 (= 20 × 16 = 4 × 80) is a multiple of 80, so
// it satisfies the cache/decim alignment as strictly as safe_len does
// and ext_len is always an exact multiple of DIDECIM_DECIM with no
// tail loss. Still ≥ DIDECIM_NTAPS - 1 (143) with margin to spare.
//
// T45 REVERTED (2026-07-05): bumping this 288→320 was NOT decode-neutral.
// The dropped tail samples are in WB_EXTRACT_SAFETY past the burst (as the
// comment above says — they don't matter), but WB_PRE_PAD is the *pre-roll*:
// +32 leading samples shifts the whole extraction window, moving the
// decim phase / D13-start / UW alignment. Measured: RAW GOLDEN overall BER
// 1.39%→2.42%, exact matches 44→43, divergent 1→3. The "tail-loss fix"
// wasn't worth an alignment shift that degrades every burst's bits. Keep 288.
#define WB_PRE_PAD_SAMPLES 288
#define WB_MAX_BURST_SAMPLES ((int)(FS_DETECT_HZ / 4)) // 250 ms = 625000
#define WB_EXTRACT_SAFETY 1024
#define WB_EXTRACT_MAX (WB_MAX_BURST_SAMPLES + WB_PRE_PAD_SAMPLES + WB_EXTRACT_SAFETY)

// 250 ksps output is at most WB_EXTRACT_MAX / 10 + 1.
#define WB_DECIM_MAX ((WB_EXTRACT_MAX / DIDECIM_DECIM) + 8)

// (T45's WB_PRE_PAD % DIDECIM_DECIM static_assert removed with the 320
// revert — 288 % 10 != 0 by design; the harmless tail-sample drop is
// accepted in exchange for decode-alignment stability. See the define.)
_Static_assert(WB_PRE_PAD_SAMPLES % 16 == 0,
               "WB_PRE_PAD_SAMPLES must be 16-complex aligned (64 B) for "
               "esp_cache_msync on the extracted PSRAM range");
_Static_assert(WB_PRE_PAD_SAMPLES >= DIDECIM_NTAPS - 1,
               "WB_PRE_PAD_SAMPLES must cover the decim FIR's group delay");

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
#define DECIM_CHUNK_IN 4000

static int16_t *s_extract_buf = NULL; // 2.5 MSPS wideband window
static int16_t *s_decim_buf   = NULL; // 250 ksps post-decim
static int16_t *s_chunk_iq    = NULL; // INTERNAL — per-chunk IQ scratch
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
    uint64_t                t_bch_accum; // accumulated BCH+log+queue time
} wb_worker_ctx_t;

static void worker_emit_frame(burst_pipeline_result_t *bres, void *ctx)
{
    wb_worker_ctx_t *wctx   = (wb_worker_ctx_t *)ctx;
    int64_t          t_bch0 = esp_timer_get_time();

    // Verbose D13/UW info -- one line per FRAME now (with multi-frame
    // this fires multiple times per burst). Demoted to ESP_LOGD; the
    // BCH outcome below stays at ESP_LOGI as the per-frame outcome
    // marker.
    ESP_LOGD(TAG, "D13 start=%d  UW dir=%s off=%d corr=%.3f SNR=%.1f omega=%.3f",
             bres->burst_start,
             bres->uw_res.direction == UW_DIR_DOWNLINK ? "DL" : bres->uw_res.direction == UW_DIR_UPLINK ? "UL"
                                                                                                        : "??",
             bres->uw_res.uw_offset,
             (double)bres->uw_res.correction,
             (double)bres->uw_res.snr_estimate_db,
             (double)bres->uw_res.omega_per_sym);

    if (!bres->demod_ok) {
        // Failed sub-frames still fire the callback for diagnostic
        // logging; just free the bits and return.
        free(bres->frame.bits);
        free(bres->frame.soft_bits); // #112
        wctx->t_bch_accum += (uint64_t)(esp_timer_get_time() - t_bch0);
        return;
    }

    decoded_frame_t frame = bres->frame;
    ESP_LOGI(TAG, "DEMOD SUCCESS: %s frame (%d bits)",
             frame.direction == DIR_DOWNLINK ? "DL" : "UL",
             frame.n_bits);

    int  e1_bch = -1, e2_bch = -1;
    bool chase_used = false;
    bool real_known = false; // BCH-passed AND classified to a known type (#111)
    (void)real_known;        // only read in WORKER/COMBINED builds
    if (frame.n_bits >= 24 + 64) {
        const uint8_t *payload = frame.bits + 24;
        uint8_t        block1[32], block2[32];
        uint8_t        data1[21], data2[21];

        iridium_deinterleave(payload, block1, block2);
        e1_bch = bch_decode_block(block1, data1);
        e2_bch = bch_decode_block(block2, data2);
        hist_bch_record(e1_bch, e2_bch); // #116 — records pre-Chase outcome

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
                if (e >= 0) {
                    e1_bch     = e;
                    chase_used = true;
                }
            }
            if (e2_bch < 0) {
                int e = bch_decode_block_soft(soft2, data2, 3);
                if (e >= 0) {
                    e2_bch     = e;
                    chase_used = true;
                }
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
            iridium_frame_t      classified = {0};
            ir_frame_direction_t fdir       = (frame.direction == DIR_DOWNLINK)
                                                  ? IR_FRM_DIR_DOWNLINK
                                                  : IR_FRM_DIR_UPLINK;
            int                  crc        = iridium_frame_classify(frame.bits, frame.n_bits,
                                                                     fdir, &classified);
            if (crc == 0 && classified.type != IR_FRAME_UNKNOWN) {
                ESP_LOGD(TAG, "BCH PASS: errors=%d/%d type=%s (real decode)",
                         e1_bch, e2_bch,
                         iridium_frame_type_name(classified.type));
                s_bursts_bch_decoded++;
                real_known = true;
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
    // Capture timestamp from the burst's sample position (not decode time),
    // so a freshness-first/LIFO queue can't reorder emitted timestamps.
    // stream_epoch is 0 only before the first push (never here); then this is
    // just esp_timer at index 0 + the burst's age in sample-time.
    const uint64_t cap_us = signal_buffer_stream_epoch_us() +
                            wctx->burst->start_sample_idx * 1000000ULL / FS_DETECT_HZ;
#if CONFIG_DEVICE_ROLE_WORKER || CONFIG_DEVICE_ROLE_COMBINED_LOOPBACK
    // Distributed front end (#135): ship only known-type frames as PDUs to
    // the aggregator (preserves the #111 bandwidth saving vs BCH false
    // positives). No local frame_decoder — that runs on the aggregator.
    if (real_known) {
        iridium_frame_pdu_t pdu = {0};
        pdu.timestamp_us        = cap_us;
        pdu.source_id           = frame_pdu_source_id();
        pdu.rel_freq_hz         = (int32_t)wctx->burst->rel_freq_hz;
        pdu.peak_snr_db         = wctx->burst->peak_snr_db;
        pdu.peak_bin            = (int16_t)wctx->burst->peak_bin;
        pdu.direction           = (frame.direction == DIR_DOWNLINK) ? 0 : 1;
        pdu.bch_e1              = (int8_t)e1_bch;
        pdu.bch_e2              = (int8_t)e2_bch;
        pdu.flags               = chase_used ? FRAME_PDU_FLAG_CHASE : 0;
        frame_pdu_pack_bits(&pdu, frame.bits, frame.n_bits);
        frame_pdu_queue_push(&pdu);
    }
#else // STANDALONE (and AGGREGATOR, which never reaches here)
    frame_decoder_push(frame.bits, frame.n_bits,
                       frame.direction, 0u,
                       wctx->burst->peak_bin, wctx->burst->peak_snr_db,
                       cap_us);
#endif
    free(frame.bits);
    free(frame.soft_bits); // #112
    wctx->t_bch_accum += (uint64_t)(esp_timer_get_time() - t_bch0);
}

void worker_task(void *arg)
{
    ESP_LOGI(TAG, "Worker Task started on Core %d", xPortGetCoreID());
    detected_burst_t burst;

    while (1) {
        {
            // Block until a burst is available, then take the STRONGEST one.
            xSemaphoreTake(s_pq_items, portMAX_DELAY);
            xSemaphoreTake(s_pq_lock, portMAX_DELAY);
            pq_extract_max_locked(&burst);
            xSemaphoreGive(s_pq_lock);
            int64_t burst_t0 = esp_timer_get_time();
            ESP_LOGD(TAG, "Worker burst: start=%llu len=%lu rel=%+.0f Hz SNR=%.1f dB",
                     (unsigned long long)burst.start_sample_idx,
                     (unsigned long)burst.length_samples,
                     (double)burst.rel_freq_hz,
                     (double)burst.peak_snr_db);
            hist_snr_record(burst.peak_snr_db); // #116

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
            // T44: check_start is the 64-bit absolute (cumulative) index for
            // burst_valid; the ring-offset callers below take (uint32_t)ext_start,
            // which is congruent mod total_cap so the ring mapping is unchanged.
            uint64_t check_start = burst.start_sample_idx - WB_PRE_PAD_SAMPLES;
            uint32_t check_len   = burst.length_samples + WB_PRE_PAD_SAMPLES;
            if (!signal_buffer_burst_valid(check_start, check_len)) {
                // T59 lag instrument: how far behind the producer is this
                // burst's start when we finally look at it? >ring-span means
                // the detect chain is lagging live ingest; the magnitude tells
                // fixed-latency (shrinkable) vs unbounded-deficit apart.
                if ((s_stale_log_throttle++ & 0x7F) == 0) {
                    uint64_t lag = signal_buffer_head_total() - burst.start_sample_idx;
                    ESP_LOGW(TAG, "stale burst: start=%llu lag=%lu ms (%.2f ring-spans) — drop",
                             (unsigned long long)burst.start_sample_idx,
                             (unsigned long)(lag * 1000ULL / FS_DETECT_HZ),
                             (double)lag / (double)(SIGNAL_BUF_SIZE / 4));
                }
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
            // 80 satisfies both, and (T45) WB_PRE_PAD_SAMPLES is ALSO a
            // multiple of 80 (see its definition above), so
            // ext_len = safe_len + WB_PRE_PAD_SAMPLES stays a multiple
            // of 80 too -- both alignments hold for ext_len, not just
            // safe_len.
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
            uint32_t ext_start = (uint32_t)check_start; // ring offset (mod total_cap)
            int64_t  t_ext0    = esp_timer_get_time();
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
            int64_t t_rot0     = esp_timer_get_time();
            double  phase_step = -2.0 * M_PI * (double)burst.rel_freq_hz / (double)FS_DETECT_HZ;
            int64_t t_rot1     = esp_timer_get_time();
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

            // T38: re-check burst_valid AFTER the read loop above. The
            // read is a multi-ms operation (chunked over ext_len /
            // DECIM_CHUNK_IN calls to signal_buffer_read_chunk); the
            // producer (ingest, Core 0) can advance the ring head
            // during that window and start overwriting the tail of the
            // range we're reading from mid-read, at which point the
            // pre-read check at the top of this block is stale and we
            // may have just decimated a mix of real and overwritten
            // samples. Mirror the same conservative envelope used
            // there (check_start/check_len already cover
            // WB_PRE_PAD_SAMPLES on both sides) and drop rather than
            // hand a possibly-torn burst to the pipeline.
            if (!signal_buffer_burst_valid(check_start, check_len)) {
                uint64_t lag = signal_buffer_head_total() - burst.start_sample_idx;
                ESP_LOGW(TAG, "stale burst: start=%llu lag=%lu ms (during read) — drop",
                         (unsigned long long)burst.start_sample_idx,
                         (unsigned long)(lag * 1000ULL / FS_DETECT_HZ));
                s_bursts_skipped++;
                continue;
            }

            if (n_250k <= 64) {
                ESP_LOGD(TAG, "direct_if_decim produced %d samples — too short",
                         n_250k);
                s_bursts_skipped++;
                continue;
            }

            // 3.5 Per-burst DC removal moved INSIDE burst_pipeline_process_burst
            //     as step 0 (2026-05-31, #128). Was at this site originally
            //     (#113) but the host pipeline tests didn't replicate it,
            //     which gave the host suite a different pre-CFO statistic
            //     than the device worker — the gap that hid the #115
            //     regression. Now in one place.

            // 4. Per-burst pipeline at 250 ksps. Multi-frame callback
            //    fires once per decoded sub-frame so multi-frame bursts
            //    (gri's handle_multiple_frames_per_burst) yield all
            //    their frames instead of just the first one.
            int64_t         t_pipe0    = esp_timer_get_time();
            wb_worker_ctx_t worker_ctx = {
                .burst       = &burst,
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

        // Periodic yield so same-prio tasks (frame_decoder, both at 4)
        // and lower-prio tasks get scheduled. One vTaskDelay(1) every
        // 8 bursts gives ~10 ms of ceded CPU per 8 bursts. NB: the
        // skipped/stale-burst paths `continue` above this point, so
        // only fully processed (or demod-attempted) bursts advance the
        // counter — skips are cheap, that's the intended behaviour.
        static int yield_counter = 0;
        if (++yield_counter >= 8) {
            yield_counter = 0;
            vTaskDelay(1);
        }
    }
}

// Boot-time early-alloc dance entry point: pins s_decim's PIE FIR
// delay lines (wideband decim FIR, I+Q) in DRAM while it is plentiful.
// direct_if_decim_init() is fully self-contained (builds its own taps
// from constants, no config from worker_core1_init needed), so this
// is a thin wrapper -- no need to fold worker_core1_init's other
// allocations (burst queue, chunk/scratch buffers) forward too; those
// aren't PIE-touched and aren't part of this heap-position hazard.
// worker_core1_init() below calls direct_if_decim_init(&s_decim)
// again; that second call is a cheap idempotent no-op for the FIR
// state (see the fir_dsp_inited guard in direct_if_decim_init) that
// just re-derives the (identical) taps and resets the delay lines.
void worker_core1_prealloc_fir(void)
{
    direct_if_decim_init(&s_decim);
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
    // SNR priority queue (T59): a small bounded buffer in .bss (BURST_PQ_CAP ×
    // detected_burst_t ≈ 1.3 KB — no PSRAM needed) guarded by a mutex, with a
    // counting semaphore tracking occupied slots so the worker can block-wait.
    s_pq_lock  = xSemaphoreCreateMutex();
    s_pq_items = xSemaphoreCreateCounting(BURST_PQ_CAP, 0);
    if (!s_pq_lock || !s_pq_items) {
        ESP_LOGE(TAG, "Burst PQ semaphore create failed");
        return ESP_ERR_NO_MEM;
    }
    s_pq_count = 0;
    ESP_LOGI(TAG, "Burst SNR priority queue: cap=%d × %u B = %u B",
             BURST_PQ_CAP, (unsigned)sizeof(detected_burst_t),
             (unsigned)(BURST_PQ_CAP * sizeof(detected_burst_t)));

    // Wideband buffers. Big working surfaces stay in PSRAM (extract +
    // decim output). The PIE FIR scratch must live in INTERNAL SRAM,
    // otherwise `esp.vld.128.ip` inside dsps_fird_s16_arp4 can't
    // service PSRAM access timing and the PIE path silently degrades
    // (we previously measured 22.9 ms/burst on this stage; PIE should
    // do ~1.5 ms). Decim runs in DECIM_CHUNK_IN-sample chunks so the
    // scratch stays tiny (8 KB per I/Q channel) rather than the 100 KB
    // each that a full-burst buffer would need.
    size_t ext_bytes = (size_t)WB_EXTRACT_MAX * 2 * sizeof(int16_t);
    size_t dec_bytes = (size_t)WB_DECIM_MAX * 2 * sizeof(int16_t);
    // +16 int16 trailing pad on the FIR scratch: the arp4 PIE kernels
    // (dsps_fird_s16_arp4 inside direct_if_decim_process_split) read/
    // write up to one 128-bit vector past the nominal buffer end.
    size_t scr_in_bytes   = (size_t)(DECIM_CHUNK_IN + 16) * sizeof(int16_t);
    size_t scr_out_bytes  = (size_t)(DECIM_CHUNK_IN / DIDECIM_DECIM + 16) * sizeof(int16_t);
    size_t chunk_iq_bytes = (size_t)DECIM_CHUNK_IN * 2 * sizeof(int16_t);
    // Task #64: s_extract_buf removed -- decim loop reads directly
    // from signal_buffer via signal_buffer_read_chunk(). The static
    // pointer is kept null; init failure check below ignores it.
    // Saves ~2.5 MB of PSRAM and the 7.5 ms/burst extract-stage
    // PSRAM write.
    (void)ext_bytes;
    s_extract_buf     = NULL;
    s_decim_buf       = heap_caps_malloc(dec_bytes, MALLOC_CAP_SPIRAM);
    s_chunk_iq        = heap_caps_aligned_alloc(16, chunk_iq_bytes, MALLOC_CAP_INTERNAL);
    s_decim_scr_in_i  = heap_caps_aligned_alloc(16, scr_in_bytes, MALLOC_CAP_INTERNAL);
    s_decim_scr_in_q  = heap_caps_aligned_alloc(16, scr_in_bytes, MALLOC_CAP_INTERNAL);
    s_decim_scr_out_i = heap_caps_aligned_alloc(16, scr_out_bytes, MALLOC_CAP_INTERNAL);
    s_decim_scr_out_q = heap_caps_aligned_alloc(16, scr_out_bytes, MALLOC_CAP_INTERNAL);
    // s_extract_buf intentionally NULL since task #64; don't check it.
    if (!s_decim_buf || !s_chunk_iq || !s_decim_scr_in_i || !s_decim_scr_in_q || !s_decim_scr_out_i || !s_decim_scr_out_q) {
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
        // Don't report a healthy init with no worker: the PQ would still exist,
        // so push_burst would admit bursts that nothing ever drains. Tear down
        // so the caller sees the failure.
        ESP_LOGE(TAG, "worker_core1 task create failed — tearing down");
        vSemaphoreDelete(s_pq_items);
        vSemaphoreDelete(s_pq_lock);
        s_pq_items = NULL;
        s_pq_lock  = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void worker_core1_push_burst(const detected_burst_t *burst)
{
    if (!s_pq_lock) return;
    s_bursts_queued++;
    xSemaphoreTake(s_pq_lock, portMAX_DELAY);
    int r = pq_insert_locked(burst);
    if (s_pq_count > (int)s_queue_high_water) s_queue_high_water = s_pq_count;
    xSemaphoreGive(s_pq_lock);
    // Give the "item available" count only when a slot went from empty to
    // occupied. r==0 evicted the weakest in place (occupied-slot count
    // unchanged — its original give still stands); r<0 dropped this burst
    // (buffer full and it was weaker than everything already queued).
    if (r > 0) {
        xSemaphoreGive(s_pq_items);
    } else if (r < 0) {
        s_bursts_dropped++;
    }
}

void worker_core1_get_histograms(worker_histograms_t *out)
{
    if (!out) return;
    uint32_t total_snr = 0, total_bch = 0;
    for (int i = 0; i < HIST_SNR_BINS; i++) {
        out->snr[i] = s_hist_snr[i];
        total_snr += s_hist_snr[i];
    }
    for (int i = 0; i < HIST_BCH_BINS; i++) {
        out->bch[i] = s_hist_bch[i];
        total_bch += s_hist_bch[i];
    }
    out->snr_total = total_snr;
    out->bch_total = total_bch;
}

void worker_core1_get_stats(worker_stats_t *out)
{
    // Exchange-with-zero so increments landing between "read" and
    // "reset" are counted in the NEXT window instead of lost.
    uint32_t n = atomic_exchange_explicit(&s_bursts_processed, 0,
                                          memory_order_relaxed);
    out->bursts_queued =
        atomic_exchange_explicit(&s_bursts_queued, 0, memory_order_relaxed);
    out->bursts_dropped =
        atomic_exchange_explicit(&s_bursts_dropped, 0, memory_order_relaxed);
    out->bursts_processed = n;
    out->bursts_skipped =
        atomic_exchange_explicit(&s_bursts_skipped, 0, memory_order_relaxed);
    out->bursts_bch_decoded =
        atomic_exchange_explicit(&s_bursts_bch_decoded, 0, memory_order_relaxed);
    out->bursts_bch_unknown =
        atomic_exchange_explicit(&s_bursts_bch_unknown, 0, memory_order_relaxed);
    out->bursts_bch_failed =
        atomic_exchange_explicit(&s_bursts_bch_failed, 0, memory_order_relaxed);
    out->bursts_bch_chase_recovered =
        atomic_exchange_explicit(&s_bursts_bch_chase_recovered, 0,
                                 memory_order_relaxed);
    out->queue_high_water =
        atomic_exchange_explicit(&s_queue_high_water, 0, memory_order_relaxed);
    uint64_t total_us =
        atomic_exchange_explicit(&s_burst_total_us, 0, memory_order_relaxed);
    uint64_t t_extract =
        atomic_exchange_explicit(&s_t_extract_us, 0, memory_order_relaxed);
    uint64_t t_rotate =
        atomic_exchange_explicit(&s_t_rotate_us, 0, memory_order_relaxed);
    uint64_t t_decim =
        atomic_exchange_explicit(&s_t_decim_us, 0, memory_order_relaxed);
    uint64_t t_pipeline =
        atomic_exchange_explicit(&s_t_pipeline_us, 0, memory_order_relaxed);
    uint64_t t_bch =
        atomic_exchange_explicit(&s_t_bch_us, 0, memory_order_relaxed);
    if (n > 0) {
        float fn            = (float)n;
        out->avg_burst_us   = (float)total_us / fn;
        out->extract_us     = (float)t_extract / fn;
        out->freq_center_us = (float)t_rotate / fn;
        out->fir_decim_us   = (float)t_decim / fn;
        out->resample_us    = 0.0f;
        out->demod_us       = (float)t_pipeline / fn;
        out->bch_us         = (float)t_bch / fn;
    } else {
        out->avg_burst_us = out->extract_us = out->freq_center_us =
            out->fir_decim_us = out->resample_us = out->demod_us =
                out->bch_us                      = 0.0f;
    }
}
