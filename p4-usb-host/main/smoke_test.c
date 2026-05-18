// Target-side smoke test — see smoke_test.h for the high-level idea.
//
// Drives the same convert -> ingest_core1 -> dsp_processor path the
// production class_driver loop uses, with a synthetic IQ source instead
// of USB. Asserts that the FFT detector raises exactly one burst at a
// non-edge bin with sensible SNR.

#include <math.h>
#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "sdkconfig.h"
#include "ingest_core1.h"
#include "signal_buffer.h"
#include "dsp_processor.h"
#include "smoke_test.h"

#if CONFIG_SMOKE_TEST_CORPUS
#include "fixture_corpus_uint8.h"
#endif

#if CONFIG_SMOKE_TEST_REAL_IRIDIUM
#include "fixture_albq_stripes.h"
#include "frame_decoder.h"
#include "worker_core1.h"
#include "bch_decoder.h"
#include "qpsk_demod.h"
#endif

#if CONFIG_SMOKE_TEST_RAW_IRIDIUM
#include "fixture_albq_raw.h"
#include "frame_decoder.h"
#include "worker_core1.h"
#include "bch_decoder.h"
#include "qpsk_demod.h"
#endif

#if CONFIG_SMOKE_TEST_FRAME_DECODER
#include "fixture_albq_frames_corpus.h"
#include "frame_decoder.h"
#include "qpsk_demod.h"
#endif

static const char *TAG = "SMOKE";

#define TRANSFER_BYTES   (16 * 1024)         // matches class_driver's out_block_size
#define TRANSFER_SAMPLES (TRANSFER_BYTES / 2) // complex samples per transfer (1 byte I + 1 byte Q)

// Inject the tone at FFT bin 200 (positive frequency, well clear of DC at
// bin 0 and Nyquist at bin 1024). After fftshift the magnitudes index is
// (bin + 1024) % 2048 = 1224. We assert the detected peak_bin is in
// 1200..1248 — generous to absorb spectral leakage.
#define TONE_FFT_BIN     200
#define EXPECTED_BIN_LO  1200
#define EXPECTED_BIN_HI  1248

// Phases of the synthetic stream:
//   PRIMING noise transfers — let the baseline EMA settle before injecting
//     the tone. dsp_processor's PRIMING_FRAMES=16 means we need ≥16/4=4
//     transfers (each contains 4 FFT frames). Use 8 for margin.
//   TONE transfers — drive a strong tone for the burst to fire.
//   TRAILER noise transfer(s) — terminate the burst (detector logs
//     BURST DETECTED only on the tone -> noise transition).
#define PRIMING_TRANSFERS 8
#define TONE_TRANSFERS    4
#define TRAILER_TRANSFERS 2

// Detection callback state. The callback fires once per completed burst.
//
// Random noise during the priming phase will occasionally spike one bin
// above the baseline-EMA threshold (16 dB) and trigger a spurious
// "phantom" burst. That's a real detector property, not a test failure —
// the production detector handles it via the burst-vs-RFI worker filter.
// For the smoke test we assert against the strongest detection seen,
// which the tone phase will dominate by ≥30 dB.
static volatile int   s_bursts_detected = 0;
static volatile int   s_strongest_peak_bin = -1;
static volatile float s_strongest_snr_db = 0.0f;

static void on_burst(const detected_burst_t *burst)
{
    s_bursts_detected++;
    if (burst->peak_snr_db > s_strongest_snr_db) {
        s_strongest_snr_db = burst->peak_snr_db;
        s_strongest_peak_bin = burst->peak_bin;
    }
    // Log length (samples and approx milliseconds at 2.56 MSPS) to make
    // it easy to tell apart genuine tone-driven bursts (~10 ms) from
    // priming-noise-spike false positives (~1 frame ≈ 0.8 ms).
    float length_ms = (float)burst->length_samples / 2560.0f;
    ESP_LOGI(TAG, "callback: burst peak_bin=%d snr=%.2f dB "
                  "start=%lu len=%lu samples (%.2f ms)",
             burst->peak_bin, burst->peak_snr_db,
             (unsigned long)burst->start_sample_idx,
             (unsigned long)burst->length_samples,
             length_ms);
}

#if CONFIG_SMOKE_TEST_REAL_IRIDIUM || CONFIG_SMOKE_TEST_RAW_IRIDIUM
// Hybrid burst callback for full-stack smoke: counts the burst (so the
// existing detector-side assertions still work) AND forwards to
// worker_core1 so the burst gets demodulated, BCH-decoded, and
// classified by frame_decoder. Mirrors the production class_driver
// flow where dsp_processor's callback is worker_core1_push_burst.
static void on_burst_full_chain(const detected_burst_t *burst)
{
    on_burst(burst);
    worker_core1_push_burst(burst);
}
#endif

// Fill a TRANSFER_BYTES uint8 buffer with low-amplitude noise centred at
// 128 (the offset our convert path expects). Uses the hardware RNG so
// the noise has no spectral structure of its own.
static void fill_noise(uint8_t *out)
{
    for (int i = 0; i < TRANSFER_BYTES; i += 4) {
        uint32_t r = esp_random();
        // Low amplitude: ±8 around 128 — keeps the baseline well below
        // the tone's energy so SNR is large.
        out[i + 0] = 128 + (int8_t)((r >>  0) & 0x0f) - 8;
        out[i + 1] = 128 + (int8_t)((r >>  8) & 0x0f) - 8;
        out[i + 2] = 128 + (int8_t)((r >> 16) & 0x0f) - 8;
        out[i + 3] = 128 + (int8_t)((r >> 24) & 0x0f) - 8;
    }
}

// Fill a TRANSFER_BYTES uint8 buffer with a complex sinusoid at FFT bin
// `tone_bin` (relative to FFT_SIZE=2048) plus the same low-level noise.
// The phase is continuous across calls via the static `phase` accumulator
// so multiple consecutive transfers form one coherent tone.
static void fill_tone(uint8_t *out, int tone_bin)
{
    static double phase = 0.0;
    const double dphase = 2.0 * M_PI * (double)tone_bin / (double)FFT_SIZE;
    const double amplitude = 100.0;  // out of ±127 range
    for (int s = 0; s < TRANSFER_SAMPLES; s++) {
        double i_v = amplitude * cos(phase);
        double q_v = amplitude * sin(phase);
        phase += dphase;
        if (phase > 2.0 * M_PI) phase -= 2.0 * M_PI;
        // Add a touch of noise so adjacent FFT bins don't collapse to zero
        // (which would make the DC-leakage path of magnitude squared less
        // representative).
        uint32_t r = esp_random();
        i_v += ((int8_t)((r >> 0) & 0x0f) - 8) * 0.5;
        q_v += ((int8_t)((r >> 8) & 0x0f) - 8) * 0.5;
        int iv = (int)(i_v + 128.5);
        int qv = (int)(q_v + 128.5);
        if (iv < 0) { iv = 0; } else if (iv > 255) { iv = 255; }
        if (qv < 0) { qv = 0; } else if (qv > 255) { qv = 255; }
        out[s * 2 + 0] = (uint8_t)iv;
        out[s * 2 + 1] = (uint8_t)qv;
    }
}

// Drive one transfer through the production ingest -> dsp_processor path.
// `prev_slot` is the slot index from the previous call (or -1 first time);
// returns the slot index this call dispatched, for the next iteration's
// prev_slot.
static int drive_transfer(uint8_t *src, int prev_slot)
{
    int slot;
    uint8_t *raw = ingest_core1_acquire_raw(&slot);
    memcpy(raw, src, TRANSFER_BYTES);
    ingest_core1_dispatch(slot, TRANSFER_BYTES);

    if (prev_slot >= 0) {
        size_t n_int16 = 0;
        int16_t *converted = ingest_core1_take_converted(prev_slot, &n_int16);
        // n_int16 is bytes_filled; complex sample count is /2.
        dsp_processor_feed(converted, n_int16 / 2);
        ingest_core1_release(prev_slot);
    }
    return slot;
}

#if CONFIG_SMOKE_TEST_FRAME_DECODER
// Frame_decoder smoke path: bypasses USB / ingest / DSP entirely, pushes
// canned post-demod bits into frame_decoder_push() and verifies the
// classifier's per-class counts on real silicon. Catches regressions in
// the queue + classifier integration that wouldn't show up in host tests.
static void smoke_test_run_frame_decoder(void)
{
    ESP_LOGI(TAG, "=== Smoke test start (frame_decoder mode) ===");
    if (frame_decoder_init() != ESP_OK) {
        ESP_LOGE(TAG, "frame_decoder_init failed -> SMOKE_FAIL");
        return;
    }
    // Snapshot per-class counts before injection (the decoder may have
    // already classified zero frames; subtract baseline).
    frame_decoder_class_counts_t before, after;
    frame_decoder_get_class_counts(&before);
    ESP_LOGI(TAG, "Pushing %u corpus frames -> frame_decoder...",
             ALBQ_FRAME_CORPUS_LEN);
    int pushed_ok = 0, push_drops = 0;
    for (unsigned int i = 0; i < ALBQ_FRAME_CORPUS_LEN; i++) {
        const albq_frame_corpus_entry_t *e = &ALBQ_FRAME_CORPUS[i];
        // Use the per-entry expected_direction so UL frames in the
        // corpus are classified with the right UW (matches host
        // test_iridium_frame_corpus 100% agreement).
        ir_direction_t qdir = (e->expected_direction == IR_FRM_DIR_UPLINK)
                              ? DIR_UPLINK : DIR_DOWNLINK;
        // Retry-on-drop with bounded backoff. The decoder task runs at
        // ~10 ms/frame (one tick of vTaskDelay + classify), so we wait
        // at most a few ticks per push. Up to 100 attempts = 1 s
        // before giving up; far longer than realistic for this corpus.
        bool ok = false;
        for (int attempt = 0; attempt < 100; attempt++) {
            ok = frame_decoder_push(e->bits, e->n_bits, qdir,
                                    e->freq_hz, 0, e->snr_db);
            if (ok) break;
            vTaskDelay(1);   // one tick = drain a bit, then retry
        }
        if (ok) pushed_ok++;
        else    push_drops++;
    }
    // Wait for the decoder task to drain. 100 ms × 30 = up to 3 s.
    for (int i = 0; i < 30 && frame_decoder_queue_count() > 0; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    frame_decoder_get_class_counts(&after);
    uint64_t got_unknown  = after.unknown  - before.unknown;
    uint64_t got_ms       = after.ms       - before.ms;
    uint64_t got_tl       = after.tl       - before.tl;
    uint64_t got_bc       = after.bc       - before.bc;
    uint64_t got_lw_da    = after.lw_da    - before.lw_da;
    uint64_t got_lw_other = after.lw_other - before.lw_other;
    uint64_t got_total    = got_unknown + got_ms + got_tl + got_bc
                          + got_lw_da + got_lw_other;

    ESP_LOGI(TAG, "Pushed: %d ok / %d dropped (corpus size %u)",
             pushed_ok, push_drops, ALBQ_FRAME_CORPUS_LEN);
    ESP_LOGI(TAG, "Decoder counts: UNKNOWN=%llu MS=%llu TL=%llu BC=%llu "
             "LW.DA=%llu LW.other=%llu (total processed=%llu)",
             (unsigned long long)got_unknown, (unsigned long long)got_ms,
             (unsigned long long)got_tl,      (unsigned long long)got_bc,
             (unsigned long long)got_lw_da,   (unsigned long long)got_lw_other,
             (unsigned long long)got_total);

    bool pass = true;
    if (push_drops > 0) {
        ESP_LOGE(TAG, "  %d push drops — queue too small or decoder too slow",
                 push_drops);
        pass = false;
    }
    if (got_total != (uint64_t)pushed_ok) {
        ESP_LOGE(TAG, "  decoder consumed %llu vs %d pushed",
                 (unsigned long long)got_total, pushed_ok);
        pass = false;
    }
    // Reference numbers from host test_iridium_frame_corpus on the
    // 82-entry Albuquerque corpus: 1 TL + 11 BC + 6 LW.DA + 55 LW.other
    // (the LW count breaks down by ft); 9 UNKNOWN. Allow ±2 slack for
    // any classifier-tuning drift between host (gcc) and target (riscv32).
    const int EXP_TL       = 1;
    const int EXP_BC       = 11;
    const int EXP_LW_TOTAL = 61;   // matches host: 14 BC + 61 LW + 1 TL etc.
    if ((int)got_tl < EXP_TL - 2 || (int)got_tl > EXP_TL + 2) {
        ESP_LOGE(TAG, "  TL count %llu out of range [%d..%d]",
                 (unsigned long long)got_tl, EXP_TL - 2, EXP_TL + 2);
        pass = false;
    }
    if ((int)got_bc < EXP_BC - 2 || (int)got_bc > EXP_BC + 2) {
        ESP_LOGE(TAG, "  BC count %llu out of range [%d..%d]",
                 (unsigned long long)got_bc, EXP_BC - 2, EXP_BC + 2);
        pass = false;
    }
    int lw_total = (int)(got_lw_da + got_lw_other);
    if (lw_total < EXP_LW_TOTAL - 2 || lw_total > EXP_LW_TOTAL + 2) {
        ESP_LOGE(TAG, "  LW total %d out of range [%d..%d]",
                 lw_total, EXP_LW_TOTAL - 2, EXP_LW_TOTAL + 2);
        pass = false;
    }
    if (pass) ESP_LOGI(TAG, "===== SMOKE_PASS =====");
    else      ESP_LOGE(TAG, "===== SMOKE_FAIL =====");

    // Park here — smoke task is supposed to never return.
    // (The frame_decoder task on Core 1 stays running so we can keep
    // observing its log lines during manual debug.)
    vTaskSuspend(NULL);
}
#endif

void smoke_test_run(void)
{
#if CONFIG_SMOKE_TEST_FRAME_DECODER
    smoke_test_run_frame_decoder();
    // Should not return; if smoke_test_run_frame_decoder ever does,
    // park here so we don't fall off the task.
    vTaskSuspend(NULL);
    return;
#endif

    ESP_LOGI(TAG, "=== Smoke test start ===");
    ESP_LOGI(TAG, "Injecting tone at FFT bin %d (post-shift bin %d), "
             "expecting detection in [%d..%d]",
             TONE_FFT_BIN, (TONE_FFT_BIN + FFT_SIZE / 2) % FFT_SIZE,
             EXPECTED_BIN_LO, EXPECTED_BIN_HI);

    // Bring up the production DSP path. Order matches action_start_stream
    // in class_driver.c.
    if (signal_buffer_init() != ESP_OK) {
        ESP_LOGE(TAG, "signal_buffer_init failed -> SMOKE_FAIL");
        return;
    }
#if CONFIG_SMOKE_TEST_REAL_IRIDIUM || CONFIG_SMOKE_TEST_RAW_IRIDIUM
    // Full-stack mode: bring up the worker chain (qpsk_demod + BCH +
    // legacy MS decode) and the frame_decoder task so detected bursts
    // get classified, not just counted.
    worker_core1_init();
    bch_decoder_init();
    if (frame_decoder_init() != ESP_OK) {
        ESP_LOGE(TAG, "frame_decoder_init failed -> SMOKE_FAIL");
        return;
    }
    if (dsp_processor_init(on_burst_full_chain) != ESP_OK) {
        ESP_LOGE(TAG, "dsp_processor_init failed -> SMOKE_FAIL");
        return;
    }
#else
    if (dsp_processor_init(on_burst) != ESP_OK) {
        ESP_LOGE(TAG, "dsp_processor_init failed -> SMOKE_FAIL");
        return;
    }
#endif
    if (ingest_core1_init() != ESP_OK) {
        ESP_LOGE(TAG, "ingest_core1_init failed -> SMOKE_FAIL");
        return;
    }

    // Stack-borrowed scratch is too small for 16 KB; use a static buffer.
    static uint8_t synth[TRANSFER_BYTES] __attribute__((aligned(64)));

    int prev_slot = -1;

    ESP_LOGI(TAG, "Phase 1: %d priming noise transfers (let baseline settle)",
             PRIMING_TRANSFERS);
    for (int i = 0; i < PRIMING_TRANSFERS; i++) {
        fill_noise(synth);
        prev_slot = drive_transfer(synth, prev_slot);
        vTaskDelay(1);  // let ingest task make progress
    }

#if CONFIG_SMOKE_TEST_CORPUS
    // Real-signal regression mode: replace the synthetic tone with a
    // resampled+quantised slice of test_corpus/prbs15-2M-20dB.sigmf-data
    // (one Iridium burst at 2.56 MSPS, uint8 IQ). Asserts the detector
    // fires on the corpus burst's carrier (near DC bin since the corpus
    // is centred at the synthetic carrier frequency).
    ESP_LOGI(TAG, "Phase 2 (corpus): 1 corpus-fixture transfer (%u bytes)",
             CORPUS_UINT8_LEN);
    if (CORPUS_UINT8_LEN >= TRANSFER_BYTES) {
        memcpy(synth, CORPUS_UINT8, TRANSFER_BYTES);
    } else {
        memcpy(synth, CORPUS_UINT8, CORPUS_UINT8_LEN);
        // pad with mid-scale silence
        memset(synth + CORPUS_UINT8_LEN, 128, TRANSFER_BYTES - CORPUS_UINT8_LEN);
    }
    prev_slot = drive_transfer(synth, prev_slot);
    vTaskDelay(1);
    // Repeat 2 more times so the burst's 9 ms duration spans enough FFT
    // frames (4 frames per 16 KB transfer × 3 = 12 frames covers ~10 ms).
    for (int i = 0; i < 2; i++) {
        prev_slot = drive_transfer(synth, prev_slot);
        vTaskDelay(1);
    }
#elif CONFIG_SMOKE_TEST_RAW_IRIDIUM
    // End-to-end raw-mode test: a single fixture representing what an
    // SDR tuned to ALBQ_RAW_LO_HZ would actually emit. Bursts at their
    // natural offsets in the 2.56 MHz subband, no per-burst pre-shift,
    // so worker_core1's peak_bin -> freq-centre chain can demod them.
    //
    // D7+: fixture is now ~1 sec at 2.56 MSPS (5 MB) so we have parity
    // with gr-iridium's fft_burst_tagger setup latency. gr-iridium
    // decodes ~32 IDA frames from this time window at 12 MSPS, of
    // which ~15 are in our subband — that's the decode target.
    const unsigned int n_xfers = ALBQ_RAW_UINT8_LEN / TRANSFER_BYTES;
    ESP_LOGI(TAG, "Phase 2 (raw-mode @ %u Hz): %u× %u-byte transfers "
                  "(~%u ms, %u expected bursts in subband)",
             ALBQ_RAW_LO_HZ, n_xfers, TRANSFER_BYTES,
             (n_xfers * TRANSFER_BYTES) / (2u * 2560u),  // ms at 2.56 MSPS
             ALBQ_RAW_EXPECTED_BURSTS);
    for (unsigned int t = 0; t < n_xfers; t++) {
        unsigned int off = t * TRANSFER_BYTES;
        memcpy(synth, ALBQ_RAW_UINT8 + off, TRANSFER_BYTES);
        prev_slot = drive_transfer(synth, prev_slot);
        vTaskDelay(1);
    }
#elif CONFIG_SMOKE_TEST_REAL_IRIDIUM
    // Multi-stripe hardware-in-the-loop test: 8 stripes covering 1615.7-
    // 1627.2 MHz at 50% overlap, each fixture 49 KB = 9.6 ms of resampled
    // 2.56 MSPS uint8 IQ. Per stripe: PRIMING_TRANSFERS noise → 3 stripe
    // transfers → TRAILER_TRANSFERS noise (let any active burst end and
    // baseline EMA re-settle for the next stripe).
    int per_stripe_bursts[ALBQ_NUM_STRIPES] = { 0 };
    int total_bursts_after = 0;
    for (int sidx = 0; sidx < ALBQ_NUM_STRIPES; sidx++) {
        const albq_stripe_t *st = &ALBQ_STRIPES[sidx];
        int bursts_before = s_bursts_detected;

        // Don't repeat priming for stripe 0 — it ran before this loop.
        if (sidx > 0) {
            for (int i = 0; i < PRIMING_TRANSFERS; i++) {
                fill_noise(synth);
                prev_slot = drive_transfer(synth, prev_slot);
                vTaskDelay(1);
            }
        }

        ESP_LOGI(TAG, "Stripe %d/%d (center %.3f MHz, %d expected bursts): "
                      "3× %u-byte transfers",
                 sidx, ALBQ_NUM_STRIPES, st->center_hz / 1e6,
                 st->expected_bursts, TRANSFER_BYTES);
        for (int t = 0; t < 3; t++) {
            unsigned int off = t * TRANSFER_BYTES;
            if (off + TRANSFER_BYTES <= st->len) {
                memcpy(synth, st->data + off, TRANSFER_BYTES);
            } else {
                memset(synth, 128, TRANSFER_BYTES);
            }
            prev_slot = drive_transfer(synth, prev_slot);
            vTaskDelay(1);
        }

        // Trailing noise so the active burst (if any) ends and gets
        // counted before the next stripe's priming begins.
        for (int i = 0; i < TRAILER_TRANSFERS; i++) {
            fill_noise(synth);
            prev_slot = drive_transfer(synth, prev_slot);
            vTaskDelay(1);
        }

        per_stripe_bursts[sidx] = s_bursts_detected - bursts_before;
        ESP_LOGI(TAG, "  stripe %d: %d bursts detected (expected %d)",
                 sidx, per_stripe_bursts[sidx], st->expected_bursts);
        total_bursts_after = s_bursts_detected;
    }
#else
    ESP_LOGI(TAG, "Phase 2: %d tone transfers (drive the burst)", TONE_TRANSFERS);
    for (int i = 0; i < TONE_TRANSFERS; i++) {
        fill_tone(synth, TONE_FFT_BIN);
        prev_slot = drive_transfer(synth, prev_slot);
        vTaskDelay(1);
    }
#endif

#if !CONFIG_SMOKE_TEST_REAL_IRIDIUM
    // Multi-stripe mode handles its own per-stripe trailing inside the
    // stripe loop above; the other modes still need a post-burst trailer.
    ESP_LOGI(TAG, "Phase 3: %d trailing noise transfers (terminate burst)",
             TRAILER_TRANSFERS);
    for (int i = 0; i < TRAILER_TRANSFERS; i++) {
        fill_noise(synth);
        prev_slot = drive_transfer(synth, prev_slot);
        vTaskDelay(1);
    }
#endif

    // Drain the last in-flight slot so its DSP feed runs.
    if (prev_slot >= 0) {
        size_t n_int16 = 0;
        int16_t *converted = ingest_core1_take_converted(prev_slot, &n_int16);
        dsp_processor_feed(converted, n_int16 / 2);
        ingest_core1_release(prev_slot);
    }

    // Give the on_burst callback a moment in case the burst-end frame is
    // still being processed when we get here.
    vTaskDelay(pdMS_TO_TICKS(100));

    int bursts = s_bursts_detected;
    int peak_bin = s_strongest_peak_bin;
    float snr_db = s_strongest_snr_db;

    ESP_LOGI(TAG, "Result: bursts=%d strongest peak_bin=%d snr=%.2f dB",
             bursts, peak_bin, snr_db);

    // Assertions vary by mode:
    //   Synthetic tone:    bursts ≥1, peak_bin in [1200..1248], SNR > 30 dB.
    //   Synthetic corpus:  bursts ≥1, peak_bin near DC (corpus is at SDR LO).
    //   Real-RF Albq:      bursts ≥1, peak_bin near DC (we shifted the
    //                      1625.27 MHz channel to baseband). SNR ≥ 10 dB —
    //                      lower than the host-test 30 dB because the FFT
    //                      detector measures wideband SNR and the burst
    //                      only fills part of the 2.56 MHz subband.
    bool pass = true;
    if (bursts < 1) {
        ESP_LOGE(TAG, "  no bursts detected (expected ≥1)");
        pass = false;
    }
#if CONFIG_SMOKE_TEST_RAW_IRIDIUM
    // Raw-mode end-to-end assertion: the worker chain should demod
    // at least one burst (since bursts are at their natural offsets
    // in the 2.56 MHz subband, peak_bin is correct for freq centring).
    // Wait for queues to drain, then check frame_decoder counts.
    ESP_LOGI(TAG, "Waiting up to 2 s for worker + frame_decoder to drain...");
    for (int i = 0; i < 20; i++) {
        if (frame_decoder_queue_count() == 0) break;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    frame_decoder_class_counts_t fc;
    frame_decoder_get_class_counts(&fc);
    uint64_t total_classified = fc.unknown + fc.ms + fc.tl + fc.bc
                              + fc.lw_da + fc.lw_other;
    ESP_LOGI(TAG, "Frame-decoder counts: UNKNOWN=%llu MS=%llu TL=%llu BC=%llu "
             "LW.DA=%llu LW.other=%llu (total=%llu, expected=%d)",
             (unsigned long long)fc.unknown, (unsigned long long)fc.ms,
             (unsigned long long)fc.tl,      (unsigned long long)fc.bc,
             (unsigned long long)fc.lw_da,   (unsigned long long)fc.lw_other,
             (unsigned long long)total_classified, ALBQ_RAW_EXPECTED_BURSTS);
    if (total_classified < 1) {
        ESP_LOGE(TAG, "  no bursts reached the classifier — worker chain broken");
        pass = false;
    }
    if (snr_db < 10.0f) {
        ESP_LOGE(TAG, "  strongest burst SNR %.2f dB lower than expected (≥10 dB)",
                 snr_db);
        pass = false;
    }
#elif CONFIG_SMOKE_TEST_CORPUS
    const int CORPUS_BIN_LO = 1014;
    const int CORPUS_BIN_HI = 1034;
    if (peak_bin < CORPUS_BIN_LO || peak_bin > CORPUS_BIN_HI) {
        ESP_LOGE(TAG, "  strongest peak_bin %d outside corpus DC window [%d..%d]",
                 peak_bin, CORPUS_BIN_LO, CORPUS_BIN_HI);
        pass = false;
    }
    if (snr_db < 6.0f) {
        ESP_LOGE(TAG, "  corpus SNR %.2f dB lower than expected (>6 dB)", snr_db);
        pass = false;
    }
#elif CONFIG_SMOKE_TEST_REAL_IRIDIUM
    // Multi-stripe assertion. Cumulative count across all 8 stripes;
    // we expect ~16 bursts total (with 50% stripe overlap double-
    // counting some). Threshold of 5 catches a real regression while
    // tolerating that some short / low-SNR bursts may not cross the
    // detector threshold inside the 9.6 ms window.
    ESP_LOGI(TAG, "Per-stripe summary:");
    int stripes_with_bursts = 0;
    for (int s = 0; s < ALBQ_NUM_STRIPES; s++) {
        ESP_LOGI(TAG, "  stripe %d (%.3f MHz): detected=%d expected=%d",
                 s, ALBQ_STRIPES[s].center_hz / 1e6,
                 per_stripe_bursts[s], ALBQ_STRIPES[s].expected_bursts);
        if (per_stripe_bursts[s] > 0) stripes_with_bursts++;
    }
    const int ALBQ_MIN_TOTAL_BURSTS = 5;
    if (bursts < ALBQ_MIN_TOTAL_BURSTS) {
        ESP_LOGE(TAG, "  total %d bursts is below threshold %d "
                 "(across %d/%d stripes)",
                 bursts, ALBQ_MIN_TOTAL_BURSTS,
                 stripes_with_bursts, ALBQ_NUM_STRIPES);
        pass = false;
    }
    if (stripes_with_bursts < 3) {
        ESP_LOGE(TAG, "  only %d stripes detected ≥1 burst (expected ≥3 "
                 "of the 5 stripes with non-zero expected bursts)",
                 stripes_with_bursts);
        pass = false;
    }
    // Edge-bin reject still applies to the strongest match.
    const int ALBQ_BIN_EDGE_REJECT = 64;
    if (peak_bin < ALBQ_BIN_EDGE_REJECT ||
        peak_bin > FFT_SIZE - ALBQ_BIN_EDGE_REJECT) {
        ESP_LOGE(TAG, "  strongest peak_bin %d in edge-reject window "
                 "(likely DC/Nyquist artefact, not a real burst)", peak_bin);
        pass = false;
    }
    if (snr_db < 10.0f) {
        ESP_LOGE(TAG, "  strongest burst SNR %.2f dB lower than expected (≥10 dB)",
                 snr_db);
        pass = false;
    }

    // Full-stack post-check: wait for worker + frame_decoder to drain
    // any queued bursts, then report what got classified.
    //
    // ARCHITECTURE NOTE (why no hard assertion on classifier counts):
    // The 8-stripe IQ fixture is FREQUENCY-SHIFTED at fixture-build
    // time so each stripe puts its 2.56 MHz subband at baseband DC.
    // That's the right shape for the FFT detector test (peak_bin
    // lands near DC). But the WORKER expects raw SDR IQ and does its
    // OWN freq-shift driven by detector-reported peak_bin to centre
    // each channel before demod. Feeding a pre-shifted fixture
    // through the worker means it shifts the wrong amount and
    // demod fails. So qpsk_demod returns false, frame_decoder_push
    // is never called, and classifier counts stay 0.
    //
    // True end-to-end IQ -> ACARS smoke tests need raw (unshifted)
    // 2.56 MSPS IQ at the SDR LO frequency. We don't have a fixture
    // for that yet — comes with the antenna in Phase 4. Until then,
    // use CONFIG_SMOKE_TEST_FRAME_DECODER for upper-chain regression
    // (post-demod bits -> classifier directly).
    ESP_LOGI(TAG, "Waiting up to 2 s for worker + frame_decoder to drain...");
    for (int i = 0; i < 20; i++) {
        if (frame_decoder_queue_count() == 0) break;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    frame_decoder_class_counts_t fc;
    frame_decoder_get_class_counts(&fc);
    ESP_LOGI(TAG, "Frame-decoder counts: UNKNOWN=%llu MS=%llu TL=%llu BC=%llu "
             "LW.DA=%llu LW.other=%llu",
             (unsigned long long)fc.unknown, (unsigned long long)fc.ms,
             (unsigned long long)fc.tl,      (unsigned long long)fc.bc,
             (unsigned long long)fc.lw_da,   (unsigned long long)fc.lw_other);
    ESP_LOGI(TAG, "Frame-decoder queue: pushed=%llu popped=%llu dropped=%llu",
             (unsigned long long)frame_decoder_pushed(),
             (unsigned long long)frame_decoder_popped(),
             (unsigned long long)frame_decoder_dropped());
    // No hard assertion — see ARCHITECTURE NOTE above.
#else
    if (peak_bin < EXPECTED_BIN_LO || peak_bin > EXPECTED_BIN_HI) {
        ESP_LOGE(TAG, "  strongest peak_bin %d outside expected window [%d..%d]",
                 peak_bin, EXPECTED_BIN_LO, EXPECTED_BIN_HI);
        pass = false;
    }
    if (snr_db < 30.0f) {
        ESP_LOGE(TAG, "  SNR %.2f dB lower than expected (>30 dB) — could be"
                 " a noise-floor spike rather than the tone", snr_db);
        pass = false;
    }
#endif

    // ----------- Performance regression assertions -----------
    //
    // Catch silent regressions that change measured timings without
    // breaking detection (e.g., an accidental -Og rebuild, a cache-miss
    // pessimisation, a new buffer landing in PSRAM). Bars are set ~25%
    // above the latest measured baseline so normal compiler-version
    // jitter doesn't trip them, but a regression that costs us back
    // any of the Step 6/7 wins will fail the test.
    //
    // Baselines (synthetic-tone smoke, ESP-IDF v6.1, ESP32-P4 @360 MHz):
    //   Step 7 (-O2 + per-file -O3 hot files):
    //     wind=92  fft=199  mag=74  detect=58  base=146  total=570
    //     convert=191  push=86 (us)
    //   Step 3a (+ PIE Q15 windowing kernel dsp_window_arp4.S):
    //     wind=16  fft=199  mag=74  detect=58  base=146  total=493
    //   Step 7a (+ linear-write magnitude, no fftshift in mag loop):
    //     wind=15  fft=200  mag=51  detect=47  base=109  total=422
    //   Step 7b (+ eradicate floats — uint32 mag/baseline/threshold):
    //     wind=15  fft=200  mag=39  detect=33  base=119  total=407
    //   Step 7c attempted PIE int magnitude — research dead-end on
    //     ESP32-P4 PIE. Three different recipes tried, none beats
    //     scalar; full findings in dsp_mag_arp4.S. Reverted to scalar.
    //
    // If you intentionally optimise something further, lower the bar
    // (don't just raise it). If you intentionally regress for a feature
    // (e.g., adding a stage), update the comment + the bar together.
    dsp_stage_stats_t dsp_st;
    dsp_processor_get_stage_stats(&dsp_st);
    ingest_stats_t ing_st;
    ingest_core1_get_stats(&ing_st);

    ESP_LOGI(TAG, "Perf check (averaged over %lu DSP frames):", dsp_st.frames);
    ESP_LOGI(TAG, "  DSP/frame: total=%.0f wind=%.0f fft=%.0f mag=%.0f "
             "detect=%.0f base=%.0f us",
             dsp_st.total_us, dsp_st.wind_us, dsp_st.fft_us,
             dsp_st.mag_us, dsp_st.detect_us, dsp_st.baseline_us);
    if (ing_st.dispatches > 0) {
        ESP_LOGI(TAG, "  Ingest/dispatch: convert=%llu push=%llu us",
                 (unsigned long long)(ing_st.convert_us_total / ing_st.dispatches),
                 (unsigned long long)(ing_st.push_us_total / ing_st.dispatches));
    }

    // Bars are loose because the synthetic random-noise priming
    // sometimes triggers extra burst false positives, each adding
    // ~50 us of ESP_LOGI to the EMA-stage timing window. The
    // medians stay around the documented per-step baselines.
    //
    // In CONFIG_SMOKE_TEST_REAL_IRIDIUM mode the burst-callback rate is
    // ~10× higher (real RF + 8 stripes catches dozens of bursts per
    // run), so the per-frame ESP_LOGI overhead dominates and the
    // averages aren't representative of production. Skip perf
    // assertions in that mode — the other smoke modes still cover
    // the DSP-perf regression purpose.
#if !CONFIG_SMOKE_TEST_REAL_IRIDIUM
    struct { const char *name; float actual; float bar; } checks[] = {
        { "DSP total/frame",   dsp_st.total_us,    900.0f },  // Step 7b baseline 407, p99 ~750
        { "DSP wind/frame",    dsp_st.wind_us,      25.0f },  // Step 3a baseline 16 (PIE)
        { "DSP fft/frame",     dsp_st.fft_us,      280.0f },
        { "DSP mag/frame",     dsp_st.mag_us,       60.0f },  // Step 7b baseline 39
        { "DSP detect/frame",  dsp_st.detect_us,    50.0f },  // Step 7b baseline 33 (int)
        { "DSP base/frame",    dsp_st.baseline_us, 500.0f },  // Step 7b 119, but bursts inflate to ~450
    };
    for (size_t i = 0; i < sizeof(checks) / sizeof(checks[0]); i++) {
        if (checks[i].actual > checks[i].bar) {
            ESP_LOGE(TAG, "  PERF REGRESSION: %s = %.0f us > bar %.0f us",
                     checks[i].name, checks[i].actual, checks[i].bar);
            pass = false;
        }
    }
#endif
#if !CONFIG_SMOKE_TEST_REAL_IRIDIUM
    if (ing_st.dispatches > 0) {
        float convert_avg = (float)ing_st.convert_us_total / ing_st.dispatches;
        float push_avg    = (float)ing_st.push_us_total    / ing_st.dispatches;
        if (convert_avg > 280.0f) {
            ESP_LOGE(TAG, "  PERF REGRESSION: convert/dispatch %.0f us > bar 280 us",
                     convert_avg);
            pass = false;
        }
        if (push_avg > 130.0f) {
            ESP_LOGE(TAG, "  PERF REGRESSION: push/dispatch %.0f us > bar 130 us",
                     push_avg);
            pass = false;
        }
    }
#endif

    if (pass) {
        ESP_LOGI(TAG, "===== SMOKE_PASS =====");
    } else {
        ESP_LOGE(TAG, "===== SMOKE_FAIL =====");
    }

    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
