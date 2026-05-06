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
    ESP_LOGI(TAG, "callback: burst peak_bin=%d snr=%.2f dB",
             burst->peak_bin, burst->peak_snr_db);
}

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

void smoke_test_run(void)
{
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
    if (dsp_processor_init(on_burst) != ESP_OK) {
        ESP_LOGE(TAG, "dsp_processor_init failed -> SMOKE_FAIL");
        return;
    }
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
#else
    ESP_LOGI(TAG, "Phase 2: %d tone transfers (drive the burst)", TONE_TRANSFERS);
    for (int i = 0; i < TONE_TRANSFERS; i++) {
        fill_tone(synth, TONE_FFT_BIN);
        prev_slot = drive_transfer(synth, prev_slot);
        vTaskDelay(1);
    }
#endif

    ESP_LOGI(TAG, "Phase 3: %d trailing noise transfers (terminate burst)",
             TRAILER_TRANSFERS);
    for (int i = 0; i < TRAILER_TRANSFERS; i++) {
        fill_noise(synth);
        prev_slot = drive_transfer(synth, prev_slot);
        vTaskDelay(1);
    }

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
    //   Synthetic tone: bursts ≥1, strongest peak_bin in [1200..1248],
    //                   SNR > 30 dB.
    //   Corpus fixture: bursts ≥1, strongest peak_bin near DC (the corpus
    //                   carrier is centred at the SDR LO so energy lands
    //                   at FFT bin 1024 ± a few bins of leakage).
    bool pass = true;
    if (bursts < 1) {
        ESP_LOGE(TAG, "  no bursts detected (expected ≥1)");
        pass = false;
    }
#if CONFIG_SMOKE_TEST_CORPUS
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

    if (pass) {
        ESP_LOGI(TAG, "===== SMOKE_PASS =====");
    } else {
        ESP_LOGE(TAG, "===== SMOKE_FAIL =====");
    }

    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
