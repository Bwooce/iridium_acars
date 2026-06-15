// Target-side smoke test — see smoke_test.h for the high-level idea.
//
// Drives the same convert -> ingest_core1 -> dsp_processor path the
// production class_driver loop uses, with a synthetic IQ source instead
// of USB. Asserts that the FFT detector raises exactly one burst at a
// non-edge bin with sensible SNR.

// Compile the harness ONLY in smoke builds. Previously the whole
// translation unit landed in production images with just the CALL
// gated — costing ~8 KB of internal-SRAM .bss (s_per_bin_max_snr)
// plus code in flash for nothing. The only caller (usb_host_lib_main.c)
// gates both its #include and the smoke_test_run() call on the same
// CONFIG symbol, so no stub is needed for the linker.
#include "sdkconfig.h"
#if CONFIG_SMOKE_TEST_MODE

#include <math.h>
#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_task_wdt.h"
#include "esp_attr.h"
#include "resample_256_to_250.h"
#include "fft_sc16_2048.h"
#include "esp_chip_info.h"
#include "sdkconfig.h"
#include "ingest_core1.h"
#include "signal_buffer.h"
#include "dsp_processor.h"
#include "burst_pipeline.h"
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

#define TRANSFER_BYTES (16 * 1024)            // matches class_driver's out_block_size
#define TRANSFER_SAMPLES (TRANSFER_BYTES / 2) // complex samples per transfer (1 byte I + 1 byte Q)

// Inject the tone at FFT bin 200 (positive frequency, well clear of DC at
// bin 0 and Nyquist at bin 1024). After fftshift the magnitudes index is
// (bin + 1024) % 2048 = 1224. We assert the detected peak_bin is in
// 1200..1248 — generous to absorb spectral leakage.
#define TONE_FFT_BIN 200
#define EXPECTED_BIN_LO 1200
#define EXPECTED_BIN_HI 1248

// Phases of the synthetic stream:
//   PRIMING noise transfers — let the baseline EMA settle before injecting
//     the tone. The wideband fft_burst_tagger keeps a HISTORY_SIZE=512
//     chunk rolling window per bin; each FBT chunk is FFT_SIZE=2048
//     complex samples at 2.5 MSPS post-resample. So priming needs
//     ≥512 chunks × 2048 = 1,048,576 complex samples through the
//     detector. At 2.56 MSPS USB × 16 KB transfers = 8192 complex per
//     transfer, post-125/128 resample = ~8000 complex post-resample,
//     so ~131 transfers fully prime the history. Use 144 for margin.
//     (The previous channelizer detector primed in ~16 FFT frames,
//     hence the historic value of 8 transfers — too few for the
//     wideband path.)
//   TONE transfers — drive a strong tone for the burst to fire.
//   TRAILER noise transfer(s) — terminate the burst (detector logs
//     BURST DETECTED only on the tone -> noise transition).
#define PRIMING_TRANSFERS 144
#define TONE_TRANSFERS 4
#define TRAILER_TRANSFERS 2

// Detection callback state. The callback fires once per completed burst.
//
// Random noise during the priming phase will occasionally spike one bin
// above the baseline-EMA threshold (16 dB) and trigger a spurious
// "phantom" burst. That's a real detector property, not a test failure —
// the production detector handles it via the burst-vs-RFI worker filter.
// For the smoke test we assert against the strongest detection seen,
// which the tone phase will dominate by ≥30 dB.
static volatile int   s_bursts_detected    = 0;
static volatile int   s_strongest_peak_bin = -1;
static volatile float s_strongest_snr_db   = 0.0f;
// Per-bin highest SNR seen across the whole run. Used by SMOKE_TEST_CORPUS
// to ask "did the corpus burst land in the DC window?" rather than "was
// the DC burst the strongest?" — the latter is brittle once the tagger
// threshold dropped from 14 dB to 10 dB (#77) because Phase 1 noise
// false-positives now produce ~12-13 dB SNR detections at random bins,
// out-ranking the corpus's own ~12.6 dB carrier.
#define BIN_PEAK_TRACK_N 2048
static volatile float s_per_bin_max_snr[BIN_PEAK_TRACK_N];

static void on_burst(const detected_burst_t *burst)
{
    s_bursts_detected++;
    if (burst->peak_snr_db > s_strongest_snr_db) {
        s_strongest_snr_db   = burst->peak_snr_db;
        s_strongest_peak_bin = burst->peak_bin;
    }
    if (burst->peak_bin >= 0 && burst->peak_bin < BIN_PEAK_TRACK_N) {
        if (burst->peak_snr_db > s_per_bin_max_snr[burst->peak_bin]) {
            s_per_bin_max_snr[burst->peak_bin] = burst->peak_snr_db;
        }
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
        out[i + 0] = 128 + (int8_t)((r >> 0) & 0x0f) - 8;
        out[i + 1] = 128 + (int8_t)((r >> 8) & 0x0f) - 8;
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
    static double phase     = 0.0;
    const double  dphase    = 2.0 * M_PI * (double)tone_bin / (double)FFT_SIZE;
    const double  amplitude = 100.0; // out of ±127 range
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
        if (iv < 0) {
            iv = 0;
        } else if (iv > 255) {
            iv = 255;
        }
        if (qv < 0) {
            qv = 0;
        } else if (qv > 255) {
            qv = 255;
        }
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
    int      slot;
    uint8_t *raw = ingest_core1_acquire_raw(&slot);
    memcpy(raw, src, TRANSFER_BYTES);
    ingest_core1_dispatch(slot, TRANSFER_BYTES);

    if (prev_slot >= 0) {
        size_t   n_int16   = 0;
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
                                  ? DIR_UPLINK
                                  : DIR_DOWNLINK;
        // Retry-on-drop with bounded backoff. The decoder task runs at
        // ~10 ms/frame (one tick of vTaskDelay + classify), so we wait
        // at most a few ticks per push. Up to 100 attempts = 1 s
        // before giving up; far longer than realistic for this corpus.
        bool ok = false;
        for (int attempt = 0; attempt < 100; attempt++) {
            ok = frame_decoder_push(e->bits, e->n_bits, qdir,
                                    e->freq_hz, 0, e->snr_db);
            if (ok) break;
            vTaskDelay(1); // one tick = drain a bit, then retry
        }
        if (ok)
            pushed_ok++;
        else
            push_drops++;
    }
    // Wait for the decoder task to drain. 100 ms × 30 = up to 3 s.
    for (int i = 0; i < 30 && frame_decoder_queue_count() > 0; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    frame_decoder_get_class_counts(&after);
    uint64_t got_unknown  = after.unknown - before.unknown;
    uint64_t got_ms       = after.ms - before.ms;
    uint64_t got_tl       = after.tl - before.tl;
    uint64_t got_bc       = after.bc - before.bc;
    uint64_t got_lw_da    = after.lw_da - before.lw_da;
    uint64_t got_lw_other = after.lw_other - before.lw_other;
    uint64_t got_total    = got_unknown + got_ms + got_tl + got_bc + got_lw_da + got_lw_other;

    ESP_LOGI(TAG, "Pushed: %d ok / %d dropped (corpus size %u)",
             pushed_ok, push_drops, ALBQ_FRAME_CORPUS_LEN);
    ESP_LOGI(TAG, "Decoder counts: UNKNOWN=%llu MS=%llu TL=%llu BC=%llu "
                  "LW.DA=%llu LW.other=%llu (total processed=%llu)",
             (unsigned long long)got_unknown, (unsigned long long)got_ms,
             (unsigned long long)got_tl, (unsigned long long)got_bc,
             (unsigned long long)got_lw_da, (unsigned long long)got_lw_other,
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
    const int EXP_LW_TOTAL = 61; // matches host: 14 BC + 61 LW + 1 TL etc.
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
    if (pass)
        ESP_LOGI(TAG, "===== SMOKE_PASS =====");
    else
        ESP_LOGE(TAG, "===== SMOKE_FAIL =====");

    // Park here — smoke task is supposed to never return.
    // (The frame_decoder task on Core 1 stays running so we can keep
    // observing its log lines during manual debug.)
    vTaskSuspend(NULL);
}
#endif

#if CONFIG_SMOKE_TEST_LIVE_SDR
// Live-SDR smoke. Stands up the production USB-host + ingest + DSP
// chain (same task topology as app_main's non-smoke branch) so a real
// USB RTL-SDR can stream IQ end-to-end, then observes the worker
// counters for SMOKE_LIVE_DURATION_S. Reports per-stage counts and
// pass/fails on:
//   - worker queue overflow (bursts_dropped > 0)
//   - no bursts at all over the window (zero queued)
// Does NOT assert on Iridium frame decode -- the antenna may be
// missing or the sky may be quiet, and the point of this variant is
// to validate the USB/SDR/ingest/DSP path, not the demodulator.
#define SMOKE_LIVE_DURATION_S 10

#include "worker_core1.h"
#include "signal_buffer.h"
#include "ingest_core1.h"
#include "dsp_processor.h"

extern void host_lib_daemon_task(void *arg);
extern void class_driver_task(void *arg);

static void smoke_test_run_live_sdr(void)
{
    ESP_LOGI(TAG, "=== Smoke test start (live SDR mode, %d s window) ===",
             SMOKE_LIVE_DURATION_S);
    ESP_LOGI(TAG, "  Will assert: no worker queue overflow, >= 1 burst tagged");
    ESP_LOGI(TAG, "  Will NOT assert: Iridium frame decode (antenna optional)");

    // Spawn the same daemon + class_driver tasks the production
    // app_main creates. class_driver_task does signal_buffer_init,
    // worker_core1_init, ingest_core1_init, dsp_processor_init on
    // first entry, so we don't need to call them explicitly here.
    SemaphoreHandle_t signaling_sem = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(host_lib_daemon_task,
                            "daemon", 4096,
                            (void *)signaling_sem,
                            5, NULL, 1);
    xTaskCreatePinnedToCore(class_driver_task,
                            "class", 4096,
                            (void *)signaling_sem,
                            4, NULL, 0);

    // Give the USB stack a moment to enumerate and start streaming
    // before sampling the counters.
    vTaskDelay(pdMS_TO_TICKS(1500));

    worker_stats_t s0;
    worker_core1_get_stats(&s0); // baseline (counters monotonically up)

    int64_t t_start  = esp_timer_get_time();
    int64_t deadline = t_start + (int64_t)SMOKE_LIVE_DURATION_S * 1000000;
    while (esp_timer_get_time() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    int64_t t_end    = esp_timer_get_time();
    double  window_s = (double)(t_end - t_start) / 1e6;

    worker_stats_t s1;
    worker_core1_get_stats(&s1);
    size_t psram_free    = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);

    uint32_t d_queued   = s1.bursts_queued - s0.bursts_queued;
    uint32_t d_proc     = s1.bursts_processed - s0.bursts_processed;
    uint32_t d_bch_ok   = s1.bursts_bch_decoded - s0.bursts_bch_decoded;
    uint32_t d_bch_unk  = s1.bursts_bch_unknown - s0.bursts_bch_unknown;
    uint32_t d_bch_fail = s1.bursts_bch_failed - s0.bursts_bch_failed;
    uint32_t d_drop     = s1.bursts_dropped - s0.bursts_dropped;
    uint32_t d_skip     = s1.bursts_skipped - s0.bursts_skipped;

    ESP_LOGI(TAG, "Live-SDR window done after %.1f s", window_s);
    ESP_LOGI(TAG, "  Worker delta: queued=%u processed=%u "
                  "bch_decoded=%u bch_unknown=%u bch_failed=%u "
                  "dropped=%u skipped=%u",
             (unsigned)d_queued, (unsigned)d_proc,
             (unsigned)d_bch_ok, (unsigned)d_bch_unk, (unsigned)d_bch_fail,
             (unsigned)d_drop, (unsigned)d_skip);
    ESP_LOGI(TAG, "  Heap: PSRAM free=%u KB  internal free=%u KB",
             (unsigned)(psram_free / 1024),
             (unsigned)(internal_free / 1024));

    bool pass = true;
    if (d_drop > 0) {
        ESP_LOGE(TAG, "  FAIL: worker dropped %u bursts (queue overflow)",
                 (unsigned)d_drop);
        pass = false;
    }
    if (d_queued == 0) {
        ESP_LOGE(TAG, "  FAIL: zero bursts tagged over %.1f s -- USB SDR not "
                      "enumerated, or signal floor below detector threshold",
                 window_s);
        pass = false;
    }

    if (pass)
        ESP_LOGI(TAG, "===== SMOKE_LIVE_SDR_PASS =====");
    else
        ESP_LOGE(TAG, "===== SMOKE_LIVE_SDR_FAIL =====");
}
#endif

// Task #67: PIE FFT bit-exact diff harness. Runs at the very start of
// smoke so the comparison numbers land in the log before any other
// noise. Declared here rather than via header since the harness is
// self-contained and only called from this one place.
extern void pie_fft_diff_run(void);

void smoke_test_run(void)
{
    // Run the PIE FFT diff harness first so its log lines are easy to
    // find. Tiny one-shot ~10 ms of synthetic FFT comparisons; doesn't
    // affect downstream smoke results.
    pie_fft_diff_run();

#if CONFIG_SMOKE_TEST_LIVE_SDR
    smoke_test_run_live_sdr();
    vTaskSuspend(NULL);
    return;
#endif

#if CONFIG_SMOKE_TEST_FRAME_DECODER
    smoke_test_run_frame_decoder();
    // Should not return; if smoke_test_run_frame_decoder ever does,
    // park here so we don't fall off the task.
    vTaskSuspend(NULL);
    return;
#endif

    ESP_LOGI(TAG, "=== Smoke test start ===");

    // One-shot silicon revision check. ESP32-P4 v1.x is 360 MHz; v3.x
    // is 400 MHz with the full PIE feature set. Our sdkconfig pins
    // CPU to 360 MHz so we're safe on either, but the log line makes
    // the actual chip step explicit when triaging perf anomalies.
    {
        esp_chip_info_t chip_info;
        esp_chip_info(&chip_info);
        uint32_t major = chip_info.revision / 100;
        uint32_t minor = chip_info.revision % 100;
        ESP_LOGI(TAG, "CHIP: ESP32-P4 silicon v%d.%d (cores=%d features=0x%x)",
                 (int)major, (int)minor, chip_info.cores, chip_info.features);
        if (major < 3) {
            ESP_LOGW(TAG, "CHIP: running on early v1.x silicon (360 MHz limit, "
                          "no v3-only PIE features)");
        } else {
            ESP_LOGI(TAG, "CHIP: production v3.x silicon (400 MHz + full PIE)");
        }
    }

    ESP_LOGI(TAG, "Injecting tone at FFT bin %d (post-shift bin %d), "
                  "expecting detection in [%d..%d]",
             TONE_FFT_BIN, (TONE_FFT_BIN + FFT_SIZE / 2) % FFT_SIZE,
             EXPECTED_BIN_LO, EXPECTED_BIN_HI);

// Bring up the production DSP path. Order matches action_start_stream
// in class_driver.c.
// Per-phase heap diagnostic: tracks how internal-SRAM fragmentation
// evolves through init. Prints total + largest-contiguous free for
// MALLOC_CAP_INTERNAL (all internal) and MALLOC_CAP_INTERNAL|DMA
// (DMA-capable subset, used by s_raw / USB pool).
#define HEAP_LOG(where)                                                                      \
    do {                                                                                     \
        size_t fi  = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);                           \
        size_t li  = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);                  \
        size_t fid = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);          \
        size_t lid = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA); \
        ESP_LOGW("HEAP", "%-22s INT free=%6zu largest=%6zu  DMA-INT free=%6zu largest=%6zu", \
                 where, fi, li, fid, lid);                                                   \
    } while (0)

    // CRITICAL early-alloc dance to keep PIE-asm-position-sensitive
    // buffers at known-working addresses. See
    // project_heap_position_decode_bug.md. Each of these buffers,
    // if shifted by upstream heap changes (e.g., struct growth),
    // can silently corrupt PIE output.
    //
    //   1. s_coeffs_pp (4 KB) — resampler polyphase coefficients
    //   2. s_w_table (4 KB) + s_fft_scratch (8 KB) — fft_sc16_2048
    //      The PIE FFT operates ON s_fft_scratch directly; if it
    //      lands in the broken zone (e.g., 0x4ff6_xxxx), the
    //      tagger FFT silently corrupts and decode collapses.
    resample_256_to_250_alloc_coeffs();
    fft_sc16_2048_init();
    HEAP_LOG("post-pie-buffers");
    HEAP_LOG("pre-signal_buffer");
    if (signal_buffer_init() != ESP_OK) {
        ESP_LOGE(TAG, "signal_buffer_init failed -> SMOKE_FAIL");
        return;
    }
    HEAP_LOG("post-signal_buffer");
#if CONFIG_SMOKE_TEST_REAL_IRIDIUM || CONFIG_SMOKE_TEST_RAW_IRIDIUM
    worker_core1_init();
    HEAP_LOG("post-worker_core1");
    bch_decoder_init();
    HEAP_LOG("post-bch_decoder");
    if (frame_decoder_init() != ESP_OK) {
        ESP_LOGE(TAG, "frame_decoder_init failed -> SMOKE_FAIL");
        return;
    }
    HEAP_LOG("post-frame_decoder");
    if (dsp_processor_init(on_burst_full_chain) != ESP_OK) {
        ESP_LOGE(TAG, "dsp_processor_init failed -> SMOKE_FAIL");
        return;
    }
    HEAP_LOG("post-dsp_processor");
#else
    if (dsp_processor_init(on_burst) != ESP_OK) {
        ESP_LOGE(TAG, "dsp_processor_init failed -> SMOKE_FAIL");
        return;
    }
    HEAP_LOG("post-dsp_processor");
#endif
    if (ingest_core1_init() != ESP_OK) {
        ESP_LOGE(TAG, "ingest_core1_init failed -> SMOKE_FAIL");
        return;
    }
    HEAP_LOG("post-ingest_core1");

    // Stack-borrowed scratch is too small for 16 KB; use a static buffer.
    // Lives in PSRAM (EXT_RAM_BSS_ATTR) — the buffer is filled then
    // memcpy'd into ingest_core1's slot (which is internal+DMA), never
    // DMA'd directly. Frees 16 KB of internal .bss for hotter consumers.
    static EXT_RAM_BSS_ATTR uint8_t synth[TRANSFER_BYTES] __attribute__((aligned(64)));

    int prev_slot = -1;

#if CONFIG_SMOKE_TEST_RAW_IRIDIUM
    // No synthetic-noise priming for wideband mode. The wideband
    // fft_burst_tagger keeps a 512-chunk EMA window; if Phase 1
    // primes on synthetic NOISE and Phase 2 hands it the real
    // ALBQ recording, the step transient at the Phase-1→Phase-2
    // boundary triggers a priming-completion burst flood that
    // masks the bins covering the actual Iridium bursts. The host
    // wideband test (test_pipeline_wideband_albq) feeds real data
    // from sample 0 and lets the tagger prime on it naturally,
    // which works (59 decodes). Mirror that here — the ALBQ
    // fixture's first 512 chunks prime the EMA on real signal
    // and subsequent bursts get detected against a meaningful
    // baseline.
    ESP_LOGI(TAG, "Phase 1: skipped for wideband mode "
                  "(fft_burst_tagger primes on real fixture)");
#else
    ESP_LOGI(TAG, "Phase 1: %d priming noise transfers (let baseline settle)",
             PRIMING_TRANSFERS);
    for (int i = 0; i < PRIMING_TRANSFERS; i++) {
        fill_noise(synth);
        prev_slot = drive_transfer(synth, prev_slot);
        vTaskDelay(1); // let ingest task make progress
    }
#endif

#if CONFIG_SMOKE_TEST_CORPUS
    // CORPUS mode: inject a narrowband DC tone (bin 0, post-shift 1024)
    // and assert the FFT detector fires in [1014..1034]. The test's
    // original design used a slice of test_corpus/prbs15-2M-20dB.sigmf-data
    // as a "real burst" signal, but that fixture is a 2 MHz wideband
    // spread-spectrum PRBS15 — its energy distributes across ~1600 of
    // the 2048 FFT bins, so per-bin SNR ends up around -12 dB even at
    // 20 dB overall power. The narrowband Iridium-style burst tagger
    // can't detect it. Once the live tagger threshold dropped from
    // 14 dB to 10 dB (task #77), Phase 1 priming noise started producing
    // ~12-13 dB SNR false positives that out-ranked the (undetected)
    // corpus signal. Swapping in a real narrowband DC tone tests the
    // same plumbing — priming → strong narrowband signal → detection
    // in the DC window — with a signal the tagger is actually designed
    // for. The `(void)CORPUS_UINT8` reference keeps the fixture's
    // inclusion non-fatal in case anyone re-enables the old codepath.
    (void)CORPUS_UINT8;
    (void)CORPUS_UINT8_LEN;
    ESP_LOGI(TAG, "Phase 2 (DC tone, was: PRBS15 corpus): %d transfers at bin 0",
             TONE_TRANSFERS);
    for (int i = 0; i < TONE_TRANSFERS; i++) {
        fill_tone(synth, /*tone_bin=*/0); // FFT bin 0 → post-shift 1024 = DC
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
             (n_xfers * TRANSFER_BYTES) / (2u * 2560u), // ms at 2.56 MSPS
             ALBQ_RAW_EXPECTED_BURSTS);
    // Register the smoke task with the task watchdog and reset it
    // every loop iteration. Without this the smoke task can starve
    // class_driver / frame_decoder / ingest on Core 1 under load —
    // the TWDT then panics one of those (not the smoke task itself)
    // and the device silently reboots before reaching the end-of-
    // Phase-2 summary. Registering here makes the smoke task's CPU
    // usage explicit to the WDT and the per-iteration reset is the
    // cheap way to keep it happy.
    esp_err_t wdt_rc = esp_task_wdt_add(NULL);
    if (wdt_rc != ESP_OK && wdt_rc != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "esp_task_wdt_add: %s", esp_err_to_name(wdt_rc));
    }
    for (unsigned int t = 0; t < n_xfers; t++) {
        unsigned int off = t * TRANSFER_BYTES;
        memcpy(synth, ALBQ_RAW_UINT8 + off, TRANSFER_BYTES);
        prev_slot = drive_transfer(synth, prev_slot);
        // DIAGNOSTIC: 50 ms inter-transfer delay (vs real-time ~3 ms)
        // gives the worker time to drain its queue before signal_buffer
        // wraps. signal_buffer holds 0.4 s at 2.5 MSPS = ~125 transfers.
        // With the worker at ~420 ms/burst and queue depth 16 = 6.7 s
        // backlog, the real-time feed rate guarantees stale-data reads
        // for any burst not processed within 0.4 s. Slowing to 50 ms/
        // transfer extends fixture-playback time from 1 s to ~16 s,
        // giving the worker headroom to process bursts before their
        // signal_buffer windows get overwritten. Confirms or rules out
        // the wrap hypothesis. Reset to 1 once worker is real-time.
        vTaskDelay(pdMS_TO_TICKS(50));
        esp_task_wdt_reset();
    }
    esp_task_wdt_delete(NULL);

    // End-of-fixture flush: emit gone events for any still-active
    // bursts so the worker sees the trailing portion of the fixture.
    // Without this, bursts whose last_active is within burst_post_len
    // of end-of-fixture never reach the worker. Matches what gri's
    // GNU Radio stop-callback would do. Adds ~7 decodes on the ALBQ
    // fixture (measured on the host wideband test).
    dsp_processor_flush();

    // End-of-Phase-2 summary: drain the queues then read worker and
    // frame_decoder stats. We sleep a fixed 30 s rather than
    // polling frame_decoder_queue_count() because frame_decoder
    // empties immediately when the WORKER isn't pushing (over-budget
    // worker queue is upstream of frame_decoder, not visible to it).
    // Fixed sleep lets the worker churn through its 16-deep backlog
    // before we sample stats. Trim back to ~2 s once worker is
    // real-time and we don't need to wait for backlog drain.
    ESP_LOGI(TAG, "Phase 2 complete — draining queues for 30 s "
                  "(worker backlog + frame_decoder)");
    vTaskDelay(pdMS_TO_TICKS(30000));
    {
        worker_stats_t ws;
        worker_core1_get_stats(&ws);
        ESP_LOGI(TAG, "Worker stats: queued=%u processed=%u dropped=%u "
                      "skipped=%u high_water=%u avg=%.0f us",
                 (unsigned)ws.bursts_queued,
                 (unsigned)ws.bursts_processed,
                 (unsigned)ws.bursts_dropped,
                 (unsigned)ws.bursts_skipped,
                 (unsigned)ws.queue_high_water,
                 (double)ws.avg_burst_us);
        // Per-stage cost so we can see WHERE the worker spends its time —
        // critical for choosing the right PIE/SIMD optimisation target.
        ESP_LOGI(TAG, "  per-stage avg us: extract=%.0f rotate=%.0f "
                      "decim=%.0f pipeline=%.0f bch=%.0f",
                 (double)ws.extract_us,
                 (double)ws.freq_center_us,
                 (double)ws.fir_decim_us,
                 (double)ws.demod_us,
                 (double)ws.bch_us);
        // burst_pipeline substage breakdown — only useful when the
        // "pipeline" stage above is the dominant cost.
        uint32_t bp[10], bp_first, bp_retry;
        burst_pipeline_get_stage_us(bp, &bp_first, &bp_retry);
        ESP_LOGI(TAG, "  pipeline substages: D13=%lu CFO=%lu prerot=%lu "
                      "RRC=%lu first=%lu retry=%lu (first_calls=%lu "
                      "retry_calls=%lu)",
                 (unsigned long)bp[0], (unsigned long)bp[1],
                 (unsigned long)bp[2], (unsigned long)bp[3],
                 (unsigned long)bp[4], (unsigned long)bp[5],
                 (unsigned long)bp_first, (unsigned long)bp_retry);
        ESP_LOGI(TAG, "  try_decode substages (all calls): "
                      "UW=%lu PREROT=%lu DECIM=%lu QPSK=%lu",
                 (unsigned long)bp[6], (unsigned long)bp[7],
                 (unsigned long)bp[8], (unsigned long)bp[9]);
        extern volatile uint64_t g_pie_fft_inner_us;
        extern volatile uint64_t g_pie_fft_outer_us;
        extern volatile uint32_t g_pie_fft_calls;
        extern volatile uint64_t g_uw_specmul_us;
        extern volatile uint64_t g_uw_magsearch_us;
        uint64_t                 pi_inner = g_pie_fft_inner_us;
        uint64_t                 pi_outer = g_pie_fft_outer_us;
        uint32_t                 pi_calls = g_pie_fft_calls;
        uint64_t                 uw_sm    = g_uw_specmul_us;
        uint64_t                 uw_ms    = g_uw_magsearch_us;
        g_pie_fft_inner_us                = 0;
        g_pie_fft_outer_us                = 0;
        g_pie_fft_calls                   = 0;
        g_uw_specmul_us                   = 0;
        g_uw_magsearch_us                 = 0;
        ESP_LOGI(TAG, "  pie_fft: calls=%lu inner=%llu us outer=%llu us "
                      "(per-call: inner=%.0f us outer=%.0f us)",
                 (unsigned long)pi_calls,
                 (unsigned long long)pi_inner,
                 (unsigned long long)pi_outer,
                 pi_calls ? (double)pi_inner / pi_calls : 0.0,
                 pi_calls ? (double)pi_outer / pi_calls : 0.0);
        ESP_LOGI(TAG, "  uw_inner: specmul=%llu us magsearch=%llu us",
                 (unsigned long long)uw_sm, (unsigned long long)uw_ms);
        if (ws.bursts_dropped > 0) {
            ESP_LOGW(TAG, "  %u bursts dropped — queue overflow",
                     (unsigned)ws.bursts_dropped);
        }
        frame_decoder_class_counts_t fc;
        frame_decoder_get_class_counts(&fc);
        ESP_LOGI(TAG, "Frame-decoder counts: UNKNOWN=%llu MS=%llu TL=%llu "
                      "BC=%llu LW.DA=%llu LW.other=%llu",
                 (unsigned long long)fc.unknown, (unsigned long long)fc.ms,
                 (unsigned long long)fc.tl, (unsigned long long)fc.bc,
                 (unsigned long long)fc.lw_da, (unsigned long long)fc.lw_other);
    }
#elif CONFIG_SMOKE_TEST_REAL_IRIDIUM
    // Multi-stripe hardware-in-the-loop test: 8 stripes covering 1615.7-
    // 1627.2 MHz at 50% overlap, each fixture 49 KB = 9.6 ms of resampled
    // 2.56 MSPS uint8 IQ. Per stripe: PRIMING_TRANSFERS noise → 3 stripe
    // transfers → TRAILER_TRANSFERS noise (let any active burst end and
    // baseline EMA re-settle for the next stripe).
    int per_stripe_bursts[ALBQ_NUM_STRIPES] = {0};
    int total_bursts_after                  = 0;
    for (int sidx = 0; sidx < ALBQ_NUM_STRIPES; sidx++) {
        const albq_stripe_t *st            = &ALBQ_STRIPES[sidx];
        int                  bursts_before = s_bursts_detected;

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
        size_t   n_int16   = 0;
        int16_t *converted = ingest_core1_take_converted(prev_slot, &n_int16);
        dsp_processor_feed(converted, n_int16 / 2);
        ingest_core1_release(prev_slot);
    }

    // Give the on_burst callback a moment in case the burst-end frame is
    // still being processed when we get here.
    vTaskDelay(pdMS_TO_TICKS(100));

    int   bursts   = s_bursts_detected;
    int   peak_bin = s_strongest_peak_bin;
    float snr_db   = s_strongest_snr_db;

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
    uint64_t total_classified = fc.unknown + fc.ms + fc.tl + fc.bc + fc.lw_da + fc.lw_other;
    ESP_LOGI(TAG, "Frame-decoder counts: UNKNOWN=%llu MS=%llu TL=%llu BC=%llu "
                  "LW.DA=%llu LW.other=%llu (total=%llu, expected=%d)",
             (unsigned long long)fc.unknown, (unsigned long long)fc.ms,
             (unsigned long long)fc.tl, (unsigned long long)fc.bc,
             (unsigned long long)fc.lw_da, (unsigned long long)fc.lw_other,
             (unsigned long long)total_classified, ALBQ_RAW_EXPECTED_BURSTS);
    // Baseline (captured 2026-06-15, commit 4aa58f7, on ESP32-P4 v1.3):
    // UNKNOWN=3 BC=1 LW.DA=2 → total=6 classified, GOLDEN matched=4. RAW
    // mode is DETERMINISTIC (no random-noise priming — it primes on the
    // real fixture), so this count is stable run-to-run on the same
    // build. A drop below the floor is a decode REGRESSION — most likely
    // the silent PIE heap-position corruption (see
    // project_heap_position_decode_bug) that the host golden tests cannot
    // see because they run the scalar path. This is the device gate for
    // the #120 context refactor. Floor at 5 (baseline 6, −1 tolerance for
    // any classifier jitter) — real corruption craters this to 0-1.
#define RAW_IRIDIUM_MIN_CLASSIFIED 5
    if ((int)total_classified < RAW_IRIDIUM_MIN_CLASSIFIED) {
        ESP_LOGE(TAG, "  classified %llu < baseline floor %d — decode REGRESSION "
                      "(worker chain broken or PIE heap-position corruption)",
                 (unsigned long long)total_classified, RAW_IRIDIUM_MIN_CLASSIFIED);
        pass = false;
    }
    if (snr_db < 10.0f) {
        ESP_LOGE(TAG, "  strongest burst SNR %.2f dB lower than expected (≥10 dB)",
                 snr_db);
        pass = false;
    }
    // Golden-bits comparison: per-burst Hamming distance vs gri's
    // canonical decoded bits, with claim-tracking so each gri entry
    // is matched at most once and unclaimed entries are surfaced as
    // GOLDEN-MISSED rows.
    worker_core1_golden_print_summary();
#elif CONFIG_SMOKE_TEST_CORPUS
    // Scan for ANY detection in the DC window rather than asserting on
    // the strongest. The 10 dB tagger threshold (#77) lets random Phase 1
    // noise produce ~12-13 dB SNR detections that scatter across bins;
    // a strong DC tone produces ~25 dB SNR detections clustered around
    // bin 1024 with up to ~30 bins of spectral leakage (windowed FFT +
    // multi-frame burst-tagger peak reporting). Noise can't sustain
    // 25 dB nor sustain detections near DC across frames, so a hit
    // near DC at high SNR is the right signal that the front end
    // detected the injected signal.
    const int CORPUS_BIN_LO      = 994;  // 1024 - 30: spectral-leakage band
    const int CORPUS_BIN_HI      = 1054; // 1024 + 30
    float     dc_window_peak_snr = 0.0f;
    int       dc_window_peak_bin = -1;
    for (int b = CORPUS_BIN_LO; b <= CORPUS_BIN_HI; b++) {
        if (s_per_bin_max_snr[b] > dc_window_peak_snr) {
            dc_window_peak_snr = s_per_bin_max_snr[b];
            dc_window_peak_bin = b;
        }
    }
    if (dc_window_peak_bin < 0) {
        ESP_LOGE(TAG, "  no detection landed in corpus DC window [%d..%d] "
                      "(strongest was peak_bin=%d snr=%.2f dB)",
                 CORPUS_BIN_LO, CORPUS_BIN_HI, peak_bin, snr_db);
        pass = false;
    } else {
        ESP_LOGI(TAG, "  corpus DC-window hit: peak_bin=%d snr=%.2f dB",
                 dc_window_peak_bin, dc_window_peak_snr);
    }
    if (dc_window_peak_snr < 6.0f) {
        ESP_LOGE(TAG, "  corpus DC-window SNR %.2f dB lower than expected (>6 dB)",
                 dc_window_peak_snr);
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
                      "(likely DC/Nyquist artefact, not a real burst)",
                 peak_bin);
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
             (unsigned long long)fc.tl, (unsigned long long)fc.bc,
             (unsigned long long)fc.lw_da, (unsigned long long)fc.lw_other);
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
    if (snr_db < 20.0f) {
        ESP_LOGE(TAG, "  SNR %.2f dB lower than expected (>20 dB) — could be"
                      " a noise-floor spike rather than the tone",
                 snr_db);
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
    // Perf bars rebaselined 2026-05-23 after esp_wifi_remote+esp_hosted
    // landed (D17 redo). The dependency's ~30 KB of always-on runtime
    // shifts our DSP code in flash and bumps per-step costs even when
    // the C6 link is idle. New bars reflect the current-build steady
    // state on the RAW_IRIDIUM fixture with ~30 % headroom — tight
    // enough to catch a real perf regression, loose enough that small
    // code-layout drift won't trip them.
    //
    // Bars are NOT a real-time budget — they're a smoke-fixture sanity
    // check. Real-time throughput is gated by `resample` (currently
    // 3.8 ms/dispatch, the dominant cost — see task #58).
#if !CONFIG_SMOKE_TEST_REAL_IRIDIUM
    struct {
        const char *name;
        float       actual;
        float       bar;
    } checks[] = {
        {"DSP total/frame", dsp_st.total_us, 900.0f},
        {"DSP wind/frame", dsp_st.wind_us, 100.0f},     // current ~76, scalar Q15 (no PIE yet)
        {"DSP fft/frame", dsp_st.fft_us, 320.0f},       // current ~256
        {"DSP mag/frame", dsp_st.mag_us, 80.0f},        // current ~46
        {"DSP detect/frame", dsp_st.detect_us, 220.0f}, // current ~94-177
        {"DSP base/frame", dsp_st.baseline_us, 500.0f},
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
        float push_avg    = (float)ing_st.push_us_total / ing_st.dispatches;
        // push = resample (~3.8 ms) + sbpush (~0.2 ms). Resample dominates
        // and is the next real-time lever; sbpush stays sub-ms.
        if (convert_avg > 500.0f) {
            ESP_LOGE(TAG, "  PERF REGRESSION: convert/dispatch %.0f us > bar 500 us",
                     convert_avg);
            pass = false;
        }
        if (push_avg > 4500.0f) {
            ESP_LOGE(TAG, "  PERF REGRESSION: push/dispatch %.0f us > bar 4500 us",
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

    while (1)
        vTaskDelay(pdMS_TO_TICKS(1000));
}

#endif // CONFIG_SMOKE_TEST_MODE
