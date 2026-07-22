#include "scanner.h"
#include "scanner_map.h"
#include "band_health.h" // survey completion re-arms the re-survey trigger
#include "class_driver.h"
#include "app_config.h"
#include "worker_core1.h" // A6: hot_clear_all on LO retune
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h> // memcpy (publish accumulated survey map)

// Cold working buffers -> PSRAM to reclaim internal DMA-INT SRAM (dmaf).
// See docs/p4-bss-audit.md (DMA-INT reclaim, 2026-07-18).
#ifdef ESP_PLATFORM
#include "esp_attr.h"
#else
#ifndef EXT_RAM_BSS_ATTR
#define EXT_RAM_BSS_ATTR
#endif
#endif

static const char      *TAG   = "SCANNER";
static dsp_processor_t *s_dsp = NULL;
static EXT_RAM_BSS_ATTR scanner_pos_t    s_map[SCANNER_MAX_POSITIONS];
// Survey working buffers: one sweep's measurements + the running accumulation.
// Static (not stack) to match s_map's cold-PSRAM placement; a survey publishes
// s_acc into s_map only once all sweeps are done, so a concurrent `map` reprint
// mid-survey still shows the previous complete map.
static EXT_RAM_BSS_ATTR scanner_pos_t    s_sweep[SCANNER_MAX_POSITIONS];
static EXT_RAM_BSS_ATTR scanner_pos_t    s_acc[SCANNER_MAX_POSITIONS];
static int              s_map_n       = 0;
static uint32_t         s_last_hot_hz = 0;
// Live LO frequency (updated on every successful hop). 32-bit aligned: atomic
// read/write on RISC-V, so a plain volatile is fine for the status-page display.
static volatile uint32_t s_cur_hz = 0;

void scanner_init(dsp_processor_t *dsp)
{
    s_dsp = dsp;
}

esp_err_t scanner_hop(uint32_t hz, bool persist)
{
    esp_err_t err = class_driver_retune(hz);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "hop %lu Hz failed: retune err=%d", (unsigned long)hz, (int)err);
        return err;
    }
    s_cur_hz = hz; // live LO now applied
    if (s_dsp) {
        dsp_processor_reset_tagger_baseline(s_dsp);
    }
    worker_core1_hot_clear_all(); // A6: detect bins are LO-relative — stale after a retune
    if (persist) app_config_set_lo_freq_hz(hz);
    ESP_LOGI(TAG, "hopped to %lu Hz (persist=%d) — re-priming", (unsigned long)hz, persist);
    return ESP_OK;
}

void scanner_reset_baseline(void)
{
    if (s_dsp) dsp_processor_reset_tagger_baseline(s_dsp);
    worker_core1_hot_clear_all(); // A6: bins invalidated by the LO change
}

// One measurement pass over the center grid: hop, settle, drop the settle
// window's density, then integrate dwell_ms and record into out[]. A failed
// hop records a zeroed entry with dwell 0 so it contributes nothing to the
// survey accumulation (rate and burst counts both read 0 for it).
static void sweep_once(const uint32_t *centers, int n, uint32_t dwell_ms,
                       scanner_pos_t *out)
{
    for (int i = 0; i < n; i++) {
        if (scanner_hop(centers[i], false) != ESP_OK) {
            out[i] = (scanner_pos_t){centers[i], 0, 0, 0.0f, 0};
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(SCAN_SETTLE_MS));
        dsp_density_t discard;
        dsp_processor_read_reset_density(s_dsp, &discard); // drop settle window
        vTaskDelay(pdMS_TO_TICKS(dwell_ms));
        dsp_density_t d;
        dsp_processor_read_reset_density(s_dsp, &d);
        out[i] = (scanner_pos_t){centers[i], d.narrowband_bursts, d.all_bursts,
                                 d.mean_snr_db, dwell_ms};
        ESP_LOGI(TAG, "  %lu Hz: nb=%lu all=%lu snr=%.1f (%.2f nb/s)",
                 (unsigned long)centers[i], (unsigned long)d.narrowband_bursts,
                 (unsigned long)d.all_bursts, d.mean_snr_db,
                 scanner_pos_narrowband_rate(&out[i]));
    }
}

void scanner_survey(uint32_t start_hz, uint32_t stop_hz, uint32_t step_hz,
                    uint32_t dwell_ms, int n_sweeps)
{
    if (!s_dsp) {
        ESP_LOGW(TAG, "survey: no detector wired");
        return;
    }
    if (n_sweeps < 1) n_sweeps = 1;
    uint32_t centers[SCANNER_MAX_POSITIONS];
    int      n = scanner_enumerate_centers(start_hz, stop_hz, step_hz, centers, SCANNER_MAX_POSITIONS);
    if (n == 0) {
        ESP_LOGW(TAG, "survey: bad range/step");
        return;
    }

    for (int i = 0; i < n; i++)
        s_acc[i] = (scanner_pos_t){centers[i], 0, 0, 0.0f, 0};

    for (int s = 0; s < n_sweeps; s++) {
        if (n_sweeps > 1) ESP_LOGI(TAG, "--- survey sweep %d/%d ---", s + 1, n_sweeps);
        sweep_once(centers, n, dwell_ms, s_sweep);
        for (int i = 0; i < n; i++)
            scanner_pos_accumulate(&s_acc[i], &s_sweep[i]);
    }

    // Publish the accumulated map as "the last map": dwell_ms holds each
    // center's SUMMED dwell, so the printed nb/s is the mean over all sweeps.
    memcpy(s_map, s_acc, (size_t)n * sizeof(scanner_pos_t));
    s_map_n = n;

    int hot = scanner_rank_hottest(s_map, n);
    if (n_sweeps > 1)
        ESP_LOGI(TAG, "=== integrated over %d sweeps (%lu ms dwell/center each) ===",
                 n_sweeps, (unsigned long)dwell_ms);
    scanner_print_last_map();
    if (hot >= 0) {
        ESP_LOGI(TAG, "parking on %shottest: %lu Hz (%.2f nb/s)",
                 n_sweeps > 1 ? "integrated " : "",
                 (unsigned long)s_map[hot].center_hz,
                 scanner_pos_narrowband_rate(&s_map[hot]));
        scanner_hop(s_map[hot].center_hz, false);
        s_last_hot_hz = s_map[hot].center_hz;
    }
    band_health_note_survey(n_sweeps); // integrated surveys re-arm the trigger
}

void scanner_scan(uint32_t start_hz, uint32_t stop_hz, uint32_t step_hz, uint32_t dwell_ms)
{
    scanner_survey(start_hz, stop_hz, step_hz, dwell_ms, 1);
}

uint32_t scanner_cur_hz(void)
{
    return s_cur_hz;
}

uint32_t scanner_last_hot_hz(void)
{
    return s_last_hot_hz;
}

int scanner_get_last_map(scanner_pos_t *out, int max)
{
    if (!out || max <= 0) return 0;
    int n = s_map_n < max ? s_map_n : max;
    memcpy(out, s_map, (size_t)n * sizeof(scanner_pos_t));
    return n;
}

void scanner_print_last_map(void)
{
    if (s_map_n == 0) {
        ESP_LOGI(TAG, "no scan yet");
        return;
    }
    ESP_LOGI(TAG, "=== density map (nb/s = narrowband bursts/sec) ===");
    for (int i = 0; i < s_map_n; i++)
        ESP_LOGI(TAG, "  %lu Hz  nb/s=%.2f  all=%lu  snr=%.1f dB",
                 (unsigned long)s_map[i].center_hz,
                 scanner_pos_narrowband_rate(&s_map[i]),
                 (unsigned long)s_map[i].all_bursts, s_map[i].mean_snr_db);
}
