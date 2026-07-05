#include "scanner.h"
#include "scanner_map.h"
#include "class_driver.h"
#include "app_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char      *TAG   = "SCANNER";
static dsp_processor_t *s_dsp = NULL;
static scanner_pos_t    s_map[SCANNER_MAX_POSITIONS];
static int              s_map_n = 0;

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
    if (s_dsp) {
        dsp_processor_reset_tagger_baseline(s_dsp);
    }
    if (persist) app_config_set_lo_freq_hz(hz);
    ESP_LOGI(TAG, "hopped to %lu Hz (persist=%d) — re-priming", (unsigned long)hz, persist);
    return ESP_OK;
}

void scanner_scan(uint32_t start_hz, uint32_t stop_hz, uint32_t step_hz, uint32_t dwell_ms)
{
    if (!s_dsp) {
        ESP_LOGW(TAG, "scan: no detector wired");
        return;
    }
    uint32_t centers[SCANNER_MAX_POSITIONS];
    int      n = scanner_enumerate_centers(start_hz, stop_hz, step_hz, centers, SCANNER_MAX_POSITIONS);
    if (n == 0) {
        ESP_LOGW(TAG, "scan: bad range/step");
        return;
    }

    for (int i = 0; i < n; i++) {
        if (scanner_hop(centers[i], false) != ESP_OK) continue;
        vTaskDelay(pdMS_TO_TICKS(SCAN_SETTLE_MS));
        dsp_density_t discard;
        dsp_processor_read_reset_density(s_dsp, &discard); // drop settle window
        vTaskDelay(pdMS_TO_TICKS(dwell_ms));
        dsp_density_t d;
        dsp_processor_read_reset_density(s_dsp, &d);
        s_map[i] = (scanner_pos_t){centers[i], d.narrowband_bursts, d.all_bursts,
                                   d.mean_snr_db, dwell_ms};
        ESP_LOGI(TAG, "  %lu Hz: nb=%lu all=%lu snr=%.1f (%.2f nb/s)",
                 (unsigned long)centers[i], (unsigned long)d.narrowband_bursts,
                 (unsigned long)d.all_bursts, d.mean_snr_db,
                 scanner_pos_narrowband_rate(&s_map[i]));
    }
    s_map_n = n;

    int hot = scanner_rank_hottest(s_map, n);
    scanner_print_last_map();
    if (hot >= 0) {
        ESP_LOGI(TAG, "parking on hottest: %lu Hz (%.2f nb/s)",
                 (unsigned long)s_map[hot].center_hz,
                 scanner_pos_narrowband_rate(&s_map[hot]));
        scanner_hop(s_map[hot].center_hz, false);
    }
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
