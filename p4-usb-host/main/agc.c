// See agc.h.

#include "agc.h"
#include "app_config.h"
#include "class_driver.h"
#include "ingest_core1.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "AGC";

// AGC tunables. Conservative — designed to nudge gain DOWN when
// saturating, leave it alone otherwise.
#define AGC_TICK_MS            1000   // re-evaluate every 1 second
#define AGC_SAT_THRESHOLD      110    // peak_dev > this -> reduce gain
#define AGC_MIN_GAIN_DBX10     0      // 0.0 dB lower bound
#define AGC_GAIN_STEP_DBX10    25     // 2.5 dB per step (matches R820T step size)

static TaskHandle_t       s_agc_task    = NULL;
static volatile uint8_t   s_last_peak   = 0;

static void agc_task(void *arg)
{
    (void)arg;
    int my_gain = -1;     // -1 = not yet set (will sync to current driver gain)
    bool was_software_agc = false;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(AGC_TICK_MS));

        app_config_t cfg;
        app_config_snapshot(&cfg);

        // Detect mode changes. When entering SOFTWARE_AGC mode,
        // initialise my_gain from the configured manual value;
        // when leaving, just reset state so the next entry restarts
        // cleanly.
        bool is_software_agc = (cfg.gain_mode == GAIN_MODE_SOFTWARE_AGC);
        if (is_software_agc && !was_software_agc) {
            my_gain = cfg.gain_db_x10;
            ESP_LOGI(TAG, "AGC enabled (start gain %d.%d dB)",
                     my_gain / 10, my_gain % 10);
            class_driver_set_tuner_gain_dbx10(my_gain);
        } else if (!is_software_agc && was_software_agc) {
            ESP_LOGI(TAG, "AGC disabled");
        }
        was_software_agc = is_software_agc;
        if (!is_software_agc) continue;

        // Read input peak deviation since last sample.
        uint8_t peak = 0;
        uint32_t dispatches = 0;
        ingest_core1_agc_sample(&peak, &dispatches);
        s_last_peak = peak;
        if (dispatches == 0) continue;     // no data yet

        if (peak > AGC_SAT_THRESHOLD) {
            int next = my_gain - AGC_GAIN_STEP_DBX10;
            if (next < AGC_MIN_GAIN_DBX10) next = AGC_MIN_GAIN_DBX10;
            if (next != my_gain) {
                if (class_driver_set_tuner_gain_dbx10(next)) {
                    ESP_LOGI(TAG, "peak=%u dispatches=%u -> reduce gain %d.%d -> %d.%d dB",
                             peak, (unsigned)dispatches,
                             my_gain / 10, my_gain % 10,
                             next / 10, next % 10);
                    my_gain = next;
                }
            }
        }
        // Else: hold. Conservative — no upward chasing.
    }
}

esp_err_t agc_init(void)
{
    if (s_agc_task) return ESP_OK;
    BaseType_t ok = xTaskCreatePinnedToCore(
        agc_task, "agc",
        /*stack=*/ 3072, NULL,
        /*prio=*/ 3,       // low priority; AGC is not time-critical
        &s_agc_task,
        /*core=*/ 1);
    return (ok == pdPASS) ? ESP_OK : ESP_FAIL;
}

uint8_t agc_get_last_peak_dev(void)
{
    return s_last_peak;
}
