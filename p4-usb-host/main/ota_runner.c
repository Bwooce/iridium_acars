// D19 — pull-style OTA worker.
//
// One-shot task per OTA attempt. esp_https_ota does the heavy lifting:
// fetches the image, verifies the app descriptor magic, writes into
// the inactive ota partition, flips otadata, and on success returns
// ESP_OK. We then esp_restart() so the new image boots.
//
// Rollback: ota_runner_mark_valid() is called once from app_main after
// boot is known-healthy. Until then the bootloader is counting boots
// against esp_ota_set_boot_partition's pending state — if we crash
// before mark_valid runs N times, the bootloader reverts to the
// previous partition automatically.
//
// HTTP-vs-HTTPS: this module accepts both. Mutual TLS, cert pinning,
// etc. are future work (see ota_runner.h). For an internal LAN
// deployment serving the .bin from a local box, http:// is fine; for
// the eventual public update server, we'll need to embed a CA cert
// (esp_app_format provides hooks).

#include "ota_runner.h"

#include <string.h>

#include "esp_log.h"
#include "esp_https_ota.h"
#include "esp_http_client.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/idf_additions.h"
#include "esp_heap_caps.h"

#include "app_config.h"

static const char *TAG = "OTA";

static SemaphoreHandle_t s_status_mu = NULL;
static ota_status_t      s_status    = {0};
static volatile bool     s_running   = false;

static void set_status_error(const char *fmt, ...)
{
    if (!s_status_mu) return;
    xSemaphoreTake(s_status_mu, portMAX_DELAY);
    s_status.state = OTA_FAILED;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_status.last_error, sizeof(s_status.last_error), fmt, ap);
    va_end(ap);
    xSemaphoreGive(s_status_mu);
    ESP_LOGE(TAG, "%s", s_status.last_error);
}

static void ota_task(void *arg)
{
    (void)arg;

    app_config_t cfg;
    app_config_snapshot(&cfg);
    if (cfg.ota_url[0] == '\0') {
        set_status_error("ota_url not set in NVS — set via /config first");
        goto done;
    }

    ESP_LOGI(TAG, "starting OTA from %s", cfg.ota_url);

    esp_http_client_config_t http_cfg = {
        .url               = cfg.ota_url,
        .timeout_ms        = 30000,
        .keep_alive_enable = true,
        // No cert bundle by default — works on http://; for https://
        // requires CA bundle (CONFIG_MBEDTLS_CERTIFICATE_BUNDLE) which
        // IDF defaults on, but skipping CN check makes self-signed
        // dev servers easier.
        .crt_bundle_attach    = NULL,
        .skip_cert_common_name_check = true,
    };
    esp_https_ota_config_t ota_cfg = {
        .http_config = &http_cfg,
    };

    esp_https_ota_handle_t handle = NULL;
    esp_err_t r = esp_https_ota_begin(&ota_cfg, &handle);
    if (r != ESP_OK || !handle) {
        set_status_error("esp_https_ota_begin failed: %s", esp_err_to_name(r));
        goto done;
    }

    int status = esp_https_ota_get_status_code(handle);
    if (s_status_mu) {
        xSemaphoreTake(s_status_mu, portMAX_DELAY);
        s_status.http_status = status;
        xSemaphoreGive(s_status_mu);
    }
    if (status < 200 || status >= 300) {
        esp_https_ota_abort(handle);
        set_status_error("HTTP %d from %s", status, cfg.ota_url);
        goto done;
    }

    esp_app_desc_t new_desc;
    r = esp_https_ota_get_img_desc(handle, &new_desc);
    if (r == ESP_OK) {
        ESP_LOGI(TAG, "new image: project='%s' version='%s' time='%s'",
                 new_desc.project_name, new_desc.version, new_desc.date);
    }

    while (1) {
        r = esp_https_ota_perform(handle);
        if (r != ESP_ERR_HTTPS_OTA_IN_PROGRESS) break;
        int bytes = esp_https_ota_get_image_len_read(handle);
        if (s_status_mu) {
            xSemaphoreTake(s_status_mu, portMAX_DELAY);
            s_status.bytes_written = bytes;
            xSemaphoreGive(s_status_mu);
        }
        // Yield so the rest of the firmware (DSP pipeline, /status
        // polls, etc.) keeps making progress while we download.
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    int final_bytes = esp_https_ota_get_image_len_read(handle);

    if (!esp_https_ota_is_complete_data_received(handle)) {
        esp_https_ota_abort(handle);
        set_status_error("incomplete data (read=%d)", final_bytes);
        goto done;
    }
    if (r != ESP_OK) {
        esp_https_ota_abort(handle);
        set_status_error("esp_https_ota_perform: %s", esp_err_to_name(r));
        goto done;
    }

    r = esp_https_ota_finish(handle);
    if (r != ESP_OK) {
        set_status_error("esp_https_ota_finish: %s", esp_err_to_name(r));
        goto done;
    }

    if (s_status_mu) {
        xSemaphoreTake(s_status_mu, portMAX_DELAY);
        s_status.state         = OTA_SUCCESS;
        s_status.bytes_written = final_bytes;
        s_status.last_error[0] = '\0';
        xSemaphoreGive(s_status_mu);
    }
    ESP_LOGI(TAG, "OTA success — %d bytes staged, rebooting in 2 s", final_bytes);
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();

done:
    s_running = false;
    vTaskDelete(NULL);
}

esp_err_t ota_runner_start(void)
{
    if (!s_status_mu) {
        s_status_mu = xSemaphoreCreateMutex();
        if (!s_status_mu) return ESP_ERR_NO_MEM;
    }
    if (s_running) return ESP_ERR_INVALID_STATE;

    xSemaphoreTake(s_status_mu, portMAX_DELAY);
    s_status.state         = OTA_RUNNING;
    s_status.http_status   = 0;
    s_status.bytes_written = 0;
    s_status.last_error[0] = '\0';
    xSemaphoreGive(s_status_mu);
    s_running = true;

    // OTA task MUST have an internal-SRAM stack. esp_https_ota_perform
    // and esp_https_ota_finish do flash writes via spi_flash_disable_
    // interrupts_caches_and_other_cpu(), which makes PSRAM (cached)
    // inaccessible. A PSRAM-stacked task hits an assert and aborts the
    // moment it dereferences any local during that cache-disabled
    // window — same bug as the /config save crash (commit a23b6b8).
    // 8 KB of internal SRAM is fine here because the task is only
    // spawned on user-triggered OTA (not boot), so it can't fragment
    // tagger init the way a boot-time internal stack would.
    BaseType_t ok = xTaskCreatePinnedToCore(ota_task, "ota", 8192,
                                             NULL, 5, NULL,
                                             tskNO_AFFINITY);
    if (ok != pdPASS) {
        s_running = false;
        set_status_error("xTaskCreate failed");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void ota_runner_get_status(ota_status_t *out)
{
    if (!out) return;
    if (!s_status_mu) {
        memset(out, 0, sizeof(*out));
        return;
    }
    xSemaphoreTake(s_status_mu, portMAX_DELAY);
    *out = s_status;
    xSemaphoreGive(s_status_mu);
}

void ota_runner_mark_valid(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (!running) return;
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) != ESP_OK) return;
    if (state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_err_t r = esp_ota_mark_app_valid_cancel_rollback();
        if (r == ESP_OK) {
            ESP_LOGI(TAG, "marked running partition '%s' valid (rollback cancelled)",
                     running->label);
        } else {
            ESP_LOGW(TAG, "mark_app_valid failed: %s", esp_err_to_name(r));
        }
    }
}
