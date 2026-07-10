// See c6_ota.h. Streams a C6 slave firmware image from an HTTP(S) URL to the
// C6 over the esp_hosted OTA RPC (SDIO). Mirrors the flow in the esp_hosted
// examples/host_performs_slave_ota, split into transfer (safe) + activate
// (risky).
#include "c6_ota.h"

#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_hosted_ota.h" // esp_hosted_slave_ota_begin/write/end/activate
#include "class_driver.h"   // class_driver_set_maintenance() — quiesce DSP during OTA

static const char *TAG = "C6OTA";

#define C6OTA_CHUNK   1400 // matches the esp_hosted example's SDIO OTA chunk
#define C6OTA_URL_MAX 200

static atomic_bool s_busy  = false;
static atomic_bool s_abort = false; // request the in-flight transfer to stop
static char        s_status[160] = "idle";
static char        s_url[C6OTA_URL_MAX];

const char *c6_ota_status(void) { return s_status; }
bool        c6_ota_busy(void) { return atomic_load(&s_busy); }

static void set_status(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_status, sizeof(s_status), fmt, ap);
    va_end(ap);
    ESP_LOGW(TAG, "%s", s_status);
}

static void c6_ota_task(void *arg)
{
    (void)arg;
    // Quiesce the DSP (pause the bulk stream) + disarm the stall/health
    // watchdogs for the whole transfer, so the SDIO link + CPU aren't contended
    // and a multi-minute pause isn't rebooted as a wedge. Give usb_pump a moment
    // to actually pause + the ring to drain before we start hammering the link.
    class_driver_set_maintenance(true);
    vTaskDelay(pdMS_TO_TICKS(600));

    esp_http_client_config_t http_cfg = {
        .url                   = s_url,
        .timeout_ms            = 20000,
        .keep_alive_enable     = true,
        // HTTP only for a LAN-hosted image; add .crt_bundle_attach for HTTPS.
    };
    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
    if (!client) {
        set_status("FAIL: http_client_init");
        goto done;
    }
    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        set_status("FAIL: http open %s (%s)", s_url, esp_err_to_name(err));
        goto cleanup;
    }
    int64_t content_len = esp_http_client_fetch_headers(client);
    int     status_code = esp_http_client_get_status_code(client);
    if (status_code != 200) {
        set_status("FAIL: HTTP %d fetching image", status_code);
        goto cleanup;
    }
    set_status("downloading %lld B, streaming to C6...", (long long)content_len);

    // Begin the slave OTA session. If THIS fails, the stale slave's OTA RPC
    // isn't usable — the whole Method B is a no-go and we've learned that
    // safely (C6 untouched).
    err = esp_hosted_slave_ota_begin();
    if (err != ESP_OK) {
        set_status("FAIL: slave_ota_begin %s (stale slave may not support OTA RPC)",
                   esp_err_to_name(err));
        goto cleanup;
    }

    uint8_t *buf = malloc(C6OTA_CHUNK);
    if (!buf) {
        set_status("FAIL: chunk alloc");
        (void)esp_hosted_slave_ota_end();
        goto cleanup;
    }

    int      total        = 0;
    int      r;
    bool     write_failed = false;
    while ((r = esp_http_client_read(client, (char *)buf, C6OTA_CHUNK)) > 0) {
        if (atomic_load(&s_abort)) {
            set_status("ABORTED @%d B — re-POST /c6ota?url=... to restart", total);
            write_failed = true;
            break;
        }
        // A C6/SDIO stall makes this RPC time out and return an error (esp_hosted
        // RPCs are bounded) rather than hanging. On that we bail cleanly, run
        // ota_end below to reset the slave session, and the transfer is fully
        // restartable — begin() at the next /c6ota starts a fresh session.
        err = esp_hosted_slave_ota_write(buf, (uint32_t)r);
        if (err != ESP_OK) {
            set_status("STALLED: slave_ota_write @%d B: %s — re-POST /c6ota?url=... to restart",
                       total, esp_err_to_name(err));
            write_failed = true;
            break;
        }
        total += r;
        if ((total & 0xFFFF) < C6OTA_CHUNK) { // ~every 64 KB
            set_status("transferring: %d B to C6...", total);
        }
    }
    free(buf);

    if (r < 0 && !write_failed) {
        set_status("FAIL: http read error @%d B", total);
        (void)esp_hosted_slave_ota_end();
        goto cleanup;
    }

    esp_err_t end_err = esp_hosted_slave_ota_end();
    if (write_failed) {
        goto cleanup; // status already set by the write failure
    }
    if (end_err != ESP_OK) {
        set_status("FAIL: slave_ota_end after %d B: %s", total, esp_err_to_name(end_err));
        goto cleanup;
    }
    set_status("OK: transferred %d B to C6 inactive partition. NOT activated — "
               "POST /c6ota?activate=1 to switch (risky).", total);

cleanup:
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
done:
    class_driver_set_maintenance(false); // resume the DSP stream
    atomic_store(&s_busy, false);
    vTaskDelete(NULL);
}

esp_err_t c6_ota_transfer_start(const char *url)
{
    if (!url || url[0] == '\0') return ESP_ERR_INVALID_ARG;
    bool expected = false;
    if (!atomic_compare_exchange_strong(&s_busy, &expected, true)) {
        return ESP_ERR_INVALID_STATE; // already running
    }
    atomic_store(&s_abort, false);
    strlcpy(s_url, url, sizeof(s_url));
    set_status("starting: %s", s_url);
    if (xTaskCreate(c6_ota_task, "c6ota", 6144, NULL, 5, NULL) != pdPASS) {
        atomic_store(&s_busy, false);
        set_status("FAIL: task spawn");
        return ESP_FAIL;
    }
    return ESP_OK;
}

void c6_ota_abort(void)
{
    if (atomic_load(&s_busy)) {
        atomic_store(&s_abort, true);
        ESP_LOGW(TAG, "abort requested — transfer will stop at the next chunk");
        // If the RPC is hard-hung (no timeout) the task can't see this; a reboot
        // recovers (the C6's half-written INACTIVE partition is harmless).
    }
}

esp_err_t c6_ota_activate(void)
{
    if (atomic_load(&s_busy)) return ESP_ERR_INVALID_STATE;
    ESP_LOGW(TAG, "activating C6 OTA image (risky — version-gated, may drop Wi-Fi)");
    esp_err_t err = esp_hosted_slave_ota_activate();
    set_status("activate -> %s", esp_err_to_name(err));
    return err;
}
