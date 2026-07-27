#include "net_time.h"

#include <sys/time.h>
#include "esp_netif_sntp.h"
#include "esp_log.h"

static const char *TAG = "net_time";

// s_started guards one-time init; s_synced flips true in the sync callback.
// Both are only written from the event-loop / SNTP task and read elsewhere;
// a torn read is benign (worst case: one extra "not synced" cycle).
static volatile bool s_started = false;
static volatile bool s_synced  = false;

static void on_time_sync(struct timeval *tv)
{
    s_synced = true;
    ESP_LOGI(TAG, "SNTP sync: epoch %lld s", (long long)(tv ? tv->tv_sec : 0));
}

void net_time_start(void)
{
    if (s_started) return;

    // ESP_NETIF_SNTP_DEFAULT_CONFIG starts polling immediately on init.
    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    cfg.sync_cb = on_time_sync;

    esp_err_t err = esp_netif_sntp_init(&cfg);
    if (err != ESP_OK) {
        // Leave s_started false so a later got-IP retries. (INVALID_STATE
        // would mean it's already up — treat that as started.)
        if (err == ESP_ERR_INVALID_STATE) {
            s_started = true;
            return;
        }
        ESP_LOGW(TAG, "esp_netif_sntp_init failed: %s", esp_err_to_name(err));
        return;
    }
    s_started = true;
    ESP_LOGI(TAG, "SNTP started (pool.ntp.org)");
}

bool net_time_synced(void)
{
    return s_synced;
}

int64_t net_time_epoch_us(void)
{
    if (!s_synced) return 0;
    struct timeval tv;
    if (gettimeofday(&tv, NULL) != 0) return 0;
    return (int64_t)tv.tv_sec * 1000000 + (int64_t)tv.tv_usec;
}
