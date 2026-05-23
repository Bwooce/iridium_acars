#include "wifi_link.h"

#include <string.h>

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"

static const char *TAG = "WIFI";

static volatile bool s_connected = false;
static char           s_ssid[33];     // Wi-Fi SSID max 32 + NUL

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)data;
    if (base == WIFI_EVENT) {
        switch (id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "STA_START → esp_wifi_connect()");
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "STA_CONNECTED to '%s'", s_ssid);
            break;
        case WIFI_EVENT_STA_DISCONNECTED: {
            s_connected = false;
            wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)data;
            ESP_LOGW(TAG, "STA_DISCONNECTED reason=%d → reconnect in 5s",
                     e ? e->reason : -1);
            // Simple linear retry. Could exponential-backoff later.
            vTaskDelay(pdMS_TO_TICKS(5000));
            esp_wifi_connect();
            break;
        }
        default:
            break;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "STA_GOT_IP " IPSTR " gw=" IPSTR,
                 IP2STR(&e->ip_info.ip), IP2STR(&e->ip_info.gw));
        s_connected = true;
    }
}

esp_err_t wifi_link_start(void)
{
    app_config_t cfg;
    app_config_snapshot(&cfg);
    if (cfg.wifi_ssid[0] == '\0') {
        ESP_LOGI(TAG, "no SSID in NVS, skipping Wi-Fi STA bring-up "
                       "(set via app_config_set_wifi_ssid())");
        return ESP_OK;
    }

    // Copy SSID for stable logging (NVS may be mutated later via app_config_set_*).
    strlcpy(s_ssid, cfg.wifi_ssid, sizeof(s_ssid));

    // NVS may already be initialised by app_config_init(); nvs_flash_init() is
    // idempotent (returns ESP_ERR_NVS_NO_FREE_PAGES on fresh flash, in which
    // case erase+retry). app_config_init() handles this already, so a second
    // call here is just a guard.
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &on_wifi_event, NULL, NULL));

    wifi_config_t wc = {0};
    strlcpy((char *)wc.sta.ssid,     cfg.wifi_ssid, sizeof(wc.sta.ssid));
    strlcpy((char *)wc.sta.password, cfg.wifi_psk,  sizeof(wc.sta.password));
    // WPA2-PSK is the common case; threshold=OPEN lets us also join open
    // networks if psk is empty.
    wc.sta.threshold.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "STA started for SSID '%s' (connect runs async)", s_ssid);
    return ESP_OK;
}

bool wifi_link_is_connected(void)
{
    return s_connected;
}

const char *wifi_link_ssid(void)
{
    return s_ssid;
}
