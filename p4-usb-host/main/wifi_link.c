#include "wifi_link.h"

#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"

static const char *TAG = "WIFI";

typedef enum {
    LINK_OFF = 0,
    LINK_STA,
    LINK_AP,
} link_mode_t;

static volatile link_mode_t s_mode = LINK_OFF;
static volatile bool        s_up   = false;     // STA: got IP; AP: started
static char                 s_ssid[33];         // active SSID (STA target, or AP self-SSID)
static esp_ip4_addr_t       s_ip   = {0};

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
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
            s_up = false;
            wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)data;
            ESP_LOGW(TAG, "STA_DISCONNECTED reason=%d → reconnect in 5s",
                     e ? e->reason : -1);
            vTaskDelay(pdMS_TO_TICKS(5000));
            esp_wifi_connect();
            break;
        }
        case WIFI_EVENT_AP_START:
            s_up = true;
            ESP_LOGI(TAG, "AP_START SSID='%s' (captive portal mode)", s_ssid);
            break;
        case WIFI_EVENT_AP_STACONNECTED: {
            wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)data;
            ESP_LOGI(TAG, "AP client connected: " MACSTR " aid=%d",
                     MAC2STR(e->mac), e->aid);
            break;
        }
        case WIFI_EVENT_AP_STADISCONNECTED: {
            wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)data;
            ESP_LOGI(TAG, "AP client disconnected: " MACSTR " aid=%d",
                     MAC2STR(e->mac), e->aid);
            break;
        }
        default:
            break;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        s_ip = e->ip_info.ip;
        ESP_LOGI(TAG, "STA_GOT_IP " IPSTR " gw=" IPSTR,
                 IP2STR(&e->ip_info.ip), IP2STR(&e->ip_info.gw));
        s_up = true;
    }
}

static void start_sta(const app_config_t *cfg)
{
    strlcpy(s_ssid, cfg->wifi_ssid, sizeof(s_ssid));
    esp_netif_t *sta = esp_netif_create_default_wifi_sta();

    // Advertise station_id as the DHCP hostname so the device appears by
    // name (e.g. "p4-iridium-1") in the router's client list and can be
    // reached without hardcoding its IP. Set before connect so it's sent
    // in the first DHCP request. station_id is kept DHCP-safe by config.
    if (sta && cfg->station_id[0]) {
        esp_err_t hr = esp_netif_set_hostname(sta, cfg->station_id);
        if (hr != ESP_OK) {
            ESP_LOGW(TAG, "set_hostname('%s') -> %s",
                     cfg->station_id, esp_err_to_name(hr));
        }
    }

    wifi_config_t wc = {0};
    strlcpy((char *)wc.sta.ssid,     cfg->wifi_ssid, sizeof(wc.sta.ssid));
    strlcpy((char *)wc.sta.password, cfg->wifi_psk,  sizeof(wc.sta.password));
    wc.sta.threshold.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    s_mode = LINK_STA;
    ESP_LOGI(TAG, "STA configured for SSID '%s' (connect runs async)", s_ssid);
}

static void start_ap(void)
{
    // Build a unique SSID from the C6's MAC. The 3 trailing bytes are
    // distinctive enough on a typical desk; the prefix tells the user
    // what device they're looking at.
    uint8_t mac[6] = {0};
    esp_wifi_get_mac(WIFI_IF_AP, mac);     // safe to call before mode set; returns
                                            // factory MAC if not yet up.
    snprintf(s_ssid, sizeof(s_ssid), "iridium-%02X%02X%02X",
             mac[3], mac[4], mac[5]);

    esp_netif_create_default_wifi_ap();

    wifi_config_t wc = {0};
    strlcpy((char *)wc.ap.ssid, s_ssid, sizeof(wc.ap.ssid));
    wc.ap.ssid_len      = (uint8_t)strlen(s_ssid);
    wc.ap.channel       = 6;
    wc.ap.max_connection = 4;
    wc.ap.authmode      = WIFI_AUTH_OPEN;    // open AP for frictionless config

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    s_mode = LINK_AP;
    ESP_LOGI(TAG, "AP configured: SSID='%s' open, channel 6", s_ssid);
}

esp_err_t wifi_link_start(void)
{
    app_config_t cfg;
    app_config_snapshot(&cfg);

    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &on_wifi_event, NULL, NULL));

    if (cfg.wifi_ssid[0] != '\0') {
        start_sta(&cfg);
    } else {
        ESP_LOGI(TAG, "no SSID in NVS → starting open AP for first-time config");
        start_ap();
    }

    ESP_ERROR_CHECK(esp_wifi_start());
    return ESP_OK;
}

bool wifi_link_is_connected(void)
{
    return s_up;
}

bool wifi_link_is_ap_mode(void)
{
    return s_mode == LINK_AP;
}

const char *wifi_link_ssid(void)
{
    return s_ssid;
}

uint32_t wifi_link_ip_u32(void)
{
    return s_ip.addr;
}
