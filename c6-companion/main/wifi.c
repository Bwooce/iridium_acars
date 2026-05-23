// See wifi.h. Standalone Wi-Fi: STA mode if credentials present,
// AP fallback if not.

#include "wifi.h"
#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"

static const char *TAG = "WIFI";
static const char *NVS_NS = "iridium_c";

static volatile bool s_connected = false;
static const char   *s_mode_str  = "init";
static EventGroupHandle_t s_ev = NULL;
#define EV_CONNECTED  BIT0

static void event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)data;
    if (base == WIFI_EVENT) {
        switch (id) {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            s_connected = false;
            ESP_LOGW(TAG, "STA disconnected; reconnecting...");
            esp_wifi_connect();
            break;
        case WIFI_EVENT_AP_STACONNECTED:
            ESP_LOGI(TAG, "Client joined captive-portal AP");
            break;
        case WIFI_EVENT_AP_STADISCONNECTED:
            ESP_LOGI(TAG, "Client left captive-portal AP");
            break;
        }
    } else if (base == IP_EVENT) {
        switch (id) {
        case IP_EVENT_STA_GOT_IP: {
            ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
            s_connected = true;
            s_mode_str = "STA-connected";
            ESP_LOGI(TAG, "STA got IP " IPSTR, IP2STR(&e->ip_info.ip));
            xEventGroupSetBits(s_ev, EV_CONNECTED);
            break;
        }
        }
    }
}

static bool load_creds(char *ssid_out, size_t ssid_cap,
                       char *psk_out,  size_t psk_cap)
{
    nvs_handle_t h;
    esp_err_t r = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (r != ESP_OK) return false;
    size_t s_len = ssid_cap, p_len = psk_cap;
    bool ok = (nvs_get_str(h, "wifi_ssid", ssid_out, &s_len) == ESP_OK
            && ssid_out[0] != '\0');
    if (ok) {
        nvs_get_str(h, "wifi_psk", psk_out, &p_len);   // optional
    }
    nvs_close(h);
    return ok;
}

static esp_err_t start_ap_mode(void)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    char ssid[32];
    snprintf(ssid, sizeof(ssid), "iridium-%02X%02X%02X",
             mac[3], mac[4], mac[5]);

    wifi_config_t apc = {0};
    strncpy((char *)apc.ap.ssid, ssid, sizeof(apc.ap.ssid));
    apc.ap.ssid_len      = strlen(ssid);
    apc.ap.channel       = 6;
    apc.ap.max_connection= 4;
    apc.ap.authmode      = WIFI_AUTH_OPEN;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &apc));
    ESP_ERROR_CHECK(esp_wifi_start());
    s_mode_str = "AP-portal";
    ESP_LOGI(TAG, "AP mode: SSID=%s open at 192.168.4.1", ssid);
    return ESP_OK;
}

static esp_err_t start_sta_mode(const char *ssid, const char *psk)
{
    wifi_config_t sc = {0};
    strncpy((char *)sc.sta.ssid, ssid, sizeof(sc.sta.ssid));
    if (psk) strncpy((char *)sc.sta.password, psk, sizeof(sc.sta.password));
    sc.sta.threshold.authmode = (psk && psk[0])
        ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sc));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "STA connecting to SSID '%s'...", ssid);
    return ESP_OK;
}

esp_err_t wifi_init(void)
{
    s_ev = xEventGroupCreate();
    if (!s_ev) return ESP_ERR_NO_MEM;

    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        r = nvs_flash_init();
    }
    ESP_ERROR_CHECK(r);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    char ssid[33] = {0};
    char psk [65] = {0};
    bool have_creds = load_creds(ssid, sizeof(ssid), psk, sizeof(psk));

    if (have_creds) {
        esp_netif_create_default_wifi_sta();
    } else {
        esp_netif_create_default_wifi_ap();
    }

    wifi_init_config_t wc = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wc));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,   IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    if (have_creds) {
        return start_sta_mode(ssid, psk);
    } else {
        return start_ap_mode();
    }
}

bool wifi_is_connected(void) { return s_connected; }
const char *wifi_mode_str(void) { return s_mode_str; }
int8_t wifi_rssi(void)
{
    wifi_ap_record_t rec = {0};
    if (esp_wifi_sta_get_ap_info(&rec) == ESP_OK) return rec.rssi;
    return -127;
}
