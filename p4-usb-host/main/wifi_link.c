#include "wifi_link.h"

#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_system.h"     // esp_restart() — link-loss watchdog (#104)
#include "ping/ping_sock.h" // gateway-ping reachability watchdog
#include "lwip/ip_addr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h" // xTaskCreatePinnedToCoreWithCaps
#include "freertos/semphr.h"

#include "app_config.h"
#include "esp_libusb.h"   // usb.completed liveness for the health wdt
#include "class_driver.h" // class_driver_dump_stall_diag()

static const char *TAG = "WIFI";

typedef enum {
    LINK_OFF = 0,
    LINK_STA,
    LINK_AP,
} link_mode_t;

static volatile link_mode_t s_mode = LINK_OFF;
static volatile bool        s_up   = false; // STA: got IP; AP: started
static char                 s_ssid[33];     // active SSID (STA target, or AP self-SSID)
static esp_ip4_addr_t       s_ip = {0};

// WiFi link-loss watchdog (#104) state.
static volatile uint32_t s_gw_addr      = 0;     // STA gateway IPv4 (ping target)
static volatile bool     s_ever_got_ip  = false; // gate: don't reboot pre-first-IP
static volatile bool     s_ping_ever_ok = false; // gate: gateway answered ICMP once
static SemaphoreHandle_t s_ping_done    = NULL;
static volatile uint32_t s_ping_replies = 0;
static volatile int      s_wdt_fails    = 0; // consecutive failed gw-ping cycles
// USB stream liveness (folded into the same health watchdog, #105).
static volatile bool s_stream_live   = false; // usb.completed advanced at least once
static volatile int  s_stream_stalls = 0;     // consecutive frozen cycles

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
            s_up                             = false;
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
        s_ip                 = e->ip_info.ip;
        ESP_LOGI(TAG, "STA_GOT_IP " IPSTR " gw=" IPSTR,
                 IP2STR(&e->ip_info.ip), IP2STR(&e->ip_info.gw));
        s_up          = true;
        s_gw_addr     = e->ip_info.gw.addr; // ping target for the link-wdt
        s_ever_got_ip = true;
    }
}

// --- WiFi link-loss watchdog (#104) --------------------------------------
// The WiFi/C6 (esp_hosted-over-SDIO) link can drop — or the RPC link wedge —
// leaving the P4 alive and decoding but unreachable over IP, with no event
// or auto-recovery (observed: ~4 h stranded). This watchdog pings the
// gateway; after sustained unreachability it esp_restart()s to re-enumerate
// the link. It only arms after a gateway ping has succeeded once, so a
// network whose gateway ignores ICMP can never trigger a reboot loop.
static void ping_end_cb(esp_ping_handle_t hdl, void *args)
{
    (void)args;
    uint32_t recv = 0;
    esp_ping_get_profile(hdl, ESP_PING_PROF_REPLY, &recv, sizeof(recv));
    s_ping_replies = recv;
    if (s_ping_done) xSemaphoreGive(s_ping_done);
}

static bool ping_gateway_once(void)
{
    uint32_t gw = s_gw_addr;
    if (gw == 0 || !s_ping_done) return false;

    ip_addr_t target       = {0};
    target.type            = IPADDR_TYPE_V4;
    target.u_addr.ip4.addr = gw;

    esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
    cfg.target_addr       = target;
    cfg.count             = 3;
    cfg.interval_ms       = 500;
    cfg.timeout_ms        = 1000;

    esp_ping_callbacks_t cbs = {0};
    cbs.on_ping_end          = ping_end_cb;

    esp_ping_handle_t h = NULL;
    if (esp_ping_new_session(&cfg, &cbs, &h) != ESP_OK || !h) return false;

    s_ping_replies = 0;
    xSemaphoreTake(s_ping_done, 0); // drain any stale signal
    bool ok = false;
    if (esp_ping_start(h) == ESP_OK) {
        // 3 × (500 ms interval + ≤1000 ms timeout) ≈ 4.5 s worst case.
        if (xSemaphoreTake(s_ping_done, pdMS_TO_TICKS(8000)) == pdTRUE) {
            ok = (s_ping_replies > 0);
        }
        esp_ping_stop(h);
    }
    esp_ping_delete_session(h);
    return ok;
}

// One independent health watchdog (runs OUTSIDE the class_driver/DSP loops,
// so a wedged pipeline can't stop it). Each cycle it checks two liveness
// signals and esp_restart()s on either: the USB stream (usb.completed
// advancing, #105) and WiFi reachability (gateway ping, #104). It replaces
// the in-loop stream-stall esp_restart that couldn't fire when that loop
// blocked.
static void health_wdt_task(void *arg)
{
    (void)arg;
    const int        GW_FAIL_LIMIT      = 6; // ~3 min gateway unreachable
    const int        STREAM_STALL_LIMIT = 3; // ~90 s USB stream frozen
    const TickType_t CYCLE              = pdMS_TO_TICKS(30000);
    uint64_t         last_completed     = 0;
    for (;;) {
        vTaskDelay(CYCLE);
        // Only in STA mode and only after we've held an IP, so a never-
        // associating boot or AP config mode can't reboot-loop.
        if (s_mode != LINK_STA || !s_ever_got_ip) {
            s_wdt_fails     = 0;
            s_stream_stalls = 0;
            continue;
        }

        // --- USB stream liveness (#105) ---------------------------------
        // usb.completed advancing = the RTL-SDR stream is feeding the
        // pipeline. If it freezes, the stream wedged (dongle silent halt
        // and/or the class loop blocked on the Core-1 handoff) — the in-loop
        // #103 watchdog can't fire then, so reboot from out here. Only after
        // it has advanced once (s_stream_live), so a non-streaming dongle at
        // boot can't reboot-loop.
        usb_stream_totals_t ut = {0};
        esp_libusb_get_stream_totals(&ut);
        if (ut.completed > last_completed) {
            last_completed  = ut.completed;
            s_stream_live   = true;
            s_stream_stalls = 0;
        } else if (s_stream_live) {
            s_stream_stalls++;
            ESP_LOGW(TAG, "health-wdt: USB stream frozen at %llu (%d/%d)",
                     (unsigned long long)ut.completed, s_stream_stalls, STREAM_STALL_LIMIT);
            if (s_stream_stalls >= STREAM_STALL_LIMIT) {
                ESP_LOGE(TAG, "health-wdt: USB stream frozen %d cycles — dumping diag then esp_restart() [#105]",
                         s_stream_stalls);
                class_driver_dump_stall_diag(); // forensics → serial before reboot
                fflush(stdout);
                vTaskDelay(pdMS_TO_TICKS(200));
                esp_restart();
            }
        }

        // --- WiFi/gateway reachability (#104) ---------------------------
        // Reboot only if the gateway has answered before (proves ICMP works
        // here) — guards against a non-pingable gateway looping the device.
        if (ping_gateway_once()) {
            s_ping_ever_ok = true;
            s_wdt_fails    = 0;
        } else {
            s_wdt_fails++;
            ESP_LOGW(TAG, "health-wdt: gateway unreachable (%d/%d)", s_wdt_fails, GW_FAIL_LIMIT);
            if (s_ping_ever_ok && s_wdt_fails >= GW_FAIL_LIMIT) {
                ESP_LOGE(TAG, "health-wdt: gateway unreachable %d cycles — esp_restart() [#104]",
                         s_wdt_fails);
                fflush(stdout);
                vTaskDelay(pdMS_TO_TICKS(200));
                esp_restart();
            }
        }
    }
}

// Link-watchdog state for /status: gateway IPv4 (network order, 0 if none),
// armed = a gateway ping has succeeded at least once (so the wdt can fire),
// fails = current consecutive failed cycles. See #104.
void wifi_link_wdt_status(uint32_t *gw_addr, bool *gw_armed, int *gw_fails,
                          bool *stream_live, int *stream_stalls)
{
    if (gw_addr) *gw_addr = s_gw_addr;
    if (gw_armed) *gw_armed = s_ping_ever_ok;
    if (gw_fails) *gw_fails = s_wdt_fails;
    if (stream_live) *stream_live = s_stream_live;
    if (stream_stalls) *stream_stalls = s_stream_stalls;
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
    strlcpy((char *)wc.sta.ssid, cfg->wifi_ssid, sizeof(wc.sta.ssid));
    strlcpy((char *)wc.sta.password, cfg->wifi_psk, sizeof(wc.sta.password));
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
    esp_wifi_get_mac(WIFI_IF_AP, mac); // safe to call before mode set; returns
                                       // factory MAC if not yet up.
    snprintf(s_ssid, sizeof(s_ssid), "iridium-%02X%02X%02X",
             mac[3], mac[4], mac[5]);

    esp_netif_create_default_wifi_ap();

    wifi_config_t wc = {0};
    strlcpy((char *)wc.ap.ssid, s_ssid, sizeof(wc.ap.ssid));
    wc.ap.ssid_len       = (uint8_t)strlen(s_ssid);
    wc.ap.channel        = 6;
    wc.ap.max_connection = 4;
    wc.ap.authmode       = WIFI_AUTH_OPEN; // open AP for frictionless config

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

    // Independent health watchdog (#104 gateway + #105 USB stream). Stack in
    // PSRAM — latency-tolerant, and internal/DMA SRAM is reserved for the USB
    // transfer pool. Runs outside all work loops; self-gates to STA mode.
    s_ping_done = xSemaphoreCreateBinary();
    if (s_ping_done) {
        xTaskCreatePinnedToCoreWithCaps(health_wdt_task, "health_wdt", 4096, NULL,
                                        2, NULL, tskNO_AFFINITY,
                                        MALLOC_CAP_SPIRAM);
    } else {
        ESP_LOGW(TAG, "health-wdt: semaphore alloc failed — watchdog disabled");
    }
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
