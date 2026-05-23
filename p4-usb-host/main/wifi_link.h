#pragma once

#include "esp_err.h"
#include <stdbool.h>

// Bring up Wi-Fi STA from NVS-stored credentials (wifi_ssid / wifi_psk,
// set via D18's app_config). If either is empty, this is a no-op
// (logs "Wi-Fi STA: no credentials in NVS, skipping") and Wi-Fi stays
// inactive — esp_hosted's SDIO transport to the C6 is still up, just
// idle.
//
// Safe to call once at boot, after app_config_init(). Async — STA
// connection happens in the background; check wifi_link_is_connected()
// or subscribe to WIFI_EVENT / IP_EVENT from the default event loop
// if you need to know when the IP is up.
esp_err_t wifi_link_start(void);

// True once IP_EVENT_STA_GOT_IP has fired.
bool wifi_link_is_connected(void);

// Returns the configured SSID (or empty string if none). Pointer
// remains valid for the lifetime of the firmware.
const char *wifi_link_ssid(void);
