// Wi-Fi bring-up. Uses NVS-stored credentials (namespace "iridium_c"
// keys "wifi_ssid", "wifi_psk"). Falls back to AP mode (open SSID
// "iridium-XXX") if no credentials are stored.

#pragma once

#include <stdbool.h>
#include "esp_err.h"

esp_err_t wifi_init(void);
bool      wifi_is_connected(void);
const char *wifi_mode_str(void);          // "STA-connected" / "AP-portal" / "init"
int8_t    wifi_rssi(void);                 // current RSSI, dBm; -127 if unknown
