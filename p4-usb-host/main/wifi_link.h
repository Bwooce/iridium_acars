#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// Bring up Wi-Fi via the C6 esp_hosted slave.
//
// - If NVS has wifi_ssid set (D18 app_config), starts in STA mode and
//   connects to that AP (esp_wifi_connect runs async on STA_START).
// - If NVS has no SSID, starts an open soft-AP named "iridium-XXXXXX"
//   (last 3 bytes of MAC) so the user's phone can connect and fill in
//   credentials via the HTTP /config form.
//
// Safe to call once at boot after app_config_init(). Mode is locked in
// at start; restart the firmware after writing NVS to flip modes.
esp_err_t wifi_link_start(void);

// True once an IP address is reachable on the active interface:
//   STA mode: IP_EVENT_STA_GOT_IP fired.
//   AP mode:  AP_START fired (AP IP is 192.168.4.1 always).
bool wifi_link_is_connected(void);

// True if we came up in AP-fallback mode (no SSID in NVS).
bool wifi_link_is_ap_mode(void);

// Active SSID (STA target or AP self-SSID). Empty until wifi_link_start
// runs. Pointer stable for the lifetime of the firmware.
const char *wifi_link_ssid(void);

// STA-mode IP (network order). Zero in AP mode (use 192.168.4.1).
uint32_t wifi_link_ip_u32(void);

// Link-loss watchdog (#104) state for diagnostics. gw_addr: STA gateway
// IPv4 (network order, 0 if none). armed: a gateway ping has succeeded at
// least once (watchdog can fire). fails: current consecutive failed cycles.
void wifi_link_wdt_status(uint32_t *gw_addr, bool *armed, int *fails);
