#pragma once

#include "esp_err.h"

// Tiny UDP/53 DNS server that responds to every A query with the AP's
// own IP (192.168.4.1). Phones use this to detect captive portals:
// after associating, iOS / Android probe a known URL; getting the AP
// IP back triggers their captive-portal browser instead of a
// "no internet" badge, and pops the config form automatically.
//
// Only useful in AP mode; call captive_dns_start() after wifi_link
// reports AP_START. No-op if already running. Memory cost: ~3 KB
// task stack + a single UDP socket.
esp_err_t captive_dns_start(void);
