#pragma once

#include "esp_err.h"

// Start the HTTP server (port 80). Call after wifi_link_start() — works
// in both STA and AP modes; the lwIP stack will accept on whichever
// interface has an IP.
//
// Current endpoints:
//   GET /status   — JSON snapshot of firmware state (build, mode, IP,
//                   decode counters).
//
// Future endpoints (D17 follow-ups): GET /config, POST /config, /ota.
esp_err_t http_server_start(void);
