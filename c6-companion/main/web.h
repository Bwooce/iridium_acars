// HTTP server: serves static HTML/JS from SPIFFS partition + REST
// API endpoints (/api/messages, /api/stats, /api/status, /api/config).

#pragma once

#include "esp_err.h"

esp_err_t web_init(void);
