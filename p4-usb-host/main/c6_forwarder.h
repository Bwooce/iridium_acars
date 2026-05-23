// P4-side UART transport to the ESP32-C6 companion (D17).
//
// Sets up the application UART, runs a forwarder task that drains
// a small queue of outgoing frames, and provides non-blocking post
// APIs so the DSP hot path can hand ACARS messages and status
// snapshots to the C6 without blocking on UART.
//
// See docs/c6-companion-firmware-design.md sections 7, 9.

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "iridium_protocol.h"

esp_err_t c6_forwarder_init(void);

// Non-blocking. Returns ESP_OK if queued, ESP_FAIL if queue full
// (item silently dropped — DSP path never stalls on UART).
esp_err_t c6_forwarder_post_status(const irp_status_snap_t *snap);
esp_err_t c6_forwarder_post_acars(const irp_acars_msg_t *msg);

// Sent once after P4 init completes.
esp_err_t c6_forwarder_post_boot_complete(const char *fw_ver);
