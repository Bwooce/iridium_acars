// C6 (esp_hosted slave) firmware update over the SDIO link — Method B.
//
// The P4 downloads a version-matched esp_hosted C6 slave binary from a URL and
// streams it to the C6 over the esp_hosted OTA RPC. Split into two explicit
// steps so the risky part is opt-in:
//
//   c6_ota_transfer(url)  — download + begin/write/end. Writes the image to the
//                           C6's INACTIVE OTA partition; the C6 keeps running
//                           its current firmware. SAFE to run: a failed or
//                           incompatible transfer leaves the C6 unchanged. This
//                           is also the test of whether SDIO OTA works at all on
//                           the stale (reported 0.0.0) slave.
//   c6_ota_activate()     — switch the C6 to the newly-written image + reboot it.
//                           RISKY: version-gated (needs slave FW > v2.5.x), and
//                           a bad activate can drop Wi-Fi. On a headless outdoor
//                           device that means losing all remote access. Only run
//                           after a clean transfer, and prefer to have physical
//                           (UART/PROG_C6) recovery access available.
//
// See docs/c6-firmware-update.md for the full context + the version catch-22.
#pragma once

#include "esp_err.h"

// Kick off a C6 OTA transfer from `url` on a background task (non-blocking).
// Returns ESP_OK if the task started, ESP_ERR_INVALID_STATE if one is already
// running. Progress/result is reported via c6_ota_status().
esp_err_t c6_ota_transfer_start(const char *url);

// Activate the previously-transferred image (the risky switch). Blocks briefly.
// Returns the esp_hosted_slave_ota_activate() result; ESP_ERR_INVALID_STATE if a
// transfer is still running.
esp_err_t c6_ota_activate(void);

// Request the in-flight transfer to stop at its next chunk (so you can restart
// after a C6/SDIO stall). A fresh c6_ota_transfer_start() then begins a clean
// session. If the RPC is hard-hung, a reboot recovers (inactive partition is
// harmless). No-op if idle.
void c6_ota_abort(void);

// Human-readable one-line status of the last/in-progress transfer.
const char *c6_ota_status(void);

// True while a transfer task is running.
bool c6_ota_busy(void);
