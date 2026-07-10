// USB host-stack reinstall probe (reinstall test).
//
// A diagnostic, operator-triggered experiment: tear the USB layer down
// cleanly, uninstall the ESP-IDF USB Host Library, reinstall it, and re-power
// the root port — WITHOUT rebooting — to find out whether the host stack can
// be cycled in place (a possible recovery for a wedged dongle that this board
// can't power-cycle; see project_no_switchable_vbus_confirmed). This first
// cut is a PROBE: it stops after reinstall + re-power and does NOT re-arm the
// stream (that re-touches the PIE/DSP layer — project_heap_position_decode_bug
// — and is a deliberate later step). After a probe the device has a fresh host
// with no client/stream; a reboot restores full streaming.
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    bool      quiesced;     // class_driver tore down + deregistered in time
    bool      no_clients;   // NO_CLIENTS lib event seen before uninstall
    bool      all_free;     // ALL_FREE lib event seen before uninstall
    esp_err_t uninstall_rc; // usb_host_uninstall() return
    esp_err_t install_rc;   // usb_host_install() return (only if uninstall OK)
    esp_err_t power_rc;     // root-port power-on return (only if install OK)
    bool      done;         // sequence ran to completion (not timed out early)
} usb_reinstall_result_t;

// Request a probe (POST /usbreinstall). The daemon picks it up within ~2 s.
// Safe from any task.
void usb_reinstall_request(void);

// Daemon side: has a probe been requested? clear consumes the request.
bool usb_reinstall_pending(void);
void usb_reinstall_clear_pending(void);

// Daemon publishes the result; the HTTP handler waits (bounded) for it.
void usb_reinstall_report(const usb_reinstall_result_t *res);
bool usb_reinstall_wait(usb_reinstall_result_t *out, uint32_t timeout_ms);
