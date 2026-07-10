// Public interface to the class_driver task — currently just the
// runtime tuner-gain hook used by D16 software AGC.

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// Apply a manual tuner gain at runtime (tenths of dB, e.g., 350 = 35.0 dB).
// Returns true if applied, false if the device is not yet ready.
// Safe to call from any task: the underlying control transfers
// (esp_libusb_control_transfer, esp_libusb.c) are serialised by
// adsbdev->xfer_mutex, so this can run concurrently with the
// class_driver task's own USB traffic without racing the shared
// transfer/response_buf state (#T8).
bool class_driver_set_tuner_gain_dbx10(int gain_dbx10);

// Like the above, but sets the gain with the bulk stream QUIESCED on the
// usb_pump task (pause -> set -> resume), so the R828D gain-register control
// transfers don't race in-flight bulk URBs. Use this from the gain-cal sweep:
// the bare version fails intermittently under the IRA burst-flood ("set
// failed"). Blocks up to ~3 s; returns false on timeout/failure. Must NOT be
// called from the usb_pump task itself (it posts an action to that task).
bool class_driver_set_gain_quiesced(int gain_dbx10);

// Returns the most recently applied tuner gain in tenths of dB,
// or -1 if not yet set. Useful for AGC state tracking.
int class_driver_get_tuner_gain_dbx10(void);

// Live-retune the SDR LO without stopping the stream (Approach A). Safe to
// call from any task once streaming has started; returns ESP_ERR_INVALID_STATE
// before the device is open.
esp_err_t class_driver_retune(uint32_t hz);

// Forensic dump for a wedged USB stream: which consumer-loop stage is stuck
// (and for how long), USB transfer totals, and all task states + stack
// high-water. Logged to serial. Called by the health watchdog right before it
// reboots a stalled stream (#105), so every recovery leaves a trace. Safe
// from any task (no flash ops).
void class_driver_dump_stall_diag(void);

// Park the dongle cleanly just before an esp_restart(): drain the in-flight
// bulk stream and put the tuner in standby so the next boot re-enumerates a
// quiescent tuner rather than one latched mid-I2C (the reboot-wedge). Signals
// the usb_pump task — the only task permitted to pause the stream / drive
// tuner I2C — and waits a bounded time for it. Best-effort and always
// bounded: if usb_pump is wedged (exactly the health-wdt reboot case) the
// wait times out and the caller must reboot regardless. Safe to call from any
// task; a no-op if no device is open. Call immediately before esp_restart().
void class_driver_prepare_for_reboot(void);

// Maintenance mode for the C6 OTA (c6_ota.c): while ON, usb_pump pauses the
// bulk stream (DSP quiesces) and both the in-loop stall watchdog and the
// wifi_link health watchdog skip their stream-stall reboot — so a deliberate
// multi-minute pause during an OTA isn't mistaken for a wedge. Safe from any
// task. Always pair set(true) before the OTA with set(false) after.
void class_driver_set_maintenance(bool on);
bool class_driver_in_maintenance(void);
