// Public interface to the class_driver task — currently just the
// runtime tuner-gain hook used by D16 software AGC.

#pragma once

#include <stdbool.h>
#include <stdint.h>

// Apply a manual tuner gain at runtime (tenths of dB, e.g., 350 = 35.0 dB).
// Returns true if applied, false if the device is not yet ready.
// Safe to call from any task; serialised by the RTL-SDR driver.
bool class_driver_set_tuner_gain_dbx10(int gain_dbx10);

// Returns the most recently applied tuner gain in tenths of dB,
// or -1 if not yet set. Useful for AGC state tracking.
int  class_driver_get_tuner_gain_dbx10(void);
