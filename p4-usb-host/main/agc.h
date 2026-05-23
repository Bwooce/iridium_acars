// D16 Software AGC.
//
// Reactive (descending-only) automatic gain control. Periodically
// samples the RTL-SDR input's peak deviation from the uint8 midpoint
// (via ingest_core1_agc_sample). If the peak indicates saturation,
// reduces the tuner gain. If well below saturation, holds — does
// NOT increase gain. This keeps behaviour conservative: AGC only
// reacts to overload, never chases noise floor.
//
// Activation: app_config.gain_mode == GAIN_MODE_SOFTWARE_AGC.
// In TUNER_AGC or MANUAL mode the task is idle (just sleeps).
//
// Why not full bidirectional AGC? Without a clean noise-floor signal
// (only saturation, which is asymmetric), bidirectional AGC tends
// to chase its tail. The asymmetric descending design is a known-
// safe baseline; richer AGC can be layered on once we have antenna
// telemetry to validate against.

#pragma once

#include "esp_err.h"

// Start the AGC task (Core 1, low priority). Idempotent.
// Reads app_config at each iteration so live-switching gain_mode
// is supported without restart.
esp_err_t agc_init(void);

// Returns the last-known peak deviation reading from the most
// recent AGC iteration. 0..127. Useful for the C6 web UI.
uint8_t agc_get_last_peak_dev(void);
