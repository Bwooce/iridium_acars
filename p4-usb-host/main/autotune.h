#pragma once
// Manual autotune RF-recalibration pass (2026-07-08 autotune design).
//
// Runs in the caller's task (serial_cmd), the same discipline as
// scanner_scan: hop to the IRA reference LO, sweep the configured coarse
// gain set counting bch_decoded per gain (inverted-U objective — MAXIMIZE
// decodes, tolerate USB drops), pick the peak gain, then park back at the
// original ACARS LO with the chosen gain and persist it. The curve + chosen
// values are logged over serial (ESP_LOG on the console UART).
//
// Live-apply only — no reboot. Gain changes go through the xfer-mutex-safe
// class_driver_set_tuner_gain_dbx10(); LO changes reuse scanner_hop()'s
// quiesced retune + baseline reset. See the design doc for why the objective
// is a coarse sweep (not binary/ternary search) at a long dwell.
//
// PRECONDITION: gain_mode must be MANUAL. In SOFTWARE_AGC the agc_task
// rewrites the tuner gain every second and fights the sweep; in TUNER_AGC the
// tuner runs its own hardware AGC so manual gain writes won't hold. The
// routine refuses (with a serial message) rather than silently flipping the
// mode (which would persist to NVS).
void autotune_run_manual(void);
