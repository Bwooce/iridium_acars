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
//
// Concurrency: refuses (logs + returns) if another autotune pass -- this
// one, autotune_run_lo_rescan(), or a periodic re-run from autotune_sched.c
// -- is already in progress. Safe to call from any task.
void autotune_run_manual(void);

// Periodic/boot LO density re-scan (2026-07-08 boot/periodic extension,
// design doc step 3): sweeps the ACARS band with the existing scanner
// (SCAN_START_HZ..SCAN_STOP_HZ, see scanner.h) and parks + persists on the
// hottest center. This is "SLOW center-tracking" per the design's empirical
// finding (best-LO is mean-reverting, not momentum) -- callers should use a
// long interval (autotune_lo_interval_s, default hourly), not a fast one.
//
// Same MANUAL-gain-mode precondition and in-progress guard as
// autotune_run_manual(). CAUTION: relies on scanner_scan()'s automated
// multi-hop retune, whose DMA-internal-heap-churn safety under sustained
// unattended use is not yet confirmed (backlog #7); autotune_lo_interval_s=0
// disables this path if it proves unsafe. Requires device-smoke validation
// before merge.
void autotune_run_lo_rescan(void);

// Scan-progress for the status page. Returns 0=idle, 1=gain cal, 2=LO rescan,
// and fills elapsed_s / remaining_s (estimated) when a scan is active (both 0
// when idle). Either pointer may be NULL. Cross-task safe (relaxed atomics).
int autotune_scan_status(int *elapsed_s, int *remaining_s);
