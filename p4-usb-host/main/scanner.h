#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "dsp_processor.h"

#define SCAN_START_HZ 1616000000u
#define SCAN_STOP_HZ 1626000000u
// 1.25 MHz step = half the 2.5 MHz RX window (50% overlap), so centers resolve
// the density peak instead of the old 2.5 MHz grid whose tiles straddled it
// (the ACARS peak ~1620.6 fell between 1618.5 and 1621). ~9 centers over the band.
#define SCAN_STEP_HZ 1250000u
// 5 s dwell (was 2 s): density swings wildly second-to-second with satellite
// passes (observed 0.5 -> 295 nb/s on one center in ~1 min), so a longer dwell
// averages it out for a more stable best-LO pick. ~9 centers x ~6.5 s = ~1 min/scan, hourly.
#define SCAN_DWELL_MS 5000u
#define SCAN_SETTLE_MS 500u

// Wire the scanner to the running detector (for density reads + baseline
// reset). Call once at boot after dsp_processor_create.
void scanner_init(dsp_processor_t *dsp);

// Live-retune to hz and re-prime the tagger noise floor. persist=true also
// writes NVS lo_hz. Returns class_driver_retune's status.
esp_err_t scanner_hop(uint32_t hz, bool persist);

// Sweep [start,stop] by step; at each center hop, settle, then measure
// narrowband density over dwell_ms; print a ranked map and park on the
// hottest center. Runs in the caller's (serial_cmd) task.
void scanner_scan(uint32_t start_hz, uint32_t stop_hz, uint32_t step_hz, uint32_t dwell_ms);

// Reprint the last density map (or "no scan yet").
void scanner_print_last_map(void);

// Reset the tagger noise-floor baseline so it re-learns after a live change
// that shifts the floor (e.g. a gain step during autotune) without changing
// the LO. Reuses the same reset the LO hop performs. No-op if the detector
// isn't wired yet.
void scanner_reset_baseline(void);

// The center_hz of the most recent hot bin any scanner_scan() call found
// (0 if no scan has ever found one yet). Sticky: a scan that finds nothing
// hot leaves this at its previous value rather than clearing it.
// scanner_scan() only live-retunes (persist=false); callers that want the
// discovered center to survive a reboot (e.g. autotune's periodic LO
// re-scan) read this and persist it themselves via
// app_config_set_lo_freq_hz().
uint32_t scanner_last_hot_hz(void);
