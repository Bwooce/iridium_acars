#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "dsp_processor.h"

#define SCAN_START_HZ 1616000000u
#define SCAN_STOP_HZ 1626000000u
#define SCAN_STEP_HZ 2500000u
#define SCAN_DWELL_MS 2000u
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
