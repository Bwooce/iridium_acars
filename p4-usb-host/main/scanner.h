#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "dsp_processor.h"

// Start at 1618, not 1616: the bottom ~1.8 MHz of the RTL band is a legal
// dead zone — Iridium's regulatory floor is ~1617.8 MHz (FCC 07-194) and our
// 40 h HydraSDR ground truth had 0.00% of IDA below 1617.3 MHz. Sweeping 1616
// spent ~2 dwell tiles on empty spectrum. Bonus: from 1618 the 1.25 MHz grid
// lands a center on 1620.5 — right on the IDA density peak (~1620.6) — instead
// of straddling it at 1619.75/1621 as the old 1616-based grid did. A center at
// 1618 still hears down to 1616.75 via the ±1.25 MHz window, so no IDA is lost.
// (Non-IDA energy below 1618 — e.g. Globalstar 1616-1618.7 — is an interference
// question for the reception heuristic at the parked LO, not the density scan.)
#define SCAN_START_HZ 1618000000u
#define SCAN_STOP_HZ 1626000000u
// 1.25 MHz step = half the 2.5 MHz RX window (50% overlap), so centers resolve
// the density peak instead of the old 2.5 MHz grid whose tiles straddled it
// (the ACARS peak ~1620.6 fell between 1618.5 and 1621). ~7 centers over the band.
#define SCAN_STEP_HZ 1250000u
// 5 s dwell (was 2 s): density swings wildly second-to-second with satellite
// passes (observed 0.5 -> 295 nb/s on one center in ~1 min), so a longer dwell
// averages it out for a more stable best-LO pick. ~7 centers x ~5.5 s ≈ 40 s/sweep.
#define SCAN_DWELL_MS 5000u
#define SCAN_SETTLE_MS 500u
// Default sweep count for an integrated survey (scanner_survey). At the
// defaults above one sweep is ~40 s, so 10 sweeps ≈ 6.5 min — spanning several
// satellite-beam periods (~1 min), which is what makes the accumulated map
// beam-averaged rather than a snapshot of whichever beam one sweep landed on.
#define SCAN_SURVEY_SWEEPS 10

// Wire the scanner to the running detector (for density reads + baseline
// reset). Call once at boot after dsp_processor_create.
void scanner_init(dsp_processor_t *dsp);

// Live-retune to hz and re-prime the tagger noise floor. persist=true also
// writes NVS lo_hz. Returns class_driver_retune's status.
esp_err_t scanner_hop(uint32_t hz, bool persist);

// Sweep [start,stop] by step; at each center hop, settle, then measure
// narrowband density over dwell_ms; print a ranked map and park on the
// hottest center. Runs in the caller's (serial_cmd) task. Equivalent to
// scanner_survey(..., 1) — kept for existing callers, but note a single
// sweep parks on whichever satellite beam happened to be overhead; prefer
// scanner_survey() for any decision that should outlive the current beam.
void scanner_scan(uint32_t start_hz, uint32_t stop_hz, uint32_t step_hz, uint32_t dwell_ms);

// Integrated multi-sweep survey (band-finder commissioning). Repeats the
// scanner_scan measurement n_sweeps times and ACCUMULATES per-center
// narrowband-burst density (summed bursts over summed dwell — see
// scanner_pos_accumulate), then prints the ranked accumulated map and parks
// on the INTEGRATED peak. Rationale (40 h HydraSDR band-strategy analysis):
// the instantaneous density hotspot rides the satellite beams (~1 min
// period), so any single sweep picks a near-random hot center; only the
// multi-sweep average is time-stationary (IDA hump median ~1620.58 MHz).
// n_sweeps <= 1 degrades to exactly scanner_scan(). Blocks in the caller's
// task for ~n_sweeps x 40 s at the SCAN_* defaults. Live-parks only
// (persist=false); callers that want the pick to survive a reboot read
// scanner_last_hot_hz() and persist it themselves.
// ONE-TIME commissioning tool: do NOT wire this to a periodic clock — the
// park-don't-steer policy stands (chasing the instantaneous hotspot is
// anti-predictive; see autotune_lo_interval_s rationale in app_config.h).
void scanner_survey(uint32_t start_hz, uint32_t stop_hz, uint32_t step_hz,
                    uint32_t dwell_ms, int n_sweeps);

// Reprint the last density map (or "no scan yet").
void scanner_print_last_map(void);

// Copy the last published density map into out[] (up to max entries); returns
// the number of centers copied (0 if no scan has run yet). Lets a caller (the
// decode-based band survey, decode_survey.c) read the density pre-pass result
// to build a decode-ranking shortlist instead of only parking on the peak.
#include "scanner_map.h"
int scanner_get_last_map(scanner_pos_t *out, int max);

// Reset the tagger noise-floor baseline so it re-learns after a live change
// that shifts the floor (e.g. a gain step during autotune) without changing
// the LO. Reuses the same reset the LO hop performs. No-op if the detector
// isn't wired yet.
void scanner_reset_baseline(void);

// The center_hz of the most recent hot bin any scanner_scan() /
// scanner_survey() call found (0 if no scan has ever found one yet) — after
// an integrated survey this is the ACCUMULATED peak, not any single sweep's.
// Sticky: a scan that finds nothing hot leaves this at its previous value
// rather than clearing it. Scans only live-retune (persist=false); callers
// that want the discovered center to survive a reboot (e.g. autotune's
// periodic LO re-scan) read this and persist it themselves via
// app_config_set_lo_freq_hz().
uint32_t scanner_last_hot_hz(void);

// Live LO frequency (Hz) currently applied — updated on every scanner_hop(),
// so during an LO rescan it tracks the sweep. 0 before the first hop. For the
// status page's "config → now" frequency display.
uint32_t scanner_cur_hz(void);
