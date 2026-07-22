#pragma once
// Band-health monitor + automatic re-survey trigger (band-finder phase 2).
//
// Tracks the hourly IDA (LW.DA) frame rate inside the current ±1.25 MHz RX
// window and detects when the parked LO has gone STALE: the 7-day trailing
// median dropping below 60% of the frozen commissioning baseline (see
// band_health_core.h for the model and thresholds). On a fire it always logs
// a "re-survey recommended" WARN; if the operator has opted in via the NVS
// flag resurvey_auto (app_config band_resurvey_auto, DEFAULT OFF) it also
// runs the integrated multi-sweep survey (scanner_survey) and live-parks on
// the integrated peak — WITHOUT persisting the new LO to NVS, so a reboot
// reverts to the operator-blessed frequency (the device is remote with no
// out-of-band recovery; every automatic action here is reboot-reversible).
//
// This is NOT the disabled periodic autotune LO-rescan: no schedule, no
// single-sweep hotspot chasing — a sustained-decline EVENT launches ONE
// integrated survey, then a >= 24 h cooldown re-arms.
//
// Persistence: the hourly history ring + frozen baseline + cooldown live in
// their own NVS blob (~360 B, one write per completed uptime hour), so the
// 7-day window survives the frequent health-wdt/OTA reboots. Hours are
// UPTIME hours — a reboot discards the partial hour in progress and gaps
// simply compress the window (168 TRACKED hours, not 168 wall-clock hours).
// All NVS writes happen on the internal-RAM-stack hourly worker task, never
// on a PSRAM-stack task (cache_utils.c:114 landmine).

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t hours_tracked;   // completed uptime hours in the trailing window (<= 168)
    uint16_t last_hour;       // IDA (LW.DA) frames in the most recent completed hour
    uint16_t trailing_median; // median frames/h over the trailing window (0 until
                              // BH_MIN_EVAL_HOURS of history exist)
    uint16_t baseline;        // frozen commissioning median (0 = still commissioning)
    uint16_t cooldown_h;      // hours until another fire is allowed (0 = armed)
    uint32_t fired_total;     // lifetime fire count (persisted)
    bool     below_baseline;  // LEVEL: trailing median currently < 60% of baseline
    bool     survey_running;  // an auto-launched integrated survey is in progress
} band_health_status_t;

// Load persisted history from NVS and start the hourly tick. Call once at
// boot (after app_config_init; frame_decoder counters read as 0 until that
// subsystem is up, which is fine — the first bucket completes an hour later).
void band_health_init(void);

// Snapshot for /status. Fields are written by the hourly worker and read
// here without a lock — torn reads are benign for a diagnostic.
void band_health_get_status(band_health_status_t *out);

// Called by scanner_survey() on completion (any caller: serial `survey`,
// POST /scan?n=..., or the auto path). An INTEGRATED survey (n_sweeps >= 2)
// re-arms the cooldown so the trigger doesn't refire right after someone
// just acted on it; single sweeps (n_sweeps <= 1) are ignored.
void band_health_note_survey(int n_sweeps);
