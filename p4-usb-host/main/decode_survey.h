#pragma once
// Decode-based band-finder survey (commissioning tool). Ranks candidate 2.5 MHz
// LO centers by ACTUAL LW.DA (IDA/ACARS-bearing) decode rate instead of raw
// burst density — density counts IDA, ring-alert IRA and interference
// indistinguishably and once wandered the parked LO into the IDA-barren IRA
// simplex sub-band. Decodes are Poisson-sparse (~sub-1/min at a good park) and
// ride Iridium's ~1 min spot beams, so this integrates each candidate over many
// beam periods with INTERLEAVED round-robin revisits (beam/diurnal common-mode
// cancels in the pairwise ranking), eliminating provably-worse centers as it
// goes. Three phases:
//   A  density pre-pass (existing scanner_survey) → shortlist of K centers
//   B  round-robin 5 min visits, LW.DA-rate elimination (decode_survey_core.h)
//   C  place the final 2.5 MHz window on the peak of the accumulated,
//      LO-independent LW.DA absolute-freq histogram (fine grid)
//
// ONE-TIME / operator- (or rare trigger-) gated. NOT a periodic steerer — the
// park-don't-steer policy stands (chasing the instantaneous hotspot is
// anti-predictive). The pick is live-parked only (scanner_hop persist=false);
// NVS lo_freq_hz is NEVER written, so a health-wdt reboot reverts to the
// operator's frequency. The RESULT (ranked table + recommended Hz + timestamp)
// IS persisted to its own NVS namespace so a 24 h outcome survives a reboot.

#include "esp_err.h"
#include "decode_survey_core.h"
#include <stdbool.h>
#include <stdint.h>

#define DS_MAX_CENTERS      8   // shortlist cap (union of top-K + peak neighbours)
#define DS_DEFAULT_K        4   // shortlisted centers when caller doesn't specify
#define DS_DEFAULT_BUDGET_H 24  // wall-clock cap; a true near-tie runs to here
#define DS_VISIT_MS         300000u // 5 min per round-robin visit (~5 beam periods)
#define DS_PLACE_STEP_HZ    250000u // Phase-C fine placement grid step

// Survey phase, published in status.
typedef enum {
    DS_PHASE_IDLE      = 0,
    DS_PHASE_SHORTLIST = 1, // A: density pre-pass
    DS_PHASE_RR        = 2, // B: round-robin decode ranking
    DS_PHASE_PLACE     = 3, // C: histogram window placement
    DS_PHASE_DONE      = 4,
    DS_PHASE_ABORTED   = 5,
} ds_phase_t;

typedef struct {
    bool        running;
    ds_phase_t  phase;
    uint32_t    cycle;      // completed round-robin cycles
    uint32_t    elapsed_s;  // since Phase B start
    uint32_t    budget_s;   // Phase B wall-clock cap
    int         n_centers;  // shortlist size
    int         alive;      // centers still in the running
    uint32_t    leader_hz;  // current best center (0 if none yet)
    uint32_t    pick_hz;    // Phase-C placement (0 until done)
    ds_center_t centers[DS_MAX_CENTERS];
    uint32_t    abs_hist[DS_ABS_HIST_BINS]; // LW.DA absolute-freq map
} decode_survey_status_t;

// Start a survey on an internal-RAM-stack task (REQUIRED — it does the NVS
// result-commit; an nvs_commit from a PSRAM-stack task aborts at
// cache_utils.c:114). budget_h <= 0 uses DS_DEFAULT_BUDGET_H; k out of
// [1, DS_MAX_CENTERS] uses DS_DEFAULT_K. Returns ESP_ERR_INVALID_STATE if a
// survey is already running or a gain-cal / LO scan is in progress (never stack
// RF actions), ESP_FAIL if the task can't be spawned.
esp_err_t decode_survey_start(int budget_h, int k);

// Request an abort: the task finishes its current step, parks back on the NVS
// lo_freq_hz it started from, and exits without persisting a result.
void decode_survey_stop(void);

// True while a survey task is live (used by band_health to defer staleness
// evaluation, and by callers to avoid stacking RF actions).
bool decode_survey_running(void);

// Snapshot the survey state for /status and /diag/survey. Torn reads of the
// table/histogram are benign for a diagnostic (single-task writer).
void decode_survey_get_status(decode_survey_status_t *out);
