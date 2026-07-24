// band_select — maps the NVS band id to a concrete band_pipeline_t
// (VHF/VDL2 foundation). Lives in main/ (not common/band_pipeline/) so
// the generic interface component stays dependency-free: this is the
// ONLY place that references every concrete band implementation.

#pragma once

#include "band_pipeline.h"
#include "band_profile.h"
#include "band_decode_stats.h"

// Resolve the per-burst pipeline for a band. Out-of-range ids fall back
// to Iridium (same clamp rule as band_profile_get). Never returns NULL.
const band_pipeline_t *band_select_pipeline(band_id_t id);

// Band-agnostic decode-funnel snapshot (cumulative since boot). Maps the
// active band's native counters onto band_decode_stats_t so band-independent
// consumers (autotune success metric, reception classifier) avoid the
// bch-vs-vdl2 getter fork. Out-of-range id clamps to Iridium. `out` may not
// be NULL. This is the ONLY place besides band_select_pipeline that references
// both concrete bands (keeps common/ dependency-free).
void band_decode_stats_get(band_id_t id, band_decode_stats_t *out);

// Resolved per-band runtime bundle (band-mode overhaul phase 3): the active
// band's id + profile params + demod pipeline vtable, from ONE call — so the
// init-time consumers (dsp_processor, worker_core1, frame_decoder) stop each
// independently re-deriving band -> profile/pipeline. Out-of-range clamps to
// Iridium; never returns NULL. Returns a pointer to a stable static entry
// (valid for the process lifetime). Resolve at init, after app_config_init.
typedef struct {
    band_id_t              band;
    const band_profile_t  *profile;  // band_profile_get(band)
    const band_pipeline_t *pipeline; // band_select_pipeline(band)
} band_runtime_t;
const band_runtime_t *band_runtime_resolve(band_id_t id);
