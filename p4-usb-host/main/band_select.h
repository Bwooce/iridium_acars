// band_select — maps the NVS band id to a concrete band_pipeline_t
// (VHF/VDL2 foundation). Lives in main/ (not common/band_pipeline/) so
// the generic interface component stays dependency-free: this is the
// ONLY place that references every concrete band implementation.

#pragma once

#include "band_pipeline.h"
#include "band_profile.h"

// Resolve the per-burst pipeline for a band. Out-of-range ids fall back
// to Iridium (same clamp rule as band_profile_get). Never returns NULL.
const band_pipeline_t *band_select_pipeline(band_id_t id);
