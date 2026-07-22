// iridium_band_pipeline — Iridium's implementation of the generic
// band_pipeline_t interface (common/band_pipeline/band_pipeline.h).
//
// Thin, allocation-free adapter over the existing per-burst modules:
//   prefilter     -> burst_prefilter()            (burst_prefilter.c)
//   process_burst -> burst_pipeline_process_burst() (burst_pipeline.c)
// The adapter adds NO processing — it only maps types (verdict struct,
// burst_pipeline_result_t -> band_frame_t). Bit-identity with the
// direct calls is pinned by tests/host/test_band_pipeline_iridium.c.

#pragma once

#include "band_pipeline.h"

// Returns the (static, immutable) Iridium pipeline vtable.
const band_pipeline_t *iridium_band_pipeline(void);
