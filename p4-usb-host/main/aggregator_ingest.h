// Aggregator-side frame ingest (#119 / #137). Consumes decoded-frame PDUs
// emitted by one or more worker front ends and feeds them into the local
// frame_decoder (classify -> SBD reassembly -> outputs), exactly as a
// STANDALONE board's worker_core1 would via frame_decoder_push().
//
// Two PDU sources feed the same consumer:
//   - COMBINED_LOOPBACK: the in-process worker pushes to the frame_pdu
//     queue and this task drains it (one-board end-to-end validation).
//   - AGGREGATOR: the SPI slave ingest (Phase 3) pushes received PDUs to
//     the same queue; this task is source-agnostic.
//
// Cross-receiver dedupe (same frame seen by two workers) lands with the
// multi-worker SPI transport; with a single worker there are no
// duplicates so it is a no-op for now.

#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Spawn the ingest consumer task (Core 1, priority 4 — same tier as
// frame_decoder, below worker/ingest). frame_pdu_queue_init() and
// frame_decoder_init() must already have run. Idempotent.
esp_err_t aggregator_ingest_init(void);

// Total PDUs popped from the queue and pushed into frame_decoder.
uint32_t aggregator_ingest_count(void);

// Per-source liveness, for receiver health/observability (#138). With a
// single worker (COMBINED) only one source appears; the table is sized for
// the small worker fleet a Phase-3 SPI aggregator will serve.
#define AGG_MAX_SOURCES 4

typedef struct {
    uint32_t source_id;    // low 4 bytes of the worker STA MAC
    uint32_t count;        // PDUs forwarded from this source
    uint64_t last_seen_us; // esp_timer time of the most recent PDU
} aggregator_source_stat_t;

typedef struct {
    uint32_t                 forwarded;       // total PDUs -> frame_decoder
    uint32_t                 pdu_queue_depth; // current frame_pdu queue depth
    uint32_t                 pdu_dropped;     // PDUs dropped at the queue
    uint32_t                 n_sources;       // distinct sources seen
    aggregator_source_stat_t sources[AGG_MAX_SOURCES];
} aggregator_ingest_stats_t;

// Snapshot the ingest stats (safe to call from another task, e.g. the
// HTTP /status handler).
void aggregator_ingest_get_stats(aggregator_ingest_stats_t *out);

#ifdef __cplusplus
}
#endif
