#ifndef WORKER_CORE1_H
#define WORKER_CORE1_H

#include "esp_err.h"
#include "dsp_processor.h"

esp_err_t worker_core1_init();
void worker_core1_push_burst(const detected_burst_t *burst);

// Diagnostic stats for the burst worker. Each call returns the values
// accumulated since the previous call and resets the internal counters.
typedef struct {
    uint32_t bursts_queued;       // pushed to the queue (incl. dropped)
    uint32_t bursts_dropped;      // dropped because queue was full
    uint32_t bursts_processed;    // ran end-to-end through the worker
    uint32_t bursts_skipped;      // dropped by edge/length/zero-output guards
    uint32_t bursts_bch_decoded;  // subset whose frame passed BCH AND classified as a known type (real decodes)
    uint32_t bursts_bch_unknown;  // subset that passed BCH but iridium_frame_classify returned UNKNOWN (BCH false positives)
    uint32_t bursts_bch_failed;   // subset that demod'd but BCH was uncorrectable (qpsk_demod false positives)
    uint32_t queue_high_water;    // peak observed queue depth
    float    avg_burst_us;        // mean wall-clock per processed burst

    // Per-stage means (microseconds), averaged over processed bursts only.
    float    extract_us;          // signal_buffer_extract
    float    freq_center_us;      // dsps_cplx_gen + complex multiply loop
    float    fir_decim_us;        // Stage 1 FIR + I/Q split
    float    resample_us;         // Stage 2 polyphase resample
    float    demod_us;            // qpsk_demod_process + interleave
    float    bch_us;              // BCH decode + de-interleave (when run)
} worker_stats_t;

void worker_core1_get_stats(worker_stats_t *out);

// Smoke-only: dumps per-burst golden-bits comparison summary at the
// end of the smoke run. Compiled to a no-op outside the
// CONFIG_SMOKE_TEST_RAW_IRIDIUM build.
void worker_core1_golden_print_summary(void);

#endif
