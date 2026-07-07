#ifndef WORKER_CORE1_H
#define WORKER_CORE1_H

#include "esp_err.h"
#include "dsp_processor.h"

esp_err_t worker_core1_init();
void      worker_core1_push_burst(const detected_burst_t *burst);

// Pre-allocate/pin the wideband decim FIR's PIE delay lines (I+Q) in
// DRAM. Part of the boot-time early-alloc dance (see
// uw_correlator_prealloc_pie_fft / uw_correlator_prealloc_fir);
// MUST be called before other heap-touching init so the delay lines
// don't spill into RTCRAM under DRAM pressure
// (project_heap_position_decode_bug). Idempotent; worker_core1_init()
// calling direct_if_decim_init(&s_decim) again afterwards is a no-op
// for this state.
void worker_core1_prealloc_fir(void);

// Diagnostic stats for the burst worker. Each call returns the values
// accumulated since the previous call and resets the internal counters.
typedef struct {
    uint32_t bursts_queued;              // pushed to the queue (incl. dropped)
    uint32_t bursts_dropped;             // dropped because queue was full (newcomer weakest)
    uint32_t bursts_evicted;             // displaced from a full PQ by a stronger newcomer
                                         // before the worker popped them (churn indicator:
                                         // under junk storms this ticks while dropped stays 0)
    uint32_t bursts_processed;           // ran end-to-end through the worker
    uint32_t bursts_skipped;             // dropped by edge/length/zero-output guards
    uint32_t bursts_bch_decoded;         // subset whose frame passed BCH AND classified as a known type (real decodes)
    uint32_t bursts_bch_unknown;         // subset that passed BCH but iridium_frame_classify returned UNKNOWN (BCH false positives)
    uint32_t bursts_bch_failed;          // subset that demod'd but BCH was uncorrectable (qpsk_demod false positives)
    uint32_t bursts_bch_chase_recovered; // subset where hard-BCH failed but Chase-2 soft decoder rescued it (#112; subset of bursts_bch_decoded + bursts_bch_unknown)
    uint32_t bursts_triage_rejected;     // P1.5a fast-pass verdict said "no frame" — dropped without the full decode cost (not in bursts_processed)
    uint32_t queue_high_water;           // peak observed queue depth
    float    avg_burst_us;               // mean wall-clock per processed burst
    float    triage_rej_us;              // mean pop→drop wall per triage-REJECTED burst (capacity accounting; rejects aren't in avg_burst_us)

    // Per-stage means (microseconds), averaged over processed bursts only.
    float extract_us;     // signal_buffer_extract
    float freq_center_us; // dsps_cplx_gen + complex multiply loop
    float fir_decim_us;   // Stage 1 FIR + I/Q split
    float resample_us;    // Stage 2 polyphase resample
    float demod_us;       // qpsk_demod_process + interleave
    float bch_us;         // BCH decode + de-interleave (when run)
    float triage_us;      // P1.5a triage pass (extract+decim+verdict) of accepted bursts
} worker_stats_t;

void worker_core1_get_stats(worker_stats_t *out);

// Diagnostic histograms (#116). Cumulative since boot; clients compute
// deltas if they want a rate. Closes the design-review gap that /status
// alone can't distinguish "antenna empty" from "demod broken in a new way."
//   snr[32]  bin i = bursts with floor(SNR_dB) == i
//   bch[16]  bin ((e1+1)*4 + (e2+1)) for e in {-1=fail, 0,1,2=corrected}.
// snr_total = sum(snr); bch_total = sum(bch). UW Hamming histogram is
// deliberately deferred (requires decoded_frame_t API change to plumb).
//
// P1.5c: snr/bch/freq above are POP-side (or, for freq, all detections) —
// snr[] specifically only sees bursts the worker actually extracted off
// the PQ. snr_pushed[]/duration_pushed[] are recorded for EVERY burst
// handed to worker_core1_push_burst (before the stale-reject and PQ
// eviction), so the population the PQ sheds is visible too:
//   snr_pushed[32]      same bin layout as snr[], but ALL pushes
//   duration_pushed[2]  0 = length_samples < BURST_DURATION_CLASS_MIN_SAMPLES
//                       (impulse-length), 1 = at/above (plausible-length)
typedef struct {
    uint32_t snr[32];
    uint32_t bch[16];
    uint32_t freq[40]; // band occupancy: ALL detections bucketed by rel_freq
    uint32_t snr_pushed[32];
    uint32_t duration_pushed[2];
    uint32_t snr_total;
    uint32_t bch_total;
    uint32_t freq_total;
    uint32_t snr_pushed_total;
    uint32_t duration_pushed_total;
} worker_histograms_t;
void worker_core1_get_histograms(worker_histograms_t *out);

// Smoke-only: dumps per-burst golden-bits comparison summary at the
// end of the smoke run. Compiled to a no-op outside the
// CONFIG_SMOKE_TEST_RAW_IRIDIUM build.
void worker_core1_golden_print_summary(void);

// Smoke-only: read the golden-match counters (RAW_IRIDIUM fixture). Used
// by the RAW smoke to gate on real decode correctness (matched vs the 65
// gr-iridium golden frames) instead of a coarse classified-count that
// includes UNKNOWN/BCH false positives. matched cratering (e.g. 62 -> 4)
// is the PIE RTCRAM-spill decode regression (project_heap_position_decode_bug).
void worker_core1_golden_get(uint32_t *matched, uint32_t *decoded, int *gri_total);

#endif
