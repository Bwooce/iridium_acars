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
    uint32_t bursts_triage_rejected;     // burst_prefilter (P1.5b) rejected the burst (width/duration/channel-SNR gate) — dropped without the full decode cost (not in bursts_processed)
    uint32_t queue_high_water;           // peak observed queue depth
    float    avg_burst_us;               // mean wall-clock per processed burst
    float    triage_rej_us;              // mean pop→drop wall per triage-REJECTED burst (capacity accounting; rejects aren't in avg_burst_us)

    // Per-stage means (microseconds), averaged over processed bursts only.
    float extract_us;     // signal_buffer_extract
    float freq_center_us; // dsps_cplx_gen + complex multiply loop
    float fir_decim_us;   // Stage 1 FIR + I/Q split
    float demod_us;       // qpsk_demod_process + interleave
    float bch_us;         // BCH decode + de-interleave (when run)
    float triage_us;      // P1.5a triage pass (extract+decim+verdict) of accepted bursts
} worker_stats_t;

void worker_core1_get_stats(worker_stats_t *out);

// Cumulative-since-boot BCH decode counters (bch_decoded / bch_unknown).
// Unlike worker_core1_get_stats(), this does NOT reset — it is a plain read
// so multiple readers (status_logger's periodic drain and the autotune gain
// sweep) don't steal counts from each other. Autotune snapshots at the start
// and end of each dwell window and uses the delta. Either pointer may be NULL.
void worker_core1_get_decode_counts(uint32_t *decoded, uint32_t *unknown);

// Full cumulative-since-boot BCH funnel: decoded / unknown / failed /
// chase_recovered (worker-side Chase-2, #112). Non-resetting reads (same
// no-race rationale as worker_core1_get_decode_counts) so /status can show the
// raw pre-mask BER and Chase-2 rescue rate that were previously visible only on
// the UDP status_logger stream. Each pointer may be NULL. Note: chase_recovered
// is a subset of decoded+unknown (a rescued block makes BCH "pass").
void worker_core1_get_bch_cumulative(uint32_t *decoded, uint32_t *unknown,
                                     uint32_t *failed, uint32_t *chase_recovered);

// Dropped/lost-burst SNR histograms (see worker_core1.c). stale[] = bursts lost
// to a ring-lap before demod (recoverable by a sample backlog / an owned-sample
// queue); pri[] = dropped by SNR-priority (the weakest — junk). 6 buckets of
// 4 dB: <8, 8-12, 12-16, 16-20, 20-24, >=24 dB. Cumulative since boot.
#define WORKER_DROP_SNR_NBUCKET 6
void worker_core1_get_drop_snr(uint32_t stale[WORKER_DROP_SNR_NBUCKET],
                               uint32_t pri[WORKER_DROP_SNR_NBUCKET]);

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
// the PQ. snr_pushed[] is recorded for EVERY burst handed to
// worker_core1_push_burst (before the stale-reject and PQ eviction), so
// the population the PQ sheds is visible too:
//   snr_pushed[32]      same bin layout as snr[], but ALL pushes
typedef struct {
    uint32_t snr[32];
    uint32_t bch[16];
    uint32_t freq[40]; // band occupancy: ALL detections bucketed by rel_freq
    uint32_t snr_pushed[32];
    uint32_t snr_bchok[32]; // tagger-SNR of frames that DECODED (BCH-ok + classified known) — task #26
    uint32_t snr_total;
    uint32_t bch_total;
    uint32_t freq_total;
    uint32_t snr_pushed_total;
    uint32_t snr_bchok_total;
} worker_histograms_t;
void worker_core1_get_histograms(worker_histograms_t *out);

// Copy the fine near-DC histogram (cumulative since boot). Copies
// min(max, WORKER_DCFINE_BINS) entries; *total_out gets the sum (may be NULL).
void worker_core1_get_dcfine(uint32_t *out, int max, uint32_t *total_out);

// ---- A6: continuation-priority boost (hot-bin table) ----
// The frame_decoder task (Core 0) publishes the detect-FFT bin of any OPEN IDA
// chain into a small hot-bin table; burst_priority() boosts bursts within
// HOT_BIN_DEADBAND of a live entry so a chain's continuation is decoded before
// its ring samples lapse. See docs/2026-07-15-a6-continuation-priority-boost-spec.md.
// The table is owned by worker_core1.c; these are the write/query API from the
// decoder task (single writer). `now_us` is the caller's esp_timer_get_time()
// (wall-clock), NOT the frame's RF timestamp (which lags under queueing).
void worker_core1_hot_publish(int bin, uint64_t now_us); // open/refresh a chain's bin (TTL-managed)
void worker_core1_hot_clear(int bin);                    // clear-on-complete
void worker_core1_hot_clear_all(void);                   // LO-retune hook (bins become meaningless)
void worker_core1_hot_set_enabled(bool on);             // runtime A/B gate (default ON)
bool worker_core1_hot_enabled(void);

typedef struct {
    uint32_t published;     // publish/refresh calls
    uint32_t cleared;       // clear-on-complete that found a live entry
    uint32_t boost_pops;    // pq_extract_max winner was boosted
    uint32_t boost_inserts; // pq_insert admitted/evicted-for a boosted newcomer
    // Triage/prefilter rejects on a HOT (open-chain) channel — a candidate 0x7608
    // continuation killed by the fast-pass. Per failing gate (docs/2026-07-15-triage-
    // acars-continuation-review.md §5, measurement M-A). pf_rej_hot_snr>0 = the
    // channel-SNR gate is eating continuations (the suspected zero-ACARS cause).
    uint32_t pf_rej_hot;       // total prefilter rejects whose bin matched an open chain
    uint32_t pf_rej_hot_width; //   ...of which the WIDTH gate failed
    uint32_t pf_rej_hot_dur;   //   ...the DURATION gate failed
    uint32_t pf_rej_hot_snr;   //   ...the channel-SNR gate failed (THE suspect)
    // (pf_rej_margin removed 2026-07-18: the global SNR-margin opener rescue was
    // disabled after the 2026-07-17 A/B — see the WORKER_PF_SNR_MARGIN_DB history
    // in git — leaving the counter permanently 0.)
    // Continuation-fate: bursts DROPPED (never decoded) on a bin that had an OPEN chain (hot).
    // Candidate lost continuations, split by cause. Large vs ida.expired => worker saturation is
    // eating continuations (de-saturate); ~0 while expired>0 => weak-SNR/never-detected instead.
    uint32_t hot_cont_stale; // ring lapped before decode (saturation)
    uint32_t hot_cont_pri;   // evicted from a full queue
} worker_hot_stats_t;
void worker_core1_get_hot_stats(worker_hot_stats_t *out); // plain read, no reset

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
