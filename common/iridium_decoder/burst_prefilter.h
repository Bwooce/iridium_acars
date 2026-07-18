// burst_prefilter.h — cheap, feature-based burst pre-discriminator
// (P1.5b redesign, 2026-07-07).
//
// PURPOSE
// -------
// Replaces the P1.5a "fast-pass" triage (burst_pipeline_triage), which
// ran the EXPENSIVE UW-correlation + CFO chain as its screen — the very
// FFT-bound operations it was meant to avoid, single-attempt, and
// measured to reject ~everything (0 escalations in a soak). This
// pre-discriminator instead rejects impulsive / flat-spectrum junk using
// only cheap features BEFORE any correlation, then lets survivors run the
// FULL retry pipeline (burst_pipeline_process_burst) so recall is
// preserved.
//
// It runs on the decimated 250 ksps, DC-centred (post rotate-to-dc) IQ
// window — the same buffer burst_pipeline_process_burst consumes.
//
// FEATURES & gr-iridium GROUNDING (no test-fit thresholds)
// --------------------------------------------------------
// 1. Active duration (time domain, O(N)). A real Iridium frame is at
//    least MIN_FRAME_LENGTH_* symbols long; gr-iridium's burst_downmix
//    drops anything shorter without decoding:
//        burst_downmix_impl.cc:502  `if (burst_size - start <
//                                     min_frame_length) return 0;`
//    with min_frame_length = MIN_FRAME_LENGTH_{NORMAL|SIMPLEX} ×
//    output_samples_per_symbol (iridium.h:18/21). We use the SIMPLEX
//    minimum (80 symbols, the smallest decodable frame class) so we
//    never reject a real burst of any class — the recall-safe bound.
//    Short impulsive junk (the ≤2.5 ms bench flood) is shorter than one
//    frame and dies here at O(N) cost.
//
// 2. Integrated in-band channel SNR (one 2048-pt FFT). gr-iridium's
//    fft_burst_tagger only ever declares a burst present when a
//    burst_width-wide channel's magnitude exceeds the rolling noise
//    baseline by the detection threshold:
//        fft_burst_tagger_impl.cc:386  `if (d_relative_magnitude_f[bin]
//                                        > d_threshold)`
//        fft_burst_tagger_impl.cc:248-250 (center bin ±1 channel)
//    with burst_width = 40 kHz (iridium-extractor:126,
//    iridium_extractor_flowgraph.py:30) and threshold = 18 dB
//    (iridium-extractor:130). In OUR baseline-scale domain the
//    equivalent threshold is 14 dB (memory: ours ≈ gri − 4 dB;
//    the tagger in these host tests runs at 14 dB). We measure the
//    integrated power in the ±20 kHz (= burst_width) channel around DC
//    (the tagger centres every burst; rotate-to-dc has already put it at
//    0 Hz) relative to a robust out-of-band noise-floor estimate, and
//    require ≥ threshold. Flat noise and broadband/out-of-band impulses
//    have no in-band channel excess and die here.
//
// 3. Spectral width (tagger metadata, O(1) — no samples touched). A real
//    Iridium burst spans ~one 41.67 kHz channel (~34 bins at the 2.5 MSPS
//    / 2048-pt tagger FFT); broadband RFI spreads far wider. The tagger
//    already measures the contiguous above-threshold width at detection
//    (fbt_burst_t.width_bins, packed into detected_burst_t via
//    BURST_WIDTH_BINS). This gate rejects bursts wider than ~3 channels
//    BEFORE any sample work. It is intentionally the coarsest / most
//    conservative gate: a burst only a channel wide is never rejected.
//    width_bins <= 0 means "unmeasured" (host tags that don't carry it) and
//    is treated as narrow — never rejected. Runs FIRST (cheapest).
//
// Both time/spectral gates are recall-biased: gate 1 uses the smallest
// frame class, gate 2 uses the same channel/threshold model that admitted
// the burst in the first place. Neither is tuned to a fixture; every
// constant traces to a gr-iridium source line. The width gate uses the
// same channel-width model the tagger uses to declare a burst present.

#pragma once

#include <stdbool.h>
#include <stdint.h>

// Gate-2 detection threshold, in this build's baseline-scale dB. gr-iridium's
// default is 18 dB (iridium-extractor:130); our tagger runs ~4 dB lower (memory
// note "Tagger threshold scale: ours ≈ gri − 4 dB"), so the parity value is 14.
// Exposed here (not just internal to the .c) so a receiver-integration layer can
// apply its own MARGIN below this without re-testing at strict parity — the
// prefilter's SNR estimator never cross-calibrated with the tagger, so shaving
// exactly at parity kills marginal-but-real bursts (see worker_core1 opener rescue).
#define PF_THRESH_DB 14.0

typedef struct {
    bool  accept;         // final verdict
    int   width_bins;     // gate-0 tagger spectral width (echoed back)
    bool  width_ok;       // gate-0 passed (width_bins <= PF_MAX_WIDTH_BINS)
    int   active_len;     // gate-1 active-envelope sample count
    bool  dur_ok;         // gate-1 passed (active_len >= min frame)
    int   fft_seg_start;  // gate-2 segment offset (complex samples)
    float channel_snr_db; // gate-2 integrated in-band channel SNR
    bool  snr_ok;         // gate-2 passed
} burst_prefilter_result_t;

// Cheap feature verdict on a decimated (250 ksps), DC-centred IQ window.
// `iq250` is interleaved int16 I,Q; `n_complex` is the complex length.
// `width_bins` is the tagger's measured spectral width in tagger-FFT bins
// (BURST_WIDTH_BINS(burst) on device); pass <= 0 when unmeasured to skip
// the width gate (never rejects). The buffer is NOT mutated. Returns true
// (accept → escalate to the full retry pipeline) or false (reject → drop).
// Optional `out` receives the per-gate diagnostics (pass NULL to ignore).
bool burst_prefilter(const int16_t *iq250, int n_complex, int width_bins,
                     burst_prefilter_result_t *out);
