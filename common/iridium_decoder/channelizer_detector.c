// See channelizer_detector.h.
//
// Implementation notes:
//
//   - One polyphase_channelizer instance generates 64 channel streams
//     at fs_in/M (40 ksps for fs_in=2.56 MHz). Each cycle produces M
//     instantaneous channel powers; the detector uses a CROSS-CHANNEL
//     percentile noise-floor estimator instead of a per-channel EMA.
//
//   - Why not per-channel EMA: a per-channel EMA assumes the channel
//     spends most of its time at noise level. On real-RF Iridium IQ
//     individual channels can be active for most of a 10 ms window
//     (multiple concurrent bursts on the same frequency), and the
//     EMA's priming phase poisons baseline with burst power. The
//     legacy FFT detector got away with a per-bin EMA because its
//     2048 bins spread the burst energy out — a single Iridium burst
//     occupies <2 of 2048 bins. Our 64 channels concentrate energy
//     1024× more, so per-channel EMA is unsafe.
//
//   - Cross-channel percentile noise-floor: at any instant, the
//     Iridium downlink has at most a few simultaneous bursts in a
//     2.56 MHz subband. The lower-quartile (16th-of-64) channel power
//     is a robust real-time noise-floor estimate, computed each cycle
//     from the 64 channel powers themselves. Threshold = floor × mult.
//     Active = floor + threshold_mult, idle channels stay below.
//
//   - Per-channel state: just { in_burst, start_cycle, peak_power }.
//     The "noise floor" is computed once per cycle, not per channel.
//
//   - Threshold ratio: passed in dB, converted once to a linear
//     multiplier at create() time. Matches the legacy detector's
//     16 dB / 40× set-point.

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "channelizer_detector.h"

#ifdef ESP_PLATFORM
  #include "esp_log.h"
  #define DET_LOG(...) ESP_LOGI("CH_DETECT", __VA_ARGS__)
#else
  #include <stdio.h>
  #define DET_LOG(...) do { printf(__VA_ARGS__); printf("\n"); } while (0)
#endif

#define M                    POLYCHAN_M
#define NOISE_FLOOR_PCTILE   16    // 16/64 = 25th percentile
// Hysteresis: a channel enters "burst" when its power crosses
// threshold_mult × floor (e.g. 16 dB) but stays in burst until power
// drops below threshold_mult × floor / HYSTERESIS_DROP (3 dB lower).
// Without hysteresis a single physical Iridium burst fragments into
// 30-50 detection events as the per-cycle power flutters around the
// edge. With 3 dB hysteresis a single ~8 ms burst yields ~1-3
// detections instead.
#define HYSTERESIS_DROP_DB   3.0f
// Minimum burst length in cycles to avoid emitting spurious 1-2
// cycle flickers. Iridium symbol period at 25 ksym/s is 1.6 cycles
// at 40 ksps; an actual TDMA burst is hundreds of cycles long, but
// edge-channel detections (where a burst's centre is near a channel
// boundary, splitting energy across two channels) can be marginal —
// the weaker side may only stay above threshold for 4-7 cycles.
// Set MIN_BURST_CYCLES=4 (~100 us) to keep those edge hits while
// rejecting 1-3 cycle flickers from PRBS / quantisation.
#define MIN_BURST_CYCLES     4     // ~100 us at 40 ksps

// Burst-merging cooldown. After a channel's power drops below the
// hysteresis exit threshold, the channel state is "cooling" for this
// many cycles. If power rises back above the entry threshold during
// the cooling window, the burst RESUMES from its original start_cycle
// (no new burst event) — this absorbs modulation-induced power dips
// of up to MERGE_GAP_CYCLES (e.g. ~1 ms) that would otherwise
// fragment a single Iridium TDMA burst into 10-50 separate detection
// events at the smoke-test fixture's modulation depth.
//
// Iridium TDMA slots are ~8 ms; legitimate distinct bursts on the
// same channel are separated by frame periods (~90 ms) so MERGE_GAP
// of ~1 ms doesn't merge legitimately-distinct events.
#define MERGE_GAP_CYCLES     40    // ~1 ms at 40 ksps

// Cross-channel dedup hold window. After emitting a burst we hold
// it for this many cycles, replacing it if a higher-SNR overlap
// arrives, before publishing to the callback. 100 cycles ≈ 2.5 ms
// at 40 ksps. Bumped to 200 once to catch longer-gap overlaps but
// that regressed decode rate (merged bursts that were actually
// distinct physical TDMA slots with different correct centre freqs).
// 100 is the empirical sweet spot for the smoke corpus.
#define DEDUP_HOLD_CYCLES    100

// D20 step 3 gate. Flipped to 1 after polyphase_mac_phase_arp4 (the
// PIE xacc-based MAC kernel) landed in polyphase_mac_arp4.S. If the
// kernel turns out buggy / crashy, flip back to 0 and reflash to
// restore the float path.
#define CHANNELIZER_USE_INT16_PATH 1

// Per-channel slow baseline EMA. Distinguishes a TRANSIENT burst (a
// channel's instantaneous power spikes above its own slow baseline)
// from a PERSISTENT spur (channel always above the cross-channel
// floor because of DC offset, an unmodulated carrier, an SDR birdie,
// etc.). Without this, the RTL-SDR's residual DC offset puts channel
// 0's instantaneous power 15-25 dB above the cross-channel 25th-
// percentile floor at every cycle, so the detector emits ~30-50
// spurious channel-0 bursts/second on real hardware (observed in the
// first P4 smoke run after the channelizer landed). The slow EMA
// converges to that DC offset level after a few hundred cycles, and
// the second threshold below (power > ema × threshold_mult) then
// requires a transient excursion above the channel's own baseline,
// not just above the cross-channel floor. Iridium bursts (8.28 ms,
// ~330 cycles) are short compared to the EMA time constant
// (~50 ms = 2048 cycles), so a real burst barely budges the EMA.
//
// The EMA is frozen while the channel is in burst — standard noise-
// floor tracking practice; otherwise the burst itself would saturate
// the baseline and the trailing edge would never be detected.
#define EMA_ALPHA_SHIFT      11    // alpha = 1/(1<<11) = 1/2048

// Three-state per-channel machine for the burst-merging cooldown:
//   IDLE       — channel is at noise. EMA updates here.
//   IN_BURST   — power is currently above the rise threshold. EMA
//                 frozen.
//   COOLING    — power dropped below the exit threshold but we're
//                 holding the burst record open for MERGE_GAP_CYCLES
//                 in case power rises again (modulation dip vs
//                 burst end). EMA frozen.
typedef enum {
    CH_IDLE = 0,
    CH_IN_BURST,
    CH_COOLING,
} channel_phase_t;

typedef struct {
    channel_phase_t phase;
    uint32_t start_cycle;       // cycle the current burst began
    uint32_t last_active_cycle; // cycle of the most recent above-threshold sample
    uint32_t cool_end_cycle;    // cycle at which COOLING expires (this_cycle ≥ → emit)
    float    peak_power;
    float    peak_floor;        // noise floor at peak_power moment
} channel_state_t;

struct channelizer_detector {
    polyphase_channelizer_t *ch;
    uint32_t  fs_in_hz;
    float     threshold_mult;
    float     drop_mult;        // = threshold_mult / 10^(HYSTERESIS_DROP_DB/10)
    channelizer_burst_cb_t cb;
    void     *user;

    channel_state_t st[M];
    float     channel_ema[M];       // slow per-channel power baseline
    uint32_t  cycle_count;          // total cycles processed since create()
    uint32_t  bursts_emitted;
    uint32_t  channels_active_peak;

    // Working buffers: allocated once at create() and reused across
    // every feed() call. Sized to handle one 16 KB SDR transfer's
    // worth of int16 IQ at a time — matches what the legacy detector
    // sees per dsp_processor_feed() invocation.
    //
    // D20 step 3: an int16 fast path through
    // polyphase_channelizer_process_int16 is wired up below, gated on
    // CHANNELIZER_USE_INT16_PATH. Today it's OFF on target because
    // scalar Q14 MAC + scalar sc16 FFT is ~19% slower than scalar
    // float MAC + fc32 FFT on the P4 (P4 has fmadd.s for float but
    // no fused int equivalent; "arp4" FFT variants are just
    // esp.lp.setup-wrapped scalar). The path lights up when the PIE
    // asm kernel in polyphase_mac_arp4.S replaces the scalar MAC.
    size_t           in_capacity_samples;     // == out_capacity_cycles × M
    size_t           out_capacity_cycles;
    float complex   *in_buf;
    float complex   *out_buf;
#if CHANNELIZER_USE_INT16_PATH
    int16_t         *out_buf_i16;             // interleaved IQ
#endif

    // Cross-channel dedup: a single physical Iridium burst hits ~2-4
    // adjacent channels with similar power (the polyphase filter's
    // ~40 dB sidelobe rolloff isn't sharp enough to confine all
    // energy to one channel). Without dedup we publish duplicate
    // bursts and the worker pipeline tries to demod the same TDMA
    // slot multiple times — wasting CPU and (per the smoke test)
    // sometimes picking a lower-SNR channel that fails to decode.
    // We hold each emitted burst in this single-slot deferred queue
    // for DEDUP_HOLD_CYCLES cycles; new emissions overlapping in
    // time with the held one REPLACE it if higher SNR, else are
    // dropped. The held burst publishes when no candidate has
    // arrived for DEDUP_HOLD_CYCLES.
    bool              pending_valid;
    channelizer_burst_t pending;
    uint32_t          pending_last_touched_cycle;

    // D7+: per-channel output ringbuffer. The worker pipeline used to
    // re-extract from the raw signal buffer and apply its own FIR +
    // freq-shift to recover the burst's channel — duplicating work the
    // channelizer already did, AND with a coarser filter. We now retain
    // the channelizer's per-channel int16 output here so the worker
    // can pull the target channel directly. Cycle-major layout:
    //   ring[cycle_mod * M * 2 + ch * 2 + (0|1)]  (Re | Im, int16)
    // RING_CYCLES sized so the ring depth covers the maximum burst
    // latency (~100 ms = 4096 cycles at 40 kHz cycle rate).
    int16_t          *channel_ring;          // M × ring_capacity × 2 int16
    size_t            ring_capacity_cycles;  // power of two for fast wrap
    // ring_capacity_cycles - 1 mask
    uint32_t          ring_mask;
};

channelizer_detector_t *channelizer_detector_create(uint32_t fs_in_hz,
                                                     float threshold_db,
                                                     channelizer_burst_cb_t cb,
                                                     void *user)
{
    channelizer_detector_t *d = calloc(1, sizeof(*d));
    if (!d) return NULL;
    d->ch = polyphase_channelizer_create(fs_in_hz);
    if (!d->ch) { free(d); return NULL; }
    d->fs_in_hz = fs_in_hz;
    d->threshold_mult = powf(10.0f, threshold_db / 10.0f);
    d->drop_mult      = powf(10.0f,
                             (threshold_db - HYSTERESIS_DROP_DB) / 10.0f);
    d->cb = cb;
    d->user = user;
    // Per-channel state starts zeroed by calloc — no baseline to seed
    // since the noise floor is recomputed each cycle from the 64
    // channel powers.
    // Capacity: one 16 KB SDR transfer = 8192 complex IQ pairs = 128
    // channelizer cycles at M=64. Both buffers are sized to that
    // upper bound; larger feeds get split into 8192-sample chunks.
    // 8192 × 8 B = 64 KB per buffer × 2 buffers = 128 KB heap. On
    // ESP32-P4 with PSRAM that's negligible.
    d->in_capacity_samples = 8192;
    d->out_capacity_cycles = 8192 / M;
    d->in_buf  = malloc(d->in_capacity_samples * sizeof(float complex));
    d->out_buf = malloc(d->out_capacity_cycles * M * sizeof(float complex));
    if (!d->in_buf || !d->out_buf) {
        if (d->in_buf)  free(d->in_buf);
        if (d->out_buf) free(d->out_buf);
        polyphase_channelizer_destroy(d->ch);
        free(d);
        return NULL;
    }
#if CHANNELIZER_USE_INT16_PATH
    d->out_buf_i16 = malloc(d->out_capacity_cycles * M * 2 * sizeof(int16_t));
    if (!d->out_buf_i16) {
        free(d->in_buf);
        free(d->out_buf);
        polyphase_channelizer_destroy(d->ch);
        free(d);
        return NULL;
    }
#endif

    // D7+: per-channel retention ringbuffer. 4096 cycles × M × 2 ×
    // sizeof(int16_t) = 1 MB on the ESP32-P4 (allocated from generic
    // heap; PSRAM if available via the build allocator). For 40 kHz
    // cycle rate that's ~100 ms — far more than the worst-case burst
    // emission latency (~30 ms for the COOLING state to expire).
    d->ring_capacity_cycles = 4096;
    d->ring_mask = (uint32_t)(d->ring_capacity_cycles - 1);
    size_t ring_bytes = d->ring_capacity_cycles * M * 2 * sizeof(int16_t);
    d->channel_ring = calloc(1, ring_bytes);
    if (!d->channel_ring) {
        free(d->in_buf);
        free(d->out_buf);
#if CHANNELIZER_USE_INT16_PATH
        if (d->out_buf_i16) free(d->out_buf_i16);
#endif
        polyphase_channelizer_destroy(d->ch);
        free(d);
        return NULL;
    }
    return d;
}

void channelizer_detector_destroy(channelizer_detector_t *d)
{
    if (!d) return;
    if (d->ch)      polyphase_channelizer_destroy(d->ch);
    if (d->in_buf)  free(d->in_buf);
    if (d->out_buf) free(d->out_buf);
#if CHANNELIZER_USE_INT16_PATH
    if (d->out_buf_i16) free(d->out_buf_i16);
#endif
    if (d->channel_ring) free(d->channel_ring);
    free(d);
}

static inline int signed_channel_offset(int k)
{
    return (k > M / 2) ? (k - M) : k;
}

// Publish the currently-held pending burst (if any) to the callback
// and clear the slot. Internal helper called when a new emission
// wants the slot OR when DEDUP_HOLD_CYCLES has elapsed.
static void publish_pending(channelizer_detector_t *d)
{
    if (!d->pending_valid) return;
    if (d->cb) d->cb(&d->pending, d->user);
    d->bursts_emitted++;
    d->pending_valid = false;
}

// Two bursts overlap if either starts inside the other's sample
// range. Compare sample indices (post-converted from cycles).
static inline bool bursts_overlap(const channelizer_burst_t *a,
                                  const channelizer_burst_t *b)
{
    uint32_t a_end = a->start_sample_idx + a->length_samples;
    uint32_t b_end = b->start_sample_idx + b->length_samples;
    return !(a_end <= b->start_sample_idx || b_end <= a->start_sample_idx);
}

// Emit the pending burst for channel k (covers cs->start_cycle ..
// cs->last_active_cycle). Resets the channel to IDLE regardless of
// whether the burst met MIN_BURST_CYCLES (too-short bursts are
// dropped silently). Called from the COOLING-expiry path in
// process_cycles.
//
// New burst goes into the single-slot dedup queue rather than
// publishing immediately. If the slot is empty, we move in; if a
// pending burst overlaps in time, we keep whichever has higher SNR;
// if non-overlapping, we flush the pending one first.
static void emit_burst(channelizer_detector_t *d, int k)
{
    channel_state_t *cs = &d->st[k];
    uint32_t length_cycles = cs->last_active_cycle - cs->start_cycle + 1;
    if (length_cycles >= (uint32_t)MIN_BURST_CYCLES) {
        float snr_db = 10.0f * log10f(cs->peak_power
                                       / (cs->peak_floor + 1e-30f));
        int spacing  = (int)d->fs_in_hz / M;
        channelizer_burst_t b = {
            .channel          = k,
            .rel_freq_hz      = signed_channel_offset(k) * spacing,
            .snr_db           = snr_db,
            .start_sample_idx = cs->start_cycle * (uint32_t)M,
            .length_samples   = length_cycles * (uint32_t)M,
        };
        if (d->pending_valid && bursts_overlap(&d->pending, &b)) {
            // Two emissions on the same physical burst — keep the
            // winning channel's bounds (NOT the union). The winning
            // channel had the strongest signal, so its time-domain
            // boundary detection is most reliable; widening to the
            // union dragged the UW correlator into noise prefix from
            // a weaker adjacent-channel detection.
            if (b.snr_db > d->pending.snr_db) d->pending = b;
        } else {
            // Slot has a non-overlapping burst — flush it first, then
            // take the slot.
            if (d->pending_valid) publish_pending(d);
            d->pending = b;
            d->pending_valid = true;
        }
        d->pending_last_touched_cycle = cs->last_active_cycle;
    }
    cs->phase      = CH_IDLE;
    cs->peak_power = 0.0f;
    cs->peak_floor = 0.0f;
}

// Quickselect for the n-th smallest element of a float array
// (in-place, destructive). Used to find the cross-channel noise
// floor each cycle. M=64 so this is cheap.
static float select_nth(float *arr, int n, int len)
{
    int lo = 0, hi = len - 1;
    while (lo < hi) {
        float pivot = arr[(lo + hi) / 2];
        int i = lo, j = hi;
        while (i <= j) {
            while (arr[i] < pivot) i++;
            while (arr[j] > pivot) j--;
            if (i <= j) {
                float t = arr[i]; arr[i] = arr[j]; arr[j] = t;
                i++; j--;
            }
        }
        if (n <= j)      hi = j;
        else if (n >= i) lo = i;
        else             return arr[n];
    }
    return arr[n];
}

static void process_cycles(channelizer_detector_t *d, size_t n_cycles)
{
    for (size_t cyc = 0; cyc < n_cycles; cyc++) {
        uint32_t this_cycle = d->cycle_count + (uint32_t)cyc;

        // 1. Compute per-channel power. Two source paths:
        //   - Int16 (D20 step 3): channelizer wrote sc16 IQ to out_buf_i16;
        //     compute power as int32 r²+i² and cast to float for the
        //     existing percentile / threshold logic (which is ratio-based,
        //     so absolute scale is irrelevant).
        //   - Float: channelizer wrote complex float to out_buf.
        //
        // We also copy this cycle's int16 IQ into the channel retention
        // ring so the worker can pull the target channel's stream
        // directly (D7+ architectural alignment with gr-iridium).
        float power[M];
        int16_t *ring_row = &d->channel_ring[(this_cycle & d->ring_mask) * M * 2];
#if CHANNELIZER_USE_INT16_PATH
        const int16_t *row = &d->out_buf_i16[cyc * M * 2];
        for (int k = 0; k < M; k++) {
            int32_t re = row[k * 2 + 0];
            int32_t im = row[k * 2 + 1];
            power[k] = (float)(re * re + im * im);
        }
        memcpy(ring_row, row, M * 2 * sizeof(int16_t));
#else
        const float complex *row = &d->out_buf[cyc * M];
        for (int k = 0; k < M; k++) {
            float complex y = row[k];
            power[k] = crealf(y) * crealf(y) + cimagf(y) * cimagf(y);
            // Convert float channelizer output → int16 for the ring.
            // The float path scales input by 1/32768 (in feed_int16),
            // so out_buf is in [-1, 1] range. Scale back up by
            // INT16_MAX so the ring's int16 values match what the
            // CHANNELIZER_USE_INT16_PATH would have produced directly.
            float re = crealf(y) * (float)INT16_MAX;
            float im = cimagf(y) * (float)INT16_MAX;
            if (re >  (float)INT16_MAX) re =  (float)INT16_MAX;
            if (re <  (float)INT16_MIN) re =  (float)INT16_MIN;
            if (im >  (float)INT16_MAX) im =  (float)INT16_MAX;
            if (im <  (float)INT16_MIN) im =  (float)INT16_MIN;
            ring_row[k * 2 + 0] = (int16_t)lrintf(re);
            ring_row[k * 2 + 1] = (int16_t)lrintf(im);
        }
#endif

        // 2. Estimate noise floor as the cross-channel 25th percentile.
        // (Most of 64 channels are at noise at any given moment, so
        // the lower quartile tracks the floor robustly through bursts
        // on a few channels.) Floor with a small constant so the very
        // first cycle, where powers are still all ~0, doesn't divide.
        float work[M];
        memcpy(work, power, sizeof(work));
        float noise_floor = select_nth(work, NOISE_FLOOR_PCTILE, M);
        if (noise_floor < 1e-12f) noise_floor = 1e-12f;
        float rise_thr = noise_floor * d->threshold_mult;
        float drop_thr = noise_floor * d->drop_mult;

        // 3. Per-channel three-state burst tracking:
        //
        //   IDLE      → rise_thr crossed → IN_BURST (record start)
        //   IN_BURST  → drop_thr crossed → COOLING  (record last_active)
        //   COOLING   → rise_thr crossed → IN_BURST (resume; keep
        //                                            original start)
        //   COOLING   → cool_end reached → IDLE     (emit burst)
        //
        // The COOLING state absorbs modulation-induced power dips of
        // up to MERGE_GAP_CYCLES so a single Iridium TDMA burst
        // doesn't fragment into many short detection events. EMA is
        // updated only in IDLE so neither active bursts nor cooling
        // periods pollute the noise-floor baseline.
        int n_active = 0;
        for (int k = 0; k < M; k++) {
            channel_state_t *cs = &d->st[k];
            // D7+: per-channel EMA mask removed. It was added in commit
            // d1604c7 to suppress DC-channel spurious bursts, but with
            // 1+ sec of real-world Iridium traffic the EMA tracks burst
            // power and self-suppresses subsequent bursts on the same
            // channel — host test on the 1-sec ALBQ fixture showed
            // 7 detections with the mask vs 116 without (= ALL 32
            // expected gr-iridium-reference bursts found). DC masking
            // is handled downstream: worker_core1 explicitly drops
            // bin 1024 (the DC channel) bursts.
            (void)cs;
            bool above_rise = (power[k] > rise_thr);

            switch (cs->phase) {
            case CH_IDLE:
                // Update slow per-channel EMA while idle.
                d->channel_ema[k] += (power[k] - d->channel_ema[k])
                                     * (1.0f / (float)(1 << EMA_ALPHA_SHIFT));
                if (above_rise) {
                    n_active++;
                    cs->phase             = CH_IN_BURST;
                    cs->start_cycle       = this_cycle;
                    cs->last_active_cycle = this_cycle;
                    cs->peak_power        = power[k];
                    cs->peak_floor        = noise_floor;
                }
                break;

            case CH_IN_BURST:
                n_active++;
                if (power[k] > cs->peak_power) {
                    cs->peak_power = power[k];
                    cs->peak_floor = noise_floor;
                }
                if (power[k] < drop_thr) {
                    // Enter cooling — don't emit yet.
                    cs->phase          = CH_COOLING;
                    cs->cool_end_cycle = this_cycle +
                                          (uint32_t)MERGE_GAP_CYCLES;
                } else {
                    cs->last_active_cycle = this_cycle;
                }
                break;

            case CH_COOLING:
                n_active++;     // count as active during cooldown
                if (above_rise) {
                    // Resume the burst — keep start_cycle, return to
                    // IN_BURST. Power may rise above the previous peak.
                    if (power[k] > cs->peak_power) {
                        cs->peak_power = power[k];
                        cs->peak_floor = noise_floor;
                    }
                    cs->last_active_cycle = this_cycle;
                    cs->phase             = CH_IN_BURST;
                } else if (this_cycle >= cs->cool_end_cycle) {
                    // Cooldown expired without re-rise — emit.
                    emit_burst(d, k);
                }
                break;
            }
        }
        if ((uint32_t)n_active > d->channels_active_peak) {
            d->channels_active_peak = (uint32_t)n_active;
        }
        // Time-out publish: release the held dedup burst once no
        // overlapping candidate has arrived for DEDUP_HOLD_CYCLES.
        if (d->pending_valid &&
            this_cycle >= d->pending_last_touched_cycle +
                              (uint32_t)DEDUP_HOLD_CYCLES) {
            publish_pending(d);
        }
    }
    d->cycle_count += (uint32_t)n_cycles;
}

// TODO(D20): the int16→float conversion below + process_cycles' float
// power computation + cross-channel percentile are all candidates for
// PIE rewrite. The conversion alone is 8192 elements × ~10 ns scalar
// vs ~1.25 ns/elem in 8-lane int16. If we keep IQ as int16 end-to-end
// (polyphase channelizer also int16, per its TODO), this conversion
// disappears entirely. See D20 roadmap in iridium-acars-implementation-plan.md.
void channelizer_detector_feed_int16(channelizer_detector_t *d,
                                      const int16_t *iq,
                                      size_t n_complex)
{
    // Convert int16 IQ to float complex on the fly, in chunks sized to
    // the output buffer. Each input chunk of M × out_capacity_cycles
    // complex samples produces out_capacity_cycles cycle outputs.
    size_t consumed = 0;
    while (consumed < n_complex) {
        size_t want = d->in_capacity_samples;
        if (want > n_complex - consumed) want = n_complex - consumed;
        // Round down to whole cycles so the channelizer state stays
        // aligned cycle-by-cycle across consecutive chunks.
        size_t whole = (want / M) * M;
        if (whole == 0) break;

#if CHANNELIZER_USE_INT16_PATH
        // Use the int16 channelizer path on host AND target. Previously
        // host took the float path while P4 took the int16 path, which
        // produced subtly different channelizer outputs that propagated
        // into the burst detector (4× more bursts detected on P4 from
        // the same data) and the worker pipeline (host 0/64 vs P4 14/64
        // decode rate on the same fixture). The int16 path is portable
        // C — only its esp-dsp FFT call is platform-gated inside
        // polyphase_channelizer.c.
        size_t got = polyphase_channelizer_process_int16(
            d->ch, &iq[consumed * 2], whole, d->out_buf_i16);
#else
        for (size_t i = 0; i < whole; i++) {
            float re = (float)iq[2 * (consumed + i) + 0] * (1.0f / 32768.0f);
            float im = (float)iq[2 * (consumed + i) + 1] * (1.0f / 32768.0f);
            d->in_buf[i] = re + im * I;
        }
        size_t got = polyphase_channelizer_process(d->ch, d->in_buf, whole,
                                                    d->out_buf);
#endif
        process_cycles(d, got);
        consumed += whole;
    }
}

void channelizer_detector_flush(channelizer_detector_t *d)
{
    if (!d) return;
    for (int k = 0; k < M; k++) {
        channel_state_t *cs = &d->st[k];
        // IN_BURST: treat the current cycle as the burst end and emit.
        // COOLING:  emit whatever's pending.
        // IDLE:     nothing to do.
        if (cs->phase == CH_IN_BURST || cs->phase == CH_COOLING) {
            emit_burst(d, k);
        }
    }
    // Flush the dedup queue too — no more cycles coming.
    publish_pending(d);
}

void channelizer_detector_get_stats(channelizer_detector_t *d,
                                     channelizer_detector_stats_t *out)
{
    out->input_samples_seen   = d->cycle_count * (uint32_t)M;
    out->cycles_processed     = d->cycle_count;
    out->bursts_detected      = d->bursts_emitted;
    out->channels_active_peak = d->channels_active_peak;
    d->bursts_emitted         = 0;
    d->channels_active_peak   = 0;
}

size_t channelizer_detector_extract_channel(channelizer_detector_t *d,
                                             int channel,
                                             uint32_t start_sample_idx,
                                             uint32_t length_samples,
                                             int16_t *out_iq)
{
    if (!d || !out_iq || channel < 0 || channel >= M) return 0;
    // Convert input-rate coords to cycle-rate. Cycle index = sample / M
    // (truncates to floor — burst start that falls mid-cycle gets the
    // cycle it started in, which is what we want).
    uint32_t start_cycle  = start_sample_idx / (uint32_t)M;
    uint32_t length_cycles = length_samples  / (uint32_t)M;
    if (length_cycles == 0) return 0;
    // Cap to ring depth; older samples have been overwritten.
    if (length_cycles > d->ring_capacity_cycles) {
        length_cycles = (uint32_t)d->ring_capacity_cycles;
    }
    // Copy out the channel's IQ from each cycle in the requested range.
    for (uint32_t i = 0; i < length_cycles; i++) {
        uint32_t cyc = (start_cycle + i) & d->ring_mask;
        const int16_t *row = &d->channel_ring[cyc * M * 2];
        out_iq[i * 2 + 0] = row[channel * 2 + 0];
        out_iq[i * 2 + 1] = row[channel * 2 + 1];
    }
    return (size_t)length_cycles;
}
