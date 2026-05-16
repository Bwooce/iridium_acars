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

typedef struct {
    bool     in_burst;
    uint32_t start_cycle;
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
    uint32_t  cycle_count;          // total cycles processed since create()
    uint32_t  bursts_emitted;
    uint32_t  channels_active_peak;

    // Working buffers: allocated once at create() and reused across
    // every feed() call. Sized to handle one 16 KB SDR transfer's
    // worth of int16 IQ at a time — matches what the legacy detector
    // sees per dsp_processor_feed() invocation.
    //   in_buf  : float complex, capacity in_capacity_samples
    //   out_buf : float complex, capacity (in_capacity_samples / M) × M
    size_t           in_capacity_samples;     // == out_capacity_cycles × M
    size_t           out_capacity_cycles;
    float complex   *in_buf;
    float complex   *out_buf;
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
    return d;
}

void channelizer_detector_destroy(channelizer_detector_t *d)
{
    if (!d) return;
    if (d->ch)      polyphase_channelizer_destroy(d->ch);
    if (d->in_buf)  free(d->in_buf);
    if (d->out_buf) free(d->out_buf);
    free(d);
}

static inline int signed_channel_offset(int k)
{
    return (k > M / 2) ? (k - M) : k;
}

static void emit_burst(channelizer_detector_t *d, int k, uint32_t cur_cycle)
{
    channel_state_t *cs = &d->st[k];
    if (!cs->in_burst) return;
    uint32_t length_cycles = cur_cycle - cs->start_cycle;
    // Drop too-short flickers without emitting (still resets state).
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
        if (d->cb) d->cb(&b, d->user);
        d->bursts_emitted++;
    }
    cs->in_burst   = false;
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
        const float complex *row = &d->out_buf[cyc * M];
        uint32_t this_cycle = d->cycle_count + (uint32_t)cyc;

        // 1. Compute per-channel power.
        float power[M];
        for (int k = 0; k < M; k++) {
            float complex y = row[k];
            power[k] = crealf(y) * crealf(y) + cimagf(y) * cimagf(y);
        }

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

        // 3. Per-channel threshold check + burst tracking with
        // hysteresis: enter at rise_thr, exit at drop_thr (3 dB lower).
        int n_active = 0;
        for (int k = 0; k < M; k++) {
            channel_state_t *cs = &d->st[k];
            if (cs->in_burst) {
                n_active++;
                if (power[k] > cs->peak_power) {
                    cs->peak_power = power[k];
                    cs->peak_floor = noise_floor;
                }
                if (power[k] < drop_thr) {
                    emit_burst(d, k, this_cycle);
                }
            } else {
                if (power[k] > rise_thr) {
                    n_active++;
                    cs->in_burst    = true;
                    cs->start_cycle = this_cycle;
                    cs->peak_power  = power[k];
                    cs->peak_floor  = noise_floor;
                }
            }
        }
        if ((uint32_t)n_active > d->channels_active_peak) {
            d->channels_active_peak = (uint32_t)n_active;
        }
    }
    d->cycle_count += (uint32_t)n_cycles;
}

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

        for (size_t i = 0; i < whole; i++) {
            float re = (float)iq[2 * (consumed + i) + 0] * (1.0f / 32768.0f);
            float im = (float)iq[2 * (consumed + i) + 1] * (1.0f / 32768.0f);
            d->in_buf[i] = re + im * I;
        }
        size_t got = polyphase_channelizer_process(d->ch, d->in_buf, whole,
                                                    d->out_buf);
        process_cycles(d, got);
        consumed += whole;
    }
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
