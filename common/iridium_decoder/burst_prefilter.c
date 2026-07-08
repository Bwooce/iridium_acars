// See burst_prefilter.h for the design rationale and gr-iridium
// grounding of every constant used here.

#include "burst_prefilter.h"
#include "fft_sc16_2048.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

// On-device: pull the real EXT_RAM_BSS_ATTR (PSRAM .ext_ram.bss placement)
// from esp_attr.h. WITHOUT this the empty fallback below leaves s_seg (8 KB)
// + s_oob (16 KB) = 24 KB in INTERNAL .bss, which starves the boot-time
// 144 KB DMA-internal reserve -> esp_psram abort + crash-loop (caught by the
// RAW-smoke GOLDEN 2026-07-08). Same fix a prior session applied; the port
// from the pre-Path-A branch reintroduced the omission. Host builds have no
// esp_attr.h, so the #ifndef fallback (empty = normal .bss) applies there.
#ifdef ESP_PLATFORM
#include "esp_attr.h"
#endif

#ifndef EXT_RAM_BSS_ATTR
#define EXT_RAM_BSS_ATTR
#endif

// ---- gr-iridium-grounded constants (see header for citations) ----

// Symbols/sec = 25000 (iridium.h:10); working rate 250 ksps
// (direct_if_decim.h:31, DIDECIM 10×) ⇒ 10 samples/symbol.
#define PF_SPS 10

// Smallest decodable frame class: MIN_FRAME_LENGTH_SIMPLEX = 80 symbols
// (iridium.h:21). Using the SIMPLEX minimum (not NORMAL's 131) keeps the
// duration gate recall-safe for every frame class. burst_downmix drops
// bursts shorter than this without decoding (burst_downmix_impl.cc:502).
#define PF_MIN_FRAME_SYMBOLS 80
#define PF_MIN_FRAME_SAMPLES (PF_MIN_FRAME_SYMBOLS * PF_SPS) // 800

// Duration-gate boxcar: smooth the power envelope over one symbol so the
// QPSK/RRC amplitude dips at symbol transitions don't fragment a real
// burst's active span.
#define PF_ENV_WIN PF_SPS

// Spectral gate: 2048-pt FFT at 250 ksps ⇒ 122.07 Hz/bin.
#define PF_FFT_N FFT_SC16_2048_N // 2048
#define PF_FS_HZ 250000.0
#define PF_BIN_HZ (PF_FS_HZ / (double)PF_FFT_N)

// Iridium channel width = burst_width = 40 kHz (iridium-extractor:126,
// iridium_extractor_flowgraph.py:30). Half-width ±20 kHz around DC (the
// tagger centres every burst; rotate-to-dc has already done so here).
#define PF_BURST_WIDTH_HZ 40000.0
// round(20000 / 122.07) = 164 bins each side ⇒ 329-bin in-band channel.
#define PF_HALF_BAND_BINS 164

// Gate-0 spectral width bound (tagger-FFT bins). One Iridium channel is
// ~34 bins at 2.5 MSPS / 2048-pt tagger FFT (fft_burst_tagger.h:111,
// dsp_processor.c:61). A real burst is ~1 channel; broadband RFI is far
// wider. Reject above ~3 channels. CONSERVATIVE by design: 100 bins is
// ~3× one channel and ~2× the worker PQ's BURST_NARROW_MAX_BINS (48)
// demotion line, so a ~1-channel real burst is never width-rejected.
#define PF_MAX_WIDTH_BINS 100

// Detection threshold. gr-iridium default = 18 dB (iridium-extractor:130);
// in our baseline-scale domain the equivalent is 14 dB (memory note
// "Tagger threshold scale: ours ≈ gri − 4 dB"; these host tests run the
// tagger at 14 dB). Same threshold the tagger applied to admit the burst.
#define PF_THRESH_DB 14.0

static inline int16_t pf_sat16(int32_t x)
{
    if (x > 32767) return 32767;
    if (x < -32768) return -32768;
    return (int16_t)x;
}

// Static scratch (single-threaded worker / host test, like burst_pipeline).
static EXT_RAM_BSS_ATTR int16_t s_seg[2 * PF_FFT_N]; // DC-removed FFT input
static EXT_RAM_BSS_ATTR int64_t s_oob[PF_FFT_N];     // out-of-band powers

static int cmp_i64(const void *a, const void *b)
{
    int64_t x = *(const int64_t *)a, y = *(const int64_t *)b;
    return (x > y) - (x < y);
}

bool burst_prefilter(const int16_t *iq250, int n_complex, int width_bins,
                     burst_prefilter_result_t *out)
{
    burst_prefilter_result_t r;
    memset(&r, 0, sizeof(r));
    r.width_bins = width_bins;

    if (!iq250 || n_complex < PF_ENV_WIN) {
        if (out) *out = r;
        return false;
    }

    // ---- Gate 0: spectral width (tagger metadata, O(1)) ----
    // width_bins <= 0 == unmeasured → treat as narrow (never reject). A
    // burst wider than ~3 Iridium channels is broadband RFI, not a frame.
    r.width_ok = (width_bins <= 0) || (width_bins <= PF_MAX_WIDTH_BINS);
    if (!r.width_ok) {
        if (out) *out = r;
        return false; // broadband RFI — far wider than one Iridium channel
    }

    // ---- Gate 1: active-envelope duration (time domain, O(N)) ----
    // Per-sample instantaneous power (int64: full-scale |x|^2 ≈ 2.15e9
    // exceeds int32). Boxcar-smoothed over one symbol.
    int64_t peak_env  = 0;
    int64_t floor_env = INT64_MAX;
    int     n_env     = n_complex - PF_ENV_WIN + 1;

    // Running boxcar sum of power.
    int64_t win_sum = 0;
    for (int k = 0; k < PF_ENV_WIN; k++) {
        int32_t re = iq250[k * 2 + 0];
        int32_t im = iq250[k * 2 + 1];
        win_sum += (int64_t)re * re + (int64_t)im * im;
    }
    // First pass: find peak and floor of the smoothed envelope.
    {
        int64_t s = win_sum;
        for (int i = 0; i < n_env; i++) {
            if (i > 0) {
                int32_t re_out = iq250[(i - 1) * 2 + 0];
                int32_t im_out = iq250[(i - 1) * 2 + 1];
                int32_t re_in  = iq250[(i + PF_ENV_WIN - 1) * 2 + 0];
                int32_t im_in  = iq250[(i + PF_ENV_WIN - 1) * 2 + 1];
                s -= (int64_t)re_out * re_out + (int64_t)im_out * im_out;
                s += (int64_t)re_in * re_in + (int64_t)im_in * im_in;
            }
            if (s > peak_env) peak_env = s;
            if (s < floor_env) floor_env = s;
        }
    }
    if (peak_env <= 0) {
        // Silent window — nothing to decode.
        if (out) *out = r;
        return false;
    }
    if (floor_env < 1) floor_env = 1;

    // Threshold = geometric mean of floor and peak envelope power (the
    // log-domain midpoint / half-power-in-dB crossing — a standard,
    // per-burst pulse-width definition, not a tuned level). Samples this
    // loud are "on"; the count approximates the burst's active duration.
    double thr_env    = sqrt((double)peak_env * (double)floor_env);
    int    active_len = 0;
    {
        int64_t s = win_sum;
        for (int i = 0; i < n_env; i++) {
            if (i > 0) {
                int32_t re_out = iq250[(i - 1) * 2 + 0];
                int32_t im_out = iq250[(i - 1) * 2 + 1];
                int32_t re_in  = iq250[(i + PF_ENV_WIN - 1) * 2 + 0];
                int32_t im_in  = iq250[(i + PF_ENV_WIN - 1) * 2 + 1];
                s -= (int64_t)re_out * re_out + (int64_t)im_out * im_out;
                s += (int64_t)re_in * re_in + (int64_t)im_in * im_in;
            }
            if ((double)s >= thr_env) active_len++;
        }
    }
    r.active_len = active_len;
    r.dur_ok     = (active_len >= PF_MIN_FRAME_SAMPLES);
    if (!r.dur_ok) {
        if (out) *out = r;
        return false; // short impulse — gr-iridium burst_downmix:502
    }

    // ---- Gate 2: integrated in-band channel SNR (one 2048-pt FFT) ----
    // Pick the highest-energy PF_FFT_N-sample segment so the burst (not
    // the ~16 ms noise post-padding) fills the FFT window. Running sum of
    // per-sample power over PF_FFT_N.
    int seg_start = 0;
    if (n_complex > PF_FFT_N) {
        int64_t seg_sum = 0;
        for (int k = 0; k < PF_FFT_N; k++) {
            int32_t re = iq250[k * 2 + 0];
            int32_t im = iq250[k * 2 + 1];
            seg_sum += (int64_t)re * re + (int64_t)im * im;
        }
        int64_t best = seg_sum;
        for (int start = 1; start + PF_FFT_N <= n_complex; start++) {
            int32_t re_out = iq250[(start - 1) * 2 + 0];
            int32_t im_out = iq250[(start - 1) * 2 + 1];
            int32_t re_in  = iq250[(start + PF_FFT_N - 1) * 2 + 0];
            int32_t im_in  = iq250[(start + PF_FFT_N - 1) * 2 + 1];
            seg_sum -= (int64_t)re_out * re_out + (int64_t)im_out * im_out;
            seg_sum += (int64_t)re_in * re_in + (int64_t)im_in * im_in;
            if (seg_sum > best) {
                best      = seg_sum;
                seg_start = start;
            }
        }
    }
    r.fft_seg_start = seg_start;

    int seg_n = n_complex - seg_start;
    if (seg_n > PF_FFT_N) seg_n = PF_FFT_N;

    // Copy segment, DC-remove (matches pipeline_head step 0 — kills the
    // residual RTL DC spike that would otherwise fake an in-band peak),
    // zero-pad the tail if the window is short.
    int64_t sum_re = 0, sum_im = 0;
    for (int i = 0; i < seg_n; i++) {
        sum_re += iq250[(seg_start + i) * 2 + 0];
        sum_im += iq250[(seg_start + i) * 2 + 1];
    }
    int16_t dc_re = (int16_t)(sum_re / seg_n);
    int16_t dc_im = (int16_t)(sum_im / seg_n);
    for (int i = 0; i < seg_n; i++) {
        s_seg[i * 2 + 0] = pf_sat16((int32_t)iq250[(seg_start + i) * 2 + 0] - dc_re);
        s_seg[i * 2 + 1] = pf_sat16((int32_t)iq250[(seg_start + i) * 2 + 1] - dc_im);
    }
    for (int i = seg_n; i < PF_FFT_N; i++) {
        s_seg[i * 2 + 0] = 0;
        s_seg[i * 2 + 1] = 0;
    }

    fft_sc16_2048_init();
    fft_sc16_2048(s_seg);

    // Per-bin power. Natural order: bin 0 = DC, bins [1..N/2-1] positive
    // freq, bins [N/2..N-1] negative freq. In-band = DC ± burst_width/2.
    int64_t p_in  = 0;
    int     n_oob = 0;
    for (int b = 0; b < PF_FFT_N; b++) {
        int32_t re      = s_seg[b * 2 + 0];
        int32_t im      = s_seg[b * 2 + 1];
        int64_t pw      = (int64_t)re * re + (int64_t)im * im;
        bool    in_band = (b <= PF_HALF_BAND_BINS) ||
                       (b >= PF_FFT_N - PF_HALF_BAND_BINS);
        if (in_band) {
            p_in += pw;
        } else {
            s_oob[n_oob++] = pw;
        }
    }
    int n_in = 2 * PF_HALF_BAND_BINS + 1; // 329

    // Robust noise floor = median of out-of-band bin power (ignores RRC
    // sidelobes / stray spurs a mean would inflate).
    qsort(s_oob, n_oob, sizeof(int64_t), cmp_i64);
    int64_t n0 = s_oob[n_oob / 2];

    if (n0 <= 0) {
        // Signal-free band is dead silent (a pure tone dominates
        // everything) — treat as strong signal, let the full pipeline
        // judge rather than dividing by zero.
        r.channel_snr_db = 99.0;
        r.snr_ok         = true;
    } else {
        double expected_in_noise = (double)n0 * (double)n_in;
        double snr_lin           = (double)p_in / expected_in_noise;
        r.channel_snr_db         = 10.0 * log10(snr_lin > 1e-12 ? snr_lin : 1e-12);
        r.snr_ok                 = (r.channel_snr_db >= PF_THRESH_DB);
    }

    r.accept = r.width_ok && r.dur_ok && r.snr_ok;
    if (out) *out = r;
    return r.accept;
}
