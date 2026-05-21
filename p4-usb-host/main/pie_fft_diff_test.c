// Target-side bit-exact diff harness for PIE FFT vs scalar reference.
// Task #67.
//
// Purpose: before attempting to swap dsps_fft2r_fc32_arp4 (PIE float
// FFT) into uw_correlator.c's matched-filter path (where prior attempts
// broke decode 58 → 3, per memory note feedback_pie_fft_swap_broke_uw_
// detection.md), produce quantitative output telling us EXACTLY how
// the two FFT implementations differ.
//
// The harness runs synthetic test cases at boot, logs per-test:
//   - peak bin position (must match for matched-filter correctness)
//   - max per-bin absolute difference
//   - RMS difference
//   - count of bins exceeding a tolerance
//
// If the two implementations are bit-equivalent (or within float epsilon
// for same-order butterflies), max_diff should be < 1e-5 and peak bins
// should match exactly. If they differ structurally (e.g. dsps's
// special-case N2 ≤ 2 tail), this harness will pinpoint where.

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "dsps_fft2r.h"

#define FFTN 2048
#define LOG2_FFTN 11

static const char *TAG = "PIE_FFT_DIFF";

// Scalar reference: identical algorithm to uw_correlator.c's radix2_fft_f32
// (standard radix-2 DIT). Kept self-contained here so the harness doesn't
// depend on uw_correlator internals.
static uint16_t s_ref_brev[FFTN];
static float    s_ref_tw_re[FFTN / 2];
static float    s_ref_tw_im[FFTN / 2];
static bool     s_ref_inited = false;

static void ref_fft_init(void)
{
    if (s_ref_inited) return;
    // Bit-reversal table.
    for (int i = 0; i < FFTN; i++) {
        unsigned x = (unsigned)i;
        unsigned r = 0;
        for (int b = 0; b < LOG2_FFTN; b++) {
            r = (r << 1) | (x & 1);
            x >>= 1;
        }
        s_ref_brev[i] = (uint16_t)r;
    }
    // Twiddles: w[k] = exp(-j·2π·k / N).
    for (int k = 0; k < FFTN / 2; k++) {
        double ang = -2.0 * M_PI * (double)k / (double)FFTN;
        s_ref_tw_re[k] = (float)cos(ang);
        s_ref_tw_im[k] = (float)sin(ang);
    }
    s_ref_inited = true;
}

static void ref_radix2_fft(float *re, float *im)
{
    ref_fft_init();
    // Bit reverse.
    for (int i = 0; i < FFTN; i++) {
        int j = s_ref_brev[i];
        if (j > i) {
            float t;
            t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }
    // DIT butterflies.
    for (int stride = 1; stride < FFTN; stride <<= 1) {
        int span = stride << 1;
        int step = (FFTN / 2) / stride;
        for (int k = 0; k < stride; k++) {
            float wr = s_ref_tw_re[k * step];
            float wi = s_ref_tw_im[k * step];
            for (int i = k; i < FFTN; i += span) {
                float xr = re[i + stride];
                float xi = im[i + stride];
                float tr = wr * xr - wi * xi;
                float ti = wr * xi + wi * xr;
                re[i + stride] = re[i] - tr;
                im[i + stride] = im[i] - ti;
                re[i]          = re[i] + tr;
                im[i]          = im[i] + ti;
            }
        }
    }
}

typedef struct {
    const char *name;
    int   peak_ref;
    int   peak_pie;
    float peak_mag2_ref;
    float peak_mag2_pie;
    float max_abs_diff;
    int   max_abs_diff_bin;
    float rms_diff;
    int   bins_over_tol;
    float tol;
} pie_fft_diff_result_t;

// PIE float FFT scratch + twiddle table, lazy-init.
static float *s_pie_w_table = NULL;
static bool   s_pie_inited  = false;
static void pie_fft_init(void)
{
    if (s_pie_inited) return;
    // dsps_fft2r_init_fc32(NULL, N) uses internal heap; supply our own
    // so we control placement (internal SRAM for PIE access).
    s_pie_w_table = (float *)heap_caps_aligned_alloc(
                        16, FFTN * sizeof(float), MALLOC_CAP_INTERNAL);
    if (!s_pie_w_table) {
        ESP_LOGE(TAG, "PIE FFT w_table alloc failed");
        return;
    }
    esp_err_t e = dsps_fft2r_init_fc32(s_pie_w_table, FFTN);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "dsps_fft2r_init_fc32 failed: %d", e);
        return;
    }
    s_pie_inited = true;
}

// Run one test case: feed `in_re`/`in_im` (de-interleaved float) into
// both implementations, compare outputs.
static void run_diff_case(const char *name, const float *in_re,
                          const float *in_im, float tol,
                          pie_fft_diff_result_t *out)
{
    static float ref_re[FFTN], ref_im[FFTN];
    static float pie_buf[2 * FFTN] __attribute__((aligned(16)));

    // Scalar reference path.
    memcpy(ref_re, in_re, sizeof(ref_re));
    memcpy(ref_im, in_im, sizeof(ref_im));
    ref_radix2_fft(ref_re, ref_im);

    // PIE path: interleaved IQ. dsps_fft2r_fc32_arp4 is in-place, output
    // is bit-reversed -> needs dsps_bit_rev_fc32 to be natural-order.
    for (int i = 0; i < FFTN; i++) {
        pie_buf[2 * i + 0] = in_re[i];
        pie_buf[2 * i + 1] = in_im[i];
    }
    pie_fft_init();
    if (s_pie_inited) {
        dsps_fft2r_fc32_arp4(pie_buf, FFTN);
        dsps_bit_rev_fc32_ansi(pie_buf, FFTN);
    } else {
        // Init failed; mark result so caller sees the issue.
        memset(out, 0, sizeof(*out));
        out->name = name;
        return;
    }

    // Compare and find both peaks.
    float max_abs = 0;
    int   max_bin = 0;
    double sum_sq = 0;
    int    over_tol = 0;
    float  peak_ref_m2 = 0;
    int    peak_ref_b  = 0;
    float  peak_pie_m2 = 0;
    int    peak_pie_b  = 0;
    for (int i = 0; i < FFTN; i++) {
        float dre = fabsf(ref_re[i] - pie_buf[2 * i + 0]);
        float dim = fabsf(ref_im[i] - pie_buf[2 * i + 1]);
        float dmax = dre > dim ? dre : dim;
        if (dmax > max_abs) { max_abs = dmax; max_bin = i; }
        sum_sq += (double)dre * dre + (double)dim * dim;
        if (dmax > tol) over_tol++;

        float m_ref = ref_re[i] * ref_re[i] + ref_im[i] * ref_im[i];
        if (m_ref > peak_ref_m2) { peak_ref_m2 = m_ref; peak_ref_b = i; }
        float m_pie = pie_buf[2 * i + 0] * pie_buf[2 * i + 0]
                    + pie_buf[2 * i + 1] * pie_buf[2 * i + 1];
        if (m_pie > peak_pie_m2) { peak_pie_m2 = m_pie; peak_pie_b = i; }
    }

    out->name             = name;
    out->peak_ref         = peak_ref_b;
    out->peak_pie         = peak_pie_b;
    out->peak_mag2_ref    = peak_ref_m2;
    out->peak_mag2_pie    = peak_pie_m2;
    out->max_abs_diff     = max_abs;
    out->max_abs_diff_bin = max_bin;
    out->rms_diff         = (float)sqrt(sum_sq / FFTN);
    out->bins_over_tol    = over_tol;
    out->tol              = tol;
}

static void log_result(const pie_fft_diff_result_t *r)
{
    ESP_LOGI(TAG, "case='%s' peak: ref@%d (|x|²=%.3e)  PIE@%d (|x|²=%.3e)  %s",
             r->name, r->peak_ref, (double)r->peak_mag2_ref,
             r->peak_pie, (double)r->peak_mag2_pie,
             (r->peak_ref == r->peak_pie) ? "PEAK_OK" : "PEAK_DIFFERS");
    ESP_LOGI(TAG, "case='%s' max_abs=%.3e at bin %d  rms=%.3e  "
                  "over_tol(%.0e)=%d/%d",
             r->name, (double)r->max_abs_diff, r->max_abs_diff_bin,
             (double)r->rms_diff, (double)r->tol,
             r->bins_over_tol, FFTN);
}

void pie_fft_diff_run(void)
{
    ESP_LOGI(TAG, "=== PIE FFT diff harness vs scalar reference (N=%d) ===", FFTN);

    static float in_re[FFTN], in_im[FFTN];
    pie_fft_diff_result_t r;

    // Case 1: complex tone at bin 100. Both FFTs must spike at bin 100.
    for (int i = 0; i < FFTN; i++) {
        double ph = 2.0 * M_PI * 100.0 * (double)i / (double)FFTN;
        in_re[i] = (float)cos(ph);
        in_im[i] = (float)sin(ph);
    }
    run_diff_case("tone@100", in_re, in_im, 1e-3f, &r);
    log_result(&r);

    // Case 2: impulse at sample 0. FFT should be flat (all bins = 1).
    memset(in_re, 0, sizeof(in_re));
    memset(in_im, 0, sizeof(in_im));
    in_re[0] = 1.0f;
    run_diff_case("impulse", in_re, in_im, 1e-5f, &r);
    log_result(&r);

    // Case 3: random Q15-ish noise. Realistic spectrum-shape input.
    uint32_t st = 0xC0FFEEu;
    for (int i = 0; i < FFTN; i++) {
        st = st * 1103515245u + 12345u;
        in_re[i] = (float)((int16_t)(st & 0xFFFF)) / 32768.0f;
        st = st * 1103515245u + 12345u;
        in_im[i] = (float)((int16_t)(st & 0xFFFF)) / 32768.0f;
    }
    run_diff_case("noise", in_re, in_im, 1e-3f, &r);
    log_result(&r);

    // Case 4: chirp (linear frequency sweep). Tests phase coherence
    // across multiple bins.
    for (int i = 0; i < FFTN; i++) {
        double t  = (double)i / (double)FFTN;
        double ph = 2.0 * M_PI * (50.0 * t + 400.0 * t * t);
        in_re[i]  = (float)cos(ph);
        in_im[i]  = (float)sin(ph);
    }
    run_diff_case("chirp", in_re, in_im, 1e-3f, &r);
    log_result(&r);

    ESP_LOGI(TAG, "=== diff harness end ===");
}
