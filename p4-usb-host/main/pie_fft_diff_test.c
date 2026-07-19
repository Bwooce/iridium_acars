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
#include "esp_memory_utils.h"   // esp_ptr_in_dram (detect-screen diff)
#include "dsps_fft2r.h"
#include "fft_burst_tagger.h"   // FBT_FFT_SIZE (detect-screen diff)

// Detect-scan PIE pre-screen kernel (common/iridium_decoder/fft_burst_tagger_
// arp4.S). Signature must match the .S: per 16-bin group it stores a 4-lane
// (128-bit) OR-mask; lane l aggregates bins {g*16 + q*4 + l : q=0..3}.
extern void fbt_detect_screen_arp4(const int32_t *mag, const int32_t *base,
                                   int32_t *flags, int n, int shift);

#define FFTN 2048
#define LOG2_FFTN 11

static const char *TAG = "PIE_FFT_DIFF";

// Scalar reference: identical algorithm to uw_correlator.c's radix2_fft_f32
// (standard radix-2 DIT). Kept self-contained here so the harness doesn't
// depend on uw_correlator internals.
//
// 2026-05-23: moved 12 KB of static .bss to heap. This is a smoke-only
// diagnostic harness; static .bss kept it in internal SRAM permanently
// even in production builds. Heap-allocated lazily on first call, kept
// alive after (the smoke test re-runs aren't expected, so we could free
// — but keeping makes re-runs cheap). PSRAM is fine since these are
// scalar C accesses (no PIE on these tables).
static uint16_t *s_ref_brev   = NULL;
static float    *s_ref_tw_re  = NULL;
static float    *s_ref_tw_im  = NULL;
static bool      s_ref_inited = false;

static void ref_fft_init(void)
{
    if (s_ref_inited) return;
    if (!s_ref_brev) {
        s_ref_brev  = (uint16_t *)heap_caps_malloc(FFTN * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
        s_ref_tw_re = (float *)heap_caps_malloc((FFTN / 2) * sizeof(float), MALLOC_CAP_SPIRAM);
        s_ref_tw_im = (float *)heap_caps_malloc((FFTN / 2) * sizeof(float), MALLOC_CAP_SPIRAM);
        if (!s_ref_brev || !s_ref_tw_re || !s_ref_tw_im) {
            ESP_LOGE(TAG, "ref FFT table alloc failed");
            return;
        }
    }
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
        double ang     = -2.0 * M_PI * (double)k / (double)FFTN;
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
            t     = re[i];
            re[i] = re[j];
            re[j] = t;
            t     = im[i];
            im[i] = im[j];
            im[j] = t;
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
                float xr       = re[i + stride];
                float xi       = im[i + stride];
                float tr       = wr * xr - wi * xi;
                float ti       = wr * xi + wi * xr;
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
    int         peak_ref;
    int         peak_pie;
    float       peak_mag2_ref;
    float       peak_mag2_pie;
    float       max_abs_diff;
    int         max_abs_diff_bin;
    float       rms_diff;
    int         bins_over_tol;
    float       tol;
} pie_fft_diff_result_t;

// PIE float FFT scratch + twiddle table, lazy-init.
static float *s_pie_w_table = NULL;
static bool   s_pie_inited  = false;
static void   pie_fft_init(void)
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

// Static scratches lazy-heap-allocated. Previously these were
// 16 KB (pie_buf) + 16 KB (ref_re+ref_im) = 32 KB of static .bss
// in internal SRAM, wasted in production where this harness never
// runs. Now heap-allocated on first call.
//   pie_buf must be in INTERNAL SRAM (PIE FFT requires it).
//   ref_re / ref_im are scalar; PSRAM is fine.
static float *s_run_pie_buf = NULL;
static float *s_run_ref_re  = NULL;
static float *s_run_ref_im  = NULL;

static bool ensure_run_buffers(void)
{
    if (s_run_pie_buf && s_run_ref_re && s_run_ref_im) return true;
    if (!s_run_pie_buf) {
        s_run_pie_buf = (float *)heap_caps_aligned_alloc(
            16, 2 * FFTN * sizeof(float), MALLOC_CAP_INTERNAL);
    }
    if (!s_run_ref_re) {
        s_run_ref_re = (float *)heap_caps_malloc(FFTN * sizeof(float), MALLOC_CAP_SPIRAM);
    }
    if (!s_run_ref_im) {
        s_run_ref_im = (float *)heap_caps_malloc(FFTN * sizeof(float), MALLOC_CAP_SPIRAM);
    }
    if (!s_run_pie_buf || !s_run_ref_re || !s_run_ref_im) {
        ESP_LOGE(TAG, "diff buffer alloc failed");
        return false;
    }
    return true;
}

// Run one test case: feed `in_re`/`in_im` (de-interleaved float) into
// both implementations, compare outputs.
static void run_diff_case(const char *name, const float *in_re,
                          const float *in_im, float tol,
                          pie_fft_diff_result_t *out)
{
    if (!ensure_run_buffers()) {
        memset(out, 0, sizeof(*out));
        out->name = name;
        return;
    }
    float *ref_re  = s_run_ref_re;
    float *ref_im  = s_run_ref_im;
    float *pie_buf = s_run_pie_buf;

    // Scalar reference path.
    memcpy(ref_re, in_re, FFTN * sizeof(float));
    memcpy(ref_im, in_im, FFTN * sizeof(float));
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
    float  max_abs     = 0;
    int    max_bin     = 0;
    double sum_sq      = 0;
    int    over_tol    = 0;
    float  peak_ref_m2 = 0;
    int    peak_ref_b  = 0;
    float  peak_pie_m2 = 0;
    int    peak_pie_b  = 0;
    for (int i = 0; i < FFTN; i++) {
        float dre  = fabsf(ref_re[i] - pie_buf[2 * i + 0]);
        float dim  = fabsf(ref_im[i] - pie_buf[2 * i + 1]);
        float dmax = dre > dim ? dre : dim;
        if (dmax > max_abs) {
            max_abs = dmax;
            max_bin = i;
        }
        sum_sq += (double)dre * dre + (double)dim * dim;
        if (dmax > tol) over_tol++;

        float m_ref = ref_re[i] * ref_re[i] + ref_im[i] * ref_im[i];
        if (m_ref > peak_ref_m2) {
            peak_ref_m2 = m_ref;
            peak_ref_b  = i;
        }
        float m_pie = pie_buf[2 * i + 0] * pie_buf[2 * i + 0] + pie_buf[2 * i + 1] * pie_buf[2 * i + 1];
        if (m_pie > peak_pie_m2) {
            peak_pie_m2 = m_pie;
            peak_pie_b  = i;
        }
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

    static float          in_re[FFTN], in_im[FFTN];
    pie_fft_diff_result_t r;

    // Case 1: complex tone at bin 100. Both FFTs must spike at bin 100.
    for (int i = 0; i < FFTN; i++) {
        double ph = 2.0 * M_PI * 100.0 * (double)i / (double)FFTN;
        in_re[i]  = (float)cos(ph);
        in_im[i]  = (float)sin(ph);
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
        st       = st * 1103515245u + 12345u;
        in_re[i] = (float)((int16_t)(st & 0xFFFF)) / 32768.0f;
        st       = st * 1103515245u + 12345u;
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

#if CONFIG_SMOKE_TEST_MODE
// On-device PIE FFT heap-placement sweep (#120 prep).
//
// The PIE float FFT (dsps_fft2r_fc32_arp4) operates in-place on a scratch
// buffer and is known to silently corrupt when that buffer is NOT in main
// internal DRAM — under DRAM pressure MALLOC_CAP_INTERNAL falls back to
// RTCRAM (0x5010_xxxx), where the PIE vector loads mis-decode
// (project_heap_position_decode_bug). uw_correlator now pins its scratch
// in DRAM via uw_correlator_prealloc_pie_fft(), called from the boot-time
// early-alloc dance (class_driver.c / smoke_test.c) while DRAM is plentiful,
// with an esp_ptr_in_dram guard that fails loudly on a non-DRAM placement.
// This sweep walks the main-DRAM arena; note it does NOT exercise the
// RTCRAM fallback that actually caused the bug (that is caught by the guard
// + the RAW smoke's GOLDEN-matched gate).
//
// This walks a large internal-SRAM arena, runs the PIE FFT at many
// 16-aligned offsets within it on a fixed broadband input, and compares
// each result to a scalar golden. A corrupting placement shows up as a
// gross diff, NaN/Inf, or a shifted peak. The log maps safe vs unsafe
// address ranges; a future context allocation can be checked against it
// (or this run used as the gate: PIE_PLACEMENT_PASS = scratch is safe
// anywhere in the swept range).
void pie_fft_placement_run(void)
{
    ESP_LOGW(TAG, "=== PIE FFT heap-placement sweep (N=%d, #120 prep) ===", FFTN);
    pie_fft_init();
    if (!s_pie_inited) {
        ESP_LOGE(TAG, "PIE w_table init failed -> abort");
        return;
    }

    // Deterministic broadband input + scalar golden (PSRAM scalar buffers —
    // every output bin is non-trivial so any corruption is visible).
    float *in_re   = heap_caps_malloc(FFTN * sizeof(float), MALLOC_CAP_SPIRAM);
    float *in_im   = heap_caps_malloc(FFTN * sizeof(float), MALLOC_CAP_SPIRAM);
    float *gold_re = heap_caps_malloc(FFTN * sizeof(float), MALLOC_CAP_SPIRAM);
    float *gold_im = heap_caps_malloc(FFTN * sizeof(float), MALLOC_CAP_SPIRAM);
    if (!in_re || !in_im || !gold_re || !gold_im) {
        ESP_LOGE(TAG, "placement input/golden alloc failed");
        heap_caps_free(in_re);
        heap_caps_free(in_im);
        heap_caps_free(gold_re);
        heap_caps_free(gold_im);
        return;
    }
    uint32_t st = 0xBADC0DEu;
    for (int i = 0; i < FFTN; i++) {
        st       = st * 1103515245u + 12345u;
        in_re[i] = (float)((int16_t)(st & 0xFFFF)) / 32768.0f;
        st       = st * 1103515245u + 12345u;
        in_im[i] = (float)((int16_t)(st & 0xFFFF)) / 32768.0f;
    }
    memcpy(gold_re, in_re, FFTN * sizeof(float));
    memcpy(gold_im, in_im, FFTN * sizeof(float));
    ref_radix2_fft(gold_re, gold_im); // scalar reference spectrum

    int   gpeak = 0;
    float gpm2 = 0, gmax = 0;
    for (int i = 0; i < FFTN; i++) {
        float m = gold_re[i] * gold_re[i] + gold_im[i] * gold_im[i];
        if (m > gpm2) {
            gpm2  = m;
            gpeak = i;
        }
        float a = fabsf(gold_re[i]);
        if (a > gmax) gmax = a;
        a = fabsf(gold_im[i]);
        if (a > gmax) gmax = a;
    }
    // Scalar↔PIE float jitter is < ~1e-2 abs at this scale; corruption is
    // gross (>> 1 or NaN). 0.5 separates the two cleanly.
    const float TOL = 0.5f;

    ESP_LOGW(TAG, "golden peak bin=%d gmax=%.2f tol=%.2f", gpeak, (double)gmax, (double)TOL);

    // Walk EVERY free internal region the allocator can hand out: grab
    // scratch-sized internal blocks until the heap is nearly exhausted
    // (leaving a reserve so logging/system survive), run the PIE FFT in
    // each, then free them all. Unlike a single contiguous arena, this
    // reaches scattered free regions — incl. the historically-suspect
    // ~0x4ff6xxxx zone if it's free — because the context scratch a future
    // refactor allocates could land in any of them.
    const size_t SCRATCH_BYTES = 2 * FFTN * sizeof(float); // 16 KB, in-place IQ
#define PP_MAX_BLOCKS 96
    static float *blocks[PP_MAX_BLOCKS];
    int           n_blocks = 0;
    while (n_blocks < PP_MAX_BLOCKS) {
        if (heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) < SCRATCH_BYTES + 64 * 1024)
            break; // keep a 64 KB reserve
        float *b = heap_caps_aligned_alloc(16, SCRATCH_BYTES, MALLOC_CAP_INTERNAL);
        if (!b) break;
        blocks[n_blocks++] = b;
    }
    ESP_LOGW(TAG, "grabbed %d internal scratch blocks (16 KB each) to sweep", n_blocks);

    int       n_fail = 0;
    uintptr_t lo = 0, hi = 0;
    for (int k = 0; k < n_blocks; k++) {
        float *scr = blocks[k];
        for (int i = 0; i < FFTN; i++) {
            scr[2 * i + 0] = in_re[i];
            scr[2 * i + 1] = in_im[i];
        }
        dsps_fft2r_fc32_arp4(scr, FFTN);
        dsps_bit_rev_fc32_ansi(scr, FFTN);

        float maxd    = 0;
        int   ppeak   = 0;
        float ppm2    = 0;
        bool  bad_num = false;
        for (int i = 0; i < FFTN; i++) {
            float rr = scr[2 * i + 0], ii = scr[2 * i + 1];
            if (isnan(rr) || isinf(rr) || isnan(ii) || isinf(ii)) bad_num = true;
            float dr = fabsf(rr - gold_re[i]), di = fabsf(ii - gold_im[i]);
            float dm = dr > di ? dr : di;
            if (dm > maxd) maxd = dm;
            float m = rr * rr + ii * ii;
            if (m > ppm2) {
                ppm2  = m;
                ppeak = i;
            }
        }
        bool      ok = !bad_num && (maxd < TOL) && (ppeak == gpeak);
        uintptr_t a  = (uintptr_t)scr;
        if (!ok) {
            n_fail++;
            if (!lo || a < lo) lo = a;
            if (a > hi) hi = a;
            ESP_LOGE(TAG, "  BAD @ %p maxd=%.3e peak=%d(exp %d)%s",
                     scr, (double)maxd, ppeak, gpeak, bad_num ? " NaN/Inf" : "");
        } else {
            ESP_LOGI(TAG, "  ok  @ %p maxd=%.3e", scr, (double)maxd);
        }
    }
    for (int k = 0; k < n_blocks; k++)
        heap_caps_free(blocks[k]);

    ESP_LOGW(TAG, "placement sweep: %d positions tested, %d corrupting", n_blocks, n_fail);
    if (n_fail) {
        ESP_LOGE(TAG, "  corrupting addresses span ~[%p .. %p] — keep PIE scratch OUT of this range",
                 (void *)lo, (void *)hi);
    }
    ESP_LOGW(TAG, n_fail == 0
                      ? "===== PIE_PLACEMENT_PASS (no corrupting positions found) ====="
                      : "===== PIE_PLACEMENT_FAIL (corruption found — see BAD addresses) =====");

    heap_caps_free(in_re);
    heap_caps_free(in_im);
    heap_caps_free(gold_re);
    heap_caps_free(gold_im);
}

// ===================================================================
// Detect-scan PIE pre-screen: on-device scalar-vs-PIE bit-exact diff.
//
// The host golden test (tests/host/test_tagger_detect_screen_golden.c) proves
// the ALGEBRA — that the C screen model is a strict superset of the exact
// threshold test, and that a screened scan yields byte-identical peaks. It
// cannot run the arp4 instructions. THIS harness runs the REAL silicon kernel
// (fbt_detect_screen_arp4) against the same C model on synthetic vectors at
// boot, and asserts:
//   (1) BIT-EXACT — every lane of the kernel's flag output equals the model's
//       (per-lane nonzero-equality is the hard gate; if the raw mask encoding
//        differs — all-ones vs 1 — we WARN but do not fail, since only the
//        nonzero-ness feeds the group skip in fft_burst_tagger.c).
//   (2) SUPERSET — for every bin that passes the exact int64 test, the kernel's
//       group flag is nonzero (a real burst can never be screened out).
// It also logs the flags-buffer address + esp_ptr_in_dram verdict — the
// prescreen buffer is PIE-written, so a non-DRAM placement would silently
// corrupt exactly like the FFT scratch (project_heap_position_decode_bug).
//
// Gated under CONFIG_SMOKE_TEST_MODE — diagnostic/smoke path only, never the
// production hot path.

#define DS_N        FBT_FFT_SIZE        // bins per scan (== tagger FFT size)
#define DS_HIST     512                 // FBT_HISTORY_SIZE (2^9); mag2*HIST in exact test
#define DS_MAG_MAX  2147352578          // 32767^2 * 2

// Exact int64 threshold test — byte-for-byte fft_burst_tagger.c:above_threshold.
static int ds_exact_above(int32_t mag2, int32_t base, int32_t thr_q15)
{
    int64_t lhs = (int64_t)mag2 * (int64_t)DS_HIST;
    int64_t rhs = ((int64_t)base * (int64_t)thr_q15) >> 15;
    return lhs > rhs;
}

// Shift derivation — byte-for-byte fft_burst_tagger.c init.
static int ds_screen_shift_for(int32_t thr_q15)
{
    if (thr_q15 <= 0) return -1;
    int a = 31 - __builtin_clz((uint32_t)thr_q15);
    int s = 24 - a;
    if (s < 0)  s = 0;
    if (s > 24) s = 24;
    return s;
}

// C model of fbt_detect_screen_arp4, at the exact lane layout the .S produces:
// per 16-bin group, 4 lanes; lane l = OR over bins {g*16 + q*4 + l : q=0..3};
// a bin's lane bit is set iff mag > (base >> shift). Emits all-ones for a set
// lane (matching esp.vcmp.gt.s32 mask semantics); the kernel-vs-model compare
// tolerates a differing encoding via nonzero-equality.
static void ds_screen_model(const int32_t *mag, const int32_t *base,
                            int32_t *flags, int n, int shift)
{
    int ng = n / 16;
    for (int g = 0; g < ng; g++) {
        int32_t lane[4] = {0, 0, 0, 0};
        for (int q = 0; q < 4; q++)
            for (int l = 0; l < 4; l++) {
                int bin = g * 16 + q * 4 + l;
                if (mag[bin] > (base[bin] >> shift)) lane[l] = (int32_t)0xFFFFFFFF;
            }
        for (int l = 0; l < 4; l++) flags[g * 4 + l] = lane[l];
    }
}

// group g flagged iff any of its 4 lanes nonzero (mirrors the scan skip test).
static inline int ds_group_flagged(const int32_t *flags, int g)
{
    return (flags[g*4+0] | flags[g*4+1] | flags[g*4+2] | flags[g*4+3]) != 0;
}

void fbt_detect_screen_diff_run(void)
{
    ESP_LOGW(TAG, "=== detect-scan PIE pre-screen diff (N=%d) ===", DS_N);

    const size_t bins_bytes  = (size_t)DS_N * sizeof(int32_t);
    const size_t flags_bytes = (size_t)(DS_N / 4) * sizeof(int32_t); // 4 lanes / 16 bins
    int32_t *mag  = heap_caps_aligned_alloc(16, bins_bytes,  MALLOC_CAP_INTERNAL);
    int32_t *base = heap_caps_aligned_alloc(16, bins_bytes,  MALLOC_CAP_INTERNAL);
    int32_t *fk   = heap_caps_aligned_alloc(16, flags_bytes, MALLOC_CAP_INTERNAL); // kernel
    int32_t *fm   = heap_caps_aligned_alloc(16, flags_bytes, MALLOC_CAP_INTERNAL); // model
    if (!mag || !base || !fk || !fm) {
        ESP_LOGE(TAG, "detect-screen diff alloc failed");
        heap_caps_free(mag); heap_caps_free(base);
        heap_caps_free(fk);  heap_caps_free(fm);
        return;
    }
    // PIE writes fk — placement must be main DRAM or the vector store corrupts.
    ESP_LOGW(TAG, "flags kernel buf @ %p  esp_ptr_in_dram=%d", fk, esp_ptr_in_dram(fk));
    if (!esp_ptr_in_dram(fk))
        ESP_LOGE(TAG, "  flags buf NOT in DRAM — PIE store will corrupt (see heap-position bug)");

    // Test-vector generators; each fills mag[] and base[] for the whole scan.
    // Explicit thr_q15 points (avoid libm pow at boot) chosen to span the shift
    // range: 32768 = 0 dB (s=9), ~85800 ≈ 14 dB default (s=8), and 20000000 >
    // 2^24 which hits the a>24 shift-clamp (s=0). thr and shift stay
    // self-consistent (shift derived from thr), so the exact test — which uses
    // thr directly — and the screen stay coupled exactly as in production.
    const int32_t thrs[] = {32768, 85800, 20000000};
    uint32_t st = 0x1234abcdu;

    int total_lane_mismatch = 0;   // hard: kernel lane nonzero != model lane nonzero
    int total_raw_mismatch  = 0;   // soft: encoding differs but nonzero-ness agrees
    int total_superset_fail = 0;   // hard: exact-true bin whose group flag is 0
    int cases = 0;

    for (unsigned di = 0; di < sizeof(thrs)/sizeof(thrs[0]); di++) {
        int32_t thr = thrs[di];
        int shift = ds_screen_shift_for(thr);
        if (shift < 0) continue;

        for (int mode = 0; mode < 4; mode++) {
            for (int i = 0; i < DS_N; i++) {
                st = st * 1103515245u + 12345u;
                switch (mode) {
                case 0: // ramps
                    base[i] = (int32_t)((i * 293127) & 0x1FFFFFFF);
                    mag[i]  = (int32_t)((i * 811 + 5) & 0x3FFFFFFF);
                    break;
                case 1: // random broadband
                    base[i] = (int32_t)((st >> 3) % 600000000u);
                    st = st * 1103515245u + 12345u;
                    mag[i]  = (int32_t)((st >> 3) % (uint32_t)DS_MAG_MAX);
                    break;
                case 2: // sparse strong peaks over a noisy floor
                    base[i] = (int32_t)((st >> 8) % 20000000u);
                    mag[i]  = (int32_t)((st >> 5) % 100000u);
                    if (((st >> 20) & 0x3f) == 0) mag[i] += (int32_t)((st >> 4) % 5000000u);
                    break;
                default: // boundaries: base at extremes, mag straddling exact rhs
                    base[i] = (i & 1) ? INT32_MAX : ((i & 2) ? 0 : 512);
                    { int64_t rhs = ((int64_t)base[i] * (int64_t)thr) >> 15;
                      int64_t m = rhs / DS_HIST + ((i & 4) ? 1 : 0);
                      if (m > DS_MAG_MAX) { m = DS_MAG_MAX; }
                      if (m < 0) { m = 0; }
                      mag[i] = (int32_t)m; }
                    break;
                }
            }

            // model then real kernel (both into fresh buffers).
            ds_screen_model(mag, base, fm, DS_N, shift);
            memset(fk, 0xAA, flags_bytes); // poison so a no-write is caught
            fbt_detect_screen_arp4(mag, base, fk, DS_N, shift);

            int lane_mm = 0, raw_mm = 0, sup_fail = 0;
            int nlanes = DS_N / 4;
            for (int l = 0; l < nlanes; l++) {
                int kn = (fk[l] != 0), mn = (fm[l] != 0);
                if (kn != mn) lane_mm++;
                else if (fk[l] != fm[l]) raw_mm++;
            }
            // superset: every exact-true bin must land in a flagged group.
            int ng = DS_N / 16;
            for (int g = 0; g < ng; g++) {
                if (ds_group_flagged(fk, g)) continue;
                for (int j = 0; j < 16; j++) {
                    int bin = g*16 + j;
                    if (ds_exact_above(mag[bin], base[bin], thr)) { sup_fail++; break; }
                }
            }

            if (lane_mm || sup_fail)
                ESP_LOGE(TAG, "  thr=%d mode=%d shift=%d: lane_mm=%d superset_fail=%d raw_mm=%d",
                         (int)thr, mode, shift, lane_mm, sup_fail, raw_mm);
            else if (raw_mm)
                ESP_LOGW(TAG, "  thr=%d mode=%d shift=%d: OK (nonzero-equal); raw encoding differs on %d lanes",
                         (int)thr, mode, shift, raw_mm);
            else
                ESP_LOGI(TAG, "  thr=%d mode=%d shift=%d: bit-exact + superset OK",
                         (int)thr, mode, shift);

            total_lane_mismatch += lane_mm;
            total_raw_mismatch  += raw_mm;
            total_superset_fail += sup_fail;
            cases++;
        }
    }

    heap_caps_free(mag); heap_caps_free(base);
    heap_caps_free(fk);  heap_caps_free(fm);

    bool pass = (total_lane_mismatch == 0) && (total_superset_fail == 0);
    ESP_LOGW(TAG, "detect-screen diff: %d cases, lane_mm=%d superset_fail=%d raw_mm=%d",
             cases, total_lane_mismatch, total_superset_fail, total_raw_mismatch);
    ESP_LOGW(TAG, pass
                      ? "===== DETECT_SCREEN_DIFF_PASS ====="
                      : "===== DETECT_SCREEN_DIFF_FAIL (see lane/superset mismatches above) =====");
}
#endif // CONFIG_SMOKE_TEST_MODE
