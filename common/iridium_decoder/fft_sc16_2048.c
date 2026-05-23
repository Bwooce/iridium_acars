// See fft_sc16_2048.h. On ESP32-P4 this wraps esp-dsp's PIE-accelerated
// dsps_fft2r_sc16_arp4 (8-lane Q15 SIMD, ~3× the ANSI scalar version
// on smoke). On host the ANSI port below runs — the algorithm is the
// C reference for arp4 so output is bit-identical between the paths.
//
// CRITICAL on ESP_PLATFORM: the PIE FFT uses `esp.vld.128.ip` for its
// vector loads, which cannot service PSRAM access timing — output is
// silently garbage (peak at DC instead of the real tone) if `data`
// is in PSRAM. We absorb this with a static 8 KB internal-SRAM
// scratch (`s_fft_scratch`): copy caller's data in, run PIE FFT in
// internal SRAM, copy result back. The copy is ~30 µs at internal-
// SRAM bandwidth and dominates well below the ~1.5 ms FFT speedup
// it unlocks. The twiddle table also lives in internal SRAM for the
// same reason.

#include "fft_sc16_2048.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef ESP_PLATFORM
// Pull esp-dsp's headers BEFORE defining `N` locally — esp-dsp's
// function prototypes use `int N` as parameter names and would
// otherwise be macro-substituted to `int 2048`.
#include <string.h>
#include "dsps_fft2r.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#endif

#define N      FFT_SC16_2048_N    // 2048
#define N_HALF (N / 2)            // 1024
#define LOG2_N 11                 // log2(2048)

#ifdef ESP_PLATFORM

// Lazy-allocated from internal-SRAM HEAP (not .bss): static .bss
// arrays would claim 12 KB of internal SRAM pre-main(), reducing the
// post-boot free pool below the CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL
// threshold (psram_init then panics on "Could not reserve internal/
// DMA pool"). Allocating after boot from the internal heap is fine
// because mag2/smooth in uw_correlator.c were moved to PSRAM,
// freeing the 20 KB of .bss internal SRAM that previously made this
// the binding constraint.
//
// s_w_table     — 4 KB, esp-dsp's twiddle table for N=2048
// s_fft_scratch — 8 KB, target for in-place FFT (caller's `data`
//                 may be in PSRAM which PIE asm can't service)
static int16_t *s_w_table     = NULL;
static int16_t *s_fft_scratch = NULL;
static bool s_inited = false;

void fft_sc16_2048_init(void)
{
    if (s_inited) return;
    s_w_table     = (int16_t *)heap_caps_aligned_alloc(16, N * sizeof(int16_t),
                                                        MALLOC_CAP_INTERNAL);
    s_fft_scratch = (int16_t *)heap_caps_aligned_alloc(16, 2 * N * sizeof(int16_t),
                                                        MALLOC_CAP_INTERNAL);
    if (!s_w_table || !s_fft_scratch) return;  // alloc failed; FFT will no-op
    // dsps_fft2r_init_sc16 ignores the size arg when buffer is NULL and
    // uses CONFIG_DSP_MAX_FFT_SIZE (= 4096), producing cos(2π·i/4096)
    // twiddles — wrong for our 2048-pt FFT (the inner loop reads w[j]
    // without stride-correcting). Pass our own N=2048-sized buffer.
    (void)dsps_fft2r_init_sc16(s_w_table, N);
    s_inited = true;
    ESP_LOGI("FFT2048", "s_w_table=%p s_fft_scratch=%p [early]",
             s_w_table, s_fft_scratch);
}

void fft_sc16_2048(int16_t *data)
{
    if (!s_inited) fft_sc16_2048_init();
    if (!s_inited) return;   // alloc still failing — caller sees no-op
    memcpy(s_fft_scratch, data, 2 * N * sizeof(int16_t));
    // PIE radix-2 DIF — output is in bit-reversed order, un-reverse
    // to natural (DC at bin 0). dsps_bit_rev_sc16_ansi called directly
    // because esp-dsp's macro alias is only defined in the
    // !CONFIG_DSP_OPTIMIZED branch of dsps_fft2r.h (header bug — no
    // PIE bit-rev exists for sc16, the ANSI scalar is fine).
    dsps_fft2r_sc16(s_fft_scratch, N);
    dsps_bit_rev_sc16_ansi(s_fft_scratch, N);
    memcpy(data, s_fft_scratch, 2 * N * sizeof(int16_t));
    (void)LOG2_N;   // legacy define; unused on this path
}

#else  /* host: ANSI scalar port, identical algorithm */

static int16_t s_w_table[N];      // N int16 = N/2 complex (4 KB)
static bool    s_inited = false;

#define MULT_SHIFT_CONST  0x7fff
#define ADD_ROUND_MULT    0x7fff

static inline int16_t xtfixed_bf_1(int16_t a0, int16_t a1, int16_t a2,
                                    int16_t a3, int16_t a4,
                                    int result_shift)
{
    int32_t result = (int32_t)a0 * MULT_SHIFT_CONST;
    result -= (int32_t)a1 * (int32_t)a2 + (int32_t)a3 * (int32_t)a4;
    result += ADD_ROUND_MULT;
    result = result >> result_shift;
    return (int16_t)result;
}

static inline int16_t xtfixed_bf_2(int16_t a0, int16_t a1, int16_t a2,
                                    int16_t a3, int16_t a4,
                                    int result_shift)
{
    int32_t result = (int32_t)a0 * MULT_SHIFT_CONST;
    result -= ((int32_t)a1 * (int32_t)a2 - (int32_t)a3 * (int32_t)a4);
    result += ADD_ROUND_MULT;
    result = result >> result_shift;
    return (int16_t)result;
}

static inline int16_t xtfixed_bf_3(int16_t a0, int16_t a1, int16_t a2,
                                    int16_t a3, int16_t a4,
                                    int result_shift)
{
    int32_t result = (int32_t)a0 * MULT_SHIFT_CONST;
    result += (int32_t)a1 * (int32_t)a2 + (int32_t)a3 * (int32_t)a4;
    result += ADD_ROUND_MULT;
    result = result >> result_shift;
    return (int16_t)result;
}

static inline int16_t xtfixed_bf_4(int16_t a0, int16_t a1, int16_t a2,
                                    int16_t a3, int16_t a4,
                                    int result_shift)
{
    int32_t result = (int32_t)a0 * MULT_SHIFT_CONST;
    result += (int32_t)a1 * (int32_t)a2 - (int32_t)a3 * (int32_t)a4;
    result += ADD_ROUND_MULT;
    result = result >> result_shift;
    return (int16_t)result;
}

// Bit-reverse the index x within `order` bits. Works for orders up to 16.
static inline unsigned bit_reverse(unsigned x, int order)
{
    unsigned b = x;
    b = (b & 0xff00) >> 8 | (b & 0x00ffu) << 8;
    b = (b & 0xf0f0) >> 4 | (b & 0x0f0fu) << 4;
    b = (b & 0xcccc) >> 2 | (b & 0x3333u) << 2;
    b = (b & 0xaaaa) >> 1 | (b & 0x5555u) << 1;
    return b >> (16 - order);
}

// Bit-reverse the data array in-place (n complex samples, treated as
// n int32 pairs). Same algorithm as dsps_bit_rev_sc16_ansi.
static void bit_rev_data(int16_t *data, int n)
{
    uint32_t *in_data = (uint32_t *)data;
    int j = 0;
    for (int i = 1; i < n - 1; i++) {
        int k = n >> 1;
        while (k <= j) {
            j -= k;
            k >>= 1;
        }
        j += k;
        if (i < j) {
            uint32_t tmp = in_data[j];
            in_data[j] = in_data[i];
            in_data[i] = tmp;
        }
    }
}

void fft_sc16_2048_init(void)
{
    if (s_inited) return;
    const double e = 2.0 * 3.14159265358979323846 / (double)N;
    for (int i = 0; i < N_HALF; i++) {
        s_w_table[2 * i + 0] = (int16_t)((double)INT16_MAX * cos(i * e));
        s_w_table[2 * i + 1] = (int16_t)((double)INT16_MAX * sin(i * e));
    }
    // Bit-reverse the twiddle table by N/2 to match the inner loop's
    // sequential indexing pattern (matches dsps_fft2r_sc16_ansi_).
    bit_rev_data(s_w_table, N_HALF);
    (void)bit_reverse;     // unused — bit_rev_data uses its own pattern
    s_inited = true;
}

void fft_sc16_2048(int16_t *data)
{
    if (!s_inited) fft_sc16_2048_init();

    uint32_t *w = (uint32_t *)s_w_table;
    uint32_t *in_data = (uint32_t *)data;

    int ie = 1;
    for (int N2 = N / 2; N2 > 0; N2 >>= 1) {
        int ia = 0;
        for (int j = 0; j < ie; j++) {
            uint32_t cs_data = w[j];
            int16_t cs_re = (int16_t)(cs_data & 0xffff);
            int16_t cs_im = (int16_t)((cs_data >> 16) & 0xffff);
            for (int i = 0; i < N2; i++) {
                int m = ia + N2;
                uint32_t m_data = in_data[m];
                uint32_t a_data = in_data[ia];
                int16_t m_re = (int16_t)(m_data & 0xffff);
                int16_t m_im = (int16_t)((m_data >> 16) & 0xffff);
                int16_t a_re = (int16_t)(a_data & 0xffff);
                int16_t a_im = (int16_t)((a_data >> 16) & 0xffff);

                int16_t m1_re = xtfixed_bf_1(a_re, cs_re, m_re, cs_im, m_im, 16);
                int16_t m1_im = xtfixed_bf_2(a_im, cs_re, m_im, cs_im, m_re, 16);
                int16_t m2_re = xtfixed_bf_3(a_re, cs_re, m_re, cs_im, m_im, 16);
                int16_t m2_im = xtfixed_bf_4(a_im, cs_re, m_im, cs_im, m_re, 16);

                in_data[m]  = ((uint32_t)(uint16_t)m1_re)
                            | ((uint32_t)(uint16_t)m1_im) << 16;
                in_data[ia] = ((uint32_t)(uint16_t)m2_re)
                            | ((uint32_t)(uint16_t)m2_im) << 16;
                ia++;
            }
            ia += N2;
        }
        ie <<= 1;
    }
    bit_rev_data(data, N);
    (void)LOG2_N;     // kept as documentation; unused
}

#endif  /* ESP_PLATFORM */
