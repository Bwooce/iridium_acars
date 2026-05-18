// See fft_sc16_64.h. Direct port of esp-dsp's
// dsps_fft2r_sc16_ansi.c (modules/fft/fixed/) reduced to the
// fixed-size N=64 case used by polyphase_channelizer. Same butterfly
// math, same per-stage right-shift, same Q15 twiddles → bit-
// equivalent output to dsps_fft2r_sc16 on identical input.

#include "fft_sc16_64.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define N      FFT_SC16_64_N    // 64
#define N_HALF (N / 2)          // 32

// Q15 twiddle table: w[k] = (cos(2π·k/N), sin(2π·k/N)) for k = 0..N/2-1.
// Stored as interleaved Re/Im int16. After init, the table is
// bit-reversed in place (matches esp-dsp's dsps_fft2r_init_sc16).
static int16_t s_w_table[N];    // N int16 = N/2 complex
static bool    s_inited = false;

// Round-then-shift constants (match dsps_fft2r_sc16_ansi.c lines
// 31-32 verbatim).
#define MULT_SHIFT_CONST  0x7fff
#define ADD_ROUND_MULT    0x7fff

// xtfixed_bf_{1,2,3,4} from dsps_fft2r_sc16_ansi.c, inlined here
// with the same names so the math is line-for-line traceable.
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

// Bit-reverse the index x within `order` bits.
static inline unsigned bit_reverse(unsigned x, int order)
{
    unsigned b = x;
    b = (b & 0xff00) >> 8 | (b & 0x00ffu) << 8;
    b = (b & 0xf0f0) >> 4 | (b & 0x0f0fu) << 4;
    b = (b & 0xcccc) >> 2 | (b & 0x3333u) << 2;
    b = (b & 0xaaaa) >> 1 | (b & 0x5555u) << 1;
    return b >> (16 - order);
}

// Bit-reverse the data array in-place (N complex samples, treated
// as N int32 pairs). Same algorithm as dsps_bit_rev_sc16_ansi for
// the matching length.
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

void fft_sc16_64_init(void)
{
    if (s_inited) return;
    // Generate twiddle table (Q15 cos/sin for k=0..N/2-1) — same as
    // dsps_gen_w_r2_sc16.
    const double e = 2.0 * 3.14159265358979323846 / (double)N;
    for (int i = 0; i < N_HALF; i++) {
        s_w_table[2 * i + 0] = (int16_t)((double)INT16_MAX * cos(i * e));
        s_w_table[2 * i + 1] = (int16_t)((double)INT16_MAX * sin(i * e));
    }
    // Bit-reverse the twiddle table by N/2 to match how
    // dsps_fft2r_sc16_ansi_ indexes w[j].
    bit_rev_data(s_w_table, N_HALF);
    s_inited = true;
}

void fft_sc16_64(int16_t *data)
{
    if (!s_inited) fft_sc16_64_init();

    // Verbatim port of dsps_fft2r_sc16_ansi_'s inner loop, with N=64
    // fixed.
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

    // Standard radix-2 DIT output is in bit-reversed order; un-shuffle.
    bit_rev_data(data, N);
}
