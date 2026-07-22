// See vdl2_demod.h. Burst-mode port of dumpvdl2's phase-domain D8PSK
// demodulator (src/demod.c, src/decode.c, src/bitstream.c of
// https://github.com/szpajder/dumpvdl2). Every protocol constant below
// carries a file:line citation to that tree; the DSP structure is
// re-implemented for this repo's burst-window model (dumpvdl2 is a
// continuous-stream demod).

#include "vdl2_demod.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "firmr_s16.h" // common/iridium_decoder — host-proven polyphase core

#define VD_PI 3.14159265358979323846f

// ---- dumpvdl2 demod constants (src/demod.c:37-48) ----
#define VDL2_SYNC_BUFLEN (VDL2_PREAMBLE_SYMS * VDL2_SPS) // 160, dumpvdl2.h:43
#define VDL2_PHERR_MAX 1000.f                            // demod.c:38
#define VDL2_SYNC_SKIP 3                                 // demod.c:39
#define VDL2_SYNC_THRESHOLD 4.f                          // demod.c:40

// Cumulative phase after each preamble symbol, wrapped to (-pi, pi].
// Verbatim from dumpvdl2 demod.c:107-124 (got_sync's pr_phase). The
// differential-symbol view of this table (successive differences in
// pi/4 units, Gray-decoded) is the ICAO Annex 10 Vol III 16-symbol
// synchronisation sequence 000 010 011 110 000 001 101 110 001 100
// 011 111 101 111 100 010 — derived, not re-typed, so the two sources
// agree by construction.
const float vdl2_preamble_phase[VDL2_PREAMBLE_SYMS] = {
    0 * VD_PI / 4,  3 * VD_PI / 4,  -3 * VD_PI / 4, 1 * VD_PI / 4,
    1 * VD_PI / 4,  2 * VD_PI / 4,  0 * VD_PI / 4,  4 * VD_PI / 4,
    -3 * VD_PI / 4, 4 * VD_PI / 4,  -2 * VD_PI / 4, 3 * VD_PI / 4,
    1 * VD_PI / 4,  -2 * VD_PI / 4, -3 * VD_PI / 4, 0 * VD_PI / 4,
};

// D8PSK Gray map, dumpvdl2 demod.c:223.
const uint8_t vdl2_graycode[8] = {0, 1, 3, 2, 6, 7, 5, 4};

// ---- (25,20) header block code, dumpvdl2 decode.c:55-100 ----
// Parity-check matrix rows (25-bit words; the 5 parity bits are the 5
// LSBs, one per row). Binary literals aren't C11; the rows are spelled
// in hex with the binary original (verbatim from decode.c:55-61) in
// comments.
static const uint32_t s_hdr_h_rows[VDL2_HDR_FEC_BITS] = {
    0x001FFF0, // 0b0000000011111111111110000
    0x07E1FE8, // 0b0011111100001111111101000
    0x18E61E4, // 0b1100011100110000111100100
    0x1B6A662, // 0b1101101101010011001100010
    0x0D3CAA1, // 0b0110100111100101010100001
};
// Syndrome -> error-pattern table, decode.c:63-96 (32 entries).
static const uint32_t s_hdr_syndtable[1u << VDL2_HDR_FEC_BITS] = {
    0x0000000, // 0b0000000000000000000000000
    0x0000001, // 0b0000000000000000000000001
    0x0000002, // 0b0000000000000000000000010
    0x0800004, // 0b0100000000000000000000100
    0x0000004, // 0b0000000000000000000000100
    0x0800002, // 0b0100000000000000000000010
    0x1000000, // 0b1000000000000000000000000
    0x0800000, // 0b0100000000000000000000000
    0x0000008, // 0b0000000000000000000001000
    0x0400000, // 0b0010000000000000000000000
    0x0200000, // 0b0001000000000000000000000
    0x0100000, // 0b0000100000000000000000000
    0x0080000, // 0b0000010000000000000000000
    0x1100000, // 0b1000100000000000000000000
    0x0040000, // 0b0000001000000000000000000
    0x0020000, // 0b0000000100000000000000000
    0x0000010, // 0b0000000000000000000010000
    0x0010000, // 0b0000000010000000000000000
    0x0804000, // 0b0100000000100000000000000
    0x0008000, // 0b0000000001000000000000000
    0x0808000, // 0b0100000001000000000000000
    0x0004000, // 0b0000000000100000000000000
    0x0002000, // 0b0000000000010000000000000
    0x1010000, // 0b1000000010000000000000000
    0x0001000, // 0b0000000000001000000000000
    0x0000800, // 0b0000000000000100000000000
    0x0000400, // 0b0000000000000010000000000
    0x0000200, // 0b0000000000000001000000000
    0x0000100, // 0b0000000000000000100000000
    0x0000080, // 0b0000000000000000010000000
    0x0000040, // 0b0000000000000000001000000
    0x0000020, // 0b0000000000000000000100000
};
// Hamming weight of each syndrome's error pattern, decode.c:98-100.
static const uint8_t s_hdr_synd_weight[1u << VDL2_HDR_FEC_BITS] = {
    0, 1, 1, 2, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1,
    1, 1, 2, 1, 2, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1,
};

static int32_t parity32(uint32_t v)
{
    // decode.c:102-109 (Brian Kernighan popcount parity).
    uint32_t p = 0;
    while (v) {
        p = !p;
        v = v & (v - 1);
    }
    return (int32_t)p;
}

static uint32_t reverse_bits(uint32_t v, int numbits)
{
    uint32_t r = 0;
    for (int i = 0; i < numbits; i++) {
        r = (r << 1) | (v & 1u);
        v >>= 1;
    }
    return r;
}

uint32_t vdl2_hdr_encode(uint32_t datalen_bits)
{
    // Transmitted word: [3 reserved zeros][17-bit length, bit-reversed
    // on air (decode.c:222 datalen = reverse(field, TRLEN))][5 parity].
    uint32_t field = reverse_bits(datalen_bits & 0x1FFFFu, VDL2_HDR_TRLEN_BITS);
    uint32_t w     = field << VDL2_HDR_FEC_BITS; // reserved bits stay 0
    // Each H row contains exactly one parity-bit position among the 5
    // LSBs (row i -> bit 4-i), so parity(codeword & H[i]) == 0 fixes
    // p_i = parity(data_part & H[i]).
    for (int i = 0; i < VDL2_HDR_FEC_BITS; i++) {
        if (parity32(w & s_hdr_h_rows[i]))
            w |= 1u << (VDL2_HDR_FEC_BITS - 1 - i);
    }
    return w;
}

int vdl2_hdr_decode(uint32_t *hdr, uint32_t *datalen_bits)
{
    // decode.c:196-232 (DEC_HEADER path). Force reserved bits to 0
    // first (decode.c:209), syndrome-correct, then require the
    // corrected word to still have zero reserved bits (decode.c:215).
    uint32_t w = *hdr & ((1u << (VDL2_HDR_TRLEN_BITS + VDL2_HDR_FEC_BITS)) - 1u);
    uint32_t syndrome = 0;
    for (int i = 0; i < VDL2_HDR_FEC_BITS; i++) {
        syndrome |= ((uint32_t)parity32(w & s_hdr_h_rows[i]))
                    << (VDL2_HDR_FEC_BITS - 1 - i);
    }
    w ^= s_hdr_syndtable[syndrome];
    if ((w & ((1u << (VDL2_HDR_TRLEN_BITS + VDL2_HDR_FEC_BITS)) - 1u)) != w)
        return -1; // corrected "error" landed in the reserved bits
    *hdr = w;
    *datalen_bits =
        reverse_bits((w >> VDL2_HDR_FEC_BITS) & 0x1FFFFu, VDL2_HDR_TRLEN_BITS);
    return (int)s_hdr_synd_weight[syndrome];
}

int vdl2_burst_body_bits(uint32_t datalen_bits)
{
    // decode.c:124-133 (get_fec_octetcount) + decode.c:233-255.
    if (datalen_bits == 0 || datalen_bits > VDL2_MAX_FRAME_BITS) return -1;
    uint32_t octets = (datalen_bits + 7) / 8;
    uint32_t blocks = octets / 249;
    uint32_t last   = octets % 249;
    uint32_t fec    = blocks * 6;
    if (last) {
        if (last < 3)
            fec += 0;
        else if (last < 31)
            fec += 2;
        else if (last < 68)
            fec += 4;
        else
            fec += 6;
    }
    // decode.c:250-255: fec_octets == 0 => "unreasonably short", reject.
    if (fec == 0) return -1;
    return (int)(8u * (octets + fec));
}

void vdl2_scramble(uint8_t *bits, int n_bits, uint16_t *lfsr)
{
    // bitstream.c:94-107. 15-stage LFSR, feedback x^15 + x + 1:
    // keystream bit = stage0 ^ stage14, shifted back in at stage 14.
    uint16_t s = *lfsr;
    for (int i = 0; i < n_bits; i++) {
        uint8_t bit = (uint8_t)(((s >> 0) ^ (s >> 14)) & 1u);
        s           = (uint16_t)((s >> 1) | ((uint16_t)bit << 14));
        bits[i] ^= bit;
    }
    *lfsr = s;
}

// ---------------------------------------------------------------------------
// Kaiser-windowed-sinc polyphase designer (Q15, firmr_s16 layout).
// Same construction as resample_256_to_250.c's make_resample_coeffs
// (the repo's established resampler-design idiom), parameterised.
// ---------------------------------------------------------------------------

static double bessel_i0(double x)
{
    double sum = 1.0, term = 1.0;
    double half_x_sq = (x * x) / 4.0;
    for (int k = 1; k < 50; k++) {
        term *= half_x_sq / (double)(k * k);
        sum += term;
        if (term < 1e-12 * sum) break;
    }
    return sum;
}

bool vdl2_lpf_design_q15(int16_t *coeffs, int delay_size, int interp,
                         double fc_cycles_per_vsample, double beta)
{
    const int    nproto = delay_size * interp;
    const double pi     = 3.14159265358979323846;
    const double inv_i0 = 1.0 / bessel_i0(beta);
    const int    center = nproto / 2;

    double *w = (double *)malloc(sizeof(double) * (size_t)nproto);
    double *sum_phase = (double *)malloc(sizeof(double) * (size_t)interp);
    if (!w || !sum_phase) {
        free(w);
        free(sum_phase);
        memset(coeffs, 0, sizeof(int16_t) * (size_t)nproto);
        return false;
    }
    for (int p = 0; p < interp; p++)
        sum_phase[p] = 0.0;

    for (int k = 0; k < nproto; k++) {
        double t    = (double)(k - center);
        double sinc = (t == 0.0)
                          ? 2.0 * fc_cycles_per_vsample
                          : sin(2.0 * pi * fc_cycles_per_vsample * t) / (pi * t);
        double u   = 2.0 * (double)k / (double)(nproto - 1) - 1.0;
        double arg = beta * sqrt(1.0 - u * u);
        double kw  = bessel_i0(arg) * inv_i0;
        w[k]       = sinc * kw;
        sum_phase[k % interp] += w[k];
    }
    // Per-phase DC gain = 1.0 (firmr_s16 shift=0 path divides by 2^15;
    // taps summing to ~1.0 in Q15 give unity gain — the
    // resample_256_to_250 convention).
    for (int p = 0; p < interp; p++) {
        if (sum_phase[p] == 0.0) continue;
        double scale = 1.0 / sum_phase[p];
        for (int t = 0; t < delay_size; t++)
            w[t * interp + p] *= scale;
    }
    for (int k = 0; k < nproto; k++) {
        double v = w[k] * 32767.0;
        if (v > 32767.0) v = 32767.0;
        if (v < -32767.0) v = -32767.0;
        coeffs[k] = (int16_t)lrint(v);
    }
    free(w);
    free(sum_phase);
    return true;
}

// ---------------------------------------------------------------------------
// 250 ksps -> 105 ksps front-end resampler (channel filter + rate
// change in one polyphase). fc = 9 kHz at the 250 k input rate —
// passband covers the D8PSK occupied bandwidth (1+alpha)*Rs/2 =
// 8.4 kHz at alpha 0.6; role-equivalent to dumpvdl2's 8 kHz input LPF
// (demod.c:45 INP_LPF_CUTOFF_FREQ).
// ---------------------------------------------------------------------------

#define VDL2_RS_DSIZE 48 // taps/phase; prototype spans 48 input samples (192 us)

static int16_t s_rs_coeffs[VDL2_RS_DSIZE * VDL2_RESAMP_INTERP];
static int     s_init_done = 0;

static void vdl2_demod_init_once(void)
{
    if (s_init_done) return;
    // 9 kHz cutoff in cycles per virtual (interp x fs_in) sample.
    (void)vdl2_lpf_design_q15(s_rs_coeffs, VDL2_RS_DSIZE, VDL2_RESAMP_INTERP,
                              9000.0 / ((double)VDL2_FS_IN_HZ * VDL2_RESAMP_INTERP),
                              8.0);
    s_init_done = 1;
}

// Resample the whole window into caller-provided iq105 (capacity
// max_out complex). Returns output complex count.
static int resample_250_to_105(const int16_t *iq250, int n_complex,
                               int16_t *iq105, int max_out)
{
    firmr_s16_t fi, fq;
    int16_t     delay_i[VDL2_RS_DSIZE], delay_q[VDL2_RS_DSIZE];
    firmr_s16_init(&fi, s_rs_coeffs, delay_i, VDL2_RS_DSIZE, VDL2_RESAMP_INTERP,
                   VDL2_RESAMP_DECIM, /*start_pos=*/0, /*shift=*/0);
    firmr_s16_init(&fq, s_rs_coeffs, delay_q, VDL2_RS_DSIZE, VDL2_RESAMP_INTERP,
                   VDL2_RESAMP_DECIM, /*start_pos=*/0, /*shift=*/0);

#define VDL2_RS_CHUNK 256
    int16_t in_i[VDL2_RS_CHUNK], in_q[VDL2_RS_CHUNK];
    // Per-chunk output bound: ceil(256 * 21/50) + 1 = 109.
    int16_t out_i[VDL2_RS_CHUNK / 2 + 8], out_q[VDL2_RS_CHUNK / 2 + 8];

    int n_out = 0;
    for (int base = 0; base < n_complex; base += VDL2_RS_CHUNK) {
        int len = n_complex - base;
        if (len > VDL2_RS_CHUNK) len = VDL2_RS_CHUNK;
        for (int k = 0; k < len; k++) {
            in_i[k] = iq250[2 * (base + k) + 0];
            in_q[k] = iq250[2 * (base + k) + 1];
        }
        int ni = (int)firmr_s16_process(&fi, in_i, out_i, len);
        int nq = (int)firmr_s16_process(&fq, in_q, out_q, len);
        // I and Q instances share phase arithmetic -> identical counts.
        int n = (ni < nq) ? ni : nq;
        if (n_out + n > max_out) n = max_out - n_out;
        for (int k = 0; k < n; k++) {
            iq105[2 * (n_out + k) + 0] = out_i[k];
            iq105[2 * (n_out + k) + 1] = out_q[k];
        }
        n_out += n;
        if (n_out >= max_out) break;
    }
    return n_out;
}

// ---------------------------------------------------------------------------
// Preamble sync — port of dumpvdl2 demod.c got_sync() + the DM_INIT
// bookkeeping around it (demod.c:105-198, 229-249).
// ---------------------------------------------------------------------------

typedef struct {
    float ring[VDL2_SYNC_BUFLEN]; // phase of the last 160 samples
    int   ringidx;                // position of the newest entry
    float pherr[3];               // squared sync error at t, t-3, t-6
    float prev_dphi;              // freq estimate from the previous attempt
    // Outputs on accept:
    float dphi;       // rad/symbol carrier offset
    int   vertex_off; // samples back from the current sample to the vertex
} vdl2_sync_t;

// Linear-regression constants over the 16 preamble symbols
// (demod.c:81-96 demod_sync_init).
static float s_lr_X[VDL2_PREAMBLE_SYMS];
static float s_lr_denom;
static int   s_lr_init = 0;

static void sync_lr_init(void)
{
    if (s_lr_init) return;
    float mean_x = 0.f;
    for (int i = 0; i < VDL2_PREAMBLE_SYMS; i++)
        mean_x += (float)i;
    mean_x /= (float)VDL2_PREAMBLE_SYMS;
    s_lr_denom = 0.f;
    for (int i = 0; i < VDL2_PREAMBLE_SYMS; i++) {
        s_lr_X[i] = (float)i - mean_x;
        s_lr_denom += s_lr_X[i] * s_lr_X[i];
    }
    s_lr_init = 1;
}

static float calc_para_vertex(float x, int d, float y1, float y2, float y3)
{
    // demod.c:98-103.
    float denom = (float)(d * 2 * d * (-d));
    float a = (x * (y2 - y1) + (x - (float)d) * (y1 - y3) +
               (x - 2.f * (float)d) * (y3 - y2)) /
              denom;
    float b = (x * x * (y1 - y2) +
               (x - (float)d) * (x - (float)d) * (y3 - y1) +
               (x - 2.f * (float)d) * (x - 2.f * (float)d) * (y2 - y3)) /
              denom;
    if (a == 0.f || !isfinite(a) || !isfinite(b)) return 0.f;
    return -b / (2.f * a);
}

static bool sync_attempt(vdl2_sync_t *s)
{
    // demod.c:105-198 got_sync(), burst-local state.
    float errvec[VDL2_PREAMBLE_SYMS];
    float errvec_mean, unwrap = 0.f;
    float prev_err = errvec_mean = errvec[0] =
        s->ring[(s->ringidx + VDL2_SPS) % VDL2_SYNC_BUFLEN] -
        vdl2_preamble_phase[0];
    for (int i = 1; i < VDL2_PREAMBLE_SYMS; i++) {
        float cur_err =
            s->ring[(s->ringidx + (i + 1) * VDL2_SPS) % VDL2_SYNC_BUFLEN] -
            vdl2_preamble_phase[i];
        float errdiff = cur_err - prev_err;
        prev_err      = cur_err;
        if (errdiff > VD_PI) {
            unwrap -= 2.f * VD_PI;
        } else if (errdiff < -VD_PI) {
            unwrap += 2.f * VD_PI;
        }
        errvec[i] = cur_err + unwrap;
        errvec_mean += errvec[i];
    }
    errvec_mean /= (float)VDL2_PREAMBLE_SYMS;
    for (int i = 0; i < VDL2_PREAMBLE_SYMS; i++)
        errvec[i] -= errvec_mean;

    float freq_err = 0.f;
    for (int i = 0; i < VDL2_PREAMBLE_SYMS; i++)
        freq_err += s_lr_X[i] * errvec[i];
    freq_err /= s_lr_denom;

    float err  = 0.f;
    s->pherr[0] = 0.f;
    for (int i = 0; i < VDL2_PREAMBLE_SYMS; i++) {
        err = errvec[i] - freq_err * s_lr_X[i];
        s->pherr[0] += err * err;
    }

    if (s->pherr[1] < VDL2_SYNC_THRESHOLD && s->pherr[0] > s->pherr[1]) {
        // Passed the error minimum below threshold: sync. Parabolic
        // vertex over the last three attempts locates the symbol-clock
        // origin (demod.c:173-192). x = 0 is the current attempt.
        float vertex_x = calc_para_vertex(0.f, VDL2_SYNC_SKIP, s->pherr[2],
                                          s->pherr[1], s->pherr[0]);
        int off = (int)lroundf(-vertex_x);
        if (off < 0) off = 0;
        if (off > 2 * VDL2_SYNC_SKIP) off = 2 * VDL2_SYNC_SKIP;
        s->vertex_off = off;
        s->dphi       = s->prev_dphi;
        s->pherr[1] = s->pherr[2] = VDL2_PHERR_MAX;
        return true;
    }
    s->pherr[2]  = s->pherr[1];
    s->pherr[1]  = s->pherr[0];
    s->prev_dphi = freq_err;
    return false;
}

// ---------------------------------------------------------------------------
// Symbol reader — dumpvdl2 demod.c DM_SYNC state (demod.c:251-284):
// strobe every SPS samples, differential phase minus the preamble's
// frequency estimate, round to the nearest pi/4 grid point.
// ---------------------------------------------------------------------------

typedef struct {
    const int16_t *iq105;
    int            n105;
    int            next_strobe; // sample index of the next symbol strobe
    float          prev_phi;
    float          dphi; // rad/symbol carrier-offset correction
    double         evm_acc;
    int            n_syms;
} vdl2_symrd_t;

// Demodulate one symbol: 3 hard bits (MSB-first) + shared confidence.
// Returns false when the window has no samples left for the strobe.
static bool symrd_next(vdl2_symrd_t *r, uint8_t bits3[3], int16_t *conf_out)
{
    if (r->next_strobe >= r->n105) return false;
    float re  = (float)r->iq105[2 * r->next_strobe + 0];
    float im  = (float)r->iq105[2 * r->next_strobe + 1];
    float phi = atan2f(im, re);
    float dphi = phi - r->prev_phi - r->dphi;
    while (dphi < 0.f)
        dphi += 2.f * VD_PI;
    while (dphi >= 2.f * VD_PI)
        dphi -= 2.f * VD_PI;
    float units = dphi / (VD_PI / 4.f);
    int   idx   = (int)lroundf(units);
    float efrac = units - (float)idx; // in [-0.5, 0.5]
    idx         = ((idx % 8) + 8) % 8;

    uint8_t g = vdl2_graycode[idx];
    bits3[0]  = (uint8_t)((g >> 2) & 1u);
    bits3[1]  = (uint8_t)((g >> 1) & 1u);
    bits3[2]  = (uint8_t)(g & 1u);
    // Confidence: distance from the decision boundary, 0 at the
    // boundary, 24576 dead-centre. Shared by the symbol's 3 bits (the
    // qpsk_demod symbol-confidence convention).
    float c = (0.5f - fabsf(efrac)) * 2.f * 24576.f;
    if (c < 0.f) c = 0.f;
    *conf_out = (int16_t)c;

    float e_rad = efrac * (VD_PI / 4.f);
    r->evm_acc += (double)e_rad * (double)e_rad;
    r->n_syms++;
    r->prev_phi = phi;
    r->next_strobe += VDL2_SPS;
    return true;
}

// ---------------------------------------------------------------------------
// Burst demod driver.
// ---------------------------------------------------------------------------

bool vdl2_demod_burst(const int16_t *iq250, int n_complex,
                      vdl2_demod_result_t *out)
{
    memset(out, 0, sizeof(*out));
    if (!iq250 || n_complex < 128) return false;
    vdl2_demod_init_once();
    sync_lr_init();

    int      max105 = n_complex * VDL2_RESAMP_INTERP / VDL2_RESAMP_DECIM + 4;
    int16_t *iq105  = (int16_t *)malloc((size_t)max105 * 2 * sizeof(int16_t));
    if (!iq105) return false;
    int n105 = resample_250_to_105(iq250, n_complex, iq105, max105);

    vdl2_sync_t ss;
    memset(&ss, 0, sizeof(ss));
    ss.pherr[1] = ss.pherr[2] = VDL2_PHERR_MAX;
    int sclk = 0;

    bool got_frame = false;
    for (int i = 0; i < n105 && !got_frame; i++) {
        ss.ringidx           = (ss.ringidx + 1) % VDL2_SYNC_BUFLEN;
        ss.ring[ss.ringidx]  = atan2f((float)iq105[2 * i + 1],
                                      (float)iq105[2 * i + 0]);
        if (++sclk < VDL2_SYNC_SKIP) continue;
        sclk = 0;
        if (i < VDL2_SYNC_BUFLEN) continue; // ring not warm yet
        if (!sync_attempt(&ss)) continue;

        // --- preamble locked. Set up the symbol reader at the vertex.
        int sync_sample = i - ss.vertex_off;
        vdl2_symrd_t rd;
        rd.iq105       = iq105;
        rd.n105        = n105;
        rd.next_strobe = sync_sample + VDL2_SPS;
        rd.prev_phi    = ss.ring[(ss.ringidx - ss.vertex_off + VDL2_SYNC_BUFLEN) %
                                 VDL2_SYNC_BUFLEN];
        rd.dphi        = ss.dphi;
        rd.evm_acc     = 0.0;
        rd.n_syms      = 0;

        // Header: 25 bits = 9 symbols (27 bits, 2 spare).
        uint8_t raw27[27];
        int16_t conf27[27];
        bool    short_window = false;
        for (int s = 0; s < 9; s++) {
            int16_t c;
            if (!symrd_next(&rd, &raw27[3 * s], &c)) {
                short_window = true;
                break;
            }
            conf27[3 * s + 0] = conf27[3 * s + 1] = conf27[3 * s + 2] = c;
        }
        if (short_window) continue; // resume scanning (window nearly over)

        uint8_t  hdr_bits[25];
        memcpy(hdr_bits, raw27, 25);
        uint16_t lfsr_hdr = VDL2_LFSR_IV;
        vdl2_scramble(hdr_bits, VDL2_HDR_BITS, &lfsr_hdr);
        uint32_t w = 0;
        for (int k = 0; k < VDL2_HDR_BITS; k++)
            w = (w << 1) | hdr_bits[k];
        uint32_t datalen  = 0;
        int      syndw    = vdl2_hdr_decode(&w, &datalen);
        if (syndw < 0) continue; // header uncorrectable -> keep scanning
        // Length plausibility, decode.c:227 (tighter cap if corrected).
        if ((syndw != 0 && datalen > VDL2_MAX_FRAME_BITS_CORRECTED) ||
            datalen > VDL2_MAX_FRAME_BITS)
            continue;
        int body = vdl2_burst_body_bits(datalen);
        if (body < 0) continue;

        int needed      = VDL2_HDR_BITS + body;
        int needed_syms = (needed + VDL2_BPS - 1) / VDL2_BPS;

        uint8_t *bits = (uint8_t *)malloc((size_t)needed);
        int16_t *soft = (int16_t *)malloc((size_t)needed * sizeof(int16_t));
        if (!bits || !soft) {
            free(bits);
            free(soft);
            break; // OOM: give up on the window
        }
        int n_avail = needed < 27 ? needed : 27;
        memcpy(bits, raw27, (size_t)n_avail);
        memcpy(soft, conf27, (size_t)n_avail * sizeof(int16_t));

        for (int s = 9; s < needed_syms; s++) {
            uint8_t b3[3];
            int16_t c;
            if (!symrd_next(&rd, b3, &c)) break; // truncated by window end
            for (int k = 0; k < 3 && n_avail < needed; k++) {
                bits[n_avail] = b3[k];
                soft[n_avail] = c;
                n_avail++;
            }
        }

        // Descramble everything demodulated (fresh LFSR — keystream is
        // a pure function of bit position), then stamp soft signs from
        // the DESCRAMBLED hard decisions.
        uint16_t lfsr = VDL2_LFSR_IV;
        vdl2_scramble(bits, n_avail, &lfsr);
        for (int k = 0; k < n_avail; k++)
            soft[k] = bits[k] ? (int16_t)-soft[k] : soft[k];

        out->bits            = bits;
        out->soft_bits       = soft;
        out->n_bits          = n_avail;
        out->n_bits_needed   = needed;
        out->complete        = (n_avail == needed);
        out->datalen_bits    = datalen;
        out->hdr_synd_weight = syndw;
        out->sync_offset     = sync_sample;
        out->cfo_hz = ss.dphi * (float)VDL2_SYMBOL_RATE_HZ / (2.f * VD_PI);
        float evm =
            (rd.n_syms > 0) ? sqrtf((float)(rd.evm_acc / rd.n_syms)) : 0.f;
        out->evm_rms = evm;
        // Phase-noise SNR proxy: sigma_phi ~ 1/sqrt(SNR) for small
        // errors -> SNR_dB ~ -20 log10(sigma_phi). Clamped to a sane
        // ceiling for near-zero EVM.
        if (evm < 1e-3f) evm = 1e-3f;
        out->snr_db = -20.f * log10f(evm);
        // Consumed input through the last strobe read (250 k domain).
        int end105 = rd.next_strobe;
        if (end105 > n105) end105 = n105;
        int consumed = (int)(((int64_t)end105 * VDL2_RESAMP_DECIM) /
                             VDL2_RESAMP_INTERP);
        if (consumed > n_complex) consumed = n_complex;
        if (consumed < 1) consumed = 1;
        out->consumed_complex_250k = consumed;
        got_frame = true;
    }

    free(iq105);
    return got_frame;
}
