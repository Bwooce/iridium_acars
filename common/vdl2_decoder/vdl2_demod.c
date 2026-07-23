// See vdl2_demod.h. Burst-mode port of dumpvdl2's phase-domain D8PSK
// demodulator (src/demod.c, src/decode.c, src/bitstream.c of
// https://github.com/szpajder/dumpvdl2). Every protocol constant below
// carries a file:line citation to that tree; the DSP structure is
// re-implemented for this repo's burst-window model (dumpvdl2 is a
// continuous-stream demod).

#include "vdl2_demod.h"

#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "firmr_s16.h" // common/iridium_decoder — host-proven polyphase core

// Large per-burst transients — the resampled iq105 window (worst case
// ~110 KB for a full tagger window) and the demodulated bits/soft
// vectors (up to ~17 KB / ~34 KB at the spec-max transmission length)
// — go to PSRAM on target. Internal SRAM is almost fully committed to
// the USB URB pool (DMA-INT budget memory note, ~40 KB free), so a
// plain malloc() of any of these would fail or starve the USB path.
// free() releases heap_caps memory fine, so ownership handoff to the
// emit callback is unchanged. Host build: no heap_caps — fall back to
// malloc() (same __has_include guard family as bch_decoder.c). The
// small coefficient/scratch allocations in vdl2_lpf_design_q15 stay on
// the default heap (a few hundred bytes, freed before return).
#if __has_include("esp_heap_caps.h")
#include "esp_heap_caps.h"
#define vd_malloc_psram(sz) heap_caps_malloc((sz), MALLOC_CAP_SPIRAM)
#else
#define vd_malloc_psram(sz) malloc(sz)
#endif

// EXT_RAM_BSS_ATTR for the static preamble-regression buffer below (tiny,
// cold, single-consumer — DMA-INT/internal-.bss hygiene, same guard pattern
// as vdl2_l2.c:16-21). Host build: plain .bss (no EXT_RAM_BSS_ATTR there).
#if __has_include("esp_attr.h")
#include "esp_attr.h"
#endif
#ifndef EXT_RAM_BSS_ATTR
#define EXT_RAM_BSS_ATTR
#endif

#define VD_PI 3.14159265358979323846f

// Cumulative demod counters (never reset; the /status pattern, mirroring
// vdl2_l2_stats_t). Single writer = the demodulating worker task; torn
// reads benign for diagnostics.
static vdl2_demod_stats_t s_demod_stats;

void vdl2_demod_get_stats(vdl2_demod_stats_t *out)
{
    if (out) *out = s_demod_stats;
}

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

// Per-bit Gray-8PSK soft-demapping (see header). For each bit, find the
// nearest constellation decision boundary that flips it and scale the
// distance to [0, 24576]. Boundaries live at the midpoint j+0.5 of any
// adjacent phase pair (j, j+1) whose Gray codes differ in that bit, so
// the mapping stays correct if vdl2_graycode[] changes.
void vdl2_softbit_conf(int idx, float efrac, int16_t conf3[3])
{
    float u = (float)idx + efrac; // received position, pi/4 units, [0,8)
    for (int b = 0; b < 3; b++) {
        int   shift = 2 - b; // bit 0 is MSB (g>>2), bit 2 is LSB (g&1)
        float best  = 8.f;   // min boundary distance for this bit
        for (int j = 0; j < 8; j++) {
            int jn = (j + 1) & 7;
            if (((vdl2_graycode[j] >> shift) & 1u) ==
                ((vdl2_graycode[jn] >> shift) & 1u))
                continue; // no bit-b boundary between j and j+1
            float d = fabsf(u - ((float)j + 0.5f));
            if (d > 4.f) d = 8.f - d; // wrap around the circle
            if (d < best) best = d;
        }
        float c = best * 2.f * 24576.f;
        if (c < 0.f) c = 0.f;
        if (c > 24576.f) c = 24576.f;
        conf3[b] = (int16_t)c;
    }
}

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

// Number of least-reliable header bits the Chase fallback probes. 3 ->
// 2^3-1 = 7 non-empty flip patterns (the empty pattern is the hard path
// that already failed). The (25,20) code guarantees only single-error
// correction, so a genuine 2-3-bit header error is exactly what a hard
// syndrome decode cannot resolve — flipping the weakest bits first moves
// the candidate onto a decodable coset.
#define VDL2_HDR_SOFT_M 3
// Meaningful header bits (the 3 reserved MSBs are forced to 0 by
// vdl2_hdr_decode, so they are never error positions worth flipping).
#define VDL2_HDR_DATA_BITS (VDL2_HDR_TRLEN_BITS + VDL2_HDR_FEC_BITS) // 22

int vdl2_hdr_decode_soft(uint32_t *hdr, const int16_t *conf_air,
                         uint32_t *datalen_bits)
{
    // Received word, reserved bits dropped (mirrors vdl2_hdr_decode's
    // mask). Header bit k (air order, MSB-first) sits at w-position
    // VDL2_HDR_BITS-1-k; build a confidence array indexed by w-position
    // over the 22 meaningful bits.
    uint32_t w0 = *hdr & ((1u << VDL2_HDR_DATA_BITS) - 1u);
    int16_t  confp[VDL2_HDR_DATA_BITS];
    for (int p = 0; p < VDL2_HDR_DATA_BITS; p++) {
        int k    = VDL2_HDR_BITS - 1 - p; // air-bit index for w-position p
        confp[p] = conf_air ? conf_air[k] : (int16_t)24576;
    }

    // Least-reliable positions (lowest confidence first), insertion sort
    // into a tiny fixed array.
    int weak[VDL2_HDR_SOFT_M];
    int nweak = 0;
    for (int p = 0; p < VDL2_HDR_DATA_BITS; p++) {
        int ins = nweak;
        while (ins > 0 && confp[p] < confp[weak[ins - 1]]) ins--;
        if (ins >= VDL2_HDR_SOFT_M) continue; // not weak enough
        int last = (nweak < VDL2_HDR_SOFT_M) ? nweak : VDL2_HDR_SOFT_M - 1;
        for (int j = last; j > ins; j--) weak[j] = weak[j - 1];
        weak[ins] = p;
        if (nweak < VDL2_HDR_SOFT_M) nweak++;
    }

    uint32_t best_w = 0, best_len = 0;
    long     best_metric = LONG_MAX;
    int      best_syndw  = -1;

    // Enumerate every non-empty subset of the weak positions. For each,
    // flip those bits, run the SAME hard syndrome decode (its own
    // single-bit correction stacks on top), and keep the valid header
    // (zero reserved bits + plausible length) closest to the received
    // word in reliability-weighted distance.
    for (uint32_t mask = 1; mask < (1u << nweak); mask++) {
        uint32_t test = 0;
        for (int i = 0; i < nweak; i++)
            if (mask & (1u << i)) test |= (1u << weak[i]);
        uint32_t w   = w0 ^ test;
        uint32_t len = 0;
        int      sw  = vdl2_hdr_decode(&w, &len);
        if (sw < 0) continue;
        // Length plausibility — identical gate to the demod's hard path
        // (tighter cap when any correction was applied).
        if ((sw != 0 && len > VDL2_MAX_FRAME_BITS_CORRECTED) ||
            len > VDL2_MAX_FRAME_BITS)
            continue;
        if (vdl2_burst_body_bits(len) < 0) continue;
        // Soft metric: total unreliability of the bits the FINAL codeword
        // differs from the received word by (Chase flips + the syndrome
        // correction combined). Lowest = most likely.
        uint32_t diff   = (w ^ w0) & ((1u << VDL2_HDR_DATA_BITS) - 1u);
        long     metric = 0;
        for (int p = 0; p < VDL2_HDR_DATA_BITS; p++)
            if (diff & (1u << p)) metric += confp[p];
        if (metric < best_metric) {
            best_metric = metric;
            best_w      = w;
            best_len    = len;
            best_syndw  = sw;
        }
    }

    if (best_syndw < 0) return -1;
    *hdr          = best_w;
    *datalen_bits = best_len;
    return best_syndw;
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

// Root-raised-cosine prototype tap, x in symbol periods, standard RRC
// impulse response (peak h(0) = 1 - alpha + 4 alpha / pi; the two
// removable singularities handled explicitly). Absolute scale is
// irrelevant here — the per-phase DC normalisation below sets the
// gain.
static double rrc_tap(double x, double alpha)
{
    double ax = fabs(x);
    const double pi = 3.14159265358979323846;
    if (ax < 1e-9) return 1.0 - alpha + 4.0 * alpha / pi;
    double q = 4.0 * alpha * ax;
    if (fabs(q - 1.0) < 1e-6) {
        // |x| = 1/(4 alpha) limit.
        double s = sin(pi / (4.0 * alpha));
        double c = cos(pi / (4.0 * alpha));
        return (alpha / sqrt(2.0)) *
               ((1.0 + 2.0 / pi) * s + (1.0 - 2.0 / pi) * c);
    }
    return (sin(pi * ax * (1.0 - alpha)) +
            4.0 * alpha * ax * cos(pi * ax * (1.0 + alpha))) /
           (pi * ax * (1.0 - q * q));
}

bool vdl2_rrc_design_q15(int16_t *coeffs, int delay_size, int interp,
                         double vsamples_per_symbol, double alpha)
{
    const int nproto = delay_size * interp;
    const int center = nproto / 2;

    double *w         = (double *)malloc(sizeof(double) * (size_t)nproto);
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
        w[k] = rrc_tap((double)(k - center) / vsamples_per_symbol, alpha);
        sum_phase[k % interp] += w[k];
    }
    // Per-phase DC gain 1.0 — identical convention to
    // vdl2_lpf_design_q15 (firmr_s16 shift=0 divides by 2^15).
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
//
// Why a FLAT-passband LPF and not an RRC matched filter (V4 A/B,
// 2026-07-22): the transmitted VDL2 pulse is the FULL raised cosine,
// alpha 0.6 (ICAO Annex 10 Vol III) — Nyquist on its own — so the
// zero-ISI receive filter is anything flat across the signal band;
// root-shaping at the receiver creates ISI instead of removing noise.
// Measured on the sigidwiki golden capture (test_vdl2_real_capture,
// fractional timing in all rows):
//     RX filter                      RS-clean frames
//     Kaiser 9 kHz, 144 taps/phase   46 of 47   <- production
//     Kaiser 9 kHz,  48 taps/phase   38         (V2 prototype length)
//     RRC alpha 0.6 matched          21         (regression)
//     RC  alpha 0.6 matched          18         (regression)
// The real V2 demod-side EVM cost was the SHORT (48 taps/phase)
// prototype — its slow transition/stopband leaked noise and images —
// not the absence of a matched shape. dumpvdl2 (2-pole Chebyshev, no
// matched filter) is consistent with this: flat-ish passband receivers
// are the correct structure for a full-RC transmitter.
// ---------------------------------------------------------------------------

// Taps/phase; prototype spans 144 input samples = 576 us. 3024 Q15
// taps = 6 KB .bss (V2: 2 KB); ~288 MACs per output sample across I+Q
// = 30.2 MMAC per second of scanned window (~8 MMAC for a full 260 ms
// tagger window) — VDL2-only, scalar, no PIE.
#define VDL2_RS_DSIZE 144

// 6 KB Q15 tap bank — CPU-only (firmr_s16 is the scalar ANSI polyphase
// FIR, no PIE/esp-dsp vector read of the coeffs), cold, single-consumer.
// -> PSRAM on target to keep internal DMA-INT free for the USB URB pool
// (same DMA-INT/internal-.bss hygiene as s_lr_X above). Host build: the
// EXT_RAM_BSS_ATTR guard above makes this a plain .bss.
static EXT_RAM_BSS_ATTR int16_t s_rs_coeffs[VDL2_RS_DSIZE * VDL2_RESAMP_INTERP];
static int     s_init_done = 0;

static void vdl2_demod_init_once(void)
{
    if (s_init_done) return;
    // 9 kHz cutoff in cycles per virtual (interp x fs_in) sample.
    (void)vdl2_lpf_design_q15(s_rs_coeffs, VDL2_RS_DSIZE, VDL2_RESAMP_INTERP,
                              9000.0 /
                                  ((double)VDL2_FS_IN_HZ * VDL2_RESAMP_INTERP),
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
    float dphi;        // rad/symbol carrier offset
    int   vertex_off;  // samples back from the current sample to the vertex
    float vertex_frac; // sub-sample residual: true vertex = current
                       // sample - vertex_off - vertex_frac, in
                       // [-0.5, 0.5] samples
} vdl2_sync_t;

// Linear-regression constants over the 16 preamble symbols
// (demod.c:81-96 demod_sync_init).
static EXT_RAM_BSS_ATTR float s_lr_X[VDL2_PREAMBLE_SYMS];
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
        // dumpvdl2 rounds the vertex to the nearest whole sample (up to
        // T/20 timing error); we keep the fractional part and strobe at
        // the true instant via interpolation (symrd below).
        float vertex_x = calc_para_vertex(0.f, VDL2_SYNC_SKIP, s->pherr[2],
                                          s->pherr[1], s->pherr[0]);
        float off_f = -vertex_x;
        if (off_f < 0.f) off_f = 0.f;
        if (off_f > 2.f * VDL2_SYNC_SKIP) off_f = 2.f * VDL2_SYNC_SKIP;
        int off = (int)lroundf(off_f);
        s->vertex_off  = off;
        s->vertex_frac = off_f - (float)off;
        s->dphi        = s->prev_dphi;
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
// frequency estimate, round to the nearest pi/4 grid point. Departure
// from the reference: dumpvdl2 strobes at whole samples (T/10 grid);
// we strobe at the FRACTIONAL symbol instant from the sync parabola,
// interpolating the matched-filter output with a 4-point (cubic
// Lagrange) kernel — at 10 samples/symbol its passband error over the
// 8.4 kHz signal is negligible.
// ---------------------------------------------------------------------------

// Interpolate one complex sample at fractional index pos (values
// outside [1, n-3) fall back to edge-clamped neighbours; callers stop
// strobing before pos leaves the window).
static void interp_iq_cubic(const int16_t *iq, int n, float pos,
                            float *re, float *im)
{
    int   n0 = (int)floorf(pos);
    float mu = pos - (float)n0;
    int   im1 = n0 - 1, ip1 = n0 + 1, ip2 = n0 + 2;
    if (im1 < 0) im1 = 0;
    if (n0 < 0) n0 = 0;
    if (n0 > n - 1) n0 = n - 1;
    if (ip1 > n - 1) ip1 = n - 1;
    if (ip2 > n - 1) ip2 = n - 1;
    float c_m1 = -mu * (mu - 1.f) * (mu - 2.f) * (1.f / 6.f);
    float c_0  = (mu + 1.f) * (mu - 1.f) * (mu - 2.f) * 0.5f;
    float c_p1 = -(mu + 1.f) * mu * (mu - 2.f) * 0.5f;
    float c_p2 = (mu + 1.f) * mu * (mu - 1.f) * (1.f / 6.f);
    *re = c_m1 * (float)iq[2 * im1 + 0] + c_0 * (float)iq[2 * n0 + 0] +
          c_p1 * (float)iq[2 * ip1 + 0] + c_p2 * (float)iq[2 * ip2 + 0];
    *im = c_m1 * (float)iq[2 * im1 + 1] + c_0 * (float)iq[2 * n0 + 1] +
          c_p1 * (float)iq[2 * ip1 + 1] + c_p2 * (float)iq[2 * ip2 + 1];
}

typedef struct {
    const int16_t *iq105;
    int            n105;
    float          strobe_pos; // fractional sample index of the next strobe
    float          prev_phi;
    float          dphi; // rad/symbol carrier-offset correction
    double         evm_acc;
    int            n_syms;
} vdl2_symrd_t;

// Demodulate one symbol: 3 hard bits (MSB-first) + PER-BIT confidence.
// Returns false when the window has no samples left for the strobe.
//
// Gray-coded 8PSK soft-demapping: each bit's confidence is the angular
// distance (in pi/4 units) from the received phase to the NEAREST
// constellation decision boundary that flips THAT bit, scaled so a
// boundary-adjacent bit reads 0 and a >=0.5-unit margin saturates at
// 24576. Because adjacent phases differ in exactly one bit, the bit
// flipping toward the received offset direction keeps the old shared
// low value (its boundary is the globally nearest, at 0.5-|efrac|),
// while the other two bits earn their larger, farther-boundary margins.
// Boundaries are derived from vdl2_graycode[] at runtime (scan adjacent
// phase pairs) so the mapping stays correct if the constant changes.
static bool symrd_next(vdl2_symrd_t *r, uint8_t bits3[3], int16_t conf3[3])
{
    if (r->strobe_pos >= (float)r->n105) return false;
    float re, im;
    interp_iq_cubic(r->iq105, r->n105, r->strobe_pos, &re, &im);
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
    bits3[0]  = (uint8_t)((g >> 2) & 1u); // UNCHANGED hard decisions
    bits3[1]  = (uint8_t)((g >> 1) & 1u);
    bits3[2]  = (uint8_t)(g & 1u);

    // Per-bit confidence via nearest bit-flip boundary.
    vdl2_softbit_conf(idx, efrac, conf3);

    float e_rad = efrac * (VD_PI / 4.f);
    r->evm_acc += (double)e_rad * (double)e_rad;
    r->n_syms++;
    r->prev_phi = phi;
    r->strobe_pos += (float)VDL2_SPS;
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
    int16_t *iq105  = (int16_t *)vd_malloc_psram((size_t)max105 * 2 *
                                                 sizeof(int16_t));
    if (!iq105) return false; // PSRAM pressure: drop the burst, no crash
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

        // --- preamble locked. Set up the symbol reader at the TRUE
        // (fractional) vertex: the last preamble symbol's strobe. The
        // reference phase comes from the interpolated matched-filter
        // output at the same fractional instant, so the first data
        // symbol's differential is timing-consistent with the rest.
        int   sync_sample = i - ss.vertex_off;
        float vertex_pos  = (float)sync_sample - ss.vertex_frac;
        vdl2_symrd_t rd;
        rd.iq105      = iq105;
        rd.n105       = n105;
        rd.strobe_pos = vertex_pos + (float)VDL2_SPS;
        {
            float vre, vim;
            interp_iq_cubic(iq105, n105, vertex_pos, &vre, &vim);
            rd.prev_phi = atan2f(vim, vre);
        }
        rd.dphi    = ss.dphi;
        rd.evm_acc = 0.0;
        rd.n_syms  = 0;

        // Header: 25 bits = 9 symbols (27 bits, 2 spare).
        uint8_t raw27[27];
        int16_t conf27[27];
        bool    short_window = false;
        for (int s = 0; s < 9; s++) {
            int16_t c3[3];
            if (!symrd_next(&rd, &raw27[3 * s], c3)) {
                short_window = true;
                break;
            }
            conf27[3 * s + 0] = c3[0];
            conf27[3 * s + 1] = c3[1];
            conf27[3 * s + 2] = c3[2];
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
        uint32_t w_orig   = w; // pre-correction copy for the soft fallback
        int      syndw    = vdl2_hdr_decode(&w, &datalen);
        if (syndw < 0) {
            // Hard (25,20) syndrome decode rejected (correction landed in
            // the reserved bits). Retry with the Chase-style soft fallback
            // over the least-reliable header bits before giving up.
            w     = w_orig;
            syndw = vdl2_hdr_decode_soft(&w, conf27, &datalen);
            if (syndw < 0) continue; // still uncorrectable -> keep scanning
            s_demod_stats.soft_hdr_rescued++;
        }
        // Length plausibility, decode.c:227 (tighter cap if corrected).
        if ((syndw != 0 && datalen > VDL2_MAX_FRAME_BITS_CORRECTED) ||
            datalen > VDL2_MAX_FRAME_BITS)
            continue;
        int body = vdl2_burst_body_bits(datalen);
        if (body < 0) continue;

        int needed      = VDL2_HDR_BITS + body;
        int needed_syms = (needed + VDL2_BPS - 1) / VDL2_BPS;

        uint8_t *bits = (uint8_t *)vd_malloc_psram((size_t)needed);
        int16_t *soft = (int16_t *)vd_malloc_psram((size_t)needed *
                                                   sizeof(int16_t));
        if (!bits || !soft) {
            free(bits);
            free(soft);
            break; // OOM: give up on the window (no frame emitted)
        }
        int n_avail = needed < 27 ? needed : 27;
        memcpy(bits, raw27, (size_t)n_avail);
        memcpy(soft, conf27, (size_t)n_avail * sizeof(int16_t));

        for (int s = 9; s < needed_syms; s++) {
            uint8_t b3[3];
            int16_t c3[3];
            if (!symrd_next(&rd, b3, c3)) break; // truncated by window end
            for (int k = 0; k < 3 && n_avail < needed; k++) {
                bits[n_avail] = b3[k];
                soft[n_avail] = c3[k];
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
        int end105 = (int)rd.strobe_pos;
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
