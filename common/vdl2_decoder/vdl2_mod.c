// See vdl2_mod.h. Test-fixture modulator; float synthesis is fine here
// (host-only usage), int16 output matches the pipeline input contract.

#include "vdl2_mod.h"

#include <math.h>
#include <string.h>

#include "vdl2_demod.h" // protocol constants + vdl2_hdr_encode/body_bits

#define VM_PI 3.14159265358979323846
#define VM_PULSE_SPAN_SYMS 6 // RC pulse truncated at +/-6 T

void vdl2_mod_params_default(vdl2_mod_params_t *p)
{
    memset(p, 0, sizeof(*p));
    p->fs_hz       = 250000.0;
    p->cfo_hz      = 0.0;
    p->timing_frac = 0.0;
    p->amp         = 8000.0;
    p->awgn_sigma  = 0.0;
    p->phase0_rad  = 0.4;
    p->rolloff     = 0.6;
    p->ramp_syms   = 5;
    p->pad_pre     = 400;
    p->pad_post    = 400;
    p->seed        = 1;
}

// Independent scrambler implementation (same spec as
// vdl2_demod.c:vdl2_scramble — bitstream.c:100-102 — but written
// against the polynomial description, not shared code, so the
// round-trip cross-checks the two).
static uint16_t mod_lfsr_next_bit(uint16_t *state)
{
    uint16_t fb = (uint16_t)(((*state) ^ ((*state) >> 14)) & 1u);
    *state      = (uint16_t)(((*state) >> 1) | (fb << 14));
    return fb;
}

// xorshift32 PRNG + Box-Muller gaussian.
static uint32_t xs32(uint32_t *s)
{
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x ? x : 0x9E3779B9u;
    return *s;
}

static double frand01(uint32_t *s)
{
    return ((double)(xs32(s) >> 8) + 0.5) / 16777216.0;
}

static double gauss(uint32_t *s)
{
    double u1 = frand01(s);
    double u2 = frand01(s);
    return sqrt(-2.0 * log(u1)) * cos(2.0 * VM_PI * u2);
}

// Raised-cosine pulse, x in symbol periods. g(0)=1, g(k)=0 for integer
// k != 0 (ISI-free at strobes).
static double rc_pulse(double x, double alpha)
{
    double ax = fabs(x);
    double s  = (ax < 1e-9) ? 1.0 : sin(VM_PI * x) / (VM_PI * x);
    double d  = 1.0 - (2.0 * alpha * x) * (2.0 * alpha * x);
    if (fabs(d) < 1e-6) {
        // Singularity at |x| = 1/(2 alpha): limit = (pi/4) sinc(1/(2a)).
        double xa = 1.0 / (2.0 * alpha);
        return (VM_PI / 4.0) * (sin(VM_PI * xa) / (VM_PI * xa));
    }
    return s * cos(VM_PI * alpha * x) / d;
}

// Inverse of the receive Gray map vdl2_graycode: bits value -> phase
// index. Derived at runtime from the demod's table so the two can
// never diverge silently.
static void inv_gray(uint8_t inv[8])
{
    for (int i = 0; i < 8; i++)
        inv[vdl2_graycode[i]] = (uint8_t)i;
}

#define VM_MAX_SYMS 8192

int vdl2_mod_burst(uint32_t datalen_bits, const uint8_t *body_bits,
                   const vdl2_mod_params_t *p,
                   int16_t *out_iq, int max_complex)
{
    if (!body_bits || !p || !out_iq) return -1;
    int body = vdl2_burst_body_bits(datalen_bits);
    if (body < 0) return -1;
    int n_bits = VDL2_HDR_BITS + body;
    int n_data_syms = (n_bits + VDL2_BPS - 1) / VDL2_BPS;
    int n_syms = p->ramp_syms + VDL2_PREAMBLE_SYMS + n_data_syms;
    if (n_syms > VM_MAX_SYMS) return -1;

    // ---- bit vector: header + body, then scramble ----
    uint8_t bits[VM_MAX_SYMS * VDL2_BPS];
    uint32_t hdr = vdl2_hdr_encode(datalen_bits);
    for (int k = 0; k < VDL2_HDR_BITS; k++)
        bits[k] = (uint8_t)((hdr >> (VDL2_HDR_BITS - 1 - k)) & 1u);
    memcpy(bits + VDL2_HDR_BITS, body_bits, (size_t)body);
    // Pad to a whole symbol with zeros (transmitted, scrambled, ignored
    // by the receiver — dumpvdl2 reads only requested_bits).
    int n_padded = n_data_syms * VDL2_BPS;
    for (int k = n_bits; k < n_padded; k++)
        bits[k] = 0;
    uint16_t lfsr = VDL2_LFSR_IV;
    for (int k = 0; k < n_padded; k++)
        bits[k] ^= (uint8_t)mod_lfsr_next_bit(&lfsr);

    // ---- symbol phases + amplitudes ----
    static double phase[VM_MAX_SYMS];
    static double s_amp[VM_MAX_SYMS];
    uint8_t inv[8];
    inv_gray(inv);
    int n = 0;
    // Ramp-up: constant phase, amplitude rising to 1.
    for (int k = 0; k < p->ramp_syms; k++) {
        phase[n] = p->phase0_rad;
        s_amp[n] = (double)(k + 1) / (double)(p->ramp_syms + 1);
        n++;
    }
    // Preamble: absolute cumulative phases from the shared table.
    for (int k = 0; k < VDL2_PREAMBLE_SYMS; k++) {
        phase[n] = p->phase0_rad + (double)vdl2_preamble_phase[k];
        s_amp[n] = 1.0;
        n++;
    }
    // Data: differential D8PSK from the last preamble symbol.
    double cur = phase[n - 1];
    for (int k = 0; k < n_data_syms; k++) {
        uint8_t v = (uint8_t)((bits[3 * k] << 2) | (bits[3 * k + 1] << 1) |
                              bits[3 * k + 2]);
        cur += (double)inv[v] * (VM_PI / 4.0);
        phase[n] = cur;
        s_amp[n] = 1.0;
        n++;
    }

    // ---- waveform synthesis ----
    double T  = 1.0 / (double)VDL2_SYMBOL_RATE_HZ;
    double Ts = 1.0 / p->fs_hz;
    // Strobe time of symbol k.
    double t0 = (double)p->pad_pre * Ts + (double)VM_PULSE_SPAN_SYMS * T +
                p->timing_frac * T;
    double t_end = t0 + (double)(n - 1) * T + (double)VM_PULSE_SPAN_SYMS * T;
    int n_out = (int)(t_end / Ts) + 1 + p->pad_post;
    if (n_out > max_complex) return -1;

    uint32_t rng = p->seed ? p->seed : 1u;
    for (int i = 0; i < n_out; i++) {
        double t  = (double)i * Ts;
        double re = 0.0, im = 0.0;
        // Symbols whose pulse reaches t.
        double ks = (t - t0) / T;
        int    k0 = (int)floor(ks - VM_PULSE_SPAN_SYMS);
        int    k1 = (int)ceil(ks + VM_PULSE_SPAN_SYMS);
        if (k0 < 0) k0 = 0;
        if (k1 > n - 1) k1 = n - 1;
        for (int k = k0; k <= k1; k++) {
            double x = (t - (t0 + (double)k * T)) / T;
            double g = rc_pulse(x, p->rolloff) * s_amp[k];
            re += g * cos(phase[k]);
            im += g * sin(phase[k]);
        }
        // Carrier offset.
        if (p->cfo_hz != 0.0) {
            double c = cos(2.0 * VM_PI * p->cfo_hz * t);
            double s = sin(2.0 * VM_PI * p->cfo_hz * t);
            double r2 = re * c - im * s;
            double i2 = im * c + re * s;
            re = r2;
            im = i2;
        }
        re *= p->amp;
        im *= p->amp;
        if (p->awgn_sigma > 0.0) {
            re += p->awgn_sigma * gauss(&rng);
            im += p->awgn_sigma * gauss(&rng);
        }
        if (re > 32767.0) re = 32767.0;
        if (re < -32768.0) re = -32768.0;
        if (im > 32767.0) im = 32767.0;
        if (im < -32768.0) im = -32768.0;
        out_iq[2 * i + 0] = (int16_t)lrint(re);
        out_iq[2 * i + 1] = (int16_t)lrint(im);
    }
    return n_out;
}
