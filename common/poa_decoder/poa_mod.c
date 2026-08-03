// See poa_mod.h. Synthetic POA modulator — the inverse of acarsdec's demod.
// Host/smoke test fixture only; nothing on the device transmits.
//
// BIT <-> PHASE ENCODING (the crux — this inverts msk.c exactly)
//   The demod recovers one bit per strobe. At strobe k it forms the unit
//   phasor v = exp(i*phi_k) (phi_k a multiple of pi/2 for clean MSK), then
//   decides on an axis that alternates with k and a polarity that flips every
//   two bits (the MskS&1 / MskS&2 logic in msk.c::demodMSK):
//       phi_k = (pi/2) * m_k,   m_k in {0,1,2,3}
//       recovered bit b_k = 1  <=>  m_k == (k + 2*(1 - b_k)) mod 4
//   i.e. the intended phase state for bit k is  m_k = (k + 2*(1-b_k)) mod 4.
//   Differencing gives the per-bit MSK deviation with NO state table:
//       e_k = m_{k+1} - m_k = +1 if b_k == b_{k+1}, else -1   (mod 4)
//   so the continuous MSK phase is just theta_dev advancing by e*(pi/2) per
//   bit. A balanced pre-key (recovered pattern 1,1,0,0,...) gives e alternating
//   +1,-1 — a net-zero-deviation tone the decision-directed PLL locks to; its
//   sliding-window bytes are {0x33,0x99,0xCC,0x66}, none == SYN(0x16) or
//   ~SYN(0xE9), so no false sync.
//
// STROBE-PARITY ALIGNMENT
//   The decoder's strobe counter (MskS) free-runs from decoder-create and is
//   never reset, so strobe s reads our bit j with a fixed offset delta = s - j.
//   delta even self-corrects (delta==2 just inverts every bit -> the ~SYN
//   branch flips MskS&2); delta ODD makes the demod read the wrong axis
//   (sin where we put cos) and it never syncs. We remove that 50/50 by (a)
//   emitting an exact multiple of 6250 complex samples per burst (= 6 bit
//   periods, an integer sample count at 2.5 MSPS, an EVEN bit count, so the
//   parity is identical for every back-to-back burst) and (b) an axis_parity
//   knob that pre-rotates the constellation a quarter turn.
//
//   MEASURED: axis_parity is inert — all four quarter-turn values decode
//   byte-identically (err=0, crc_fixed=0). The decision-directed PLL in
//   msk.c::demodMSK locks to whatever rotation the pre-key presents, and the
//   data is coherent with the pre-key (same rotation), so any COMMON rotation
//   is absorbed; the 6250-sample quantization independently pins the timing
//   parity. The knob is kept only to document the axis question and to give a
//   future, more rotation-sensitive decoder variant a lever. Default 0.

#include "poa_mod.h"

#include <math.h>
#include <string.h>

#define PM_PI       3.14159265358979323846
#define PM_BITRATE  2400.0
#define PM_SUBCARR  1800.0     // MSK sub-carrier (tones at 1800 +/- 600)
#define PM_QUANTUM  6250       // complex samples per 6 bits at 2.5 MSPS (exact)

#define SYN 0x16
#define SOH 0x01
#define STX 0x02
#define ETX 0x83   // 0x03 + odd parity
#define ETB 0x97   // 0x17 + odd parity

// CRC-16 (reflected CCITT, poly 0x8408) — byte-identical to poa_syndrome.h's
// update_crc / la_crc16_ccitt. Kept as an INDEPENDENT implementation (built
// from the polynomial, not the shared 256-entry table) so the round-trip
// cross-checks the CRC in both directions, mirroring vdl2_mod.c's independent
// scrambler.
static uint16_t pm_crc16(const uint8_t *buf, int len)
{
    uint16_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0x8408) : (uint16_t)(crc >> 1);
    }
    return crc;
}

// Odd parity in bit 7: SOH->0x01, SYN->0x16, STX->0x02, ETX->0x83, ETB->0x97.
static uint8_t pm_odd_parity(uint8_t v)
{
    v &= 0x7f;
    int ones = 0;
    for (int b = 0; b < 7; b++) ones += (v >> b) & 1;
    return (ones & 1) ? v : (uint8_t)(v | 0x80);
}

void poa_mod_params_default(poa_mod_params_t *p)
{
    memset(p, 0, sizeof(*p));
    p->fs_hz       = 2500000u;
    p->lo_hz       = 130800000u;
    p->chan_hz     = 131550000u;
    p->prekey_bits = 192;   // >= 128 for reliable PLL lock (msk.c PLLG is slow)
    p->postkey_bits = 24;
    // amp low (peak = amp*(1+depth) ~ 1900) so the SNR sweep stays clipping-free
    // through the decode threshold: int16 range caps representable noise, and
    // clipping must not co-limit the reported threshold. The decoder is
    // amplitude-invariant (v /= lvl+1e-8), so the threshold itself is amp-
    // independent — verified: at amp 1000 vs 4000 the clean floor is identical,
    // but at 1000 there is zero clipping down past the floor.
    p->amp         = 1000.0;
    p->mod_depth   = 0.9;
    p->axis_parity = 0;
}

int poa_mod_build_block(const poa_mod_msg_t *msg, uint8_t *out, int max_out)
{
    if (!msg || !out) return -1;
    int tlen = msg->text_len;
    if (msg->text && tlen < 0) tlen = (int)strlen(msg->text);
    if (tlen < 0) tlen = 0;
    // mode(1)+reg(7)+ack(1)+label(2)+block_id(1)+STX(1)+text+ETX/ETB(1)
    int len = 13 + tlen + 1;
    if (len > max_out) return -1;

    int i = 0;
    out[i++] = pm_odd_parity((uint8_t)msg->mode);
    for (int r = 0; r < 7; r++) {
        char c = msg->reg[r] ? msg->reg[r] : ' ';
        out[i++] = pm_odd_parity((uint8_t)c);
    }
    out[i++] = pm_odd_parity((uint8_t)msg->ack);
    out[i++] = pm_odd_parity((uint8_t)msg->label[0]);
    out[i++] = pm_odd_parity((uint8_t)msg->label[1]);
    out[i++] = pm_odd_parity((uint8_t)msg->block_id);
    out[i++] = STX;                       // txt[12] — decoder force-sets STX
    for (int t = 0; t < tlen; t++)
        out[i++] = pm_odd_parity((uint8_t)msg->text[t]);
    out[i++] = msg->final_block ? (uint8_t)ETX : (uint8_t)ETB;
    return i;
}

// Append a byte's 8 bits, LSB first (the order putbit() assembles them).
static int push_byte_bits(uint8_t *bits, int n, uint8_t byte)
{
    for (int b = 0; b < 8; b++) bits[n++] = (byte >> b) & 1;
    return n;
}

#define PM_MAX_BITS 4096   // >= prekey + 3 sync + (13+220+1)*8 + crc + postkey

int poa_mod_block(const poa_mod_msg_t *msg, const poa_mod_params_t *p,
                  int64_t phase_ref, int16_t *out_iq, int max_complex)
{
    if (!msg || !p || !out_iq) return -1;
    if (p->fs_hz % 12500u != 0u || p->fs_hz == 0u) return -1;
    if (p->mod_depth <= 0.0 || p->mod_depth >= 1.0) return -1;

    uint8_t block[256];
    int blen = poa_mod_build_block(msg, block, (int)sizeof(block));
    if (blen < 13) return -1;
    uint16_t crc = pm_crc16(block, blen);

    // ---- recovered-bit stream d[]: prekey + SYN SYN SOH + block + CRC + postkey
    static uint8_t d[PM_MAX_BITS];
    int L = 0;
    static const uint8_t prekey_pat[4] = {1, 1, 0, 0}; // balanced tone
    int pre = p->prekey_bits < 0 ? 0 : p->prekey_bits;
    for (int k = 0; k < pre; k++) d[L++] = prekey_pat[k & 3];
    L = push_byte_bits(d, L, SYN);
    L = push_byte_bits(d, L, SYN);
    L = push_byte_bits(d, L, SOH);
    for (int i = 0; i < blen; i++) L = push_byte_bits(d, L, block[i]);
    L = push_byte_bits(d, L, (uint8_t)(crc & 0xff));
    L = push_byte_bits(d, L, (uint8_t)(crc >> 8));
    int post = p->postkey_bits < 0 ? 0 : p->postkey_bits;
    for (int k = 0; k < post; k++) d[L++] = prekey_pat[k & 3];
    // Pad the tail so the total is a multiple of 6 bits (=> integer sample
    // count, even bit count -> stable strobe parity across bursts).
    while (L % 6 != 0) { d[L] = prekey_pat[L & 3]; L++; }
    if (L >= PM_MAX_BITS) return -1;

    // ---- per-bit MSK phase state m_unwrapped[k] (in quarter-turns) ----
    // m_0 from the closed form (axis_parity rotates the whole constellation);
    // thereafter accumulate e = +1 if consecutive recovered bits match else -1.
    static long mq[PM_MAX_BITS];   // theta_dev(center_k) = (pi/2) * mq[k]
    mq[0] = (p->axis_parity + 2 * (1 - d[0]));
    for (int k = 1; k < L; k++) {
        int e = (d[k - 1] == d[k]) ? 1 : -1;
        mq[k] = mq[k - 1] + e;
    }

    // ---- synthesis geometry ----
    const double fs = (double)p->fs_hz;
    const double Tb = fs / PM_BITRATE;                 // samples per bit
    const double foff = (double)p->chan_hz - (double)p->lo_hz;
    long n_out = (long)llround((double)L * Tb);        // exact multiple of 6250
    if (n_out % PM_QUANTUM != 0) {
        // Round to the quantum defensively (should already be exact).
        n_out = ((n_out + PM_QUANTUM - 1) / PM_QUANTUM) * PM_QUANTUM;
    }
    if (n_out > (long)max_complex) return -1;

    const double w_sub = 2.0 * PM_PI * PM_SUBCARR / fs; // rad/sample, sub-carrier
    const double w_off = 2.0 * PM_PI * foff / fs;       // rad/sample, channel
    const double amp = p->amp, md = p->mod_depth;

    for (long i = 0; i < n_out; i++) {
        double g = (double)(phase_ref + i);            // global sample index
        // bit-centre coordinate: centre of bit k is at local sample (k+0.5)*Tb
        double u = (double)i / Tb - 0.5;
        long kf = (long)floor(u);
        double frac = u - (double)kf;
        long ka = kf, kb = kf + 1;
        if (ka < 0) { ka = 0; kb = 0; frac = 0.0; }
        if (kb > L - 1) { ka = L - 1; kb = L - 1; frac = 0.0; }
        double theta_dev = (PM_PI / 2.0) * ((double)mq[ka] + frac * (double)(mq[kb] - mq[ka]));

        double msk = cos(w_sub * g + theta_dev);       // audio sub-carrier, [-1,1]
        double env = 1.0 + md * msk;                   // AM envelope, > 0
        double carrier = w_off * g;
        double re = amp * env * cos(carrier);
        double im = amp * env * sin(carrier);
        if (re >  32767.0) re =  32767.0;
        if (re < -32768.0) re = -32768.0;
        if (im >  32767.0) im =  32767.0;
        if (im < -32768.0) im = -32768.0;
        out_iq[2 * i + 0] = (int16_t)lrint(re);
        out_iq[2 * i + 1] = (int16_t)lrint(im);
    }
    return (int)n_out;
}

double poa_mod_signal_power(const int16_t *iq, int n_complex)
{
    if (!iq || n_complex <= 0) return 0.0;
    double acc = 0.0;
    for (int i = 0; i < n_complex; i++) {
        double re = iq[2 * i], im = iq[2 * i + 1];
        acc += re * re + im * im;
    }
    return acc / (double)n_complex;
}

double poa_mod_sigma_for_snr(double signal_power, double snr_db)
{
    double snr_lin = pow(10.0, snr_db / 10.0);
    double var = signal_power / (2.0 * snr_lin); // per component
    return sqrt(var);
}

// xorshift32 + Box-Muller (independent of vdl2_mod's copy; same idea).
static uint32_t pm_xs32(uint32_t *s)
{
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x ? x : 0x9E3779B9u;
    return *s;
}

static double pm_gauss(uint32_t *s)
{
    double u1 = ((double)(pm_xs32(s) >> 8) + 0.5) / 16777216.0;
    double u2 = ((double)(pm_xs32(s) >> 8) + 0.5) / 16777216.0;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * PM_PI * u2);
}

int poa_mod_add_awgn(int16_t *iq, int n_complex, double sigma, uint32_t seed)
{
    if (!iq || n_complex <= 0 || sigma <= 0.0) return 0;
    uint32_t rng = seed ? seed : 1u;
    int clipped = 0;
    for (int i = 0; i < 2 * n_complex; i++) {
        double v = (double)iq[i] + sigma * pm_gauss(&rng);
        if (v >  32767.0) { v =  32767.0; clipped++; }
        if (v < -32768.0) { v = -32768.0; clipped++; }
        iq[i] = (int16_t)lrint(v);
    }
    return clipped;
}
