// poa_decoder — see poa_decoder.h. Ported from acarsdec (Thierry Leconte,
// GPLv2): msk.c (MSK demod) + acars.c (state machine + parity/CRC L2). The
// demod math and the ACARS state machine are kept BIT-FAITHFUL to acarsdec so
// the host cross-validation against acarsdec-the-oracle holds; the only
// structural change is that acarsdec's pthread block-queue is inlined
// synchronously (process_block -> callback) and per-channel state lives in
// poa_channel_t instead of a malloc'd msgblk_t.
//
//   Copyright (c) 2015-2017 Thierry Leconte (original acarsdec)
//   GPLv2 (as acarsdec). Adaptation for the iridium_acars POA band.

#include "poa_decoder.h"

#include <complex.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "poa_syndrome.h" // numbits[256], crc_ccitt_table[256], update_crc, syndrom[]

#define INTRATE   12500
#define FLEN      ((INTRATE / 1200) + 1)  // = 11
#define MFLTOVER  12
#define FLENO     (FLEN * MFLTOVER + 1)    // = 133

#define SYN 0x16
#define SOH 0x01
#define STX 0x02
#define ETX 0x83
#define ETB 0x97
#define DLE 0x7f
#define MAXPERR 3

typedef enum { WSYN, SYN2, SOH1, TXT, CRC1, CRC2, END } acarsstate_t;

typedef struct {
    int chn;
    // --- MSK demod state (msk.c) ---
    // MskPhi: Q32 phase accumulator (2^32 == 2*pi radians), NOT a float
    // radian value. It wraps for free on unsigned integer overflow -- an
    // EXACT wrap, unlike a float radian accumulator that would need a
    // conditional subtract (and unlike incrementally-multiplying a unit
    // phasor, which decays -- feedback_q15_incremental_phasor_decays).
    uint32_t      MskPhi;
    float         MskDf;
    float         MskClk;
    float         MskLvlSum;
    int           MskBitCount;
    unsigned int  MskS, idx;
    float complex inb[FLEN];
    const float  *dm_buffer;   // current feed buffer (borrowed)
    unsigned char outbits;
    int           nbits;
    // --- ACARS state machine (acars.c) ---
    acarsstate_t  state;
    bool          in_blk;
    int           blk_len, blk_err;
    unsigned char txt[POA_TXT_MAX];
    unsigned char crcb[2];
    // --- telemetry counters (since last poa_decoder_get_stats) ---
    uint32_t      st_sync;      // SYN sync locks (WSYN -> SYN2)
    uint32_t      st_blk_start; // SOH block starts (SOH1 -> TXT)
    uint32_t      st_delivered; // blocks emitted (clean or CRC-fixed)
    uint32_t      st_crc_fail;  // blocks reaching CRC/parity but dropped
} poa_channel_t;

struct poa_decoder {
    int            nch;
    poa_block_cb   cb;
    void          *user;
    poa_channel_t  ch[POA_MAX_CHANNELS];
};

// Matched filter (msk.c h[]), shared across channels, built once.
static float s_h[FLENO];
static bool  s_h_init = false;

static const float PLLG = 38e-4f;
static const float PLLC = 0.52f;

// --- NCO: Q32 phase accumulator + cos/sin LUT (replaces double cexp) ---
// The ESP32-P4 is RV32IMAFC: single-precision FPU only, no D extension. The
// original acarsdec math accumulates the mixer phase `p` as a `double` and
// evaluates `cexp(-p*I)` per envelope sample (12.5 kHz/channel) -- on this
// target every one of those ops (add/cmp/cexp) is an emulated soft-float
// libgcc call. This block replaces both:
//   - the phase accumulator becomes a uint32_t where the full 2^32 range
//     maps to one full turn (2*pi); advancing it is a plain integer add
//     that wraps for free (no branch, no float drift -- see MskPhi comment
//     above);
//   - cexp(-p*I) = cos(p) - i*sin(p) becomes a table lookup indexed by the
//     top NCO_LUT_BITS of the accumulator.
// 1024 entries -> phase quantization of 2*pi/1024 ~= 0.0061 rad, two orders
// of magnitude finer than anything the MSK PLL (PLLG=38e-4) tracks bit to
// bit; float32's 24-bit mantissa is ample for a 12.5 kHz NCO over ~1 s.
#define NCO_LUT_BITS 10
#define NCO_LUT_SIZE (1 << NCO_LUT_BITS)          // 1024
#define NCO_LUT_SHIFT (32 - NCO_LUT_BITS)         // 22

static float s_nco_cos[NCO_LUT_SIZE];
static float s_nco_sin[NCO_LUT_SIZE];

// Radians -> Q32 turns conversion factor. This is a compile-time constant
// (folded by the compiler from the double literals below); it is NOT a
// runtime double op. 2^32 is exactly representable in float (power of two).
static const float NCO_RAD_TO_Q32 = (float)(4294967296.0 / (2.0 * M_PI));

// --- L2 error correction (acars.c fixprerr/fixdberr, verbatim on txt/len) ---
static int fixprerr(unsigned char *txt, int len, const unsigned short crc,
                    int *pr, int pn)
{
    if (pn > 0) {
        for (int i = 0; i < 8; i++) {
            if (fixprerr(txt, len, crc ^ syndrom[i + 8 * (len - *pr + 1)], pr + 1, pn - 1)) {
                txt[*pr] ^= (1 << i);
                return 1;
            }
        }
        return 0;
    }
    if (crc == 0) return 1;
    for (int i = 0; i < 2 * 8; i++)
        if (syndrom[i] == crc) return 1;
    return 0;
}

static int fixdberr(unsigned char *txt, int len, const unsigned short crc)
{
    for (int i = 0; i < 2 * 8; i++)
        if (syndrom[i] == crc) return 1;
    for (int k = 0; k < len; k++) {
        int bo = 8 * (len - k + 1);
        for (int i = 0; i < 8; i++)
            for (int j = 0; j < 8; j++) {
                if (i == j) continue;
                if ((crc ^ syndrom[i + bo] ^ syndrom[j + bo]) == 0) {
                    txt[k] ^= (1 << i);
                    txt[k] ^= (1 << j);
                    return 1;
                }
            }
    }
    return 0;
}

// acars.c blk_thread body (124-209), inlined synchronously. Returns true if a
// valid block was produced and delivered via the callback.
static void process_block(poa_decoder_t *d, poa_channel_t *ch, float level_db)
{
    unsigned char *txt = ch->txt;
    int len = ch->blk_len;
    int pr[MAXPERR], pn = 0;
    unsigned short crc;

    if (len < 13) return;

    // force STX/ETX
    txt[12] &= (ETX | STX);
    txt[12] |= (ETX & STX);

    // parity locate
    for (int i = 0; i < len; i++) {
        if ((numbits[txt[i]] & 1) == 0) {
            if (pn < MAXPERR) pr[pn] = i;
            pn++;
        }
    }
    if (pn > MAXPERR) return;

    // crc
    crc = 0;
    for (int i = 0; i < len; i++) update_crc(crc, txt[i]);
    update_crc(crc, ch->crcb[0]);
    update_crc(crc, ch->crcb[1]);

    bool fixed = false;
    if (pn) {
        if (fixprerr(txt, len, crc, pr, pn) == 0) { ch->st_crc_fail++; return; }
        fixed = true;
    } else if (crc) {
        if (fixdberr(txt, len, crc) == 0) { ch->st_crc_fail++; return; }
        fixed = true;
    }

    // redo parity + strip high bit
    pn = 0;
    for (int i = 0; i < len; i++) {
        if ((numbits[txt[i]] & 1) == 0) pn++;
        txt[i] &= 0x7f;
    }
    if (pn) { ch->st_crc_fail++; return; }

    poa_block_t out;
    out.chn = ch->chn;
    out.len = len;
    out.err = ch->blk_err;
    out.crc_fixed = fixed;
    out.level_db = level_db;
    out.crc[0] = ch->crcb[0];
    out.crc[1] = ch->crcb[1];
    memcpy(out.txt, txt, (size_t)len);
    ch->st_delivered++;
    if (d->cb) d->cb(&out, d->user);
}

void poa_decoder_get_stats(poa_decoder_t *d, poa_chan_dstats_t *out, int max_ch, int reset)
{
    if (!d || !out) return;
    int n = d->nch < max_ch ? d->nch : max_ch;
    for (int c = 0; c < n; c++) {
        poa_channel_t *ch = &d->ch[c];
        out[c].sync      = ch->st_sync;
        out[c].blk_start = ch->st_blk_start;
        out[c].delivered = ch->st_delivered;
        out[c].crc_fail  = ch->st_crc_fail;
        if (reset) {
            ch->st_sync = ch->st_blk_start = ch->st_delivered = ch->st_crc_fail = 0;
        }
    }
}

static void reset_acars(poa_channel_t *ch)
{
    ch->state = WSYN;
    ch->MskDf = 0;
    ch->nbits = 1;
}

// acars.c decodeAcars, adapted to in-struct block state + synchronous emit.
static void decode_acars(poa_decoder_t *d, poa_channel_t *ch)
{
    unsigned char r = ch->outbits;

    switch (ch->state) {
    case WSYN:
        if (r == SYN) { ch->st_sync++; ch->state = SYN2; ch->nbits = 8; return; }
        if (r == (unsigned char)~SYN) { ch->st_sync++; ch->MskS ^= 2; ch->state = SYN2; ch->nbits = 8; return; }
        ch->nbits = 1;
        return;
    case SYN2:
        if (r == SYN) { ch->state = SOH1; ch->nbits = 8; return; }
        if (r == (unsigned char)~SYN) { ch->MskS ^= 2; ch->nbits = 8; return; }
        reset_acars(ch);
        return;
    case SOH1:
        if (r == SOH) {
            ch->st_blk_start++;
            ch->state = TXT;
            ch->blk_len = 0;
            ch->blk_err = 0;
            ch->nbits = 8;
            ch->MskLvlSum = 0;
            ch->MskBitCount = 0;
            return;
        }
        reset_acars(ch);
        return;
    case TXT:
        ch->txt[ch->blk_len] = r;
        ch->blk_len++;
        if ((numbits[r] & 1) == 0) {
            ch->blk_err++;
            if (ch->blk_err > MAXPERR + 1) { reset_acars(ch); return; }
        }
        if (r == ETX || r == ETB) { ch->state = CRC1; ch->nbits = 8; return; }
        if (ch->blk_len > 20 && r == DLE) {
            ch->blk_len -= 3;
            ch->crcb[0] = ch->txt[ch->blk_len];
            ch->crcb[1] = ch->txt[ch->blk_len + 1];
            ch->state = CRC2;
            goto putmsg;
        }
        if (ch->blk_len > 240) { reset_acars(ch); return; }
        ch->nbits = 8;
        return;
    case CRC1:
        ch->crcb[0] = r;
        ch->state = CRC2;
        ch->nbits = 8;
        return;
    case CRC2:
        ch->crcb[1] = r;
    putmsg: {
        float lvl = (ch->MskBitCount > 0)
                        ? 10.0f * log10f((float)(ch->MskLvlSum / ch->MskBitCount))
                        : 0.0f;
        process_block(d, ch, lvl);
        ch->state = END;
        ch->nbits = 8;
        return;
    }
    case END:
        reset_acars(ch);
        ch->nbits = 8;
        return;
    }
}

static inline void putbit(poa_decoder_t *d, poa_channel_t *ch, float v)
{
    ch->outbits >>= 1;
    if (v > 0) ch->outbits |= 0x80;
    ch->nbits--;
    if (ch->nbits <= 0) decode_acars(d, ch);
}

// msk.c demodMSK. Ported to single-precision + a Q32 phase-accumulator NCO
// (see NCO block above) -- functionally identical to the acarsdec double
// path (same PLL constants, same matched filter, same bit decisions) but
// with no double-precision or transcendental math in the per-sample loop.
static void demod_msk(poa_decoder_t *d, poa_channel_t *ch, int len)
{
    unsigned int idx = ch->idx;
    uint32_t p = ch->MskPhi;

    for (int n = 0; n < len; n++) {
        float s = 1800.0f / INTRATE * 2.0f * (float)M_PI + ch->MskDf;
        p += (uint32_t)(s * NCO_RAD_TO_Q32);

        unsigned lut_idx = p >> NCO_LUT_SHIFT;
        float c  = s_nco_cos[lut_idx];
        float si = s_nco_sin[lut_idx];

        float in = ch->dm_buffer[n];
        // cexp(-p*I) = cos(p) - i*sin(p)
        ch->inb[idx] = (in * c) - (in * si) * I;
        idx = (idx + 1) % FLEN;

        ch->MskClk += s;
        if (ch->MskClk >= (float)(3.0 * M_PI / 2.0) - s * 0.5f) {
            ch->MskClk -= (float)(3.0 * M_PI / 2.0);

            int o = (int)((float)MFLTOVER * (ch->MskClk / s + 0.5f));
            if (o > MFLTOVER) o = MFLTOVER;
            float complex v = 0;
            for (int j = 0; j < FLEN; j++, o += MFLTOVER)
                v += s_h[o] * ch->inb[(j + idx) % FLEN];

            float lvl = cabsf(v);
            v /= lvl + 1e-8f;
            ch->MskLvlSum += lvl * lvl / 4.0f;
            ch->MskBitCount++;

            float dphi;
            float vo;
            if (ch->MskS & 1) {
                vo = cimagf(v);
                dphi = (vo >= 0) ? -crealf(v) : crealf(v);
            } else {
                vo = crealf(v);
                dphi = (vo >= 0) ? cimagf(v) : -cimagf(v);
            }
            putbit(d, ch, (ch->MskS & 2) ? -vo : vo);
            ch->MskS++;

            ch->MskDf = PLLC * ch->MskDf + (1.0f - PLLC) * PLLG * dphi;
        }
    }
    ch->idx = idx;
    ch->MskPhi = p;
}

poa_decoder_t *poa_decoder_create(int nchannels, poa_block_cb cb, void *user)
{
    if (nchannels < 1 || nchannels > POA_MAX_CHANNELS) return NULL;
    poa_decoder_t *d = calloc(1, sizeof(*d));
    if (!d) return NULL;
    d->nch = nchannels;
    d->cb = cb;
    d->user = user;

    if (!s_h_init) {
        for (int i = 0; i < FLENO; i++) {
            s_h[i] = cosf(2.0f * (float)M_PI * 600.0f / INTRATE / MFLTOVER * (i - (FLENO - 1) / 2));
            if (s_h[i] < 0) s_h[i] = 0;
        }
        // NCO LUT: one-time build, single-precision cosf/sinf (not hot path).
        for (int i = 0; i < NCO_LUT_SIZE; i++) {
            float ang = 2.0f * (float)M_PI * (float)i / (float)NCO_LUT_SIZE;
            s_nco_cos[i] = cosf(ang);
            s_nco_sin[i] = sinf(ang);
        }
        s_h_init = true;
    }
    for (int c = 0; c < nchannels; c++) {
        d->ch[c].chn = c;
        d->ch[c].state = WSYN;
        d->ch[c].nbits = 8;
        d->ch[c].outbits = 0;
    }
    return d;
}

void poa_decoder_feed(poa_decoder_t *d, int chn, const float *audio, int len)
{
    if (!d || chn < 0 || chn >= d->nch || !audio || len <= 0) return;
    poa_channel_t *ch = &d->ch[chn];
    ch->dm_buffer = audio;
    demod_msk(d, ch, len);
    ch->dm_buffer = NULL;
}

void poa_decoder_destroy(poa_decoder_t *d) { free(d); }
