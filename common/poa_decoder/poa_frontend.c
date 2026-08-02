// poa_frontend — see poa_frontend.h. Ported from acarsdec rtl.c (Thierry
// Leconte, GPLv2). The per-channel mix+integrate-dump+envelope is bit-faithful
// to in_callback(); acarsdec's fixed RTLOUTBUFSZ/rtlInBufSize block structure
// is generalized to a streaming feed (a partial input block carries across
// feeds; output is flushed to poa_decoder per OUTBUF or at feed end).
//
//   Copyright (c) 2015-2017 Thierry Leconte (original acarsdec rtl.c).
//   GPLv2. Adaptation for the iridium_acars POA band.
//
// Hot path (consume_block) does the mix+integrate as explicit real/imag float
// MACs rather than C99 `float complex` operators. GCC lowers `a*b` on two
// `float complex` to an out-of-line __mulsc3 call (Inf/NaN rescue) even at -O2;
// at ~10 M complex-MAC/s (2.5 MSPS x nch) that call dominated the DSP budget.
// The manual expansion below is the same arithmetic __mulsc3 performs (minus
// the non-finite rescue, which bounded RTL samples never trigger) and inlines
// to hardware single-precision FMAs.
//
// Removing __mulsc3 was necessary but NOT sufficient: at 360 MHz a saturated
// core is ~144 cyc/complex-sample and the channelizer still pinned dsp=100%
// (POA has no burst tagger — consume_block runs on 100% of the 2.5 MSPS
// stream, x nch, continuously; unlike Iridium/VDL2 which only run heavy DSP on
// short tagged burst windows). Two structural changes cut that:
//   1) Single fused pass. The feed takes int16 straight from the USB stream and
//      consume_block reads whole rtlMult-blocks DIRECTLY from it, converting to
//      float once per sample on the stack. This drops the caller's int16->float
//      pass + 32 KB fbuf and this file's old float deinterleave copy (two full
//      passes over the 2.5 MSPS stream, gone).
//   2) Latency-hidden accumulate. The integrate-dump is a serial float add
//      chain; a single Dre/Dim accumulator stalls on FPU add latency every
//      iteration. Four accumulators (2-way unroll x re/im) give four
//      independent chains so the adds pipeline.
// The weight table is still built in `double` at create() so it stays bit-
// comparable to the acarsdec oracle. The 2-way accumulator split reorders the
// float summation vs a single accumulator (~1e-6 relative), far below the MSK
// slice margin — host test_poa_frontend still reproduces JQ0404 byte-exact.

#include "poa_frontend.h"

#include <complex.h> // create() only: double cexp for the weight table
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define RTLMULTMAX 320
#define OUTBUF     1024   // acarsdec RTLOUTBUFSZ: output samples flushed per batch

struct poa_frontend {
    poa_decoder_t *dec;
    int            nch;
    int            rtlMult;
    // mix+integrate weights per channel, split re/im (plain float — NOT
    // float complex, so the hot loop can't accidentally reacquire __mulsc3).
    float         *wf_re[POA_MAX_CHANNELS]; // [rtlMult]
    float         *wf_im[POA_MAX_CHANNELS]; // [rtlMult]
    // partial input block (< rtlMult complex samples) carried across feeds,
    // held as raw int16 I/Q (consume_block converts to float on the stack).
    int16_t        pend[2 * RTLMULTMAX];
    int            pend_n;
    // per-channel output (envelope) buffer, flushed to poa_decoder at OUTBUF
    float          out[POA_MAX_CHANNELS][OUTBUF];
    int            out_n;
};

poa_frontend_t *poa_frontend_create(uint32_t fs_hz, uint32_t lo_hz,
                                    const uint32_t *chan_hz, int nch,
                                    poa_block_cb cb, void *user)
{
    if (nch < 1 || nch > POA_MAX_CHANNELS || fs_hz % POA_INTRATE != 0) return NULL;
    int rtlMult = (int)(fs_hz / POA_INTRATE);
    if (rtlMult < 1 || rtlMult > RTLMULTMAX) return NULL;

    poa_frontend_t *fe = calloc(1, sizeof(*fe));
    if (!fe) return NULL;
    fe->nch = nch;
    fe->rtlMult = rtlMult;
    fe->dec = poa_decoder_create(nch, cb, user);
    if (!fe->dec) { free(fe); return NULL; }

    for (int n = 0; n < nch; n++) {
        fe->wf_re[n] = malloc((size_t)rtlMult * sizeof(float));
        fe->wf_im[n] = malloc((size_t)rtlMult * sizeof(float));
        if (!fe->wf_re[n] || !fe->wf_im[n]) { poa_frontend_destroy(fe); return NULL; }
        // AMFreq = 2*pi*(Fr - Fc)/fs (rad/sample). wf = exp(-i*AMFreq*ind)/rtlMult
        // (rtlMult = integrate-dump normalization; acarsdec's extra /127.5 cu8
        // scale is dropped — the decode is amplitude-invariant). Built in double
        // to stay bit-comparable to the oracle; stored as split float.
        double amf = 2.0 * M_PI * ((double)chan_hz[n] - (double)lo_hz) / (double)fs_hz;
        for (int ind = 0; ind < rtlMult; ind++) {
            double complex w = cexp(-I * amf * ind) / rtlMult;
            fe->wf_re[n][ind] = (float)creal(w);
            fe->wf_im[n][ind] = (float)cimag(w);
        }
    }
    return fe;
}

static void flush_out(poa_frontend_t *fe)
{
    if (fe->out_n <= 0) return;
    for (int n = 0; n < fe->nch; n++)
        poa_decoder_feed(fe->dec, n, fe->out[n], fe->out_n);
    fe->out_n = 0;
}

// consume one complete rtlMult-sample input block (interleaved int16 I/Q) ->
// one output envelope sample/ch. Per channel D = sum(vb[ind] * wf[ind]) as
// explicit re/im MACs (see file header); envelope = |D|. The int16->float
// conversion is done once per sample into stack scratch, then the nch mixes
// read the hot float scratch (converting inside each channel loop would redo
// the conversion nch times).
static void consume_block(poa_frontend_t *fe, const int16_t *blk)
{
    const int R = fe->rtlMult;
    float vb_re[RTLMULTMAX], vb_im[RTLMULTMAX];
    for (int i = 0; i < R; i++) {
        vb_re[i] = (float)blk[2 * i];
        vb_im[i] = (float)blk[2 * i + 1];
    }
    int m = fe->out_n;
    for (int n = 0; n < fe->nch; n++) {
        const float *wre = fe->wf_re[n];
        const float *wim = fe->wf_im[n];
        // 2-way unroll -> 4 independent accumulator chains hide FPU add latency.
        float Dre0 = 0.0f, Dim0 = 0.0f, Dre1 = 0.0f, Dim1 = 0.0f;
        int ind = 0;
        for (; ind + 1 < R; ind += 2) {
            float x0 = vb_re[ind],     y0 = vb_im[ind],     a0 = wre[ind],     b0 = wim[ind];
            float x1 = vb_re[ind + 1], y1 = vb_im[ind + 1], a1 = wre[ind + 1], b1 = wim[ind + 1];
            Dre0 += x0 * a0 - y0 * b0;
            Dim0 += x0 * b0 + y0 * a0;
            Dre1 += x1 * a1 - y1 * b1;
            Dim1 += x1 * b1 + y1 * a1;
        }
        for (; ind < R; ind++) { // odd rtlMult tail (rtlMult=fs/12500 is even in practice)
            float x = vb_re[ind], y = vb_im[ind], a = wre[ind], b = wim[ind];
            Dre0 += x * a - y * b;
            Dim0 += x * b + y * a;
        }
        float Dre = Dre0 + Dre1;
        float Dim = Dim0 + Dim1;
        fe->out[n][m] = sqrtf(Dre * Dre + Dim * Dim);
    }
    fe->out_n++;
    if (fe->out_n >= OUTBUF) flush_out(fe);
}

void poa_frontend_feed(poa_frontend_t *fe, const int16_t *iq, int nsamp)
{
    if (!fe || !iq || nsamp <= 0) return;
    const int R = fe->rtlMult;
    int i = 0;

    // 1) top up a carried-over partial block
    if (fe->pend_n > 0) {
        while (fe->pend_n < R && i < nsamp) {
            fe->pend[2 * fe->pend_n]     = iq[2 * i];
            fe->pend[2 * fe->pend_n + 1] = iq[2 * i + 1];
            fe->pend_n++; i++;
        }
        if (fe->pend_n == R) {
            consume_block(fe, fe->pend);
            fe->pend_n = 0;
        }
    }
    // 2) whole blocks consumed DIRECTLY from the input (no float copy)
    while (nsamp - i >= R) {
        consume_block(fe, &iq[2 * i]);
        i += R;
    }
    // 3) stash the remainder
    while (i < nsamp) {
        fe->pend[2 * fe->pend_n]     = iq[2 * i];
        fe->pend[2 * fe->pend_n + 1] = iq[2 * i + 1];
        fe->pend_n++; i++;
    }
    // flush whatever output we have so nothing is stranded across feeds
    flush_out(fe);
}

void poa_frontend_destroy(poa_frontend_t *fe)
{
    if (!fe) return;
    for (int n = 0; n < fe->nch; n++) {
        free(fe->wf_re[n]);
        free(fe->wf_im[n]);
    }
    poa_decoder_destroy(fe->dec);
    free(fe);
}
