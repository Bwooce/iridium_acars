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
// at ~10 M complex-MAC/s (2.5 MSPS x nch) that call dominated the DSP budget
// (dsp_cap ~100%, mass rb_full drops). The manual expansion below is the same
// arithmetic __mulsc3 performs (minus the non-finite rescue, which bounded RTL
// samples never trigger) and inlines to hardware single-precision FMAs. The
// weight table is still built in `double` at create() so it stays bit-
// comparable to the acarsdec oracle (host test_poa_frontend).

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
    // partial input block (< rtlMult complex samples) carried across feeds
    float          pend_re[RTLMULTMAX];
    float          pend_im[RTLMULTMAX];
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

// consume one complete rtlMult-sample input block -> one output envelope sample/ch.
// D = sum(vb[ind] * wf[ind]) done as explicit re/im MACs (see file header);
// envelope = |D|. Same math as `D += vb*wf; cabsf(D)` without the __mulsc3 call.
static void consume_block(poa_frontend_t *fe, const float *vb_re, const float *vb_im)
{
    int m = fe->out_n;
    for (int n = 0; n < fe->nch; n++) {
        const float *wre = fe->wf_re[n];
        const float *wim = fe->wf_im[n];
        float Dre = 0.0f, Dim = 0.0f;
        for (int ind = 0; ind < fe->rtlMult; ind++) {
            float x = vb_re[ind], y = vb_im[ind];
            float a = wre[ind], b = wim[ind];
            Dre += x * a - y * b;
            Dim += x * b + y * a;
        }
        fe->out[n][m] = sqrtf(Dre * Dre + Dim * Dim);
    }
    fe->out_n++;
    if (fe->out_n >= OUTBUF) flush_out(fe);
}

void poa_frontend_feed(poa_frontend_t *fe, const float *iq, int nsamp)
{
    if (!fe || !iq || nsamp <= 0) return;
    int i = 0;

    // 1) top up a carried-over partial block
    if (fe->pend_n > 0) {
        while (fe->pend_n < fe->rtlMult && i < nsamp) {
            fe->pend_re[fe->pend_n] = iq[2*i];
            fe->pend_im[fe->pend_n] = iq[2*i+1];
            fe->pend_n++; i++;
        }
        if (fe->pend_n == fe->rtlMult) {
            consume_block(fe, fe->pend_re, fe->pend_im);
            fe->pend_n = 0;
        }
    }
    // 2) whole blocks straight from the input (deinterleaved into re/im scratch)
    while (nsamp - i >= fe->rtlMult) {
        float vb_re[RTLMULTMAX];
        float vb_im[RTLMULTMAX];
        for (int ind = 0; ind < fe->rtlMult; ind++, i++) {
            vb_re[ind] = iq[2*i];
            vb_im[ind] = iq[2*i+1];
        }
        consume_block(fe, vb_re, vb_im);
    }
    // 3) stash the remainder
    while (i < nsamp) {
        fe->pend_re[fe->pend_n] = iq[2*i];
        fe->pend_im[fe->pend_n] = iq[2*i+1];
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
