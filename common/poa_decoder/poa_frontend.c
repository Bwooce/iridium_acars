// poa_frontend — see poa_frontend.h. Ported from acarsdec rtl.c (Thierry
// Leconte, GPLv2). The per-channel mix+integrate-dump+envelope is bit-faithful
// to in_callback(); acarsdec's fixed RTLOUTBUFSZ/rtlInBufSize block structure
// is generalized to a streaming feed (a partial input block carries across
// feeds; output is flushed to poa_decoder per OUTBUF or at feed end).
//
//   Copyright (c) 2015-2017 Thierry Leconte (original acarsdec rtl.c).
//   GPLv2. Adaptation for the iridium_acars POA band.

#include "poa_frontend.h"

#include <complex.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define RTLMULTMAX 320
#define OUTBUF     1024   // acarsdec RTLOUTBUFSZ: output samples flushed per batch

struct poa_frontend {
    poa_decoder_t *dec;
    int            nch;
    int            rtlMult;
    float complex *wf[POA_MAX_CHANNELS];   // [rtlMult] mix+integrate weights per channel
    // partial input block (< rtlMult complex samples) carried across feeds
    float complex  pend[RTLMULTMAX];
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
        fe->wf[n] = malloc((size_t)rtlMult * sizeof(float complex));
        if (!fe->wf[n]) { poa_frontend_destroy(fe); return NULL; }
        // AMFreq = 2*pi*(Fr - Fc)/fs (rad/sample). wf = exp(-i*AMFreq*ind)/rtlMult
        // (rtlMult = integrate-dump normalization; acarsdec's extra /127.5 cu8
        // scale is dropped — the decode is amplitude-invariant).
        double amf = 2.0 * M_PI * ((double)chan_hz[n] - (double)lo_hz) / (double)fs_hz;
        for (int ind = 0; ind < rtlMult; ind++)
            fe->wf[n][ind] = (float complex)(cexp(-I * amf * ind) / rtlMult);
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

// consume one complete rtlMult-sample input block -> one output envelope sample/ch
static void consume_block(poa_frontend_t *fe, const float complex *vb)
{
    int m = fe->out_n;
    for (int n = 0; n < fe->nch; n++) {
        const float complex *wf = fe->wf[n];
        float complex D = 0;
        for (int ind = 0; ind < fe->rtlMult; ind++)
            D += vb[ind] * wf[ind];
        fe->out[n][m] = cabsf(D);
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
            fe->pend[fe->pend_n++] = iq[2*i] + iq[2*i+1] * I; i++;
        }
        if (fe->pend_n == fe->rtlMult) { consume_block(fe, fe->pend); fe->pend_n = 0; }
    }
    // 2) whole blocks straight from the input
    while (nsamp - i >= fe->rtlMult) {
        float complex vb[RTLMULTMAX];
        for (int ind = 0; ind < fe->rtlMult; ind++, i++)
            vb[ind] = iq[2*i] + iq[2*i+1] * I;
        consume_block(fe, vb);
    }
    // 3) stash the remainder
    while (i < nsamp) { fe->pend[fe->pend_n++] = iq[2*i] + iq[2*i+1] * I; i++; }
    // flush whatever output we have so nothing is stranded across feeds
    flush_out(fe);
}

void poa_frontend_destroy(poa_frontend_t *fe)
{
    if (!fe) return;
    for (int n = 0; n < fe->nch; n++) free(fe->wf[n]);
    poa_decoder_destroy(fe->dec);
    free(fe);
}
