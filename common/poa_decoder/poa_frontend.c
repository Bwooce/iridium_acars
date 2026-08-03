// poa_frontend — see poa_frontend.h. Ported from acarsdec rtl.c (Thierry
// Leconte, GPLv2). The per-channel mix+integrate-dump+envelope is bit-faithful
// to in_callback(); acarsdec's fixed RTLOUTBUFSZ/rtlInBufSize block structure
// is generalized to a streaming feed (a partial input block carries across
// feeds; output is flushed to poa_decoder per OUTBUF or at feed end).
//
//   Copyright (c) 2015-2017 Thierry Leconte (original acarsdec rtl.c).
//   GPLv2. Adaptation for the iridium_acars POA band.
//
// Hot path: the per-channel complex integrate-dump is a Q15 int16 dot product,
//   Dre = sum(xr*wre) + sum(xi*(-wim)),  Dim = sum(xr*wim) + sum(xi*wre)
// over rtlMult samples, accumulated in a >=40-bit accumulator. On device this
// is the 8-lane PIE vector MAC poa_mix_q15_arp4 (poa_mix_arp4.S); on host it is
// the bit-identical ANSI-C reference below.
//
// History of the cost fight (all measured on the P4 at 360 MHz, 4 ch,
// cyc/complex-sample): float complex operators compiled to out-of-line
// __mulsc3 (~990); explicit float re/im MACs (817); Q15 int16 scalar (209);
// PIE 8-lane vector (this file's device path). POA has no burst tagger — the
// mix runs on 100% of the 2.5 MSPS stream, x nch, continuously, unlike
// Iridium/VDL2 which only run heavy DSP on short tagged burst windows.
//
// The Q15 weights are unit magnitude (the old 1/rtlMult integrate-dump
// normalization is dropped — the decode is amplitude-invariant, so unit
// magnitude keeps 15 bits of precision). wf_re/wf_im hold cos(amf*ind) and
// -sin(amf*ind) scaled by 32767; wf_nim holds -wf_im so the subtraction in Dre
// folds into a MAC. Weights are clamped to [-32767, 32767] so negating never
// overflows to -32768.

#include "poa_frontend.h"

#include <complex.h> // create() only: double cexp for the weight table
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifdef ESP_PLATFORM
#include "soc/soc_caps.h"     // SOC_CPU_HAS_PIE
#include "esp_heap_caps.h"    // heap_caps_aligned_alloc / MALLOC_CAP_INTERNAL
#include "esp_memory_utils.h" // esp_ptr_in_dram
#endif

#define RTLMULTMAX 320
#define OUTBUF     1024   // acarsdec RTLOUTBUFSZ: output samples flushed per batch
#define PIE_LANES  8      // s16 lanes per PIE vector op
#define READAHEAD  8      // extra zero-padded int16 tail (PIE vld read-ahead guard)

// ---- Q15 complex integrate-dump kernel (device: poa_mix_arp4.S) ----
// Dre = sum(xr*wre) + sum(xi*negwim); Dim = sum(xr*wim) + sum(xi*wre), over
// nvec*8 samples (buffers zero-padded to nvec*8 + read-ahead, 16-byte aligned).
void poa_mix_q15_arp4(const int16_t *xr, const int16_t *xi,
                      const int16_t *wre, const int16_t *wim,
                      const int16_t *negwim, int nvec,
                      int64_t *Dre, int64_t *Dim);

#if !(defined(__riscv) && defined(SOC_CPU_HAS_PIE))
// ANSI-C reference — the device PIE kernel is bit-identical by construction
// (same int16*int16 products summed; the 40-bit PIE accumulator holds the
// max ~4.3e11 result without overflow, matching this int64 sum).
void poa_mix_q15_arp4(const int16_t *xr, const int16_t *xi,
                      const int16_t *wre, const int16_t *wim,
                      const int16_t *negwim, int nvec,
                      int64_t *Dre, int64_t *Dim)
{
    const int n = nvec * PIE_LANES;
    int64_t dre = 0, dim = 0;
    for (int i = 0; i < n; i++) {
        int32_t x = xr[i], y = xi[i];
        dre += (int64_t)x * wre[i] + (int64_t)y * negwim[i];
        dim += (int64_t)x * wim[i] + (int64_t)y * wre[i];
    }
    *Dre = dre;
    *Dim = dim;
}
#endif

struct poa_frontend {
    poa_decoder_t *dec;
    int            nch;
    int            rtlMult;
    int            nvec;     // ceil(rtlMult / 8): PIE vectors per dot product
    int            npad;     // nvec * 8: padded sample count (>= rtlMult)
    // Q15 int16 mix weights per channel + the negated imag table, split re/im.
    // All are internal-DRAM, 16-byte aligned, and zero-padded to npad+READAHEAD
    // (PIE reads whole 8-lane vectors; the pad taps contribute 0).
    int16_t       *wf_re[POA_MAX_CHANNELS];
    int16_t       *wf_im[POA_MAX_CHANNELS];
    int16_t       *wf_nim[POA_MAX_CHANNELS]; // -wf_im
    // deinterleaved input scratch (shared across channels), same placement.
    int16_t       *xr;
    int16_t       *xi;
    // partial input block (< rtlMult complex samples) carried across feeds,
    // held as raw int16 I/Q (scalar-accessed only, not PIE).
    int16_t        pend[2 * RTLMULTMAX];
    int            pend_n;
    // per-channel output (envelope) buffer, flushed to poa_decoder at OUTBUF
    float          out[POA_MAX_CHANNELS][OUTBUF];
    int            out_n;
};

// Allocate n int16 in internal DRAM, 16-byte aligned, zeroed. PIE vector loads
// require internal DRAM (PSRAM/RTCRAM/TCM silently corrupt) — guard it.
static int16_t *poa_alloc_i16(int n)
{
    const size_t bytes = ((size_t)n * sizeof(int16_t) + 15u) & ~(size_t)15u;
#ifdef ESP_PLATFORM
    int16_t *p = heap_caps_aligned_alloc(16, bytes, MALLOC_CAP_INTERNAL);
    if (p && !esp_ptr_in_dram(p)) { heap_caps_free(p); return NULL; }
#else
    int16_t *p = aligned_alloc(16, bytes);
#endif
    if (p) memset(p, 0, bytes);
    return p;
}

static void poa_free_i16(int16_t *p)
{
    if (!p) return;
#ifdef ESP_PLATFORM
    heap_caps_free(p);
#else
    free(p);
#endif
}

poa_frontend_t *poa_frontend_create(uint32_t fs_hz, uint32_t lo_hz,
                                    const uint32_t *chan_hz, int nch,
                                    poa_block_cb cb, void *user)
{
    if (nch < 1 || nch > POA_MAX_CHANNELS || fs_hz % POA_INTRATE != 0) return NULL;
    int rtlMult = (int)(fs_hz / POA_INTRATE);
    if (rtlMult < 1 || rtlMult > RTLMULTMAX) return NULL;

    poa_frontend_t *fe = calloc(1, sizeof(*fe));
    if (!fe) return NULL;
    fe->nch     = nch;
    fe->rtlMult = rtlMult;
    fe->nvec    = (rtlMult + PIE_LANES - 1) / PIE_LANES;
    fe->npad    = fe->nvec * PIE_LANES;
    fe->dec     = poa_decoder_create(nch, cb, user);
    if (!fe->dec) { free(fe); return NULL; }

    const int alloc_n = fe->npad + READAHEAD; // zero-padded tail + read-ahead
    fe->xr = poa_alloc_i16(alloc_n);
    fe->xi = poa_alloc_i16(alloc_n);
    if (!fe->xr || !fe->xi) { poa_frontend_destroy(fe); return NULL; }

    for (int n = 0; n < nch; n++) {
        fe->wf_re[n]  = poa_alloc_i16(alloc_n);
        fe->wf_im[n]  = poa_alloc_i16(alloc_n);
        fe->wf_nim[n] = poa_alloc_i16(alloc_n);
        if (!fe->wf_re[n] || !fe->wf_im[n] || !fe->wf_nim[n]) {
            poa_frontend_destroy(fe);
            return NULL;
        }
        // AMFreq = 2*pi*(Fr - Fc)/fs (rad/sample). wf = exp(-i*AMFreq*ind).
        // Built in double, stored Q15 (round-to-nearest via lrint), clamped to
        // [-32767, 32767] so wf_nim = -wf_im never wraps to -32768. Taps
        // [rtlMult..npad) stay zero (poa_alloc_i16 zeroes the whole buffer).
        double amf = 2.0 * M_PI * ((double)chan_hz[n] - (double)lo_hz) / (double)fs_hz;
        for (int ind = 0; ind < rtlMult; ind++) {
            double complex w = cexp(-I * amf * ind);
            long wr = lrint(creal(w) * 32767.0);
            long wi = lrint(cimag(w) * 32767.0);
            if (wr >  32767) wr =  32767;
            if (wr < -32767) wr = -32767;
            if (wi >  32767) wi =  32767;
            if (wi < -32767) wi = -32767;
            fe->wf_re[n][ind]  = (int16_t)wr;
            fe->wf_im[n][ind]  = (int16_t)wi;
            fe->wf_nim[n][ind] = (int16_t)(-wi);
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
// one output envelope sample/ch. Deinterleaves into the padded xr/xi scratch,
// runs the Q15 complex integrate-dump per channel (PIE on device), envelope=|D|.
static void consume_block(poa_frontend_t *fe, const int16_t *blk)
{
    const int R = fe->rtlMult;
    for (int i = 0; i < R; i++) {
        fe->xr[i] = blk[2 * i];
        fe->xi[i] = blk[2 * i + 1];
    }
    // xr/xi[R..npad) stay zero (never written after the zeroed alloc), so the
    // dot product over nvec*8 equals the dot over rtlMult.
    int m = fe->out_n;
    for (int n = 0; n < fe->nch; n++) {
        int64_t Dre, Dim;
        poa_mix_q15_arp4(fe->xr, fe->xi, fe->wf_re[n], fe->wf_im[n],
                         fe->wf_nim[n], fe->nvec, &Dre, &Dim);
        float fre = (float)Dre, fim = (float)Dim;
        fe->out[n][m] = sqrtf(fre * fre + fim * fim);
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
        poa_free_i16(fe->wf_re[n]);
        poa_free_i16(fe->wf_im[n]);
        poa_free_i16(fe->wf_nim[n]);
    }
    poa_free_i16(fe->xr);
    poa_free_i16(fe->xi);
    poa_decoder_destroy(fe->dec);
    free(fe);
}
