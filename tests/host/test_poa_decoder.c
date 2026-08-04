// test_poa_decoder — cross-validate the ported POA demod+L2 against acarsdec
// (the oracle) on the HydraSDR golden capture (~/iridium_capture/poa_ref/,
// see its README + docs/2026-08-01-poa-onband-plan.md §8).
//
// Feeds the 4-channel 12.5 kHz AM-audio golden WAV (make_golden.py output)
// through poa_decoder and asserts it reproduces the oracle decode:
//   #1 (L:+0.8 E:0)  VH-VGD JQ0404 2 3L M31A S 33.849/E151.307 /UTC 2243
// i.e. at least one block whose text carries the distinctive position string.
//
// SKIPs (exit 0) if the golden WAV is absent — it lives outside the repo
// (~6 GB source IQ + derived WAV), like the vdl2_ref oracle. Regenerate with
// ~/iridium_capture/poa_ref/make_golden.py.

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "poa_decoder.h"

#define GOLDEN_WAV "/Users/bruce/iridium_capture/poa_ref/poa_golden_10min_12k5_4ch.wav"
#define NCH 4

static int   g_nblk = 0;
static int   g_hit  = 0; // block carrying the oracle's distinctive substring

static void on_block(const poa_block_t *b, void *user)
{
    (void)user;
    g_nblk++;
    // block bytes are 7-bit ACARS chars; print printable, look for the marker
    char s[POA_TXT_MAX + 1];
    int n = b->len < POA_TXT_MAX ? b->len : POA_TXT_MAX;
    for (int i = 0; i < n; i++) {
        unsigned char c = b->txt[i];
        s[i] = (c >= 0x20 && c < 0x7f) ? (char)c : '.';
    }
    s[n] = '\0';
    printf("  block ch=%d len=%d err=%d fixed=%d lvl=%.1f: %s\n",
           b->chn, b->len, b->err, (int)b->crc_fixed, b->level_db, s);
    if (strstr(s, "33.849") || strstr(s, "VGD")) g_hit++;
}

int main(void)
{
    FILE *f = fopen(GOLDEN_WAV, "rb");
    if (!f) {
        printf("SKIP: golden WAV not present (%s) — regenerate via poa_ref/make_golden.py\n",
               GOLDEN_WAV);
        return 0;
    }
    if (fseek(f, 44, SEEK_SET) != 0) { fclose(f); return 0; } // skip PCM16 header

    poa_decoder_t *d = poa_decoder_create(NCH, on_block, NULL);
    assert(d);

    // Chunked, deinterleaved feed (state carries across chunks).
    enum { FR = 12500 };            // 1 s of frames per chunk
    static short inter[FR * NCH];
    static float chan[NCH][FR];
    size_t got;
    long total = 0;
    while ((got = fread(inter, sizeof(short) * NCH, FR, f)) > 0) {
        for (int c = 0; c < NCH; c++)
            for (size_t i = 0; i < got; i++)
                chan[c][i] = (float)inter[i * NCH + c];
        for (int c = 0; c < NCH; c++)
            poa_decoder_feed(d, c, chan[c], (int)got);
        total += (long)got;
    }
    fclose(f);
    poa_decoder_destroy(d);

    printf("poa_decoder: %ld frames, %d block(s) decoded, %d matched the oracle marker\n",
           total, g_nblk, g_hit);
    // Cross-validation: the port must reproduce acarsdec's JQ0404 block.
    assert(g_nblk >= 1);
    assert(g_hit >= 1);
    printf("PASS: test_poa_decoder (reproduced oracle JQ0404)\n");
    return 0;
}
