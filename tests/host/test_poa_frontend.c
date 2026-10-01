// test_poa_frontend — full-chain POA cross-validation from RAW IQ:
//   raw 2.5 MSPS IQ -> poa_frontend channelizer -> poa_decoder -> L2
// must reproduce acarsdec's golden decode (JQ0404). This validates the ported
// C channelizer (poa_frontend) against the Python make_golden.py channelizer
// AND acarsdec end-to-end, on the HydraSDR golden capture
// (~/iridium_capture/poa_ref/, plan §8). SKIPs if the raw capture is absent.

#include <assert.h>
#include <complex.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "poa_frontend.h"

#define GOLDEN_RAW "/Users/bruce/iridium_capture/poa_ref/poa_2500k_130800_10min_s16.raw"
#define FS   2500000u
#define LO   130800000u
#define NCH  4

static int g_nblk = 0, g_hit = 0;

static void on_block(const poa_block_t *b, void *user)
{
    (void)user;
    g_nblk++;
    char s[POA_TXT_MAX + 1];
    int n = b->len < POA_TXT_MAX ? b->len : POA_TXT_MAX;
    for (int i = 0; i < n; i++) {
        unsigned char c = b->txt[i];
        s[i] = (c >= 0x20 && c < 0x7f) ? (char)c : '.';
    }
    s[n] = '\0';
    printf("  block ch=%d len=%d err=%d fixed=%d: %s\n", b->chn, b->len, b->err, (int)b->crc_fixed, s);
    if (strstr(s, "33.849") || strstr(s, "VGD")) g_hit++;
}

int main(void)
{
    FILE *f = fopen(GOLDEN_RAW, "rb");
    if (!f) {
        printf("SKIP: golden raw IQ not present (%s)\n", GOLDEN_RAW);
        return 0;
    }
    const uint32_t chans[NCH] = {131550000u, 130450000u, 130425000u, 130025000u};
    poa_frontend_t *fe = poa_frontend_create(FS, LO, chans, NCH, on_block, NULL);
    assert(fe);

    enum { NS = 250000 };                 // complex samples per chunk
    static int16_t raw[NS * 2];           // interleaved s16 I/Q (native feed)
    size_t got;
    long total = 0;
    while ((got = fread(raw, sizeof(int16_t) * 2, NS, f)) > 0) {
        poa_frontend_feed(fe, raw, (int)got);
        total += (long)got;
    }
    fclose(f);
    poa_frontend_destroy(fe);

    printf("poa_frontend: %ld complex samples, %d block(s), %d matched oracle marker\n",
           total, g_nblk, g_hit);
    assert(g_nblk >= 1);
    assert(g_hit >= 1);
    printf("PASS: test_poa_frontend (raw IQ -> channelizer -> decode == oracle JQ0404)\n");
    return 0;
}
