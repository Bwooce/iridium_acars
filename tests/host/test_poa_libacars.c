// test_poa_libacars — P3 bridge validation. Decodes the POA golden through
// poa_decoder, then runs each decoded block through the SAME libacars call
// frame_decoder's acars_deliver() uses (la_acars_parse), asserting it yields
// the structured ACARS fields the oracle showed:
//   VH-VGD JQ0404 label 3L
// This proves the poa_decoder block format (mode..STX..text..ETX, no SOH)
// is exactly what libacars expects — de-risking the P3 device bridge
// (dsp poa_on_block -> decoder-task queue -> acars_deliver) BEFORE wiring the
// shared decoder task. SKIPs if the golden WAV is absent.

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <libacars/libacars.h>
#include <libacars/acars.h>

#include "poa_decoder.h"

#define GOLDEN_WAV "/Users/bruce/iridium_capture/poa_ref/poa_golden_10min_12k5_4ch.wav"
#define NCH 4

static int g_parsed = 0, g_ok = 0, g_crc_ok = 0;

static la_acars_msg *find_acars_msg(la_proto_node *node)
{
    while (node) {
        if (node->td == &la_DEF_acars_message) return (la_acars_msg *)node->data;
        node = node->next;
    }
    return NULL;
}

static void on_block(const poa_block_t *b, void *user)
{
    (void)user;
    // Reconstruct the frame libacars expects: [mode..ETX] + CRC(2) + DEL(0x7f)
    // (acarsdec keeps the CRC separate and drops the DEL; the real device bridge
    // does the same reassembly before acars_deliver).
    uint8_t buf[POA_TXT_MAX + 3];
    int n = b->len < POA_TXT_MAX ? b->len : POA_TXT_MAX;
    memcpy(buf, b->txt, (size_t)n);
    // poa_decoder strips the odd-parity high bit before emit, but libacars
    // computes the CRC over the RAW parity-bearing bytes (it strips parity
    // itself, after the CRC). Reconstruct odd parity per text byte exactly as
    // frame_decoder_push_poa does on device, so libacars' CRC validates. The 2
    // CRC bytes stay raw. (Reconstruction is lossless: poa_decoder guaranteed
    // valid odd parity before emitting.)
    for (int i = 0; i < n; i++) {
        unsigned v = buf[i] & 0x7f, bits = 0, t = v;
        while (t) { bits += t & 1; t >>= 1; }
        buf[i] = (bits & 1) ? v : (v | 0x80); // ACARS = odd parity in bit 7
    }
    buf[n++] = b->crc[0];
    buf[n++] = b->crc[1];
    buf[n++] = 0x7f; // DEL terminator
    // Aircraft->ground (JQ0404 position report) -> downlink.
    la_proto_node *node = la_acars_parse(buf, n, LA_MSG_DIR_AIR2GND);
    if (!node) return;
    la_acars_msg *m = find_acars_msg(node);
    if (m) {
        g_parsed++;
        printf("  parsed: reg='%s' flight='%.6s' label='%.2s' crc_ok=%d err=%d\n",
               m->reg, m->flight_id, m->label, (int)m->crc_ok, (int)m->err);
        if (memcmp(m->flight_id, "JQ0404", 6) == 0 && strstr(m->reg, "VH-VGD") &&
            memcmp(m->label, "3L", 2) == 0) {
            g_ok++;
            // The golden is a KNOWN-CLEAN decode (poa_block err=0, crc_fixed=0),
            // so libacars must agree the CRC checks out — else the poa->libacars
            // bridge passes the CRC bytes/boundaries wrong and EVERY POA frame
            // would read crc=BAD downstream (the untested path the live decode
            // exposed). This is the CRC-bridge gate.
            if (m->crc_ok) g_crc_ok++;
        }
    }
    la_proto_tree_destroy(node);
}

int main(void)
{
    FILE *f = fopen(GOLDEN_WAV, "rb");
    if (!f) { printf("SKIP: golden WAV not present (%s)\n", GOLDEN_WAV); return 0; }
    if (fseek(f, 44, SEEK_SET) != 0) { fclose(f); return 0; }

    poa_decoder_t *d = poa_decoder_create(NCH, on_block, NULL);
    assert(d);
    enum { FR = 12500 };
    static short inter[FR * NCH];
    static float chan[NCH][FR];
    size_t got;
    while ((got = fread(inter, sizeof(short) * NCH, FR, f)) > 0) {
        for (int c = 0; c < NCH; c++)
            for (size_t i = 0; i < got; i++) chan[c][i] = (float)inter[i * NCH + c];
        for (int c = 0; c < NCH; c++) poa_decoder_feed(d, c, chan[c], (int)got);
    }
    fclose(f);
    poa_decoder_destroy(d);

    printf("poa->libacars: %d block(s) parsed, %d matched JQ0404/VH-VGD/3L, %d crc_ok\n",
           g_parsed, g_ok, g_crc_ok);
    assert(g_parsed >= 1);
    assert(g_ok >= 1);
    // CRC-bridge gate: the golden is a clean decode, so libacars must agree the
    // CRC checks out once odd parity is reconstructed (as frame_decoder_push_poa
    // does on device). This is the assertion that was MISSING — its absence let
    // every POA frame read crc=BAD downstream. Do NOT delete it.
    assert(g_crc_ok >= 1);
    printf("PASS: test_poa_libacars (bridge format + CRC validate through libacars)\n");
    return 0;
}
