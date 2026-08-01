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

static int g_parsed = 0, g_ok = 0;

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
    buf[n++] = b->crc[0];
    buf[n++] = b->crc[1];
    buf[n++] = 0x7f; // DEL terminator
    // Aircraft->ground (JQ0404 position report) -> downlink.
    la_proto_node *node = la_acars_parse(buf, n, LA_MSG_DIR_AIR2GND);
    if (!node) return;
    la_acars_msg *m = find_acars_msg(node);
    if (m) {
        g_parsed++;
        printf("  parsed: reg='%s' flight='%.6s' label='%.2s'\n", m->reg, m->flight_id, m->label);
        if (memcmp(m->flight_id, "JQ0404", 6) == 0 && strstr(m->reg, "VH-VGD") &&
            memcmp(m->label, "3L", 2) == 0)
            g_ok++;
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

    printf("poa->libacars: %d block(s) parsed, %d matched JQ0404/VH-VGD/3L\n", g_parsed, g_ok);
    assert(g_parsed >= 1);
    assert(g_ok >= 1);
    printf("PASS: test_poa_libacars (P3 bridge format == libacars-parseable ACARS)\n");
    return 0;
}
