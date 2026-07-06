// Real-capture proof for the IDA -> SBD -> ACARS tail glue.
//
// Unlike test_acars_tail_synthetic.c (a synthetic-but-valid positive
// control), this test feeds REAL over-the-air ACARS frames from the
// 2026-07-06 HydraSDR milestone capture (fixture_acars_frames.h, see
// tests/scripts/build_acars_fixture.py for the derivation) through
// ida_reassembler_feed() -> sbd_reassembler_feed() ->
// la_acars_parse_and_reassemble() and asserts the EXACT ACARS fields
// against acars-milestone-20260706.txt (iridium-toolkit's own
// `reassembler.py -m acars` output over that capture) — an
// independent golden per the no-circular-goldens rule.
//
// Both fixture messages need 2 chained LW.DA fragments (da_cont/
// da_ctr) to assemble their SBD envelope, which is what motivated
// adding ida_reassembler.c: before it existed, frame_decoder.c fed
// each single burst straight to sbd_reassembler_feed(), which could
// only ever see a truncated (and hence rejected) SBD envelope for any
// message needing more than one physical burst -- which, per
// iridium-toolkit's own stats on this capture ("46 valid packets
// assembled from 178 fragments"), is effectively all real SBD/ACARS
// traffic here.
//
// Deliberate scope: this test constructs ida_decoded_t fragments
// directly from the fixture's real (verified-CRC-OK) post-BCH bytes,
// the same way test_sbd_reassembler.c's make_ida() does, rather than
// calling iridium_frame_classify()/ida_decode() on raw pre-BCH bits --
// genuine raw bits for these specific captured bursts are not
// available (see fixture_acars_frames.h's header comment). The
// classify+BCH layer is already regression-gated against real RF by
// test_iridium_frame_corpus / test_ida_decode_corpus on the
// Albuquerque corpus; this test's unique value is proving the
// SBD/ACARS reassembly layers reproduce a real, independently-verified
// multi-message capture exactly.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "ida_decode.h"
#include "ida_reassembler.h"
#include "sbd_reassembler.h"
#include "acars_tail.h"
#include "fixture_acars_frames.h"
#include <libacars/reassembly.h>

static int passed = 0, failed = 0;

#define CHECK(cond, fmt, ...)                                             \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("  FAIL line %d: " fmt "\n", __LINE__, ##__VA_ARGS__); \
            failed++;                                                     \
        } else {                                                          \
            passed++;                                                     \
        }                                                                 \
    } while (0)

// Build a synthetic ida_decoded_t carrying one fixture fragment's real
// captured bytes. Mirrors test_sbd_reassembler.c / decode_burst_capture's
// contract: ida_decode() only ever returns fields on ok/header_ok/crc_ok
// == true, which is exactly what gr-iridium's "CRC:OK" tag on the source
// IDA: line already confirms for each of these fragments.
static void make_ida(ida_decoded_t *ida, const acars_fixture_fragment_t *fr)
{
    memset(ida, 0, sizeof(*ida));
    ida->ok          = true;
    ida->header_ok   = true;
    ida->crc_ok      = true;
    ida->blocks_ok   = 10;
    ida->n_blocks    = 10;
    ida->da_cont     = fr->da_cont;
    ida->da_ctr      = fr->da_ctr;
    ida->payload_len = fr->payload_len;
    memcpy(ida->payload, fr->payload, fr->payload_len);
}

static void test_message(int idx)
{
    const acars_fixture_message_t *m = &ACARS_FIXTURE_MESSAGES[idx];
    printf("Test: real-capture ACARS message %d: %s\n", idx, m->description);

    sbd_reassembler_t sbd_ctx;
    sbd_reassembler_init(&sbd_ctx);
    ida_reassembler_t ida_reasm;
    ida_reassembler_init(&ida_reasm);
    la_reasm_ctx *reasm = la_reasm_ctx_new();
    CHECK(reasm != NULL, "la_reasm_ctx_new() returned NULL");
    if (!reasm) return;

    CHECK(m->n_fragments >= 2, "expected >=2 fragments (this capture's "
                               "envelopes always need cross-burst "
                               "reassembly), got %d",
          m->n_fragments);

    acars_tail_result_t tail     = {0};
    int                 final_rc = -2;
    for (int i = 0; i < m->n_fragments; i++) {
        const acars_fixture_fragment_t *fr = &m->fragments[i];
        ida_decoded_t                   ida;
        make_ida(&ida, fr);

        int rc = acars_tail_feed(&sbd_ctx, reasm, &ida_reasm, &ida, m->uplink,
                                 fr->freq_hz, fr->timestamp_us, &tail);
        if (i < m->n_fragments - 1) {
            CHECK(rc == 0, "fragment %d/%d: rc=%d (expected 0=chain still open)",
                  i + 1, m->n_fragments, rc);
        } else {
            final_rc = rc;
        }
    }

    CHECK(final_rc == 1, "final fragment: rc=%d (expected 1=SBD+ACARS complete)",
          final_rc);
    CHECK(tail.sbd_ready, "tail.sbd_ready is false");
    CHECK(tail.acars_ready, "tail.acars_ready is false — libacars parse "
                            "did not complete");

    CHECK(tail.mode == m->expected_mode, "tail.mode='%c' (expected '%c')",
          tail.mode, m->expected_mode);
    // libacars remaps a raw 0x7f (DEL) second label byte to the
    // printable 'd' (libacars/acars.c: `if (msg->label[1] == 0x7f)
    // msg->label[1] = 'd';`) -- the fixture's expected_label is the
    // raw over-the-air byte, so apply the same remap before comparing.
    uint8_t exp_label1 = (m->expected_label[1] == 0x7f) ? 'd' : m->expected_label[1];
    CHECK((uint8_t)tail.label[0] == m->expected_label[0] &&
              (uint8_t)tail.label[1] == exp_label1,
          "tail.label=%02x:%02x (expected %02x:%02x, raw wire %02x:%02x)",
          (uint8_t)tail.label[0], (uint8_t)tail.label[1],
          m->expected_label[0], exp_label1,
          m->expected_label[0], m->expected_label[1]);
    CHECK(tail.block_id == m->expected_block_id,
          "tail.block_id='%c' (expected '%c')", tail.block_id, m->expected_block_id);
    // tail.reg is the raw fixed-width 7-byte field (libacars doesn't
    // strip padding); short registrations like "A62001" (6 chars) are
    // left-padded with '.'. iridium-toolkit's own pretty-printer
    // strips that same padding before display (sbd.py: `while
    // q.f_reg[0:1]==b'.': q.f_reg=q.f_reg[1:]`) -- match that
    // convention here so expected_reg reads as the human-readable REG.
    const char *reg_nodot = tail.reg;
    while (*reg_nodot == '.')
        reg_nodot++;
    CHECK(strcmp(reg_nodot, m->expected_reg) == 0,
          "tail.reg=\"%s\" (expected REG \"%s\", raw field \"%s\")",
          reg_nodot, m->expected_reg, tail.reg);
    CHECK(tail.sbd.uplink == false, "tail.sbd.uplink=%d (expected downlink)",
          tail.sbd.uplink);
    CHECK(tail.ack == m->expected_ack, "tail.ack='%c' (expected '%c')",
          tail.ack, m->expected_ack);
    CHECK(strcmp(tail.txt, m->expected_txt) == 0,
          "tail.txt=\"%s\" (expected \"%s\" — this capture's demand-mode "
          "pings carry no text)",
          tail.txt, m->expected_txt);

    printf("  OK: %s -> mode=%c REG=%s label=%02x:%02x block=%c txt=\"%s\"\n",
           m->description, tail.mode, tail.reg, (uint8_t)tail.label[0],
           (uint8_t)tail.label[1], tail.block_id, tail.txt);

    la_reasm_ctx_destroy(reasm);
}

int main(void)
{
    printf("Real-capture fixture: %d messages (2026-07-06 HydraSDR, REG A62001)\n",
           ACARS_FIXTURE_NUM_MESSAGES);
    for (int i = 0; i < ACARS_FIXTURE_NUM_MESSAGES; i++) {
        test_message(i);
    }
    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
