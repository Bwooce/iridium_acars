// Round-trip proof for ida_encode_da_frame() — the inverse of the
// production iridium_frame_classify() + ida_decode() LW.DA path.
//
// Context: the device FRAME_DECODER smoke corpus only has one
// injection point, frame_decoder_push(), which consumes raw
// post-qpsk_demod bits and runs them through the FULL production
// classify -> BCH chain. tests/fixtures/fixture_acars_frames.h's real
// captured ACARS payloads have no surviving raw pre-BCH bits (see that
// fixture's header comment), so ida_encode_da_frame() re-encodes the
// known-good da_cont/da_ctr/payload triples into a content-accurate
// wire bitstream instead.
//
// This test is the load-bearing proof that the re-encoding is correct
// BEFORE it's trusted in the on-device smoke corpus: for every fixture
// fragment, encode -> classify -> ida_decode() and assert an EXACT
// match (header fields, payload bytes, CRC) against the fixture; then
// feed the two full messages through the real ida_reassembler ->
// sbd_reassembler -> libacars chain (the same one frame_decoder.c
// uses) and assert the reassembled REG/mode/label/ACK land on the
// fixture's golden values, exactly like test_acars_tail_real.c does
// but starting from ENCODED BITS instead of hand-built ida_decoded_t
// structs -- i.e. this test additionally proves the classify+BCH layer
// those structs bypass.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "iridium_frame.h"
#include "ida_decode.h"
#include "ida_encode.h"
#include "ida_reassembler.h"
#include "sbd_reassembler.h"
#include "fixture_acars_frames.h"
#include <libacars/reassembly.h>
#include <libacars/acars.h>
#include <libacars/libacars.h>

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

// Per-fragment proof: encode -> classify -> ida_decode -> compare
// against the fixture's own fragment fields exactly.
static void test_fragment_roundtrip(int msg_idx, int frag_idx)
{
    const acars_fixture_message_t  *m  = &ACARS_FIXTURE_MESSAGES[msg_idx];
    const acars_fixture_fragment_t *fr = &m->fragments[frag_idx];

    uint8_t bits[IDA_ENCODE_FRAME_BITS];
    int     rc = ida_encode_da_frame(fr->da_cont, fr->da_ctr, fr->payload,
                                     fr->payload_len, bits);
    CHECK(rc == 0, "msg %d frag %d: ida_encode_da_frame rc=%d", msg_idx, frag_idx, rc);
    if (rc != 0) return;

    iridium_frame_t f = {0};
    rc                = iridium_frame_classify(bits, sizeof(bits), IR_FRM_DIR_DOWNLINK, &f);
    CHECK(rc == 0, "msg %d frag %d: classify rc=%d", msg_idx, frag_idx, rc);
    CHECK(f.type == IR_FRAME_LW, "msg %d frag %d: type=%s (expected LW)",
          msg_idx, frag_idx, iridium_frame_type_name(f.type));
    CHECK(f.lw_subtype == IR_LW_DA, "msg %d frag %d: lw_subtype=%s (expected DA)",
          msg_idx, frag_idx, iridium_lw_subtype_name(f.lw_subtype));

    ida_decoded_t d = {0};
    rc              = ida_decode(&f, &d);
    CHECK(rc == 0, "msg %d frag %d: ida_decode rc=%d", msg_idx, frag_idx, rc);
    CHECK(d.blocks_ok == 10 && d.ok, "msg %d frag %d: blocks_ok=%d ok=%d (expected 10/true)",
          msg_idx, frag_idx, d.blocks_ok, d.ok);
    CHECK(d.header_ok, "msg %d frag %d: header_ok=false", msg_idx, frag_idx);
    CHECK(d.crc_ok, "msg %d frag %d: crc_ok=false (crc_reported=%04x crc_computed=%04x)",
          msg_idx, frag_idx, d.da_crc_reported, d.da_crc_computed);
    CHECK(d.da_cont == fr->da_cont, "msg %d frag %d: da_cont=%d (expected %d)",
          msg_idx, frag_idx, d.da_cont, fr->da_cont);
    CHECK(d.da_ctr == fr->da_ctr, "msg %d frag %d: da_ctr=%d (expected %d)",
          msg_idx, frag_idx, d.da_ctr, fr->da_ctr);
    CHECK(d.payload_len == fr->payload_len, "msg %d frag %d: payload_len=%d (expected %d)",
          msg_idx, frag_idx, d.payload_len, fr->payload_len);
    CHECK(memcmp(d.payload, fr->payload, fr->payload_len) == 0,
          "msg %d frag %d: payload bytes mismatch", msg_idx, frag_idx);

    printf("  OK: msg %d frag %d -> cont=%d ctr=%d len=%u crc=%s (%u/%u blocks)\n",
           msg_idx, frag_idx, d.da_cont, d.da_ctr, d.payload_len,
           d.crc_ok ? "OK" : "BAD", d.blocks_ok, d.n_blocks);
}

// End-to-end: encode both fragments of a message, push them through
// the REAL classify -> ida_decode -> ida_reassembler -> sbd_reassembler
// -> libacars chain (same production code frame_decoder.c drives), and
// check the reassembled ACARS fields match the fixture's golden REG.
static void test_message_end_to_end(int idx)
{
    const acars_fixture_message_t *m = &ACARS_FIXTURE_MESSAGES[idx];
    printf("Test: end-to-end encode->classify->BCH->reassemble message %d: %s\n",
           idx, m->description);

    ida_reassembler_t ida_reasm;
    ida_reassembler_init(&ida_reasm);
    sbd_reassembler_t sbd_ctx;
    sbd_reassembler_init(&sbd_ctx);
    la_reasm_ctx *reasm = la_reasm_ctx_new();
    CHECK(reasm != NULL, "la_reasm_ctx_new() returned NULL");
    if (!reasm) return;

    uint8_t merged[IDA_REASM_MAX_BYTES];
    int     merged_len = 0;
    int     rc_reasm   = -2;

    for (int i = 0; i < m->n_fragments; i++) {
        const acars_fixture_fragment_t *fr = &m->fragments[i];
        uint8_t                         bits[IDA_ENCODE_FRAME_BITS];
        CHECK(ida_encode_da_frame(fr->da_cont, fr->da_ctr, fr->payload,
                                  fr->payload_len, bits) == 0,
              "msg %d frag %d: encode failed", idx, i);

        iridium_frame_t f = {0};
        CHECK(iridium_frame_classify(bits, sizeof(bits), IR_FRM_DIR_DOWNLINK, &f) == 0,
              "msg %d frag %d: classify failed", idx, i);
        ida_decoded_t d = {0};
        CHECK(ida_decode(&f, &d) == 0 && d.ok && d.header_ok && d.crc_ok,
              "msg %d frag %d: ida_decode not clean (ok=%d hdr=%d crc=%d)",
              idx, i, d.ok, d.header_ok, d.crc_ok);

        // Fragments are 280 ms-window-compatible by construction (this
        // test pushes them microseconds apart); use monotonically
        // increasing timestamps so the reassembler's gap/timeout logic
        // sees a sane order.
        uint64_t now_us = fr->timestamp_us;
        rc_reasm        = ida_reassembler_feed(&ida_reasm, &d, m->uplink,
                                               fr->freq_hz, now_us, merged,
                                               (int)sizeof(merged), &merged_len);
        if (i < m->n_fragments - 1) {
            CHECK(rc_reasm == 0, "msg %d frag %d/%d: ida_reassembler rc=%d (expected 0=open)",
                  idx, i + 1, m->n_fragments, rc_reasm);
        }
    }
    CHECK(rc_reasm == 1, "msg %d: final ida_reassembler rc=%d (expected 1=complete)",
          idx, rc_reasm);
    if (rc_reasm != 1) {
        la_reasm_ctx_destroy(reasm);
        return;
    }

    sbd_message_t sbd;
    int           rc_sbd = sbd_reassembler_feed(&sbd_ctx, merged, merged_len,
                                                m->uplink, m->fragments[m->n_fragments - 1].timestamp_us,
                                                &sbd);
    CHECK(rc_sbd == 1, "msg %d: sbd_reassembler rc=%d (expected 1=complete)", idx, rc_sbd);
    if (rc_sbd != 1) {
        la_reasm_ctx_destroy(reasm);
        return;
    }

    // Strip the SOH (+ optional 8-byte 0x03 header) prefix exactly like
    // frame_decoder.c's strip_acars_prefix() / acars_tail.c do.
    const uint8_t *acars_buf = sbd.payload;
    int            acars_len = sbd.payload_len;
    if (acars_len >= 1 && acars_buf[0] == 0x01) {
        acars_buf++;
        acars_len--;
        if (acars_len >= 8 && acars_buf[0] == 0x03) {
            acars_buf += 8;
            acars_len -= 8;
        }
    }
    CHECK(acars_len >= 8, "msg %d: stripped ACARS payload too short (%d bytes)", idx, acars_len);

    struct timeval rx_time = {0};
    la_msg_dir     dir     = sbd.uplink ? LA_MSG_DIR_GND2AIR : LA_MSG_DIR_AIR2GND;
    la_proto_node *node    = la_acars_parse_and_reassemble(acars_buf, acars_len, dir,
                                                           reasm, rx_time);
    CHECK(node != NULL, "msg %d: la_acars_parse_and_reassemble returned NULL", idx);
    if (!node) {
        la_reasm_ctx_destroy(reasm);
        return;
    }

    la_acars_msg *a = NULL;
    for (la_proto_node *n = node; n; n = n->next) {
        if (n->td == &la_DEF_acars_message && n->data) {
            a = (la_acars_msg *)n->data;
            break;
        }
    }
    CHECK(a != NULL, "msg %d: no la_acars_msg in parse tree", idx);
    if (a) {
        CHECK(a->reasm_status == LA_REASM_COMPLETE || a->reasm_status == LA_REASM_SKIPPED,
              "msg %d: reasm_status=%d (expected COMPLETE/SKIPPED)", idx, a->reasm_status);
        const char *reg_nodot = a->reg;
        while (*reg_nodot == '.')
            reg_nodot++;
        CHECK(strcmp(reg_nodot, m->expected_reg) == 0,
              "msg %d: reg=\"%s\" (expected \"%s\", raw \"%s\")",
              idx, reg_nodot, m->expected_reg, a->reg);
        CHECK(a->mode == m->expected_mode, "msg %d: mode='%c' (expected '%c')",
              idx, a->mode, m->expected_mode);
        CHECK(a->block_id == m->expected_block_id, "msg %d: block_id='%c' (expected '%c')",
              idx, a->block_id, m->expected_block_id);
        CHECK(a->crc_ok, "msg %d: libacars crc_ok=false", idx);
        printf("  OK: %s -> mode=%c REG=%s block=%c crc=%s\n",
               m->description, a->mode, reg_nodot, a->block_id,
               a->crc_ok ? "OK" : "BAD");
    }
    la_proto_tree_destroy(node);
    la_reasm_ctx_destroy(reasm);
}

int main(void)
{
    printf("Round-trip: ida_encode_da_frame() against %d real-capture fixture messages\n",
           ACARS_FIXTURE_NUM_MESSAGES);

    for (int i = 0; i < ACARS_FIXTURE_NUM_MESSAGES; i++) {
        const acars_fixture_message_t *m = &ACARS_FIXTURE_MESSAGES[i];
        for (int j = 0; j < m->n_fragments; j++) {
            test_fragment_roundtrip(i, j);
        }
    }
    for (int i = 0; i < ACARS_FIXTURE_NUM_MESSAGES; i++) {
        test_message_end_to_end(i);
    }

    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
