// Positive control for the IDA -> SBD -> ACARS tail glue (acars_tail.c),
// the shared helper decode_burst_capture.c now uses to turn a decoded
// LW.DA frame into printed SBD/ACARS output.
//
// This test never runs ida_decode() itself (no BCH/de-interleave
// machinery involved) — like test_sbd_reassembler.c it constructs
// synthetic ida_decoded_t values directly at the post-BCH byte level.
// What's new here is going one stage further: the synthetic IDA
// payloads wrap a real, CRC-correct ACARS frame (built the same way
// test_libacars_link.c's canonical-frame case does), split across
// multiple 0x76/0x08 SBD fragments the way a real multi-block message
// would arrive. Feeding those fragments through sbd_reassembler_feed()
// and then la_acars_parse_and_reassemble() (both via acars_tail_feed())
// proves the tail glue shared with decode_burst_capture works
// end-to-end: IDA payload bytes in, exact ACARS text out.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "ida_decode.h"
#include "sbd_reassembler.h"
#include "acars_tail.h"
#include <libacars/crc.h>
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

// Build a synthetic ida_decoded_t the way test_sbd_reassembler.c's
// make_ida() does: just drop bytes into payload[], skip BCH state.
static void make_ida(ida_decoded_t *ida, const uint8_t *bytes, int n)
{
    memset(ida, 0, sizeof(*ida));
    ida->ok          = true;
    ida->header_ok   = true;
    ida->crc_ok      = true;
    ida->blocks_ok   = 10;
    ida->n_blocks    = 10;
    ida->payload_len = (uint8_t)(n > (int)sizeof(ida->payload) ? sizeof(ida->payload) : n);
    memcpy(ida->payload, bytes, ida->payload_len);
}

// Fragment overhead for a 0x76/0x08 downlink data frame per
// sbd_reassembler.c: 2 B type prefix + 7 B "0x26" prehdr + 3 B 0x10
// sub-header = 12 B, leaving (IDA payload cap 24 B - 12 B) = 12 B of
// message content per fragment.
#define FRAG_OVERHEAD 12
#define FRAG_MAX_CONTENT (24 - FRAG_OVERHEAD)

static void test_acars_tail_roundtrip(void)
{
    printf("Test: multi-fragment IDA/SBD -> ACARS tail round-trips exact text\n");

    // 1. Build a canonical downlink ACARS frame (mirrors
    //    test_libacars_link.c's "Canonical ACARS frame" case), with a
    //    distinctive message text that forces >1 SBD fragment.
    const char *flight_text = "PROOF OK";
    uint8_t     acars[64];
    size_t      i = 0;

    acars[i++] = '2'; // mode
    memcpy(acars + i, "ABCDEFG", 7);
    i += 7;            // 7-byte address
    acars[i++] = 0x15; // ack = NAK
    acars[i++] = 'H';
    acars[i++] = '1';  // label "H1"
    acars[i++] = '5';  // block_id digit -> downlink
    acars[i++] = 0x02; // STX
    // msg_num "M04" + seq 'A': la_acars_parse_and_reassemble()'s
    // downlink reassembly path keys sequencing off msg_num_seq (must
    // be 'A'-relative -> seq_num 0 here) with is_final_fragment=true
    // (this message ends in ETX below), which the reassembly engine
    // recognises as a complete, unfragmented message and returns
    // LA_REASM_SKIPPED for -- one of the two "ready" states try_acars()
    // (and acars_tail_feed) accept alongside LA_REASM_COMPLETE.
    memcpy(acars + i, "M04A", 4);
    i += 4;
    memcpy(acars + i, "FLT123", 6);
    i += 6;
    memcpy(acars + i, flight_text, strlen(flight_text));
    i += strlen(flight_text);
    acars[i++] = 0x03; // ETX

    uint16_t crc = la_crc16_ccitt(acars, i, 0);
    acars[i++]   = (uint8_t)(crc & 0xff);
    acars[i++]   = (uint8_t)((crc >> 8) & 0xff);
    acars[i++]   = 0x7f; // DEL

    size_t acars_len = i;
    CHECK(acars_len > FRAG_MAX_CONTENT, "test setup: acars_len=%zu should need >1 fragment", acars_len);

    // 2. Split into 0x76/0x08 SBD fragments, each carrying up to
    //    FRAG_MAX_CONTENT bytes of the ACARS frame.
    int msg_cnt = (int)((acars_len + FRAG_MAX_CONTENT - 1) / FRAG_MAX_CONTENT);
    CHECK(msg_cnt >= 2 && msg_cnt <= 32, "msg_cnt=%d out of expected range", msg_cnt);

    sbd_reassembler_t ctx;
    sbd_reassembler_init(&ctx);
    la_reasm_ctx *reasm = la_reasm_ctx_new();
    CHECK(reasm != NULL, "la_reasm_ctx_new() returned NULL");
    if (!reasm) return;

    acars_tail_result_t tail     = {0};
    int                 final_rc = -2;
    size_t              off      = 0;
    for (int frag = 1; frag <= msg_cnt; frag++) {
        size_t remain = acars_len - off;
        size_t take   = remain < FRAG_MAX_CONTENT ? remain : FRAG_MAX_CONTENT;

        uint8_t bytes[24];
        int     n  = 0;
        bytes[n++] = 0x76;
        bytes[n++] = 0x08;
        bytes[n++] = 0x26; // prehdr[0]
        bytes[n++] = 0x00;
        bytes[n++] = 0x00;
        bytes[n++] = (uint8_t)msg_cnt; // prehdr[3] = msgcnt
        bytes[n++] = 0x00;
        bytes[n++] = 0x00;
        bytes[n++] = 0x00;
        bytes[n++] = 0x10;          // sub-header marker
        bytes[n++] = (uint8_t)take; // sub-header length
        bytes[n++] = (uint8_t)frag; // sub-header msgno
        memcpy(bytes + n, acars + off, take);
        n += (int)take;

        ida_decoded_t ida;
        make_ida(&ida, bytes, n);

        int rc = acars_tail_feed(&ctx, reasm, &ida, /*uplink=*/false,
                                 1000000ULL + (uint64_t)frag * 100000ULL, &tail);
        if (frag < msg_cnt) {
            CHECK(rc == 0, "fragment %d/%d: rc=%d (expected 0=partial)", frag, msg_cnt, rc);
        } else {
            final_rc = rc;
        }
        off += take;
    }
    CHECK(off == acars_len, "fragment offsets summed to %zu, expected %zu", off, acars_len);

    // 3. Final fragment must complete the SBD message and the ACARS parse.
    CHECK(final_rc == 1, "final fragment: rc=%d (expected 1=complete)", final_rc);
    CHECK(tail.sbd_ready, "tail.sbd_ready is false");
    CHECK(tail.sbd.payload_len == acars_len,
          "sbd.payload_len=%u (expected %zu)", tail.sbd.payload_len, acars_len);
    CHECK(memcmp(tail.sbd.payload, acars, acars_len) == 0,
          "reassembled SBD payload does not match the source ACARS frame bytes");

    CHECK(tail.acars_ready, "tail.acars_ready is false — libacars parse did not complete");
    CHECK(tail.crc_ok, "tail.crc_ok is false (expected true — CRC was computed correctly)");
    CHECK(tail.mode == '2', "tail.mode='%c' (expected '2')", tail.mode);
    CHECK(memcmp(tail.label, "H1", 2) == 0, "tail.label='%.2s' (expected 'H1')", tail.label);
    CHECK(tail.block_id == '5', "tail.block_id='%c' (expected '5')", tail.block_id);
    CHECK(memcmp(tail.flight_id, "FLT123", 6) == 0,
          "tail.flight_id='%.6s' (expected 'FLT123')", tail.flight_id);
    // The exact proof: the text that went in over the (synthetic) air
    // interface comes back out, byte for byte, after IDA -> SBD ->
    // ACARS.
    CHECK(strcmp(tail.txt, flight_text) == 0,
          "tail.txt=\"%s\" (expected \"%s\")", tail.txt, flight_text);

    printf("  round-trip OK: %d-fragment SBD message (%zu B) -> ACARS txt=\"%s\"\n",
           msg_cnt, acars_len, tail.txt);

    la_reasm_ctx_destroy(reasm);
}

int main(void)
{
    test_acars_tail_roundtrip();
    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
