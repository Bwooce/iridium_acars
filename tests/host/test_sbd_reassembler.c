// Unit tests for sbd_reassembler.
//
// We construct synthetic ida_decoded_t inputs (just the post-D4 bytes,
// no need to drive the BCH/de-interleave machinery) and verify the
// reassembler's state machine: filtering, single-frame emit,
// multi-frame assembly, session timeout.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "ida_decode.h"
#include "sbd_reassembler.h"

static int passed = 0, failed = 0;

#define CHECK(cond, fmt, ...)                                             \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("  FAIL line %d: " fmt "\n", __LINE__, ##__VA_ARGS__); \
            failed++;                                                     \
            return;                                                       \
        } else {                                                          \
            passed++;                                                     \
        }                                                                 \
    } while (0)

// Helper: build a fake ida_decoded_t with the given payload bytes
// (skipping the header parsing; just put the bytes in payload[]).
static void make_ida(ida_decoded_t *ida, const uint8_t *bytes, int n)
{
    memset(ida, 0, sizeof(*ida));
    ida->ok          = true;
    ida->blocks_ok   = 10;
    ida->n_blocks    = 10;
    ida->payload_len = (uint8_t)(n > (int)sizeof(ida->payload) ? sizeof(ida->payload) : n);
    memcpy(ida->payload, bytes, ida->payload_len);
}

// --- 1. Non-SBD frame is filtered.
static void test_filtered_non_sbd(void)
{
    printf("Test: non-SBD payload -> filtered (returns -1)\n");
    sbd_reassembler_t ctx;
    sbd_reassembler_init(&ctx);
    uint8_t       bytes[] = {0x12, 0x34, 0x56, 0x78, 0x9a};
    ida_decoded_t ida;
    make_ida(&ida, bytes, sizeof(bytes));
    sbd_message_t out;
    int           rc = sbd_reassembler_feed(&ctx, ida.payload, ida.payload_len, false, 1000000, &out);
    CHECK(rc == -1, "rc=%d", rc);
    CHECK(ctx.cnt_filtered == 1, "cnt_filtered=%u", ctx.cnt_filtered);
}

// --- 2. SBD HELLO (0x06 0x00) with msgcnt==0 -> short/mailbox emit.
static void test_hello_mailbox_check(void)
{
    printf("Test: 0x06 0x00 with msgcnt=0 -> mailbox check, emit immediately\n");
    sbd_reassembler_t ctx;
    sbd_reassembler_init(&ctx);

    // Type prefix 06 00, then 0x20 (HELLO marker), then 22 bytes of
    // prehdr (truncated — IDA can carry at most ~22 bytes; the
    // reassembler matches Python's slice-truncation behaviour).
    // Set prehdr[15]==0 so msgcnt==0 (mailbox check).
    uint8_t bytes[24] = {0};
    bytes[0]          = 0x06;
    bytes[1]          = 0x00;
    bytes[2]          = 0x20;
    ida_decoded_t ida;
    make_ida(&ida, bytes, sizeof(bytes));
    sbd_message_t out;
    int           rc = sbd_reassembler_feed(&ctx, ida.payload, ida.payload_len, false, 1000000, &out);
    CHECK(rc == 1, "rc=%d (expected 1)", rc);
    CHECK(out.type == SBD_TYPE_HELLO_0600, "type=%d", out.type);
    CHECK(ctx.cnt_short == 1, "cnt_short=%u", ctx.cnt_short);
}

// --- 3. Single-frame DL data: 0x76 0x08 with msgcnt=1 in prehdr,
//        msgno=1 in 0x10 sub-header.
static void test_single_frame_dl(void)
{
    printf("Test: 0x76 0x08 single-frame DL data -> emit immediately\n");
    sbd_reassembler_t ctx;
    sbd_reassembler_init(&ctx);

    // Construct: 76 08 [26 ?? ?? 01 ?? ?? ??] [10 04 01 AA BB CC DD]
    //                  | prehdr (7 bytes, starts with 0x26, prehdr[3]=msgcnt=1
    //                  | 0x10 hdr: len=4, msgno=1
    //                  | 4 bytes payload AA BB CC DD
    uint8_t bytes[16];
    int     i  = 0;
    bytes[i++] = 0x76;
    bytes[i++] = 0x08;
    bytes[i++] = 0x26; // prehdr[0]
    bytes[i++] = 0x00;
    bytes[i++] = 0x00; // prehdr[1..2]
    bytes[i++] = 0x01; // prehdr[3] = msgcnt
    bytes[i++] = 0x00;
    bytes[i++] = 0x00;
    bytes[i++] = 0x00; // prehdr[4..6]
    bytes[i++] = 0x10;
    bytes[i++] = 0x04;
    bytes[i++] = 0x01;
    bytes[i++] = 0xAA;
    bytes[i++] = 0xBB;
    bytes[i++] = 0xCC;
    bytes[i++] = 0xDD;

    ida_decoded_t ida;
    make_ida(&ida, bytes, i);
    sbd_message_t out;
    int           rc = sbd_reassembler_feed(&ctx, ida.payload, ida.payload_len, false, 1000000, &out);
    CHECK(rc == 1, "rc=%d", rc);
    CHECK(out.type == SBD_TYPE_DATA_DL_7608, "type=%d", out.type);
    CHECK(out.payload_len == 4, "payload_len=%u", out.payload_len);
    CHECK(out.payload[0] == 0xAA, "p[0]=0x%x", out.payload[0]);
    CHECK(out.payload[3] == 0xDD, "p[3]=0x%x", out.payload[3]);
    CHECK(ctx.cnt_single == 1, "cnt_single=%u", ctx.cnt_single);
}

// --- 4. Multi-frame DL: msgcnt=2, frames msgno=1 and msgno=2.
static void test_multi_frame_assembly(void)
{
    printf("Test: 2-fragment multi-frame DL data -> assemble both, emit\n");
    sbd_reassembler_t ctx;
    sbd_reassembler_init(&ctx);

    // Frame 1: 76 08 [26 .. .. 02 ..] [10 02 01 11 22]
    uint8_t f1[14];
    int     i = 0;
    f1[i++]   = 0x76;
    f1[i++]   = 0x08;
    f1[i++]   = 0x26;
    f1[i++]   = 0;
    f1[i++]   = 0;
    f1[i++]   = 0x02; // msgcnt=2
    f1[i++]   = 0;
    f1[i++]   = 0;
    f1[i++]   = 0;
    f1[i++]   = 0x10;
    f1[i++]   = 0x02;
    f1[i++]   = 0x01;
    f1[i++]   = 0x11;
    f1[i++]   = 0x22;

    // Frame 2: 76 08 [26 .. .. 02 ..] [10 02 02 33 44]
    uint8_t f2[14];
    int     j = 0;
    f2[j++]   = 0x76;
    f2[j++]   = 0x08;
    f2[j++]   = 0x26;
    f2[j++]   = 0;
    f2[j++]   = 0;
    f2[j++]   = 0x02;
    f2[j++]   = 0;
    f2[j++]   = 0;
    f2[j++]   = 0;
    f2[j++]   = 0x10;
    f2[j++]   = 0x02;
    f2[j++]   = 0x02;
    f2[j++]   = 0x33;
    f2[j++]   = 0x44;

    ida_decoded_t ida;
    sbd_message_t out;

    make_ida(&ida, f1, i);
    int rc = sbd_reassembler_feed(&ctx, ida.payload, ida.payload_len, false, 1000000, &out);
    CHECK(rc == 0, "frame 1 rc=%d (expected 0=partial)", rc);

    make_ida(&ida, f2, j);
    rc = sbd_reassembler_feed(&ctx, ida.payload, ida.payload_len, false, 1100000, &out);
    CHECK(rc == 1, "frame 2 rc=%d (expected 1=complete)", rc);
    CHECK(out.payload_len == 4, "payload_len=%u (expected 4)", out.payload_len);
    CHECK(out.payload[0] == 0x11 && out.payload[1] == 0x22 &&
              out.payload[2] == 0x33 && out.payload[3] == 0x44,
          "merged payload mismatch: %02x %02x %02x %02x",
          out.payload[0], out.payload[1], out.payload[2], out.payload[3]);
    CHECK(ctx.cnt_multi == 1, "cnt_multi=%u", ctx.cnt_multi);
}

// --- 5. Session timeout: open a multi-frame, advance time past 5 s,
//        verify the session gets purged.
static void test_session_timeout(void)
{
    printf("Test: stale multi-frame session expires after 5 s\n");
    sbd_reassembler_t ctx;
    sbd_reassembler_init(&ctx);

    // Frame 1 of a 2-frame message at t=1s.
    uint8_t f1[14];
    int     i = 0;
    f1[i++]   = 0x76;
    f1[i++]   = 0x08;
    f1[i++]   = 0x26;
    f1[i++]   = 0;
    f1[i++]   = 0;
    f1[i++]   = 0x02;
    f1[i++]   = 0;
    f1[i++]   = 0;
    f1[i++]   = 0;
    f1[i++]   = 0x10;
    f1[i++]   = 0x02;
    f1[i++]   = 0x01;
    f1[i++]   = 0x99;
    f1[i++]   = 0xAA;

    ida_decoded_t ida;
    sbd_message_t out;
    make_ida(&ida, f1, i);
    sbd_reassembler_feed(&ctx, ida.payload, ida.payload_len, false, 1000000, &out);

    int active = 0;
    for (int s = 0; s < SBD_MAX_SESSIONS; s++) {
        if (ctx.sessions[s].active) active++;
    }
    CHECK(active == 1, "session count after frame 1 = %d (expected 1)", active);

    // Tick at t=7s — should expire.
    sbd_reassembler_tick(&ctx, 7000000);
    active = 0;
    for (int s = 0; s < SBD_MAX_SESSIONS; s++) {
        if (ctx.sessions[s].active) active++;
    }
    CHECK(active == 0, "session count after expire = %d (expected 0)", active);
    CHECK(ctx.cnt_broken == 1, "cnt_broken=%u (expected 1)", ctx.cnt_broken);
}

int main(void)
{
    test_filtered_non_sbd();
    test_hello_mailbox_check();
    test_single_frame_dl();
    test_multi_frame_assembly();
    test_session_timeout();
    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
