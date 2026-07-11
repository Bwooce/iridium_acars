// Unit tests for sbd_salvage_parse() -- the stateless, truncation-
// tolerant SBD envelope extractor (chain-salvage Task B2).
//
// Reuses the same synthetic byte-construction style as
// test_sbd_reassembler.c (a 0x76 0x08 envelope with a 7-byte prehdr and
// a 0x10 sub-header) so the two test files stay easy to cross-check.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
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

// Full, untruncated 0x76 0x08 multi-frame-style envelope:
//   76 08                     -- type
//   26 00 00 03 00 00 00      -- 7-byte prehdr, prehdr[3]=msg_cnt=3
//   10 06 02                 -- 0x10 sub-header: len=6, msgno=2
//   11 22 33 44 55 66         -- 6-byte body
// 18 bytes total.
static const uint8_t FULL_ENV[] = {
    0x76,
    0x08,
    0x26,
    0x00,
    0x00,
    0x03,
    0x00,
    0x00,
    0x00,
    0x10,
    0x06,
    0x02,
    0x11,
    0x22,
    0x33,
    0x44,
    0x55,
    0x66,
};
#define FULL_LEN ((int)sizeof(FULL_ENV))

// --- 1. Positive control: full, untruncated envelope parses cleanly.
static void test_positive_control(void)
{
    printf("Test: full 0x76 0x08 envelope w/ 0x10 sub-header -> exact parse\n");
    sbd_salvage_info_t out;
    int                rc = sbd_salvage_parse(FULL_ENV, FULL_LEN, false, &out);
    CHECK(rc == 1, "rc=%d", rc);
    CHECK(out.type == SBD_TYPE_DATA_DL_7608, "type=%d", out.type);
    CHECK(out.truncated == false, "truncated=%d (expected false)", out.truncated);
    CHECK(out.msg_no == 2, "msg_no=%d (expected 2)", out.msg_no);
    CHECK(out.msg_cnt == 3, "msg_cnt=%d (expected 3)", out.msg_cnt);
    CHECK(out.body_len == 6, "body_len=%d (expected 6)", out.body_len);
    CHECK(out.body != NULL, "body is NULL");
    static const uint8_t expected_body[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
    CHECK(memcmp(out.body, expected_body, 6) == 0, "body mismatch");
}

// --- 2. Truncation sweep: every prefix length 0..FULL_LEN must never
//        crash / read OOB, and must hit the documented invariants at
//        the boundaries we can predict analytically:
//          L < 2                    -> rc == 0 (can't classify)
//          L == 9  (end of prehdr)  -> rc == 1, truncated == false,
//                                       body_len == 0 (natural boundary,
//                                       indistinguishable from cut-off --
//                                       matches feed()'s own semantics)
//          L in [13,17] (past the   -> rc == 1, truncated == true,
//          0x10 sub-header, short      body_len == L - 12 (bytes of the
//          of the declared 6-byte      declared 6-byte body actually
//          body)                       present)
//          L == FULL_LEN            -> rc == 1, truncated == false,
//                                       body_len == 6 (full positive
//                                       control again)
static void test_truncation_sweep(void)
{
    printf("Test: truncation sweep L=0..%d -> no crash, documented invariants hold\n", FULL_LEN);
    for (int L = 0; L <= FULL_LEN; L++) {
        sbd_salvage_info_t out;
        int                rc = sbd_salvage_parse(FULL_ENV, L, false, &out);
        CHECK(rc == 0 || rc == 1, "L=%d rc=%d (must be 0 or 1)", L, rc);

        if (rc == 1) {
            // General invariants for every successful parse: body_len
            // never exceeds what we actually gave it, and if non-NULL,
            // body points inside [payload, payload+L).
            CHECK(out.body_len >= 0 && out.body_len <= L,
                  "L=%d body_len=%d out of range", L, out.body_len);
            if (out.body_len > 0) {
                CHECK(out.body != NULL, "L=%d body_len>0 but body==NULL", L);
                CHECK(out.body >= FULL_ENV && out.body + out.body_len <= FULL_ENV + L,
                      "L=%d body range escapes [0,%d)", L, L);
            } else {
                CHECK(out.body == NULL, "L=%d body_len==0 but body!=NULL", L);
            }
        }

        if (L < 2) {
            CHECK(rc == 0, "L=%d rc=%d (expected 0, can't classify)", L, rc);
        } else if (L == 9) {
            CHECK(rc == 1, "L=%d rc=%d (expected 1)", L, rc);
            CHECK(out.truncated == false, "L=%d truncated=%d (expected false, natural boundary)", L, out.truncated);
            CHECK(out.body_len == 0, "L=%d body_len=%d (expected 0)", L, out.body_len);
        } else if (L >= 13 && L <= 17) {
            CHECK(rc == 1, "L=%d rc=%d (expected 1)", L, rc);
            CHECK(out.truncated == true, "L=%d truncated=%d (expected true)", L, out.truncated);
            CHECK(out.msg_no == 2, "L=%d msg_no=%d (expected 2)", L, out.msg_no);
            CHECK(out.msg_cnt == 3, "L=%d msg_cnt=%d (expected 3)", L, out.msg_cnt);
            int expect_body_len = L - 12;
            CHECK(out.body_len == expect_body_len, "L=%d body_len=%d (expected %d)", L, out.body_len, expect_body_len);
            CHECK(memcmp(out.body, FULL_ENV + 12, (size_t)expect_body_len) == 0,
                  "L=%d body content mismatch", L);
        } else if (L == FULL_LEN) {
            CHECK(rc == 1, "L=%d rc=%d (expected 1)", L, rc);
            CHECK(out.truncated == false, "L=%d truncated=%d (expected false)", L, out.truncated);
            CHECK(out.body_len == 6, "L=%d body_len=%d (expected 6)", L, out.body_len);
        }
    }
    // If we got here without an early CHECK-triggered return, count a
    // pass for the sweep as a whole too.
    passed++;
}

// --- 3. Unknown type is rejected outright.
static void test_unknown_type_reject(void)
{
    printf("Test: unrecognised type prefix -> rc==0\n");
    static const uint8_t bytes[] = {0x12, 0x34, 0x56, 0x78, 0x9a};
    sbd_salvage_info_t   out;
    int                  rc = sbd_salvage_parse(bytes, sizeof(bytes), false, &out);
    CHECK(rc == 0, "rc=%d (expected 0)", rc);
}

// --- 4. Short/mailbox (msg_no==0) reachable from a VALID, untruncated
//        HELLO header (not just the truncated-default path).
static void test_hello_mailbox_reachable(void)
{
    printf("Test: valid 0x06 0x00 HELLO w/ msgcnt=0 -> msg_no==0, not truncated\n");
    // 06 00 20 <22 zero bytes> -- prehdr[15] (== bytes[2+15]) is 0, so
    // msg_cnt==0 -> msg_no==0. Mirrors test_hello_mailbox_check() in
    // test_sbd_reassembler.c.
    uint8_t bytes[24] = {0};
    bytes[0]          = 0x06;
    bytes[1]          = 0x00;
    bytes[2]          = 0x20;
    sbd_salvage_info_t out;
    int                rc = sbd_salvage_parse(bytes, (int)sizeof(bytes), false, &out);
    CHECK(rc == 1, "rc=%d (expected 1)", rc);
    CHECK(out.type == SBD_TYPE_HELLO_0600, "type=%d", out.type);
    CHECK(out.truncated == false, "truncated=%d (expected false)", out.truncated);
    CHECK(out.msg_no == 0, "msg_no=%d (expected 0)", out.msg_no);
    CHECK(out.msg_cnt == 0, "msg_cnt=%d (expected 0)", out.msg_cnt);
    CHECK(out.body_len == 0, "body_len=%d (expected 0)", out.body_len);
}

// --- 5. NULL-safety: NULL payload / NULL out must not crash.
static void test_null_safety(void)
{
    printf("Test: NULL payload / NULL out -> rc==0, no crash\n");
    sbd_salvage_info_t out;
    int                rc = sbd_salvage_parse(NULL, 10, false, &out);
    CHECK(rc == 0, "rc=%d (expected 0 for NULL payload)", rc);
    rc = sbd_salvage_parse(FULL_ENV, FULL_LEN, false, NULL);
    CHECK(rc == 0, "rc=%d (expected 0 for NULL out)", rc);
}

int main(void)
{
    test_positive_control();
    test_truncation_sweep();
    test_unknown_type_reject();
    test_hello_mailbox_reachable();
    test_null_safety();
    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
