// Unit tests for ida_reassembler_reap() — the chain-salvage path (Task B1).
//
// The reap() API replaces the old auto-expire that ran inside feed() and
// silently discarded a timed-out chain's buffered bytes. Now the driver reaps
// the partial for best-effort PARTIAL salvage. These tests verify: normal
// completion is unaffected; reap returns the exact buffered payload of a
// timed-out chain (1- and 2-fragment); the reap-before-feed contract frees
// table slots; and a stale un-reaped session can never capture a late fragment.

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "ida_decode.h"
#include "ida_reassembler.h"

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

// Build a synthetic LW.DA fragment: ctr/cont header + payload bytes. Sets the
// ok/header_ok/crc_ok preconditions the reassembler's caller guarantees.
static void make_frag(ida_decoded_t *ida, uint8_t ctr, uint8_t cont,
                      const uint8_t *bytes, int n)
{
    memset(ida, 0, sizeof(*ida));
    ida->ok        = true;
    ida->header_ok = true;
    ida->crc_ok    = true;
    ida->blocks_ok = 10;
    ida->n_blocks  = 10;
    ida->da_ctr    = ctr;
    ida->da_cont   = cont;
    if (n > (int)sizeof(ida->payload)) n = (int)sizeof(ida->payload);
    ida->da_len      = (uint8_t)n;
    ida->payload_len = (uint8_t)n;
    memcpy(ida->payload, bytes, (size_t)n);
}

#define FREQ 1620600000u
#define SEC (1000000ULL)

// --- 1. Normal completion still works (regression: reap didn't break feed).
static void test_normal_completion(void)
{
    printf("Test: opener + terminal fragment completes normally\n");
    ida_reassembler_t ctx;
    ida_reassembler_init(&ctx);
    uint8_t       o[] = {0x76, 0x08, 0xAA, 0xBB};
    uint8_t       c[] = {0xCC, 0xDD};
    ida_decoded_t f;
    uint8_t       out[IDA_REASM_MAX_BYTES];
    int           outlen = 0;

    make_frag(&f, 0, 1, o, sizeof(o)); // opener, more to come
    int rc = ida_reassembler_feed(&ctx, &f, false, FREQ, 1 * SEC, out, sizeof(out), &outlen);
    CHECK(rc == 0, "opener should return 0 (open), got %d", rc);

    make_frag(&f, 1, 0, c, sizeof(c)); // terminal
    rc = ida_reassembler_feed(&ctx, &f, false, FREQ, 1 * SEC + 100000, out, sizeof(out), &outlen);
    CHECK(rc == 1, "terminal should return 1 (complete), got %d", rc);
    CHECK(outlen == 6, "merged len should be 6, got %d", outlen);
    CHECK(memcmp(out, "\x76\x08\xAA\xBB\xCC\xDD", 6) == 0, "merged bytes mismatch");
    CHECK(ctx.cnt_completed == 1, "cnt_completed should be 1, got %u", ctx.cnt_completed);
}

// --- 2. First-fragment-only chain is salvageable via reap after timeout.
static void test_salvage_first_fragment(void)
{
    printf("Test: opener then timeout -> reap returns opener payload\n");
    ida_reassembler_t ctx;
    ida_reassembler_init(&ctx);
    uint8_t       o[] = {0x76, 0x08, 0x11, 0x22, 0x33};
    ida_decoded_t f;
    uint8_t       out[IDA_REASM_MAX_BYTES];
    int           outlen = 0;
    make_frag(&f, 0, 1, o, sizeof(o));
    ida_reassembler_feed(&ctx, &f, true, FREQ, 10 * SEC, out, sizeof(out), &outlen);

    ida_salvage_t s;
    // Before the timeout: nothing due.
    CHECK(ida_reassembler_reap(&ctx, 10 * SEC + IDA_REASM_SESSION_TIMEOUT_US / 2, &s) == 0,
          "reap before timeout should return 0");
    // After the timeout: the opener is reaped, byte-exact.
    CHECK(ida_reassembler_reap(&ctx, 10 * SEC + IDA_REASM_SESSION_TIMEOUT_US + 1, &s) == 1,
          "reap after timeout should return 1");
    CHECK(s.payload_len == (int)sizeof(o), "salvage len %d != %zu", s.payload_len, sizeof(o));
    CHECK(memcmp(s.payload, o, sizeof(o)) == 0, "salvage bytes mismatch");
    CHECK(s.uplink == true, "salvage uplink flag lost");
    CHECK(s.frags == 1, "salvage frags should be 1 (opener only), got %u", s.frags);
    CHECK(ctx.cnt_expired == 1, "cnt_expired should be 1, got %u", ctx.cnt_expired);
    // Drained: a second reap finds nothing.
    CHECK(ida_reassembler_reap(&ctx, 20 * SEC, &s) == 0, "second reap should return 0");
}

// --- 3. Two-fragment (opener + 1 continuation, still incomplete) salvage.
static void test_salvage_two_fragments(void)
{
    printf("Test: opener + 1 continuation then timeout -> reap merges both\n");
    ida_reassembler_t ctx;
    ida_reassembler_init(&ctx);
    uint8_t       o[]  = {0x76, 0x08, 0x41, 0x42};
    uint8_t       c1[] = {0x43, 0x44, 0x45};
    ida_decoded_t f;
    uint8_t       out[IDA_REASM_MAX_BYTES];
    int           outlen = 0;
    make_frag(&f, 0, 1, o, sizeof(o));
    ida_reassembler_feed(&ctx, &f, false, FREQ, 5 * SEC, out, sizeof(out), &outlen);
    make_frag(&f, 1, 1, c1, sizeof(c1)); // continuation, still more expected
    int rc = ida_reassembler_feed(&ctx, &f, false, FREQ, 5 * SEC + 100000, out, sizeof(out), &outlen);
    CHECK(rc == 0, "continuation should return 0 (still open), got %d", rc);

    ida_salvage_t s;
    CHECK(ida_reassembler_reap(&ctx, 5 * SEC + 100000 + IDA_REASM_SESSION_TIMEOUT_US + 1, &s) == 1,
          "reap after timeout should return 1");
    CHECK(s.payload_len == 7, "merged salvage len should be 7, got %d", s.payload_len);
    CHECK(memcmp(s.payload, "\x76\x08\x41\x42\x43\x44\x45", 7) == 0, "merged salvage mismatch");
    CHECK(s.frags == 2, "salvage frags should be 2, got %u", s.frags);
}

// --- 4. Reap-before-feed frees table slots (the driver contract).
static void test_reap_frees_slots(void)
{
    printf("Test: full table of stale chains blocks new opener until reaped\n");
    ida_reassembler_t ctx;
    ida_reassembler_init(&ctx);
    ida_decoded_t f;
    uint8_t       out[IDA_REASM_MAX_BYTES];
    int           outlen = 0;
    uint8_t       o[]    = {0x76, 0x08, 0x01};

    // Fill every session slot with an opener at t=0.
    for (int i = 0; i < IDA_REASM_MAX_SESSIONS; i++) {
        make_frag(&f, 0, 1, o, sizeof(o));
        // Distinct freq per chain so they don't merge into each other.
        ida_reassembler_feed(&ctx, &f, false, FREQ + (uint32_t)i * 40000u, 1 * SEC, out, sizeof(out), &outlen);
    }
    CHECK(ctx.cnt_opened == IDA_REASM_MAX_SESSIONS, "should have opened %d chains, got %u",
          IDA_REASM_MAX_SESSIONS, ctx.cnt_opened);

    // Let them all go stale, then (WITHOUT reaping) feed a new opener: table
    // full -> overflow, because feed() no longer auto-expires.
    uint64_t later = 1 * SEC + IDA_REASM_SESSION_TIMEOUT_US + 1;
    make_frag(&f, 0, 1, o, sizeof(o));
    int rc = ida_reassembler_feed(&ctx, &f, false, FREQ + 900000u, later, out, sizeof(out), &outlen);
    CHECK(rc == -1, "new opener into full table should be dropped (-1), got %d", rc);
    CHECK(ctx.cnt_overflow == 1, "cnt_overflow should be 1, got %u", ctx.cnt_overflow);

    // Now drain via reap (salvaging each), which frees the slots...
    ida_salvage_t s;
    int           reaped = 0;
    while (ida_reassembler_reap(&ctx, later, &s))
        reaped++;
    CHECK(reaped == IDA_REASM_MAX_SESSIONS, "should reap %d stale chains, got %d",
          IDA_REASM_MAX_SESSIONS, reaped);

    // ...and the new opener now succeeds.
    make_frag(&f, 0, 1, o, sizeof(o));
    rc = ida_reassembler_feed(&ctx, &f, false, FREQ + 900000u, later + 1, out, sizeof(out), &outlen);
    CHECK(rc == 0, "opener after reap should open (0), got %d", rc);
}

// --- 5. A stale un-reaped session must not capture a late continuation.
static void test_stale_no_capture(void)
{
    printf("Test: continuation arriving after FRAG_GAP is an orphan, not merged\n");
    ida_reassembler_t ctx;
    ida_reassembler_init(&ctx);
    ida_decoded_t f;
    uint8_t       out[IDA_REASM_MAX_BYTES];
    int           outlen = 0;
    uint8_t       o[]    = {0x76, 0x08, 0x55};
    uint8_t       c1[]   = {0x66};
    make_frag(&f, 0, 1, o, sizeof(o));
    ida_reassembler_feed(&ctx, &f, false, FREQ, 1 * SEC, out, sizeof(out), &outlen);

    // Continuation arrives well after FRAG_GAP (but before session timeout).
    make_frag(&f, 1, 0, c1, sizeof(c1));
    int rc = ida_reassembler_feed(&ctx, &f, false, FREQ,
                                  1 * SEC + IDA_REASM_FRAG_GAP_US + 1, out, sizeof(out), &outlen);
    CHECK(rc == -1, "late continuation should be orphan (-1), got %d", rc);
    CHECK(ctx.cnt_orphan == 1, "cnt_orphan should be 1, got %u", ctx.cnt_orphan);
    CHECK(ctx.cnt_merged == 0, "nothing should have merged, cnt_merged=%u", ctx.cnt_merged);
}

// --- Task C: dirty-continuation admission via ida_reassembler_feed_ex().
// A continuation whose OWN CRC failed (frag_crc_ok=false) is still appended, but
// marks the chain dirty so a completed chain is emitted PARTIAL, never trusted.

// 6. Clean opener + a dirty terminal continuation completes and reports dirty.
static void test_dirty_terminal_completes_dirty(void)
{
    printf("Test: clean opener + dirty terminal -> completes, out_dirty=true\n");
    ida_reassembler_t ctx;
    ida_reassembler_init(&ctx);
    uint8_t       o[] = {0x76, 0x08, 0xAA, 0xBB};
    uint8_t       c[] = {0xCC, 0xDD};
    ida_decoded_t f;
    uint8_t       out[IDA_REASM_MAX_BYTES];
    int           outlen = 0;
    bool          dirty  = true; // must be cleared by the opener return

    make_frag(&f, 0, 1, o, sizeof(o));
    int rc = ida_reassembler_feed_ex(&ctx, &f, true, false, FREQ, 1 * SEC, out,
                                     sizeof(out), &outlen, &dirty);
    CHECK(rc == 0, "clean opener should open (0), got %d", rc);
    CHECK(dirty == false, "opener return should clear out_dirty");

    make_frag(&f, 1, 0, c, sizeof(c)); // terminal, but its own CRC failed
    rc = ida_reassembler_feed_ex(&ctx, &f, false, false, FREQ, 1 * SEC + 100000,
                                 out, sizeof(out), &outlen, &dirty);
    CHECK(rc == 1, "dirty terminal should complete (1), got %d", rc);
    CHECK(dirty == true, "completed chain with a dirty fragment must report dirty");
    CHECK(outlen == 6, "merged len should be 6, got %d", outlen);
    CHECK(memcmp(out, "\x76\x08\xAA\xBB\xCC\xDD", 6) == 0, "merged bytes mismatch");
    CHECK(ctx.cnt_completed == 1, "cnt_completed should be 1, got %u", ctx.cnt_completed);
}

// 7. A fully clean chain reports out_dirty=false (regression on the flag).
static void test_clean_chain_not_dirty(void)
{
    printf("Test: fully clean chain -> out_dirty=false\n");
    ida_reassembler_t ctx;
    ida_reassembler_init(&ctx);
    uint8_t       o[] = {0x76, 0x08, 0x01};
    uint8_t       c[] = {0x02};
    ida_decoded_t f;
    uint8_t       out[IDA_REASM_MAX_BYTES];
    int           outlen = 0;
    bool          dirty  = true;

    make_frag(&f, 0, 1, o, sizeof(o));
    ida_reassembler_feed_ex(&ctx, &f, true, false, FREQ, 1 * SEC, out, sizeof(out),
                            &outlen, &dirty);
    make_frag(&f, 1, 0, c, sizeof(c));
    int rc = ida_reassembler_feed_ex(&ctx, &f, true, false, FREQ, 1 * SEC + 100000,
                                     out, sizeof(out), &outlen, &dirty);
    CHECK(rc == 1, "clean terminal should complete (1), got %d", rc);
    CHECK(dirty == false, "fully clean chain must report out_dirty=false");
}

// 8. A dirty middle fragment rides through to a CLEAN terminal (dirty is sticky).
static void test_dirty_rides_through_clean_terminal(void)
{
    printf("Test: dirty middle + clean terminal -> out_dirty=true (sticky)\n");
    ida_reassembler_t ctx;
    ida_reassembler_init(&ctx);
    uint8_t       o[]  = {0x76, 0x08, 0x41};
    uint8_t       c1[] = {0x42, 0x43}; // dirty continuation, more to come
    uint8_t       c2[] = {0x44};       // clean terminal
    ida_decoded_t f;
    uint8_t       out[IDA_REASM_MAX_BYTES];
    int           outlen = 0;
    bool          dirty  = false;

    make_frag(&f, 0, 1, o, sizeof(o));
    ida_reassembler_feed_ex(&ctx, &f, true, false, FREQ, 1 * SEC, out, sizeof(out),
                            &outlen, &dirty);
    make_frag(&f, 1, 1, c1, sizeof(c1));
    int rc = ida_reassembler_feed_ex(&ctx, &f, false, false, FREQ, 1 * SEC + 100000,
                                     out, sizeof(out), &outlen, &dirty);
    CHECK(rc == 0, "dirty middle should stay open (0), got %d", rc);
    CHECK(dirty == false, "non-completing return should report out_dirty=false");
    make_frag(&f, 2, 0, c2, sizeof(c2));
    rc = ida_reassembler_feed_ex(&ctx, &f, true, false, FREQ, 1 * SEC + 200000, out,
                                 sizeof(out), &outlen, &dirty);
    CHECK(rc == 1, "clean terminal should complete (1), got %d", rc);
    CHECK(dirty == true, "dirty must ride through even when the terminal is clean");
    CHECK(outlen == 6, "merged len should be 6, got %d", outlen);
    CHECK(memcmp(out, "\x76\x08\x41\x42\x43\x44", 6) == 0, "merged bytes mismatch");
}

// 9. A dirty incomplete chain is reaped with its dirty flag set.
static void test_dirty_reap_flag(void)
{
    printf("Test: dirty incomplete chain -> reap carries dirty=true\n");
    ida_reassembler_t ctx;
    ida_reassembler_init(&ctx);
    uint8_t       o[]  = {0x76, 0x08, 0x55};
    uint8_t       c1[] = {0x66}; // dirty continuation, still more expected
    ida_decoded_t f;
    uint8_t       out[IDA_REASM_MAX_BYTES];
    int           outlen = 0;
    bool          dirty  = false;

    make_frag(&f, 0, 1, o, sizeof(o));
    ida_reassembler_feed_ex(&ctx, &f, true, false, FREQ, 5 * SEC, out, sizeof(out),
                            &outlen, &dirty);
    make_frag(&f, 1, 1, c1, sizeof(c1));
    ida_reassembler_feed_ex(&ctx, &f, false, false, FREQ, 5 * SEC + 100000, out,
                            sizeof(out), &outlen, &dirty);

    ida_salvage_t s;
    CHECK(ida_reassembler_reap(&ctx, 5 * SEC + 100000 + IDA_REASM_SESSION_TIMEOUT_US + 1, &s) == 1,
          "reap after timeout should return 1");
    CHECK(s.dirty == true, "reaped chain that had a dirty fragment must set s.dirty");
    CHECK(s.payload_len == 4, "merged salvage len should be 4, got %d", s.payload_len);
}

// 10. Standalone frame via _ex reports out_dirty=false.
static void test_standalone_not_dirty(void)
{
    printf("Test: standalone frame -> out_dirty=false\n");
    ida_reassembler_t ctx;
    ida_reassembler_init(&ctx);
    uint8_t       o[] = {0x76, 0x08, 0x99};
    ida_decoded_t f;
    uint8_t       out[IDA_REASM_MAX_BYTES];
    int           outlen = 0;
    bool          dirty  = true;

    make_frag(&f, 0, 0, o, sizeof(o)); // ctr=0, cont=0 -> standalone
    int rc = ida_reassembler_feed_ex(&ctx, &f, true, false, FREQ, 1 * SEC, out,
                                     sizeof(out), &outlen, &dirty);
    CHECK(rc == 1, "standalone should return 1, got %d", rc);
    CHECK(dirty == false, "standalone must report out_dirty=false");
    CHECK(ctx.cnt_standalone == 1, "cnt_standalone should be 1, got %u", ctx.cnt_standalone);
}

// 11. A dirty continuation with no open chain is a plain orphan (openers strict).
static void test_dirty_orphan(void)
{
    printf("Test: dirty continuation with no opener -> orphan (-1)\n");
    ida_reassembler_t ctx;
    ida_reassembler_init(&ctx);
    uint8_t       c1[] = {0x66};
    ida_decoded_t f;
    uint8_t       out[IDA_REASM_MAX_BYTES];
    int           outlen = 0;
    bool          dirty  = true;

    make_frag(&f, 1, 0, c1, sizeof(c1)); // continuation, ctr=1, no chain open
    int rc = ida_reassembler_feed_ex(&ctx, &f, false, false, FREQ, 1 * SEC, out,
                                     sizeof(out), &outlen, &dirty);
    CHECK(rc == -1, "orphan dirty continuation should return -1, got %d", rc);
    CHECK(dirty == false, "orphan return should report out_dirty=false");
    CHECK(ctx.cnt_orphan == 1, "cnt_orphan should be 1, got %u", ctx.cnt_orphan);
}

int main(void)
{
    test_normal_completion();
    test_salvage_first_fragment();
    test_salvage_two_fragments();
    test_reap_frees_slots();
    test_stale_no_capture();
    test_dirty_terminal_completes_dirty();
    test_clean_chain_not_dirty();
    test_dirty_rides_through_clean_terminal();
    test_dirty_reap_flag();
    test_standalone_not_dirty();
    test_dirty_orphan();
    printf("\n%d passed, %d failed\n", passed, failed);
    return failed ? 1 : 0;
}
