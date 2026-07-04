// Host regression for the SPSC USB-ingest ring's pure index/wrap
// arithmetic (T49a, docs/perf-decoupling-design-2026-07-04.md §T49a).
// This is chain A step 1: the test + header ARE the contract that the
// later buffer-backed implementation (usbring.c, wired into
// esp_libusb.c / class_driver.c / ingest_core1.c in place of
// dev->ringbuf) must satisfy. This test does NOT touch those files.
//
// usbring_ring.h isolates the pure head/tail/capacity arithmetic from
// memcpy, PSRAM placement, and the __sync_synchronize() publish barrier
// the real implementation needs (mirrors how signal_buffer_ring.h /
// test_signal_buffer_index.c split signal_buffer.c's pure ring-index
// math from its DMA and FreeRTOS concerns). A small buffer-backed
// wrapper (usbring_host_t below) is host-test-only glue: it exists so
// the randomised interleaving model can verify actual byte content, not
// just index numbers.
//
// Coverage:
//   (a) direct index-arithmetic edge cases: pow2 precondition,
//       full/empty distinguishability, exact-remaining-space write,
//       peek-contiguous-stops-at-wrap swept over many head/tail combos.
//   (b) pure sequence model: cumulative written/consumed byte counts
//       stay in lockstep with used(head,tail) over many pushes/pops.
//   (c) randomised producer/consumer interleaving against an
//       independent reference byte generator (a pure function of
//       absolute stream position -- NOT derived from the ring itself),
//       seeded xorshift32, thousands of iterations, checking FIFO
//       ordering, peek-before-consume idempotency, and wrap handling.
//   (d) in-suite anti-toothless guards: three deliberately broken index
//       variants (peek ignoring the wrap clamp, consume under-advancing
//       tail, write bypassing the free-space check) MUST be proven to
//       diverge from the correct model, run automatically every ctest
//       invocation (mirrors test_signal_buffer_index.c's
//       test_old_bug_would_diverge).
//   (e) --demo-fail: routes the SAME content-level interleaving harness
//       used for the positive control through the broken peek variant,
//       producing real byte-mismatch CHECK failures and a nonzero exit
//       (mirrors test_window_multiply_golden.c's --demo-fail red path).
//
// No wall-clock, no libc rand() -- xorshift32 with fixed seeds only.

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "usbring_ring.h"

static int s_passed = 0, s_failed = 0;
#define CHECK(cond, ...)                                   \
    do {                                                   \
        if (!(cond)) {                                     \
            fprintf(stderr, "  FAIL line %d: ", __LINE__); \
            fprintf(stderr, __VA_ARGS__);                  \
            fprintf(stderr, "\n");                         \
            s_failed++;                                    \
        } else {                                           \
            s_passed++;                                    \
        }                                                  \
    } while (0)

// Deterministic PRNG -- xorshift32, fixed seeds only (no time(), no rand()).
static uint32_t xs32(uint32_t *s)
{
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return x;
}

// Independent reference byte generator: a pure function of the absolute
// byte position in the (conceptually infinite) producer stream. This is
// deliberately NOT derived from any ring state -- it's the "obviously
// correct" ground truth the ring's output is checked against (avoids a
// circular golden: the ring never gets to define what its own output
// should have been).
static uint8_t gen_byte(uint64_t pos)
{
    uint64_t h = pos * 2654435761ULL + 0x9E3779B97F4A7C15ULL;
    h ^= h >> 33;
    h *= 0xff51afd7ed558ccdULL;
    h ^= h >> 33;
    return (uint8_t)h;
}

// ---------------------------------------------------------------------
// (a) direct index-arithmetic edge cases
// ---------------------------------------------------------------------

static void test_pow2_precondition(void)
{
    struct {
        uint32_t cap;
        bool     want;
    } cases[] = {
        {0, false},
        {1, true},
        {2, true},
        {3, false},
        {4, true},
        {1023, false},
        {1024, true},
        {1u << 20, true},
        {0xFFFFFFFFu, false},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        bool got = usbring_is_pow2_capacity(cases[i].cap);
        CHECK(got == cases[i].want, "is_pow2(%u) = %d want %d",
              cases[i].cap, (int)got, (int)cases[i].want);
    }
}

// Full vs empty must be unambiguously distinguishable (the whole point
// of free-running head/tail counters -- see usbring_ring.h's header
// comment on the convention).
static void test_full_empty_distinguishable(void)
{
    const uint32_t capacity = 1024;

    uint32_t head = 5000, tail = 5000; // empty: head == tail
    CHECK(usbring_used(head, tail) == 0, "empty: used should be 0");
    CHECK(usbring_free_space(head, tail, capacity) == capacity,
          "empty: free_space should equal capacity");

    head = 5000 + capacity;
    tail = 5000; // full: used == capacity exactly
    CHECK(usbring_used(head, tail) == capacity, "full: used should equal capacity");
    CHECK(usbring_free_space(head, tail, capacity) == 0,
          "full: free_space should be 0");
    CHECK(head != tail, "full state must NOT collapse to head==tail "
                        "(that would be indistinguishable from empty)");
}

// Write into exactly the remaining free space must be representable:
// usbring_write_contig() plus a wrap-around second chunk must sum to
// exactly free_space when free_space == n (no data left un-writable).
static void test_write_exactly_remaining_space(void)
{
    struct {
        uint32_t head, tail, capacity;
    } cases[] = {
        {0, 0, 16},          // empty ring, full capacity available
        {11, 0, 16},         // free=5, all before wrap
        {12, 0, 16},         // free=4, off=12, to_wrap=4 (exact edge)
        {4096 - 3, 0, 4096}, // free=3, straddling would-be wrap on write side
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        uint32_t head = cases[i].head, tail = cases[i].tail, cap = cases[i].capacity;
        uint32_t free_bytes = usbring_free_space(head, tail, cap);
        uint32_t first      = usbring_write_contig(head, tail, cap);
        uint32_t second     = free_bytes - first; // remainder after wrap
        CHECK(first <= free_bytes, "case %zu: write_contig %u exceeds free_space %u",
              i, first, free_bytes);
        uint32_t off = usbring_phys_offset(head, cap);
        CHECK(off + first == cap || free_bytes == first,
              "case %zu: first chunk (%u) should reach the wrap boundary "
              "(off=%u cap=%u) unless free_space itself is smaller",
              i, first, off, cap);
        // Writing exactly free_bytes must be split into first+second with
        // second landing at offset 0 and never exceeding capacity total.
        CHECK(first + second == free_bytes,
              "case %zu: two-part split doesn't sum to free_space", i);
    }
}

// Sweep many (head, tail, capacity) combinations and confirm
// usbring_peek_contig() NEVER reports a span that would read past the
// physical wrap boundary, and reports the exact clamp (not early, not
// late).
static void test_peek_wrap_boundary_sweep(void)
{
    const uint32_t capacities[] = {16, 64, 256, 4096, 65536};
    uint32_t       st           = 0x51EEC0DEu;

    for (size_t ci = 0; ci < sizeof(capacities) / sizeof(capacities[0]); ci++) {
        uint32_t cap = capacities[ci];
        for (int trial = 0; trial < 5000; trial++) {
            uint32_t tail = xs32(&st);
            uint32_t used = xs32(&st) % (cap + 1); // 0..cap inclusive
            uint32_t head = tail + used;

            uint32_t n_contig = 0xFFFFFFFFu;
            uint32_t off      = usbring_peek_contig(head, tail, cap, &n_contig);
            uint32_t to_wrap  = cap - usbring_phys_offset(tail, cap);

            CHECK(off == usbring_phys_offset(tail, cap),
                  "peek offset should be tail's physical offset");
            CHECK(off + n_contig <= cap,
                  "cap=%u tail=%u used=%u: peek reported off=%u n_contig=%u "
                  "(off+n_contig=%u) -- reads PAST the physical array bound",
                  cap, tail, used, off, n_contig, off + n_contig);
            uint32_t want = used < to_wrap ? used : to_wrap;
            CHECK(n_contig == want,
                  "cap=%u tail=%u used=%u: n_contig=%u want=%u (to_wrap=%u) "
                  "-- clamp is early or late, not exact",
                  cap, tail, used, n_contig, want, to_wrap);
        }
    }
}

// ---------------------------------------------------------------------
// (b) pure sequence model -- cumulative written/consumed byte counts
// stay in lockstep with used(head, tail), mirroring
// test_signal_buffer_index.c's test_sequence_mapping_consistent.
// ---------------------------------------------------------------------

static void test_sequence_used_consistent(void)
{
    const uint32_t capacity = 8192;
    uint32_t       head = 0, tail = 0;
    uint64_t       total_written = 0, total_consumed = 0;
    uint32_t       st = 0xC0FFEEu;

    for (int op = 0; op < 20000; op++) {
        st            = st * 1103515245u + 12345u;
        bool do_write = (st & 1) != 0;

        if (do_write) {
            uint32_t free_bytes = usbring_free_space(head, tail, capacity);
            uint32_t want       = (st >> 8) % (capacity / 2 + 1);
            uint32_t n          = want < free_bytes ? want : free_bytes;
            head                = usbring_next_head(head, n);
            total_written += n;
        } else {
            uint32_t used = usbring_used(head, tail);
            uint32_t want = (st >> 8) % (capacity / 2 + 1);
            uint32_t n    = want < used ? want : used;
            tail          = usbring_next_tail(tail, n);
            total_consumed += n;
        }

        uint32_t expected_used = (uint32_t)(total_written - total_consumed);
        CHECK(usbring_used(head, tail) == expected_used,
              "op %d: used(head,tail)=%u expected=%llu (written=%llu consumed=%llu)",
              op, usbring_used(head, tail), (unsigned long long)expected_used,
              (unsigned long long)total_written, (unsigned long long)total_consumed);
        CHECK(usbring_used(head, tail) <= capacity,
              "op %d: used exceeded capacity -- overrun", op);
    }
}

// ---------------------------------------------------------------------
// Host-only buffer-backed wrapper (NOT part of the pure-index header --
// this glue is test-only so the interleaving model can check real byte
// content, per the task's "keep it in the test or clearly host-only"
// guidance).
// ---------------------------------------------------------------------

typedef uint32_t (*peek_contig_fn)(uint32_t head, uint32_t tail, uint32_t capacity,
                                   uint32_t *n_contig_out);
typedef uint32_t (*next_tail_fn)(uint32_t tail, uint32_t n);
typedef uint32_t (*free_space_fn)(uint32_t head, uint32_t tail, uint32_t capacity);

typedef struct {
    uint8_t *buf;      // backing store; storage_len >= capacity (may
                       // include a trailing guard region, see below)
    uint32_t capacity; // power of two
    uint32_t head;
    uint32_t tail;
} usbring_host_t;

static void usbring_host_init(usbring_host_t *r, uint8_t *buf, uint32_t capacity)
{
    r->buf      = buf;
    r->capacity = capacity;
    r->head     = 0;
    r->tail     = 0;
}

// Producer path: check free space (via the given free_space_fn, so the
// negative controls can inject a broken one), split the memcpy at the
// wrap boundary using the ALWAYS-correct usbring_write_contig (write-
// side splitting isn't part of the bug classes under test here -- only
// the space check and the head advance are parameterised), and advance
// head. Returns false if n exceeds the (possibly broken) free-space
// computation's answer -- exactly the class of bug where a broken
// free_space always reports "plenty of room" and the producer proceeds
// to overwrite unconsumed data.
static bool usbring_host_write(usbring_host_t *r, free_space_fn free_fn,
                               const uint8_t *src, uint32_t n)
{
    uint32_t free_bytes = free_fn(r->head, r->tail, r->capacity);
    if (n > free_bytes) {
        return false;
    }
    uint32_t first = usbring_write_contig(r->head, r->tail, r->capacity);
    if (first > n) first = n;
    uint32_t off = usbring_phys_offset(r->head, r->capacity);
    memcpy(r->buf + off, src, first);
    if (n > first) {
        memcpy(r->buf, src + first, n - first);
    }
    r->head = usbring_next_head(r->head, n);
    return true;
}

static const uint8_t *usbring_host_peek(usbring_host_t *r, peek_contig_fn peek_fn,
                                        uint32_t *n_contig_out)
{
    uint32_t off = peek_fn(r->head, r->tail, r->capacity, n_contig_out);
    return r->buf + off;
}

static void usbring_host_consume(usbring_host_t *r, next_tail_fn tail_fn, uint32_t n)
{
    r->tail = tail_fn(r->tail, n);
}

// First mismatching index against the independent reference generator,
// or -1 if all n bytes matched (mirrors test_window_multiply_golden.c's
// diff_report -- report the first divergence, not every byte, so a
// broken run's stderr stays readable).
static int content_mismatch(const uint8_t *p, uint32_t n, uint64_t base_pos)
{
    for (uint32_t i = 0; i < n; i++) {
        if (p[i] != gen_byte(base_pos + i)) {
            return (int)i;
        }
    }
    return -1;
}

// ---------------------------------------------------------------------
// (c) randomised producer/consumer interleaving model, content-checked
// against the independent gen_byte() reference.
// ---------------------------------------------------------------------

// storage is sized capacity * STORAGE_GUARD_MULT: the extra region past
// `capacity` is filled once with a sentinel and never legitimately
// written. It exists so a BROKEN peek (one that ignores the wrap clamp)
// reads bounded, deterministic-but-wrong bytes instead of undefined
// memory -- the content check then fails loudly (sentinel != expected)
// instead of crashing. The correct peek never reads into this region
// (proven by test_peek_wrap_boundary_sweep above).
#define STORAGE_GUARD_MULT 2
#define SENTINEL 0xEEu

// Returns the number of NEW check failures produced during this run
// (0 == everything matched the reference model).
static int run_interleaving(uint32_t capacity, uint64_t iterations, uint32_t seed,
                            peek_contig_fn peek_fn, free_space_fn free_fn,
                            next_tail_fn tail_fn, const char *label)
{
    int before = s_failed;

    uint8_t *storage = malloc((size_t)capacity * STORAGE_GUARD_MULT);
    uint8_t *scratch = malloc(capacity);
    if (!storage || !scratch) {
        fprintf(stderr, "OOM in run_interleaving\n");
        exit(2);
    }
    memset(storage, SENTINEL, (size_t)capacity * STORAGE_GUARD_MULT);

    usbring_host_t ring;
    usbring_host_init(&ring, storage, capacity);

    uint64_t total_written = 0, total_consumed = 0;
    uint32_t st = seed;

    for (uint64_t it = 0; it < iterations; it++) {
        // --- write phase ---
        uint32_t free_bytes = usbring_free_space(ring.head, ring.tail, capacity);
        uint32_t want       = xs32(&st) % (capacity / 4 + 1);
        uint32_t n          = want < free_bytes ? want : free_bytes;
        if (n > 0) {
            for (uint32_t i = 0; i < n; i++) {
                scratch[i] = gen_byte(total_written + i);
            }
            bool ok = usbring_host_write(&ring, free_fn, scratch, n);
            CHECK(ok, "%s it=%llu: write of %u bytes (free_space said %u) was rejected",
                  label, (unsigned long long)it, n, free_bytes);
            total_written += n;
        }

        // --- peek phase (idempotency: two peeks with no consume in
        // between must return identical offset/length/content) ---
        uint32_t used = usbring_used(ring.head, ring.tail);
        if (used == 0) continue;

        uint32_t       n1, n2;
        const uint8_t *p1 = usbring_host_peek(&ring, peek_fn, &n1);
        const uint8_t *p2 = usbring_host_peek(&ring, peek_fn, &n2);
        CHECK(p1 == p2 && n1 == n2,
              "%s it=%llu: peek not idempotent without consume (p1=%p n1=%u, "
              "p2=%p n2=%u)",
              label, (unsigned long long)it, (const void *)p1, n1, (const void *)p2, n2);

        int mismatch = content_mismatch(p1, n1, total_consumed);
        CHECK(mismatch < 0,
              "%s it=%llu: FIFO content mismatch at byte offset %d within "
              "peeked span of %u (total_consumed=%llu) -- expected=%u got=%u",
              label, (unsigned long long)it, mismatch, n1,
              (unsigned long long)total_consumed,
              mismatch >= 0 ? gen_byte(total_consumed + (uint32_t)mismatch) : 0,
              mismatch >= 0 ? p1[mismatch] : 0);

        // --- consume phase: never more than what was just peeked, per
        // the documented usage pattern (peek, act on n_contig, consume
        // n_contig or less) ---
        uint32_t consume_n = (n1 == 0) ? 0 : (xs32(&st) % (n1 + 1));
        if (consume_n > 0) {
            usbring_host_consume(&ring, tail_fn, consume_n);
            total_consumed += consume_n;
        }
    }

    // Final drain: walk any remaining unconsumed bytes to exercise the
    // tail end (including a trailing wrap) and confirm content still
    // matches all the way to the last byte ever written.
    for (;;) {
        uint32_t used = usbring_used(ring.head, ring.tail);
        if (used == 0) break;
        uint32_t       n_contig;
        const uint8_t *p = usbring_host_peek(&ring, peek_fn, &n_contig);
        if (n_contig == 0) {
            CHECK(false, "%s: drain stalled with used=%u but n_contig=0", label, used);
            break;
        }
        int mismatch = content_mismatch(p, n_contig, total_consumed);
        CHECK(mismatch < 0,
              "%s: drain content mismatch at byte %d within span %u "
              "(total_consumed=%llu)",
              label, mismatch, n_contig, (unsigned long long)total_consumed);
        usbring_host_consume(&ring, tail_fn, n_contig);
        total_consumed += n_contig;
    }
    CHECK(total_consumed == total_written,
          "%s: drain finished with total_consumed=%llu != total_written=%llu",
          label, (unsigned long long)total_consumed, (unsigned long long)total_written);

    free(storage);
    free(scratch);
    return s_failed - before;
}

// ---------------------------------------------------------------------
// Broken index variants -- deliberately wrong, used ONLY by the
// anti-toothless guards and --demo-fail below. Never used by production
// code; they exist to prove the test suite would catch them.
// ---------------------------------------------------------------------

// Bug class 1 (from the design doc's suggested list): "peek that
// returns contiguous length past the wrap boundary" -- forgets to clamp
// to the physical end of the array.
static uint32_t peek_contig_BROKEN_no_wrap_clamp(uint32_t head, uint32_t tail,
                                                 uint32_t capacity, uint32_t *n_contig_out)
{
    uint32_t used = usbring_used(head, tail);
    uint32_t off  = usbring_phys_offset(tail, capacity);
    if (n_contig_out) *n_contig_out = used; // BUG: no min() against (capacity - off)
    return off;
}

// Bug class 2: "consume that under-advances" -- silently drops one byte
// of every nonzero advance, so the consumer's application-level
// bookkeeping (total_consumed) races ahead of the ring's own tail.
static uint32_t next_tail_BROKEN_under_advance(uint32_t tail, uint32_t n)
{
    return tail + (n > 0 ? n - 1 : 0); // BUG: should be tail + n
}

// Bug class 3: "write that overwrites unconsumed bytes" -- free-space
// check always reports the full capacity available, ignoring the
// consumer's tail entirely.
static uint32_t free_space_BROKEN_ignores_tail(uint32_t head, uint32_t tail,
                                               uint32_t capacity)
{
    (void)head;
    (void)tail;
    return capacity; // BUG: should be capacity - usbring_used(head, tail)
}

// ---------------------------------------------------------------------
// (d) in-suite anti-toothless guards -- prove each broken variant above
// is caught by an independent check, automatically on every ctest run
// (mirrors test_signal_buffer_index.c's test_old_bug_would_diverge).
// ---------------------------------------------------------------------

static void test_broken_peek_would_overrun(void)
{
    uint32_t st              = 0xB0B0FACEu;
    int      divergence_seen = 0;
    for (int trial = 0; trial < 20000; trial++) {
        uint32_t cap  = 16u << (xs32(&st) % 8); // 16..2048, all powers of two
        uint32_t tail = xs32(&st);
        uint32_t used = xs32(&st) % (cap + 1);
        uint32_t head = tail + used;

        uint32_t n_correct, n_broken;
        uint32_t off_correct = usbring_peek_contig(head, tail, cap, &n_correct);
        uint32_t off_broken  = peek_contig_BROKEN_no_wrap_clamp(head, tail, cap, &n_broken);

        CHECK(off_correct + n_correct <= cap,
              "correct peek must never exceed the physical array (off=%u n=%u cap=%u)",
              off_correct, n_correct, cap);
        if (off_broken + n_broken > cap) {
            divergence_seen = 1;
        }
    }
    CHECK(divergence_seen,
          "expected the broken no-wrap-clamp peek to report a span past the "
          "physical array boundary at least once over 20000 trials, but it "
          "never did -- this guard would not have caught the bug");
}

// Steady-state produce/consume cycle: correct arithmetic keeps used()
// at 0 between cycles; the broken under-advance leaks 1 byte of "stuck"
// occupancy per cycle until the ring falsely reports itself full.
static void test_broken_tail_would_stall(void)
{
    const uint32_t capacity  = 100;
    uint32_t       good_head = 0, good_tail = 0;
    uint32_t       bad_head = 0, bad_tail = 0;
    int            saw_false_full = 0;

    for (uint32_t cycle = 0; cycle < capacity + 10; cycle++) {
        const uint32_t chunk = 10;
        good_head            = usbring_next_head(good_head, chunk);
        bad_head             = usbring_next_head(bad_head, chunk);
        good_tail            = usbring_next_tail(good_tail, chunk);
        bad_tail             = next_tail_BROKEN_under_advance(bad_tail, chunk);

        CHECK(usbring_used(good_head, good_tail) == 0,
              "cycle %u: correct model should return to used=0 after matched "
              "produce/consume (produce==consume==%u every cycle)",
              cycle, chunk);

        if (usbring_free_space(bad_head, bad_tail, capacity) == 0) {
            saw_false_full = 1;
        }
    }
    CHECK(saw_false_full,
          "expected the broken under-advancing consume to eventually make the "
          "ring falsely report itself full despite produce==consume every "
          "cycle, but it never did -- this guard would not have caught the bug");
}

// Correct free_space, by construction, never lets used(head,tail)
// exceed capacity across any sequence of writes. The broken variant
// (ignores tail, always says "capacity free") lets a producer write
// straight over data the consumer hasn't taken yet.
static void test_broken_write_would_overrun_capacity(void)
{
    const uint32_t capacity  = 4096;
    uint32_t       good_head = 0, good_tail = 0;
    uint32_t       bad_head = 0, bad_tail = 0;
    int            saw_overrun = 0;
    uint32_t       st          = 0x0FF1CE00u;

    for (int op = 0; op < 5000; op++) {
        st                = st * 1103515245u + 12345u;
        bool     do_write = (st & 1) != 0;
        uint32_t amt      = (st >> 8) % (capacity / 2 + 1);

        if (do_write) {
            uint32_t good_free = usbring_free_space(good_head, good_tail, capacity);
            uint32_t good_n    = amt < good_free ? amt : good_free;
            good_head          = usbring_next_head(good_head, good_n);

            uint32_t bad_free = free_space_BROKEN_ignores_tail(bad_head, bad_tail, capacity);
            uint32_t bad_n    = amt < bad_free ? amt : bad_free; // bad_free == capacity always
            bad_head          = usbring_next_head(bad_head, bad_n);
        } else {
            uint32_t good_used = usbring_used(good_head, good_tail);
            uint32_t good_n    = amt < good_used ? amt : good_used;
            good_tail          = usbring_next_tail(good_tail, good_n);

            uint32_t bad_used = usbring_used(bad_head, bad_tail);
            uint32_t bad_n    = amt < bad_used ? amt : bad_used;
            bad_tail          = usbring_next_tail(bad_tail, bad_n);
        }

        CHECK(usbring_used(good_head, good_tail) <= capacity,
              "op %d: correct model exceeded capacity -- should be impossible", op);
        if (usbring_used(bad_head, bad_tail) > capacity) {
            saw_overrun = 1;
        }
    }
    CHECK(saw_overrun,
          "expected the broken tail-ignoring free_space to eventually let "
          "used(head,tail) exceed capacity (an overwrite of unconsumed data), "
          "but it never did -- this guard would not have caught the bug");
}

int main(int argc, char **argv)
{
    bool demo_fail = (argc > 1 && strcmp(argv[1], "--demo-fail") == 0);

    if (demo_fail) {
        printf("--demo-fail: routing the content-level interleaving harness "
               "through peek_contig_BROKEN_no_wrap_clamp (MUST go red)\n");
        int nf = run_interleaving(64, 20000, 0xD00D0001u,
                                  peek_contig_BROKEN_no_wrap_clamp,
                                  usbring_free_space, usbring_next_tail,
                                  "demo_fail[cap=64]");
        nf += run_interleaving(256, 20000, 0xD00D0002u,
                               peek_contig_BROKEN_no_wrap_clamp,
                               usbring_free_space, usbring_next_tail,
                               "demo_fail[cap=256]");
        printf("--demo-fail: %d divergence(s) detected (expected > 0)\n", nf);
        printf("usbring_index demo-fail: %d passed, %d failed\n", s_passed, s_failed);
        // Exit non-zero regardless: this mode exists to demonstrate the
        // red path, never to report success.
        return (nf > 0) ? 1 : 2; // 2 = the demo itself is broken (toothless)
    }

    test_pow2_precondition();
    test_full_empty_distinguishable();
    test_write_exactly_remaining_space();
    test_peek_wrap_boundary_sweep();
    test_sequence_used_consistent();

    // Positive control: correct index core through the full content-level
    // interleaving harness, at several capacities/seeds (small capacity
    // forces frequent wraps; larger capacity exercises scale).
    run_interleaving(64, 20000, 0xC0FFEE01u,
                     usbring_peek_contig, usbring_free_space, usbring_next_tail,
                     "positive[cap=64]");
    run_interleaving(256, 20000, 0xC0FFEE02u,
                     usbring_peek_contig, usbring_free_space, usbring_next_tail,
                     "positive[cap=256]");
    run_interleaving(65536, 4000, 0xC0FFEE03u,
                     usbring_peek_contig, usbring_free_space, usbring_next_tail,
                     "positive[cap=65536]");

    // Negative controls: three broken variants, each proven to diverge
    // from the correct model automatically -- if any of these ever stop
    // detecting the injected bug, the guard itself has gone toothless.
    test_broken_peek_would_overrun();
    test_broken_tail_would_stall();
    test_broken_write_would_overrun_capacity();

    printf("\n=== usbring_index: %d passed, %d failed ===\n", s_passed, s_failed);
    return s_failed ? 1 : 0;
}
