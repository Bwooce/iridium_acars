// Host regression for the signal_buffer ring index invariant (T2, commit
// 933ab38: "signal_buffer: preserve ring index invariant on wrap-path
// DMA drop"). signal_buffer.c is device-only (ESP-IDF/FreeRTOS deps) and
// doesn't build on the host, so this test exercises the pure head-advance
// arithmetic pulled out into signal_buffer_ring.h -- the same function
// signal_buffer_push() calls at its single head-update exit site -- via
// a model of the cumulative-index -> ring-offset mapping downstream code
// (signal_buffer_read_chunk, signal_buffer_burst_valid, and the worker's
// start_sample_idx % total_cap) relies on.
//
// The bug this guards against: an early `return;` on the wrap-path
// DMA-submit-failure branch skipped the head advance for that push while
// the producer's cumulative sample count kept moving -- permanently
// desyncing every future cumulative_index % total_cap computation.

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "signal_buffer_ring.h"

static int passed = 0, failed = 0;
#define CHECK(c, ...)                                    \
    do {                                                 \
        if (!(c)) {                                      \
            fprintf(stderr, "FAIL line %d: ", __LINE__); \
            fprintf(stderr, __VA_ARGS__);                \
            fprintf(stderr, "\n");                       \
            failed++;                                    \
        } else {                                         \
            passed++;                                    \
        }                                                \
    } while (0)

// (a) next_head always advances by aligned_count mod total_cap, for a
// spread of head/aligned_count/total_cap combinations including exact
// wrap boundaries.
static void test_next_head_arithmetic(void)
{
    const uint32_t total_cap = 1000000u; // stand-in for SIGNAL_BUF_SIZE/4

    struct {
        uint32_t head, aligned_count;
    } cases[] = {
        {0, 16},
        {16, 16},
        {total_cap - 16, 16}, // wraps to 0
        {total_cap - 8, 16},  // wraps to 8
        {500000, 999984},     // wraps most of the way around
        {0, 0},               // degenerate zero-length push
        {total_cap - 1, 1},   // wraps to exactly 0
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        uint32_t got  = signal_buffer_next_head(cases[i].head, cases[i].aligned_count, total_cap);
        uint32_t want = (uint32_t)(((uint64_t)cases[i].head + cases[i].aligned_count) % total_cap);
        CHECK(got == want, "case %zu: next_head(%u,%u,%u)=%u want %u",
              i, cases[i].head, cases[i].aligned_count, total_cap, got, want);
    }
}

// (b) Model a sequence of pushes -- some "failed" DMA submits that still
// recover and advance head per the fix -- and confirm the cumulative
// sample index -> ring offset mapping never gaps or overlaps: after
// every push, head must equal (total samples ever pushed) % total_cap.
// A "failed" push in this model still calls signal_buffer_next_head
// (mirroring signal_buffer_push's fixed single exit site), whether or
// not the data it wrote is a CPU-recovered copy or a garbage window --
// the index bookkeeping is identical in both cases.
static void test_sequence_mapping_consistent(void)
{
    const uint32_t total_cap  = 4096u;
    uint32_t       head       = 0;
    uint64_t       cumulative = 0;

    // Deterministic PRNG so the sequence is reproducible.
    uint32_t st = 0xC0FFEEu;
    for (int push = 0; push < 5000; push++) {
        st                                   = st * 1103515245u + 12345u;
        uint32_t aligned_count               = (st >> 8) % 4096u;      // 0..4095, spans multiple wraps over the run
        bool     simulated_dma_submit_failed = ((st >> 3) & 0x7) == 0; // ~1/8 of pushes "fail"

        // Fixed behaviour: head advances by aligned_count on EVERY push,
        // success or (recovered) failure alike.
        head = signal_buffer_next_head(head, aligned_count, total_cap);
        cumulative += aligned_count;

        uint32_t expected = (uint32_t)(cumulative % total_cap);
        CHECK(head == expected,
              "push %d (dma_fail=%d): head=%u expected=%u (cumulative=%llu) -- "
              "mapping desynced",
              push, (int)simulated_dma_submit_failed, head, expected,
              (unsigned long long)cumulative);
    }
}

// (c) TEETH: prove the OLD buggy behaviour (early return before the head
// update on a wrap-path DMA-submit failure) would break the mapping --
// i.e. the buggy computation diverges from the correct one as soon as a
// single failure is skipped.
static void test_old_bug_would_diverge(void)
{
    const uint32_t total_cap  = 4096u;
    uint32_t       fixed_head = 0, buggy_head = 0;
    uint64_t       cumulative      = 0;
    int            divergence_seen = 0;

    uint32_t st = 0xBADC0DEu;
    for (int push = 0; push < 2000; push++) {
        st                                      = st * 1103515245u + 12345u;
        uint32_t aligned_count                  = (st >> 8) % 512u + 1;   // 1..512, never zero-length
        bool     dma_submit_failed_on_wrap_path = ((st >> 5) & 0xF) == 0; // ~1/16

        cumulative += aligned_count;

        // Fixed behaviour: always advance (single exit site calls the
        // helper unconditionally, per commit 933ab38).
        fixed_head = signal_buffer_next_head(fixed_head, aligned_count, total_cap);

        // OLD buggy behaviour being modelled here: on a wrap-path
        // DMA-submit failure, the pre-fix code took an early `return;`
        // BEFORE reaching the head-update line, so buggy_head is simply
        // left unchanged for that push.
        if (!dma_submit_failed_on_wrap_path) {
            buggy_head = signal_buffer_next_head(buggy_head, aligned_count, total_cap);
        }
        // else: bug reproduced -- head advance skipped for this push.

        uint32_t expected = (uint32_t)(cumulative % total_cap);
        if (buggy_head != expected) {
            divergence_seen = 1;
        }
        // The fixed model must never desync regardless of the injected
        // failures.
        CHECK(fixed_head == expected, "push %d: fixed_head=%u expected=%u",
              push, fixed_head, expected);
    }

    CHECK(divergence_seen,
          "expected the old (skip-advance-on-failure) model to diverge from "
          "the cumulative-index mapping at least once over %d pushes, but it "
          "never did -- this test would not have caught the T2 bug",
          2000);
    CHECK(fixed_head != buggy_head || !divergence_seen,
          "sanity: if a divergence occurred, fixed and buggy heads must differ "
          "by the end of the run (fixed=%u buggy=%u)",
          fixed_head, buggy_head);
}

int main(void)
{
    test_next_head_arithmetic();
    test_sequence_mapping_consistent();
    test_old_bug_would_diverge();

    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed ? 1 : 0;
}
