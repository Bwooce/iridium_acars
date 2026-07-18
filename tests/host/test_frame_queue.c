// Tests for frame_queue (the SPSC ringbuffer between worker_core1 and
// the frame_decoder task).
//
// Three layers of test:
//   1. Empty/full single-thread mechanics — push to capacity, pop to
//      empty, verify counters, verify wraparound.
//   2. Drop counter — overflow the queue, confirm dropped count.
//   3. Multi-thread pthread producer/consumer — 1 producer + 1 consumer
//      moving 100k items, verify all delivered in order with no
//      duplicates / drops.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include "frame_queue.h"

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

static void make_item(frame_queue_item_t *it, uint32_t id)
{
    memset(it, 0, sizeof(*it));
    it->timestamp_us = id;
    it->freq_hz      = 1620000000u + id;
    it->n_bits       = 382;
    it->direction    = 0;
    // Encode `id` into the bit stream so the consumer can verify
    // ordering. First 32 bits are the LSB-first id.
    for (int b = 0; b < 32; b++)
        it->bits[b] = (uint8_t)((id >> b) & 1);
}

static uint32_t recover_id(const frame_queue_item_t *it)
{
    uint32_t id = 0;
    for (int b = 0; b < 32; b++)
        id |= ((uint32_t)(it->bits[b] & 1)) << b;
    return id;
}

// --- 1. Single-thread: push to capacity, pop to empty.
static void test_basic_push_pop(void)
{
    printf("Test: push/pop in a single thread\n");
    frame_queue_t *q = frame_queue_create(8); // capacity = 7
    CHECK(q != NULL, "create");
    CHECK(frame_queue_capacity(q) == 7, "capacity=%zu", frame_queue_capacity(q));
    CHECK(frame_queue_count(q) == 0, "count after create");

    frame_queue_item_t it;
    for (uint32_t i = 0; i < 7; i++) {
        make_item(&it, 1000 + i);
        CHECK(frame_queue_push(q, &it), "push %u", i);
    }
    CHECK(frame_queue_count(q) == 7, "full count");

    // 8th push should fail (queue full).
    make_item(&it, 9999);
    CHECK(!frame_queue_push(q, &it), "push beyond capacity should fail");
    CHECK(frame_queue_dropped(q) == 1, "dropped=%llu", (unsigned long long)frame_queue_dropped(q));

    // Pop them all back in FIFO order.
    for (uint32_t i = 0; i < 7; i++) {
        frame_queue_item_t out;
        CHECK(frame_queue_pop(q, &out), "pop %u", i);
        uint32_t id = recover_id(&out);
        CHECK(id == 1000 + i, "FIFO order: got id=%u expected %u", id, 1000 + i);
    }
    CHECK(frame_queue_count(q) == 0, "empty count");
    CHECK(!frame_queue_pop(q, &it), "pop on empty fails");

    frame_queue_destroy(q);
}

// --- 2. Wraparound across head/tail.
static void test_wraparound(void)
{
    printf("Test: wraparound across the ringbuffer end\n");
    frame_queue_t *q = frame_queue_create(4); // capacity = 3

    // Push 3, pop 2, push 3 more, pop 4. Tail wraps.
    frame_queue_item_t it, out;
    for (uint32_t i = 0; i < 3; i++) {
        make_item(&it, 100 + i);
        CHECK(frame_queue_push(q, &it), "first push %u", i);
    }
    for (uint32_t i = 0; i < 2; i++) {
        CHECK(frame_queue_pop(q, &out), "first pop %u", i);
        CHECK(recover_id(&out) == 100 + i, "first pop id");
    }
    for (uint32_t i = 0; i < 2; i++) {
        make_item(&it, 200 + i);
        CHECK(frame_queue_push(q, &it), "second push %u", i);
    }
    // Order should be: 102, 200, 201 still in the queue.
    uint32_t expected[3] = {102, 200, 201};
    for (uint32_t i = 0; i < 3; i++) {
        CHECK(frame_queue_pop(q, &out), "second pop %u", i);
        CHECK(recover_id(&out) == expected[i], "second pop id mismatch");
    }
    frame_queue_destroy(q);
}

// --- 3. Validation: invalid n_slots.
static void test_invalid_create(void)
{
    printf("Test: invalid n_slots returns NULL\n");
    CHECK(frame_queue_create(0) == NULL, "0 slots");
    CHECK(frame_queue_create(1) == NULL, "1 slot");
    CHECK(frame_queue_create(3) == NULL, "non-power-of-two");
    CHECK(frame_queue_create(7) == NULL, "non-power-of-two");
}

// --- 3b. Soft-metric round-trip (Chase-2, task #16) + the zero-copy
// reserve/commit producer path frame_decoder_push now uses.
static void test_soft_and_reserve(void)
{
    printf("Test: soft[] round-trip + producer reserve/commit\n");
    frame_queue_t *q = frame_queue_create(4);
    CHECK(q != NULL, "create");

    // Copy-push with soft metrics.
    frame_queue_item_t it;
    make_item(&it, 42);
    it.n_soft = 382;
    for (int i = 0; i < 382; i++)
        it.soft[i] = (int16_t)(i - 191); // signs + magnitudes both exercised
    CHECK(frame_queue_push(q, &it), "push with soft");

    frame_queue_item_t out;
    memset(&out, 0xAA, sizeof(out)); // poison: stale bytes must not matter
    CHECK(frame_queue_pop(q, &out), "pop with soft");
    CHECK(recover_id(&out) == 42, "id round-trip");
    CHECK(out.n_soft == 382, "n_soft round-trip (%u)", out.n_soft);
    int soft_ok = 1;
    for (int i = 0; i < 382; i++)
        if (out.soft[i] != (int16_t)(i - 191)) soft_ok = 0;
    CHECK(soft_ok, "soft payload round-trip");

    // Clamp: bogus n_soft must not read/write past the array.
    make_item(&it, 43);
    it.n_soft = 0xFFFF;
    CHECK(frame_queue_push(q, &it), "push with bogus n_soft");
    CHECK(frame_queue_pop(q, &out), "pop bogus n_soft");
    CHECK(out.n_soft <= FRAME_QUEUE_MAX_SOFT, "n_soft clamped (%u)", out.n_soft);

    // Zero-copy producer path: reserve, fill in place, commit.
    frame_queue_item_t *slot = frame_queue_producer_reserve(q);
    CHECK(slot != NULL, "reserve");
    make_item(slot, 44);
    slot->n_soft  = 4;
    slot->soft[0] = -7;
    slot->soft[3] = 7;
    frame_queue_producer_commit(q);
    CHECK(frame_queue_count(q) == 1, "count after commit");
    CHECK(frame_queue_pop(q, &out), "pop reserved");
    CHECK(recover_id(&out) == 44 && out.n_soft == 4 &&
              out.soft[0] == -7 && out.soft[3] == 7,
          "reserve/commit round-trip");

    // Reserve honours full-queue (capacity 3).
    for (int i = 0; i < 3; i++) {
        frame_queue_item_t *s = frame_queue_producer_reserve(q);
        CHECK(s != NULL, "fill reserve %d", i);
        make_item(s, 50 + (uint32_t)i);
        frame_queue_producer_commit(q);
    }
    uint64_t dropped_before = frame_queue_dropped(q);
    CHECK(frame_queue_producer_reserve(q) == NULL, "reserve on full");
    CHECK(frame_queue_dropped(q) == dropped_before + 1, "full counts a drop");

    frame_queue_destroy(q);
}

// --- 4. Concurrent producer/consumer with pthread.
//
// Mirrors production semantics: the worker calls push() once per
// burst with no retry — if the queue is full, the frame is dropped.
// The consumer keeps up most of the time but may occasionally fall
// behind. We verify:
//   - every popped item's ID is strictly greater than the previous
//     (ordering preserved)
//   - pushed + dropped == total attempts
//   - popped == pushed
//   - queue drains to empty
#define MT_TOTAL 100000
#define MT_QSIZE 64

static void *mt_producer(void *arg)
{
    frame_queue_t     *q = (frame_queue_t *)arg;
    frame_queue_item_t it;
    for (uint32_t i = 0; i < MT_TOTAL; i++) {
        make_item(&it, i);
        // Single attempt; failures count toward dropped. This is
        // the production behavior: real-time worker never blocks.
        (void)frame_queue_push(q, &it);
    }
    return NULL;
}

static int          mt_ordering_errors = 0;
static volatile int mt_producer_done   = 0;

static void *mt_consumer(void *arg)
{
    frame_queue_t     *q = (frame_queue_t *)arg;
    frame_queue_item_t out;
    int64_t            prev_id = -1;
    while (1) {
        if (frame_queue_pop(q, &out)) {
            int64_t got = (int64_t)recover_id(&out);
            if (got <= prev_id) {
                mt_ordering_errors++;
            }
            prev_id = got;
        } else if (mt_producer_done) {
            // Producer is done and queue is empty — exit.
            break;
        } else {
            for (volatile int k = 0; k < 100; k++)
                ;
        }
    }
    return NULL;
}

static void test_pthread_producer_consumer(void)
{
    printf("Test: pthread 1P-1C, single-attempt push, %d items, %d-slot queue\n",
           MT_TOTAL, MT_QSIZE);
    mt_ordering_errors = 0;
    mt_producer_done   = 0;
    frame_queue_t *q   = frame_queue_create(MT_QSIZE);
    CHECK(q != NULL, "create");

    pthread_t prod_thr, cons_thr;
    pthread_create(&prod_thr, NULL, mt_producer, q);
    pthread_create(&cons_thr, NULL, mt_consumer, q);
    pthread_join(prod_thr, NULL);
    mt_producer_done = 1;
    pthread_join(cons_thr, NULL);

    uint64_t pushed  = frame_queue_pushed(q);
    uint64_t popped  = frame_queue_popped(q);
    uint64_t dropped = frame_queue_dropped(q);

    printf("  pushed=%llu popped=%llu dropped=%llu (total attempts=%d)\n",
           (unsigned long long)pushed, (unsigned long long)popped,
           (unsigned long long)dropped, MT_TOTAL);

    CHECK(mt_ordering_errors == 0, "%d ordering errors", mt_ordering_errors);
    CHECK(pushed + dropped == MT_TOTAL,
          "pushed+dropped=%llu (expected %d)",
          (unsigned long long)(pushed + dropped), MT_TOTAL);
    CHECK(popped == pushed,
          "popped=%llu pushed=%llu",
          (unsigned long long)popped, (unsigned long long)pushed);
    CHECK(frame_queue_count(q) == 0, "queue not drained");

    frame_queue_destroy(q);
}

int main(void)
{
    test_invalid_create();
    test_basic_push_pop();
    test_wraparound();
    test_soft_and_reserve();
    test_pthread_producer_consumer();
    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
