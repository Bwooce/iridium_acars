// SPSC lock-free ringbuffer for frame_queue_t. See frame_queue.h.

#include "frame_queue.h"
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>

// On target use heap_caps_malloc with MALLOC_CAP_SPIRAM. On host fall
// back to calloc.
#if defined(ESP_PLATFORM)
#include "esp_heap_caps.h"
static void *fq_alloc(size_t bytes)
{
    return heap_caps_calloc(1, bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}
static void fq_free(void *p)
{
    heap_caps_free(p);
}
#else
static void *fq_alloc(size_t bytes)
{
    return calloc(1, bytes);
}
static void fq_free(void *p)
{
    free(p);
}
#endif

struct frame_queue {
    size_t              n_slots;
    size_t              mask; // n_slots - 1 (n_slots is a power of two)
    _Atomic size_t      head; // consumer index (next to read)
    _Atomic size_t      tail; // producer index (next to write)
    _Atomic uint64_t    pushed;
    _Atomic uint64_t    popped;
    _Atomic uint64_t    dropped;
    frame_queue_item_t *slots; // n_slots entries
};

static int is_power_of_two(size_t n)
{
    return n != 0 && (n & (n - 1)) == 0;
}

frame_queue_t *frame_queue_create(size_t n_slots)
{
    if (n_slots < 2 || !is_power_of_two(n_slots)) return NULL;

    frame_queue_t *q = (frame_queue_t *)fq_alloc(sizeof(*q));
    if (!q) return NULL;

    q->slots = (frame_queue_item_t *)fq_alloc(n_slots * sizeof(frame_queue_item_t));
    if (!q->slots) {
        fq_free(q);
        return NULL;
    }

    q->n_slots = n_slots;
    q->mask    = n_slots - 1;
    atomic_store_explicit(&q->head, 0, memory_order_relaxed);
    atomic_store_explicit(&q->tail, 0, memory_order_relaxed);
    atomic_store_explicit(&q->pushed, 0, memory_order_relaxed);
    atomic_store_explicit(&q->popped, 0, memory_order_relaxed);
    atomic_store_explicit(&q->dropped, 0, memory_order_relaxed);
    return q;
}

void frame_queue_destroy(frame_queue_t *q)
{
    if (!q) return;
    fq_free(q->slots);
    fq_free(q);
}

bool frame_queue_push(frame_queue_t *q, const frame_queue_item_t *item)
{
    if (!q || !item) return false;

    size_t tail      = atomic_load_explicit(&q->tail, memory_order_relaxed);
    size_t head      = atomic_load_explicit(&q->head, memory_order_acquire);
    size_t next_tail = (tail + 1) & q->mask;
    if (next_tail == head) {
        // Full — record the drop and return.
        atomic_fetch_add_explicit(&q->dropped, 1, memory_order_relaxed);
        return false;
    }

    // Copy the item into the ringbuffer slot. memcpy of ~432 bytes is
    // negligible compared to the BCH and demod work upstream.
    memcpy(&q->slots[tail], item, sizeof(*item));

    // Release-store on tail so the consumer's acquire-load sees the
    // payload write before the index update.
    atomic_store_explicit(&q->tail, next_tail, memory_order_release);
    atomic_fetch_add_explicit(&q->pushed, 1, memory_order_relaxed);
    return true;
}

bool frame_queue_pop(frame_queue_t *q, frame_queue_item_t *out)
{
    if (!q || !out) return false;

    size_t head = atomic_load_explicit(&q->head, memory_order_relaxed);
    size_t tail = atomic_load_explicit(&q->tail, memory_order_acquire);
    if (head == tail) {
        return false; // empty
    }

    memcpy(out, &q->slots[head], sizeof(*out));

    size_t next_head = (head + 1) & q->mask;
    atomic_store_explicit(&q->head, next_head, memory_order_release);
    atomic_fetch_add_explicit(&q->popped, 1, memory_order_relaxed);
    return true;
}

size_t frame_queue_count(const frame_queue_t *q)
{
    if (!q) return 0;
    size_t head = atomic_load_explicit(&q->head, memory_order_acquire);
    size_t tail = atomic_load_explicit(&q->tail, memory_order_acquire);
    return (tail - head) & q->mask;
}

size_t frame_queue_capacity(const frame_queue_t *q)
{
    return q ? (q->n_slots - 1) : 0;
}

uint64_t frame_queue_pushed(const frame_queue_t *q)
{
    return q ? atomic_load_explicit(&q->pushed, memory_order_relaxed) : 0;
}

uint64_t frame_queue_popped(const frame_queue_t *q)
{
    return q ? atomic_load_explicit(&q->popped, memory_order_relaxed) : 0;
}

uint64_t frame_queue_dropped(const frame_queue_t *q)
{
    return q ? atomic_load_explicit(&q->dropped, memory_order_relaxed) : 0;
}
