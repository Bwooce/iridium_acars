#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "usbring.h"
#include "usbring_ring.h"

static const char *TAG = "USBRING";

// Free-running byte counters (see usbring_ring.h's full/empty
// convention: head/tail are NEVER individually wrapped, only masked
// against capacity-1 at the point of use). head is producer-owned
// (only stream_transfer_cb writes it), tail is consumer-owned (only
// class_driver's drain loop writes it). _Atomic + explicit memory
// orders matches the codebase's existing cross-core counter idiom
// (sd_capture.c, frame_decoder.c, frame_pdu.c) rather than raw
// __atomic_* builtins.
static uint8_t         *s_buf      = NULL;
static uint32_t         s_capacity = 0;
static _Atomic uint32_t s_head     = 0;
static _Atomic uint32_t s_tail     = 0;

esp_err_t usbring_init(uint32_t capacity)
{
    if (!usbring_is_pow2_capacity(capacity)) {
        ESP_LOGE(TAG, "capacity %u is not a power of two", (unsigned)capacity);
        return ESP_ERR_INVALID_ARG;
    }
    s_buf = heap_caps_aligned_alloc(64, capacity, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_buf) {
        ESP_LOGE(TAG, "Failed to allocate %u KB PSRAM ring",
                 (unsigned)(capacity / 1024));
        return ESP_ERR_NO_MEM;
    }
    s_capacity = capacity;
    atomic_store_explicit(&s_head, 0, memory_order_relaxed);
    atomic_store_explicit(&s_tail, 0, memory_order_relaxed);
    ESP_LOGI(TAG, "Ring allocated: %u KB PSRAM at %p",
             (unsigned)(capacity / 1024), s_buf);
    return ESP_OK;
}

void usbring_deinit(void)
{
    free(s_buf);
    s_buf      = NULL;
    s_capacity = 0;
    atomic_store_explicit(&s_head, 0, memory_order_relaxed);
    atomic_store_explicit(&s_tail, 0, memory_order_relaxed);
}

bool usbring_write(const uint8_t *data, uint32_t n)
{
    if (!s_buf) return false;
    if (n == 0) return true;

    // Producer's own counter: relaxed (no other task writes s_head).
    uint32_t head = atomic_load_explicit(&s_head, memory_order_relaxed);
    // Consumer-owned counter: a stale (older) read here is always safe
    // — it can only make free_space look SMALLER than reality, which
    // makes the producer conservative (a possible spurious drop), never
    // lets it overwrite unconsumed data. Acquire anyway so free_space is
    // as fresh as possible (fewer spurious rb_full_drops).
    uint32_t tail = atomic_load_explicit(&s_tail, memory_order_acquire);

    uint32_t free_bytes = usbring_free_space(head, tail, s_capacity);
    if (n > free_bytes) {
        return false; // mirrors xRingbufferSend()'s all-or-nothing drop
    }

    uint32_t first = usbring_write_contig(head, tail, s_capacity);
    if (first > n) first = n;
    uint32_t off = usbring_phys_offset(head, s_capacity);
    memcpy(s_buf + off, data, first);
    if (n > first) {
        memcpy(s_buf, data + first, n - first);
    }

    // Publish: the memcpy(s) above MUST be globally visible before the
    // consumer (a different core) can observe the advanced head and
    // start reading this memory. Same pattern as ingest_core1.c:137.
    __sync_synchronize();
    atomic_store_explicit(&s_head, usbring_next_head(head, n), memory_order_relaxed);
    return true;
}

const uint8_t *usbring_peek(uint32_t *out_n)
{
    if (!s_buf) {
        if (out_n) *out_n = 0;
        return NULL;
    }
    // Acquire: must happen-after the producer's __sync_synchronize()
    // publish, so the bytes in [tail, tail+n_contig) are guaranteed to
    // be the ones described by this head value.
    uint32_t head = atomic_load_explicit(&s_head, memory_order_acquire);
    // Consumer's own counter: relaxed (no other task writes s_tail).
    uint32_t tail = atomic_load_explicit(&s_tail, memory_order_relaxed);

    uint32_t n_contig = 0;
    uint32_t off      = usbring_peek_contig(head, tail, s_capacity, &n_contig);
    if (n_contig == 0) {
        if (out_n) *out_n = 0;
        return NULL;
    }
    if (out_n) *out_n = n_contig;
    return s_buf + off;
}

void usbring_consume(uint32_t n)
{
    if (!s_buf || n == 0) return;
    uint32_t tail = atomic_load_explicit(&s_tail, memory_order_relaxed);
    // Release: the caller's reads of the consumed span (e.g. Core 1's
    // convert loop, synchronised into this call via the ingest "raw
    // done" semaphore) must complete-before the producer can observe
    // the advanced tail and reuse that space.
    atomic_store_explicit(&s_tail, usbring_next_tail(tail, n), memory_order_release);
}

void usbring_get_info(size_t *used, size_t *capacity)
{
    if (!s_buf) {
        if (used) *used = 0;
        if (capacity) *capacity = 0;
        return;
    }
    uint32_t head = atomic_load_explicit(&s_head, memory_order_acquire);
    uint32_t tail = atomic_load_explicit(&s_tail, memory_order_acquire);
    if (used) *used = usbring_used(head, tail);
    if (capacity) *capacity = s_capacity;
}
