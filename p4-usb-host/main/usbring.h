#ifndef USBRING_H
#define USBRING_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Buffer-backed single-producer/single-consumer byte ring for the USB
// ingest stream (T49a, docs/perf-decoupling-design-2026-07-04.md §T49a).
// Replaces `dev->ringbuf` (the IDF `xRingbufferCreateWithCaps` byte ring
// in esp_libusb.c), whose byte-buffer mode only allows ONE outstanding
// `xRingbufferReceiveUpTo` retrieval at a time (see the design doc's
// "blocker discovered" note). This module is the buffer-backed wrapper
// around usbring_ring.h's pure index/wrap arithmetic (head/tail/capacity
// in, offsets/lengths out): it owns the PSRAM allocation, the memcpy
// wrap-splitting, and the producer/consumer memory-ordering barriers
// usbring_ring.h documents but deliberately does not implement.
//
//   Producer: stream_transfer_cb (esp_libusb.c), the USB host client's
//             URB completion callback -- runs inside usb_pump's
//             usb_host_client_handle_events() (class_driver.c).
//   Consumer: dsp_feed's drain loop (class_driver.c, T48 split) -- the
//             ONLY call site for usbring_peek() / usbring_consume().
//             Before T48 this was the same task as the producer
//             (class_driver's single combined loop); post-T48 producer
//             and consumer are genuinely concurrent tasks (both pinned
//             to Core 0), which is exactly what the barriers below are
//             for.
//
// IMPORTANT — single outstanding span: usbring_peek() always views the
// ring from the current tail (per usbring_ring.h's contract). Calling
// it again before usbring_consume()'ing the previous peek does NOT
// return a fresh, further-along span — it re-views the same tail
// (possibly with more bytes appended). The consumer must fully consume
// one peeked span before it can see the next one; it must never try to
// hold two independent un-consumed spans concurrently. This is why the
// class_driver rewiring waits for Core 1's "raw done" signal for the
// previous dispatch before peeking the next one (T49a design doc: "one
// outstanding item serialises acquire-next against convert-done").

// Allocate the ring's PSRAM backing store and reset head/tail to 0.
// `capacity` MUST be a power of two (usbring_is_pow2_capacity() checked;
// returns ESP_ERR_INVALID_ARG otherwise, ESP_ERR_NO_MEM on alloc
// failure). Safe to call once per stream start.
esp_err_t usbring_init(uint32_t capacity);

// Free the backing store and reset state. Only safe once producer and
// consumer are both idle (no in-flight stream_transfer_cb / drain loop).
void usbring_deinit(void);

// Reset head/tail to 0 WITHOUT touching the PSRAM allocation (s_buf /
// s_capacity untouched). Drops any stale buffered samples. For the
// stream-pause-retune path (class_driver.c ACTION_RETUNE): the ring is
// paused/drained, not torn down, so resume must reuse the existing
// allocation rather than calling usbring_init() again (that would leak
// the previous 4 MB PSRAM block — usbring_init() is not idempotent).
// Only safe once producer and consumer are both idle, same precondition
// as usbring_deinit().
void usbring_reset(void);

// Producer: copy `n` bytes from `data` into the ring, splitting the
// memcpy at the physical wrap boundary via usbring_ring.h's
// usbring_write_contig(). Returns false (nothing written) if `n`
// exceeds the current free space — mirrors xRingbufferSend()'s
// all-or-nothing behaviour so stream_transfer_cb's existing
// rb_full_drops accounting keeps working unmodified. Publishes with
// `__sync_synchronize()` between the memcpy(s) and the head store
// (the same pattern already used at ingest_core1.c:137) so the
// consumer never observes an advanced head before the bytes it
// describes are actually in memory.
bool usbring_write(const uint8_t *data, uint32_t n);

// Consumer: acquire-read head, return a pointer into the ring's PSRAM
// backing store for the contiguous span starting at the current tail,
// and (via `*out_n`) the number of contiguous bytes available there —
// clamped to the physical wrap boundary, NOT to any caller-supplied
// maximum. The caller clamps `*out_n` down to whatever length it
// actually wants to use before dispatching (usbring_ring.h's
// peek-then-consume-a-prefix contract). Returns NULL / *out_n = 0 if
// the ring is empty.
const uint8_t *usbring_peek(uint32_t *out_n);

// Consumer: advance tail by `n` bytes once the caller — and anything it
// handed the peeked pointer to (e.g. the Core-1 convert step) — is
// completely done reading them. `n` MUST be <= the span most recently
// returned by usbring_peek() that hasn't already been (partially)
// consumed.
void usbring_consume(uint32_t n);

// Diagnostics: bytes currently queued (unconsumed) + total capacity.
// Both are 0 if the ring hasn't been usbring_init()'d.
void usbring_get_info(size_t *used, size_t *capacity);

// T48 (docs/perf-decoupling-design-2026-07-04.md §T48): register the task
// that usbring_write() should wake -- via a lightweight task notification
// -- every time it adds bytes to the ring. This lets the consumer (post-
// T48: the dsp_feed task) block on ring-empty with ulTaskNotifyTake()
// instead of busy-polling usbring_peek() when idle. Call once, right
// after creating the consumer task; pass NULL to disable (e.g. once the
// consumer task is stopped, before usbring_deinit()).
//
// No lock is needed around this pointer itself: unlike s_head/s_tail
// (shared between the producer and consumer tasks), both the setter and
// its only reader (usbring_write(), called from stream_transfer_cb
// inside usb_host_client_handle_events) run on usb_pump -- the producer
// task IS the task that starts/stops the consumer.
void usbring_set_consumer_task(TaskHandle_t task);

#endif // USBRING_H
