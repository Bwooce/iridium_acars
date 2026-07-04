#ifndef USBRING_RING_H
#define USBRING_RING_H

#include <stdbool.h>
#include <stdint.h>

// Pure index/wrap arithmetic for the custom SPSC (single-producer,
// single-consumer) byte ring that replaces `dev->ringbuf` (the IDF
// `xRingbufferCreateWithCaps` byte buffer) in the USB ingest path
// (T49a, docs/perf-decoupling-design-2026-07-04.md §T49a). The IDF byte
// ring only allows one outstanding `xRingbufferReceiveUpTo` retrieval at
// a time, which blocks a true zero-copy consumer that wants to hold a
// pointer across the Core 1 convert step. The replacement:
//
//   usbring_write(buf, n)   -- producer (Core 0, stream_transfer_cb):
//                              memcpy n bytes in, release-store head.
//   usbring_peek(&ptr,&len) -- consumer (feeder/class task): pointer +
//                              contiguous length up to the wrap boundary.
//   usbring_consume(n)      -- consumer: advance tail after Core 1
//                              signals conversion done. Multiple peeks
//                              before a consume are fine by construction
//                              (peeking never mutates state); consume
//                              order is FIFO, matching the existing
//                              slot handshake protocol.
//
// This header is ONLY the pure index math -- head/tail/capacity in,
// offsets/lengths out. No memcpy, no PSRAM placement, no atomics/memory
// barriers, no device headers. That mirrors how signal_buffer_ring.h
// isolates signal_buffer_next_head() from signal_buffer.c's DMA and
// FreeRTOS concerns, and for the identical reason: it lets a host unit
// test exercise the exact arithmetic the device runs, even though the
// buffer-backed, cache/barrier-aware wrapper (the actual usbring.c) is a
// LATER implementation step that is not part of this header's scope.
//
// -- Full/empty convention (MANDATORY reading for the implementer who
//    wires the real buffer-backed usbring.c against this contract) --
//
// `head` and `tail` are FREE-RUNNING byte counters: they count every
// byte ever produced/consumed since the ring was created and are never
// individually wrapped to `capacity`. The physical array offset for a
// counter is only computed at the point of use, via
// `usbring_phys_offset()` (a bitmask against capacity-1, since capacity
// is required to be a power of two).
//
// This is deliberate and resolves the classic power-of-two ring
// full-vs-empty ambiguity WITHOUT sacrificing a byte of capacity (the
// common alternative: only ever allow capacity-1 bytes to be
// outstanding, so head==tail is unambiguously empty). With free-running
// counters:
//
//   used  = head - tail             (unsigned subtraction; wraps
//                                     correctly through UINT32_MAX by
//                                     two's-complement arithmetic as
//                                     long as outstanding used bytes
//                                     never exceeds capacity, which the
//                                     producer must enforce)
//   empty <=> used == 0             (head == tail)
//   full  <=> used == capacity      (head - tail == capacity; head !=
//                                     tail even when full)
//
// These two states are always distinguishable because `used` is a
// direct byte count, not a masked/wrapped index. The real implementation
// MUST keep head/tail as full 32-bit free-running counters in the struct
// (not masked into [0, capacity)) and must only mask when computing a
// physical array offset. If a future change stores head/tail already
// masked to capacity, the full/empty ambiguity comes back and this
// header's arithmetic no longer applies -- don't do that.
//
// Producer/consumer division of labour over these functions:
//   producer (usbring_write):  usbring_free_space, usbring_write_contig,
//                               usbring_next_head
//   consumer (usbring_peek):   usbring_peek_contig, usbring_phys_offset
//   consumer (usbring_consume): usbring_used (as an upper-bound check),
//                               usbring_next_tail
//
// Concurrency note (out of scope for this header, recorded for the
// implementer): the real usbring.c needs a `__sync_synchronize()`
// publish barrier between the producer's memcpy and its head store (the
// codebase already uses this pattern, ingest_core1.c:137), and the
// consumer must read head with an acquire semantic (or an equivalent
// barrier) before trusting `usbring_used()`'s result. None of that
// changes the index arithmetic below; it only changes when a caller is
// allowed to observe a given head/tail value.

// True iff `capacity` is a nonzero power of two -- a precondition for
// every function below (physical-offset masking only works for
// power-of-two capacities).
static inline bool usbring_is_pow2_capacity(uint32_t capacity)
{
    return capacity != 0 && (capacity & (capacity - 1)) == 0;
}

// Number of bytes produced but not yet consumed. Correct across
// uint32_t wraparound of the free-running counters (two's-complement
// subtraction), provided the producer never lets `used` exceed
// `capacity` (see usbring_free_space / usbring_write_contig, which are
// how the producer enforces that).
static inline uint32_t usbring_used(uint32_t head, uint32_t tail)
{
    return head - tail;
}

// Bytes the producer may still write before catching up to the
// consumer's tail.
static inline uint32_t usbring_free_space(uint32_t head, uint32_t tail, uint32_t capacity)
{
    return capacity - usbring_used(head, tail);
}

// Physical offset into the backing array for a free-running counter.
static inline uint32_t usbring_phys_offset(uint32_t counter, uint32_t capacity)
{
    return counter & (capacity - 1);
}

// Maximum contiguous span the producer may memcpy in a single write
// starting at `head`, i.e. the smaller of (a) total free space and (b)
// the distance from head's physical offset to the end of the backing
// array. A write of n > this value must be split into two memcpy calls
// (this header does not do that split -- it only reports the boundary;
// see usbring_write() in the real implementation / the test's
// host-only wrapper for the two-part copy).
static inline uint32_t usbring_write_contig(uint32_t head, uint32_t tail, uint32_t capacity)
{
    uint32_t free_bytes = usbring_free_space(head, tail, capacity);
    uint32_t off        = usbring_phys_offset(head, capacity);
    uint32_t to_wrap    = capacity - off;
    return free_bytes < to_wrap ? free_bytes : to_wrap;
}

// Advance the producer's head by n bytes (a plain, unmasked add -- the
// free-running-counter convention documented above). The caller MUST
// have already ensured n <= usbring_free_space(...); this function does
// not check that, mirroring signal_buffer_next_head's role as pure
// arithmetic with the invariant enforced by its one caller.
static inline uint32_t usbring_next_head(uint32_t head, uint32_t n)
{
    return head + n;
}

// Consumer-side peek: returns the physical offset the consumer should
// read from (tail's physical offset) and, via `n_contig_out`, the
// number of contiguously readable bytes starting there -- clamped to
// the wrap boundary even when more unconsumed data exists beyond the
// wrap (a second usbring_peek_contig() call after consuming this span
// reports the rest). This is the crux of the "peek returns pointer +
// contiguous length up to wrap" contract: it must never report a
// length that would read past the end of the backing array.
static inline uint32_t usbring_peek_contig(uint32_t head, uint32_t tail, uint32_t capacity,
                                           uint32_t *n_contig_out)
{
    uint32_t used    = usbring_used(head, tail);
    uint32_t off     = usbring_phys_offset(tail, capacity);
    uint32_t to_wrap = capacity - off;
    uint32_t n       = used < to_wrap ? used : to_wrap;
    if (n_contig_out) {
        *n_contig_out = n;
    }
    return off;
}

// Advance the consumer's tail by n bytes after the caller has finished
// with (converted/copied out) that many bytes. The caller MUST have
// n <= usbring_used(head, tail); this function does not check that
// (same pure-arithmetic contract as usbring_next_head).
static inline uint32_t usbring_next_tail(uint32_t tail, uint32_t n)
{
    return tail + n;
}

#endif // USBRING_RING_H
