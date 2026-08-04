#ifndef FRAME_QUEUE_H
#define FRAME_QUEUE_H

// Single-producer / single-consumer ringbuffer for handing demodulated
// frames from worker_core1 (after BCH) to a frame_decoder task that
// runs the classifier + IDA/SBD/libacars stack.
//
// Why a queue:
//   - Decouples BCH-success-rate from the higher-layer processing
//     latency. libacars + reassembler can take seconds without
//     back-pressuring the real-time DSP path.
//   - Lets the higher layers run on a separate task pinned to Core 1
//     (alongside the worker — same data locality, easy to debug).
//
// Backing memory: the slot pool lives in PSRAM. A 64-slot queue with
// 400-byte items is ~26 KB — comfortably within the 16 MB PSRAM budget.
//
// SPSC lock-free using C11 atomics on head/tail. No FreeRTOS deps in
// this layer; thread blocking lives in p4-usb-host/main/frame_decoder.c
// (the consumer side wakes via task notification when push() observes
// an empty -> non-empty transition).

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Maximum demod bit count we ever queue. The queue is band-agnostic
// (bits + metadata), so this is sized to the LARGEST frame any band
// pushes:
//
// Iridium L-band TDMA (the original sizing):
//   - One TDMA slot:    8.28 ms × 25 ksym/s × 2 bit/sym  = ~414 bits
//   - Single-slot burst (most IDA / IBC / IRA frames):   ~382 bits
//   - Two-slot data burst (concatenated next-access):    ~828 bits
//   - Four-slot voice burst (theoretical max):          ~1656 bits
//   (was 2048 through 2026-07; Iridium never comes close to the new cap)
//
// VDL Mode 2 (band=vdl2, phase V3): one transmission = 25 header bits
// + 8 × (data octets + RS FEC octets). The header length cap
// (VDL2_MAX_FRAME_BITS = 0x3FFF data bits, dumpvdl2 decode.c:45) gives
// 2048 data octets + 52 FEC octets = 16825 bits worst case. Round up
// to 16832 (multiple of 64). Per-item cost: 16832 + 768 soft + 24
// metadata ≈ 17.6 KB; the 64-slot queue is ~1.13 MB in PSRAM (fits the
// ~4 MB headroom — PSRAM-budget memory note). Runtime cost for Iridium
// is UNCHANGED: push/pop copy only the n_bits/n_soft prefixes
// (frame_queue.c), so the larger slots cost PSRAM, not cycles.
#define FRAME_QUEUE_MAX_BITS 16832

// Maximum per-bit soft metrics carried alongside bits[]. Two consumers:
//   - Iridium Chase-2 soft BCH (task #16): needs soft coverage of frame
//     bits [0, 382) only (UW+LCW+data); a standard burst is ~382 bits.
//   - VDL2 RS soft-decision erasure fallback (feat/vhf-vdl2): vdl2_l2
//     needs per-bit confidence for the WHOLE transmission (data + RS FEC
//     octets), because the erasure decoder picks the least-reliable
//     symbols of EACH RS block — so soft must cover every block, not just
//     the first. Sized to FRAME_QUEUE_MAX_BITS to match.
// Producers set n_soft = n_bits; frame_decoder_push truncates to this cap.
// Iridium frames stay tiny (~382), so their push/pop copy cost is
// unchanged — the larger array costs PSRAM (int16 × 16832 ≈ 33 KB/slot,
// 64 slots ≈ 2.1 MB, within the ~3 MB PSRAM headroom), not cycles.
#define FRAME_QUEUE_MAX_SOFT FRAME_QUEUE_MAX_BITS

typedef struct {
    // Host timestamp at enqueue (esp_timer_get_time / gettimeofday).
    // Full 64-bit: a uint32_t wrapped at ~71.6 min uptime, after which
    // sbd_reassembler_tick (which runs on the unwrapped 64-bit clock)
    // saw every session as expired and silently killed multi-frame
    // reassembly.
    uint64_t timestamp_us;
    uint32_t freq_hz;   // burst center frequency from worker
    int32_t  peak_bin;  // FFT bin reported by detector (post-fftshift)
    float    snr_db;    // detection SNR
    uint16_t n_bits;    // valid count in bits[]
    uint8_t  direction; // 0 = downlink (matches qpsk_demod's DIR_DOWNLINK),
                        // 1 = uplink   (DIR_UPLINK)
    uint8_t pad;
    // Per-bit soft metrics for bits[0..n_soft) (sign = hard decision,
    // magnitude = reliability; see qpsk_demod.h). n_soft == 0 when the
    // producer had none (aggregator PDU path, demod OOM). Kept BEFORE
    // bits[] so push/pop can keep copying only the valid bits[] prefix
    // (bits[] must stay the last member).
    uint16_t n_soft;
    int16_t  soft[FRAME_QUEUE_MAX_SOFT];
    uint8_t  bits[FRAME_QUEUE_MAX_BITS]; // 0/1-per-byte demod output
} frame_queue_item_t;

typedef struct frame_queue frame_queue_t;

// Allocate a queue with `n_slots` ringbuffer entries. On target this
// allocates the slot pool from PSRAM (MALLOC_CAP_SPIRAM); on host it
// uses calloc. n_slots must be ≥ 2 and a power of two for a fast modulo.
frame_queue_t *frame_queue_create(size_t n_slots);
void           frame_queue_destroy(frame_queue_t *q);

// Non-blocking push from producer side. Returns true on enqueue, false
// if the queue is full (item is dropped; counter advances). Increments
// `dropped` regardless of caller behaviour so callers can use it for
// debug/stats. Producer callable from any task / ISR context that has
// stable access to `q`.
bool frame_queue_push(frame_queue_t *q, const frame_queue_item_t *item);

// Zero-copy producer path: reserve the tail slot for in-place fill,
// then commit to publish it. Between reserve and commit the producer
// owns the returned slot exclusively (SPSC — the consumer never reads
// past the released tail). Returns NULL when full (the drop counter
// advances, mirroring frame_queue_push). The producer MUST set n_bits/
// n_soft consistently with what it wrote before committing, and must
// not interleave another reserve/push before the commit. Added so
// frame_decoder_push can fill the (now ~2.8 KB) item directly in PSRAM
// instead of staging it on the calling task's stack and copying twice.
//
// SINGLE-PRODUCER INVARIANT: this is SPSC — exactly ONE producer task may drive
// reserve/commit (and frame_queue_push) at a time. The codebase has three
// producer sites (worker_core1 Iridium bursts, aggregator PDU ingest, and
// frame_decoder_push_poa for POA), kept mutually exclusive by band selection
// (band=poa never runs the tagger/worker; the aggregator drain is idle). Do NOT
// add a build/config that lets two of them run concurrently without making this
// queue MPSC first — concurrent reserves would publish the same/overwritten slot.
frame_queue_item_t *frame_queue_producer_reserve(frame_queue_t *q);
void                frame_queue_producer_commit(frame_queue_t *q);

// Non-blocking pop from consumer side. Returns true if an item was
// dequeued into *out, false if empty.
bool frame_queue_pop(frame_queue_t *q, frame_queue_item_t *out);

// Stats accessors (read-only from any task; values are atomic loads).
size_t   frame_queue_count(const frame_queue_t *q);    // current items in flight
size_t   frame_queue_capacity(const frame_queue_t *q); // n_slots - 1 (one slot reserved)
uint64_t frame_queue_pushed(const frame_queue_t *q);   // total successful pushes
uint64_t frame_queue_dropped(const frame_queue_t *q);  // total drops (queue full)
uint64_t frame_queue_popped(const frame_queue_t *q);   // total successful pops

#ifdef __cplusplus
}
#endif

#endif // FRAME_QUEUE_H
