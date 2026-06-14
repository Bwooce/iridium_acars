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

// Maximum demod bit count we ever queue. Sized to the Iridium L-band
// TDMA spec, not to what any one corpus frame happens to have:
//
//   - One TDMA slot:    8.28 ms × 25 ksym/s × 2 bit/sym  = ~414 bits
//   - Single-slot burst (most IDA / IBC / IRA frames):   ~382 bits
//   - Two-slot data burst (concatenated next-access):    ~828 bits
//   - Four-slot voice burst (theoretical max):          ~1656 bits
//
// Round up to 2048 (= 256 bytes packed; we store 0/1-per-byte so 2048 B)
// to cover any realistic frame including the 4-slot worst case plus
// preamble margin. Per-item cost: 2048 + 16 metadata = 2064 B; 64-slot
// queue is ~132 KB in PSRAM. (We have 32 MB free PSRAM; this is noise.)
#define FRAME_QUEUE_MAX_BITS 2048

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
    uint8_t bits[FRAME_QUEUE_MAX_BITS]; // 0/1-per-byte demod output
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
