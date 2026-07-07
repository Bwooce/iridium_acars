#ifndef INGEST_CORE1_H
#define INGEST_CORE1_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

// Two ping-pong slots, alternated each consumer cycle.
// Each slot holds the converted (uint8 -> int16 Q15) IQ samples for one
// 16 KB USB transfer. Sized for the same out_block_size (16 KB) the
// class_driver currently uses, expressed in int16 units (8192 elements
// = 32 KB per slot).
#define INGEST_NUM_SLOTS 2
#define INGEST_SLOT_ELEMS (20 * 1024) /* int16 elements per slot (40 KB each).    \
                                       * Path A (native 2.5 MSPS, no resample):   \
                                       * output = input count = 8192 complex =    \
                                       * 16384 int16 for a 16 KB dispatch — the \
                                       * old 16*1024 was an exact fit; 20*1024    \
                                       * adds safety margin. */

// T49b: size of the internal-SRAM staging tile the convert+resample loop
// processes at a time, in complex samples (4 KB = 1024 complex * 2 int16).
// Deliberately MINIMAL, well below the design doc's original 8-16 KB range:
// the DMA-INT budget is critically tight (pre-stream free ~44 KB, USB pool
// already short of its 8 transfers) and this tile is NEW internal-SRAM
// pressure on top of that (deleting s_conv[] only frees PSRAM). An 8 KB
// tile dropped the pool 5->4 transfers and boot-crashed the device (store
// fault in the tagger FFT); 4 KB keeps the pool at its working 5-transfer
// point. The resampler is chunk-continuable at any tile size (bit-exact —
// see tests/host/test_resample_tile.c), so this only trades a little loop
// overhead for the smaller footprint. Do not grow this without
// re-measuring "Pre-stream DMA-internal heap: free=" and the pool count.
#define INGEST_TILE_COMPLEX 1024

// Initialise the ingest task on Core 1 plus the ping-pong infrastructure.
// Allocates converted/resampled buffers in PSRAM, creates the queue and
// per-slot semaphores, spawns the task. (T49a: there is no more raw
// buffer allocation here — the raw USB bytes live in the usbring PSRAM
// ring, esp_libusb.c/usbring.c, and are handed to ingest_core1_dispatch
// as a pointer instead of being copied into a slot-owned buffer.) Calls
// ingest_core1_prealloc_tile() as a lazy fallback if the boot-time
// early-alloc dance hasn't already done so; fails with ESP_ERR_NO_MEM
// if the tile isn't available (see ingest_core1_prealloc_tile's doc
// comment — this must never proceed with the tile missing/misplaced).
esp_err_t ingest_core1_init(void);

// T49b: early one-time allocation of the internal-SRAM convert/resample
// staging tile. This buffer is the resample PIE MAC's (resample_arp4.S,
// via resample_256_to_250_process_explicit) INPUT — a PIE-read buffer.
// On this chip, a PIE working buffer that lands outside main DRAM (e.g.
// RTCRAM 0x5010_xxxx, which MALLOC_CAP_INTERNAL can silently fall back
// to under heap pressure) makes the PIE unit silently mis-decode — see
// memory note project_heap_position_decode_bug.md. Call this FIRST in
// boot, in the same early-alloc dance as resample_256_to_250_alloc_coeffs()
// / fft_sc16_2048_init() / uw_correlator_prealloc_pie_fft() (class_driver.c
// action_start_stream, smoke_test.c's pipeline bring-up), while DRAM is
// plentiful. Idempotent. Hard-guards the allocation with esp_ptr_in_dram()
// and logs the resulting address; a non-DRAM placement is a loud
// ESP_LOGE, not a silent fallback — ingest_core1_init() treats a missing/
// misplaced tile as a fatal ESP_ERR_NO_MEM.
void ingest_core1_prealloc_tile(void);

// Reserve the next output slot for this dispatch cycle. Blocks until
// that slot's previous DSP consumer has freed it (typically immediate).
// out_slot is set to the slot index for the matching
// ingest_core1_dispatch / ingest_core1_wait_raw_done /
// ingest_core1_take_converted / ingest_core1_release calls.
void ingest_core1_acquire_slot(int *out_slot);

// Hand off a ring region — `ptr` points directly into the usbring PSRAM
// backing store, valid for `bytes_filled` bytes — to the Core 1 ingest
// task, tagged with the output slot from ingest_core1_acquire_slot. The
// ingest task will:
//   1. Convert raw uint8 -> int16 Q15 from `ptr` into slot's int16
//      buffer, then give the slot's "raw done" semaphore (see
//      ingest_core1_wait_raw_done) — the ring bytes at `ptr` may be
//      reclaimed (usbring_consume) only after that.
//   2. signal_buffer_push the converted data into PSRAM
//   3. Mark the slot ready-for-DSP via a semaphore
void ingest_core1_dispatch(int slot, const uint8_t *ptr, size_t bytes_filled);

// Wait until the ingest task has finished READING the ring region handed
// to ingest_core1_dispatch() for this slot (i.e. the convert step is
// done, though resample+push may still be in flight). The caller must
// wait on this — and only then usbring_consume() that region's bytes —
// before peeking the ring for the next dispatch: the usbring only
// supports one outstanding un-consumed span, so "acquire the next ring
// region" is serialised against "Core 1 has finished converting the
// previous one" (T49a design doc's disclosed tradeoff). This is a NEW
// synchronisation point, separate from (and earlier than) s_ready —
// it does not change the existing s_free/s_ready acquire/dispatch/
// take/release protocol's shape.
void ingest_core1_wait_raw_done(int slot);

// Count of dispatches dropped because the queue was unexpectedly full
// (should always be 0; nonzero = the slot was recovered, not deadlocked). #106
uint32_t ingest_core1_dispatch_drops(void);

// Count of 500 ms ticks take_converted waited without s_ready being given.
// A handful per hour is fine (slow ingest cycle); a sustained climb means
// ingest is wedged and health_wdt will reboot once class can't progress (#110).
uint32_t ingest_core1_take_converted_slow_waits(void);

// Count of 500 ms ticks ingest_core1_wait_raw_done waited without the
// slot's "raw done" semaphore being given (T49a's new acquire-next
// vs. convert-done serialisation point). A handful per hour is fine;
// a sustained climb means ingest is wedged before it even reaches the
// resample/push stage.
uint32_t ingest_core1_raw_done_slow_waits(void);

// Wait for the named slot's converted int16 data to be ready. Returns a
// pointer to the int16 buffer the DSP can read. Sets *out_n_int16 to the
// number of int16 elements available (= bytes_filled).
int16_t *ingest_core1_take_converted(int slot, size_t *out_n_int16);

// Mark a slot's converted data as fully consumed by DSP. The ingest task
// can now reuse that slot for the next conversion.
void ingest_core1_release(int slot);

// Diagnostics
typedef struct {
    uint64_t convert_us_total;   // sum of convert wall-clock since last get
    uint64_t push_us_total;      // sum of resample + signal_buffer_push wall-clock
    uint64_t resample_us_total;  // resample step only
    uint64_t sbpush_us_total;    // signal_buffer_push (AXI DMA wait) only
    uint32_t dispatches;         // count of dispatches handled
    uint32_t slot_wait_total_us; // time the consumer waited for a slot
    uint32_t consumer_waits;     // count of times consumer had to block
} ingest_stats_t;

void ingest_core1_get_stats(ingest_stats_t *out);

// D16 AGC peak-sample probe. Returns the maximum |raw uint8 - 127|
// observed in the input prefix of any dispatch since the last call,
// plus the number of dispatches sampled. Resets the counters after
// reading. peak_dev=127 means a sample was at 0 or 255 (saturation).
void ingest_core1_agc_sample(uint8_t *out_peak_dev, uint32_t *out_dispatches);

#endif
