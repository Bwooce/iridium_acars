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
#define INGEST_NUM_SLOTS    2
#define INGEST_SLOT_ELEMS   (16 * 1024)   /* int16 elements per slot (32 KB each) */

// Initialise the ingest task on Core 1 plus the ping-pong infrastructure.
// Allocates raw + converted buffers in DMA-capable internal SRAM, creates
// the queue and per-slot semaphores, spawns the task.
esp_err_t ingest_core1_init(void);

// Returns a pointer to the next raw USB-fill buffer the consumer should
// read into. Blocks until that slot's previous DSP consumer has freed it
// (typically immediate). out_slot is set to the slot index for the
// matching ingest_core1_dispatch call.
//
// Each slot has 16 KB of raw uint8 storage. Caller must read up to that
// many bytes via esp_libusb_read_stream().
uint8_t *ingest_core1_acquire_raw(int *out_slot);

// Hand off a filled raw buffer (slot index from ingest_core1_acquire_raw)
// to the Core 1 ingest task. The ingest task will:
//   1. Convert raw uint8 -> int16 Q15 into slot's int16 buffer
//   2. signal_buffer_push the converted data into PSRAM
//   3. Mark the slot ready-for-DSP via a semaphore
void ingest_core1_dispatch(int slot, size_t bytes_filled);

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
    uint64_t push_us_total;      // sum of signal_buffer_push wall-clock
    uint32_t dispatches;         // count of dispatches handled
    uint32_t slot_wait_total_us; // time the consumer waited for a slot
    uint32_t consumer_waits;     // count of times consumer had to block
} ingest_stats_t;

void ingest_core1_get_stats(ingest_stats_t *out);

#endif
