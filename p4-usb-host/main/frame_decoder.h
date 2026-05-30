#ifndef FRAME_DECODER_H
#define FRAME_DECODER_H

// Higher-layer decoder task. Pulls bit packets out of a PSRAM queue
// (frame_queue), classifies them via iridium_frame_classify, and (in
// later phases) feeds IDA-LCW frames into the SBD reassembler and
// libacars.
//
// Lifecycle: frame_decoder_init() once at startup, before any worker
// pushes. Returns ESP_OK on success. The task auto-pins to Core 1
// at priority 4 (lower than worker_core1 at 5; lower than ingest at 8;
// higher than status_logger at 1).

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>
#include "qpsk_demod.h"           // ir_direction_t

#ifdef __cplusplus
extern "C" {
#endif

// Initialise the decoder: allocate PSRAM-backed queue, spawn the
// consumer task, register with the task watchdog. Idempotent —
// subsequent calls return ESP_OK without reinitialising.
esp_err_t frame_decoder_init(void);

// Producer-side push. Called by worker_core1 after qpsk_demod reports
// success (and the bits are still in scope). Bits[] must be 0/1-per-byte
// from qpsk_demod; n_bits is typically 382 for an Iridium burst.
//
// Non-blocking — if the queue is full, the frame is dropped and the
// frame_decoder_dropped() counter advances. Always safe to call from
// worker_core1's task; not ISR-safe (uses memcpy of ~400 bytes).
//
// Returns true on enqueue, false if dropped or if frame_decoder_init
// hasn't been called yet.
bool frame_decoder_push(const uint8_t *bits, size_t n_bits,
                        ir_direction_t direction,
                        uint32_t freq_hz, int peak_bin, float snr_db);

// Stats accessors for the per-second status block.
uint64_t frame_decoder_pushed(void);    // total bursts the worker handed off
uint64_t frame_decoder_popped(void);    // total bursts the decoder processed
uint64_t frame_decoder_dropped(void);   // total dropped (queue full)
size_t   frame_decoder_queue_count(void);

// Per-frame-class counts since boot. Useful for the status block.
typedef struct {
    uint64_t unknown;
    uint64_t ms;
    uint64_t tl;
    uint64_t bc;
    uint64_t lw_da;       // LW with subtype DA — these are SBD/ACARS-bearing
    uint64_t lw_other;    // LW with any other subtype (VO/IP/SY/U3/U6/...)
} frame_decoder_class_counts_t;
void frame_decoder_get_class_counts(frame_decoder_class_counts_t *out);

// Lifetime totals (since boot) of the two ACARS-pipeline counters.
// Cheap relaxed atomic loads; safe to call from any thread.
uint64_t frame_decoder_acars_decoded_total(void);
uint64_t frame_decoder_sbd_complete_total(void);

// Rolling decode-rate counters (#117). Sum of classified-as-known-type
// frames over the last 1 h and 24 h, snapped on a 1-minute esp_timer
// tick. A WARN log fires automatically when 24h>10 && 1h==0 ("we
// used to work, we no longer do") — catches silent DSP wedge.
void frame_decoder_get_rolling_rates(uint32_t *out_1h, uint32_t *out_24h);

#ifdef __cplusplus
}
#endif

#endif // FRAME_DECODER_H
