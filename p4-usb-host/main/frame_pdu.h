// Decoded-frame PDU: the unit a worker P4 ships to the aggregator P4
// (#119 / #135). The worker runs the full numerically-heavy front end
// (ingest -> tagger -> per-burst worker -> BCH -> #111 classify) and, for
// frames that classify to a known Iridium type, emits one of these.
//
// Bits are MSB-first PACKED on the wire. The decoder pipeline works on a
// 0/1-per-byte array (decoded_frame_t.bits, up to ~382 bits); we pack for
// the link and the aggregator unpacks before iridium_frame_classify.
//
// Pack/unpack are pure C (host-testable). The output queue is target-only
// (FreeRTOS) and lives behind ESP_PLATFORM.

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

// 512 bits packed = 64 bytes; comfortably covers the longest Iridium
// frame we decode (observed max n_bits = 382).
#define FRAME_PDU_MAX_BITS 512
#define FRAME_PDU_BITS_BYTES (FRAME_PDU_MAX_BITS / 8)

#define FRAME_PDU_FLAG_CHASE 0x01 // Chase-2 soft BCH rescued a block (#112)

typedef struct {
    uint64_t timestamp_us; // emit time (esp_timer), receiver-local
    uint32_t source_id;    // receiver id (low 4 bytes of STA MAC)
    int32_t  rel_freq_hz;  // burst centre vs receiver LO
    float    peak_snr_db;
    int16_t  peak_bin;
    uint16_t n_bits;    // valid bits in bits_packed
    uint8_t  direction; // 0 = DL, 1 = UL
    int8_t   bch_e1;    // 0..2 corrected, -1 fail
    int8_t   bch_e2;
    uint8_t  flags; // FRAME_PDU_FLAG_*
    uint8_t  bits_packed[FRAME_PDU_BITS_BYTES];
} iridium_frame_pdu_t;

// Fixed little-endian wire layout (must match frame_pdu_pack exactly):
//   u64 ts, u32 src, i32 freq, f32 snr, i16 bin, u16 n_bits,
//   u8 dir, i8 e1, i8 e2, u8 flags, then FRAME_PDU_BITS_BYTES of bits.
#define FRAME_PDU_WIRE_SIZE (8 + 4 + 4 + 4 + 2 + 2 + 1 + 1 + 1 + 1 + FRAME_PDU_BITS_BYTES)

// Serialize/deserialize to a fixed LE byte buffer. Return bytes
// written/read, or 0 on bad args / short buffer.
size_t frame_pdu_pack(const iridium_frame_pdu_t *pdu, uint8_t *buf, size_t buflen);
size_t frame_pdu_unpack(const uint8_t *buf, size_t buflen, iridium_frame_pdu_t *pdu);

// 0/1-per-byte decoder bits <-> packed PDU bits. pack clamps n_bits to
// FRAME_PDU_MAX_BITS and zero-fills the remainder; unpack writes exactly
// pdu->n_bits bytes into bits01_out (caller-sized >= n_bits).
void frame_pdu_pack_bits(iridium_frame_pdu_t *pdu, const uint8_t *bits01, int n_bits);
void frame_pdu_unpack_bits(const iridium_frame_pdu_t *pdu, uint8_t *bits01_out);

#ifdef ESP_PLATFORM
#include "esp_err.h"

// Worker output queue (PSRAM ring). Single producer (worker_emit_frame on
// Core 1), single consumer (SPI tx in phase 3, or the in-process
// aggregator in COMBINED_LOOPBACK). Idempotent init.
esp_err_t frame_pdu_queue_init(void);

// This receiver's id (low 4 bytes of the STA MAC), valid after init.
uint32_t frame_pdu_source_id(void);

// Non-blocking push from the worker hot path: drops (and counts) the PDU
// if the ring is full so the worker never stalls. Returns true if queued.
bool frame_pdu_queue_push(const iridium_frame_pdu_t *pdu);

// Consumer pop (blocks up to timeout_ms). Returns true if a PDU was read.
bool frame_pdu_queue_pop(iridium_frame_pdu_t *pdu, uint32_t timeout_ms);

uint32_t frame_pdu_queue_dropped(void);
#endif // ESP_PLATFORM
