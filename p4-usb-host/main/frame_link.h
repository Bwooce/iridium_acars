// Inter-chip PDU transport (#119 / #136 Phase 3). Carries decoded-frame
// PDUs from a worker P4 (SPI slave) to the aggregator P4 (SPI master) over
// a dedicated GPSPI link, leaving the C6 on its own SDIO bus untouched.
//
// Topology (per worker link):
//   worker  : frame_pdu output queue -> frame_link slave -> SPI MISO
//   aggregator: SPI master clocks the frame -> frame_pdu input queue
//               -> aggregator_ingest -> frame_decoder
//
// The slave raises a handshake GPIO when it has a frame loaded; the master
// waits on that line, then clocks exactly one fixed-size wire frame.
//
// COMBINED_LOOPBACK does NOT use this module — there the worker pushes
// straight into the local frame_pdu queue that aggregator_ingest drains.
//
// === Wire framing (pure, host-testable) ===
// A wire frame is fixed length:
//   [u16 magic LE][u8 ver][u8 flags][FRAME_PDU_WIRE_SIZE payload][u16 crc16]
// crc16 is CRC-16/CCITT-FALSE over ver..payload (everything between magic
// and the crc field). The SPI link is short but not noise-free; the CRC
// catches bit slips / clock glitches that BCH on the carried bits would
// otherwise have to absorb.

#pragma once

#include "frame_pdu.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FRAME_LINK_MAGIC 0x1D5A // "ID" + 'Z'-ish; LE on the wire
#define FRAME_LINK_VER 1
#define FRAME_LINK_HDR_BYTES 4 // magic(2) + ver(1) + flags(1)
#define FRAME_LINK_CRC_BYTES 2
#define FRAME_LINK_FRAME_SIZE \
    (FRAME_LINK_HDR_BYTES + FRAME_PDU_WIRE_SIZE + FRAME_LINK_CRC_BYTES)

// SPI DMA transfer size: padded to 64-byte cache-line boundary for P4 alignment
// and to satisfy ESP-IDF SPI-DMA requirement that rx lengths be multiples of 4.
// FRAME_LINK_FRAME_SIZE (98 bytes) is NOT a multiple of 4; this xfer size is.
#define FRAME_LINK_XFER_SIZE ((FRAME_LINK_FRAME_SIZE + 63) / 64 * 64)

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection/xorout).
uint16_t frame_link_crc16(const uint8_t *data, size_t len);

// Encode a PDU into a fixed-size wire frame. buf must be >=
// FRAME_LINK_FRAME_SIZE. Returns bytes written (FRAME_LINK_FRAME_SIZE) or
// 0 on bad args.
size_t frame_link_encode(const iridium_frame_pdu_t *pdu, uint8_t *buf, size_t buflen);

// Decode + validate a wire frame back into a PDU. Returns true only if the
// magic, version, length and CRC all check out. On any failure returns
// false and leaves *pdu untouched.
bool frame_link_decode(const uint8_t *buf, size_t buflen, iridium_frame_pdu_t *pdu);

// === Stats ===
typedef struct {
    uint32_t frames_tx;  // slave: wire frames clocked out to the master
    uint32_t frames_rx;  // master: wire frames received + CRC-valid
    uint32_t crc_errors; // master: frames dropped on bad magic/ver/CRC
    uint32_t bus_errors; // spi transaction failures (either side)
    uint32_t queue_full; // master: received PDU dropped (input queue full)
} frame_link_stats_t;

void frame_link_get_stats(frame_link_stats_t *out);

#ifdef ESP_PLATFORM
#include "esp_err.h"

// Worker role: bring up the SPI slave + handshake GPIO and spawn the task
// that drains the local frame_pdu output queue onto the link. Idempotent.
// frame_pdu_queue_init() must have run first.
esp_err_t frame_link_slave_start(void);

// Aggregator role: bring up the SPI master + handshake input and spawn the
// task that clocks frames from the worker and pushes them into the local
// frame_pdu input queue. Idempotent. frame_pdu_queue_init() must have run.
esp_err_t frame_link_master_start(void);

// One-board bring-up self-test (#136): requires the GPSPI master pins
// jumpered to the slave pins (see Kconfig FRAME_LINK_*_GPIO). Brings up
// both ends, pushes a known PDU through the slave, clocks it on the
// master, and verifies it round-trips byte-for-byte. Returns ESP_OK on a
// clean round-trip. Gated by CONFIG_FRAME_LINK_LOOPBACK_SELFTEST.
esp_err_t frame_link_loopback_selftest(void);
#endif // ESP_PLATFORM

#ifdef __cplusplus
}
#endif
