#ifndef IRIDIUM_CRC16_H
#define IRIDIUM_CRC16_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// CRC-16/CCITT-FALSE (a.k.a. CRC-16/IBM-3740):
//   poly = 0x1021, init = 0xFFFF, refin = false, refout = false, xorout = 0.
// Canonical check value for "123456789" is 0x29B1.
//
// Single shared implementation for the whole tree (byte-wise table lookup,
// 512-byte const table in flash .rodata — 0 RAM). Users:
//   - ida_decode.c: DA payload CRC (matches crcmod's "crc-ccitt-false" used
//     by iridium-toolkit/bitsparser.py:IridiumDAMessage)
//   - frame_link.c: inter-chip SPI wire framing (frame_link_crc16 wrapper)
// Pinned bit-exact by tests/host/test_crc16_ccitt.c.
//
// Deliberately ZERO dependencies beyond stdint/stddef so frame_link.c's
// "pure wire framing, no RTOS/ESP deps" property is preserved.
uint16_t crc16_ccitt_false(const uint8_t *data, size_t n_bytes);

// CRC-16/X-25 (a.k.a. CRC-16/IBM-SDLC) — the ISO/IEC 13239 HDLC FCS used by
// VDL Mode 2 AVLC frames (common/vdl2/avlc.c):
//   poly = 0x1021, init = 0xFFFF, refin = true, refout = true,
//   xorout = 0xFFFF.  Canonical check value for "123456789" is 0x906E.
// Same generator polynomial as crc16_ccitt_false above, opposite bit order +
// final complement — the two are NOT interchangeable.
//
// crc16_x25() returns the FCS to transmit: append low byte first, then high
// byte (HDLC octets are serialised LSB-first on air).
//
// crc16_x25_raw() is the accumulator WITHOUT the final complement, provided
// for the classic HDLC receive check: running it over a frame INCLUDING its
// two trailing FCS octets yields the fixed "good residue" 0xF0B8 iff the
// frame is intact. This is exactly dumpvdl2's verdict form
// (src/avlc.c:40,177-179 `GOOD_FCS 0xF0B8`, v2.6.0 3f583da) — matching it
// keeps our AVLC deframer cross-validatable line-for-line.
// Table + both entry points pinned by tests/host/test_crc16_x25.c.
#define CRC16_X25_GOOD_RESIDUE 0xF0B8u
uint16_t crc16_x25(const uint8_t *data, size_t n_bytes);
uint16_t crc16_x25_raw(const uint8_t *data, size_t n_bytes);

#ifdef __cplusplus
}
#endif

#endif // IRIDIUM_CRC16_H
