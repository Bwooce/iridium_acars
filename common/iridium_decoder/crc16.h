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

#ifdef __cplusplus
}
#endif

#endif // IRIDIUM_CRC16_H
