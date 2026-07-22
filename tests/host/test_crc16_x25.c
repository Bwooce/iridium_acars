// test_crc16_x25.c — pins the table-driven CRC-16/X-25 pair (crc16.c
// crc16_x25 / crc16_x25_raw) bit-exact against an independent bit-serial
// reference, the canonical check vector, and the HDLC "good residue"
// convention the VDL2 AVLC deframer (common/vdl2/avlc.c) relies on.
//
// CRC-16/X-25: poly 0x1021, init 0xFFFF, refin=true, refout=true,
// xorout=0xFFFF. Canonical check value for "123456789" is 0x906E; the
// residue of a frame including its FCS is 0xF0B8 — dumpvdl2's GOOD_FCS
// (src/avlc.c:40, v2.6.0 3f583da), whose crc16_ccitt (src/crc.c:21-64) is
// the same reflected algorithm as crc16_x25_raw.

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "crc16.h"

static int s_pass = 0, s_fail = 0;
#define CHECK(cond, ...)                                   \
    do {                                                   \
        if (!(cond)) {                                     \
            fprintf(stderr, "  FAIL line %d: ", __LINE__); \
            fprintf(stderr, __VA_ARGS__);                  \
            fprintf(stderr, "\n");                         \
            s_fail++;                                      \
        } else {                                           \
            s_pass++;                                      \
        }                                                  \
    } while (0)

// Independent reference: bit-serial reflected form (poly 0x8408 =
// bit-reversed 0x1021), no final complement. If the table or the lookup
// formula in crc16.c ever regresses, this diverges.
static uint16_t crc_ref_bitserial_raw(const uint8_t *data, size_t n)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < n; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0x8408u)
                             : (uint16_t)(crc >> 1);
    }
    return crc;
}

// Deterministic LCG so the corpus is identical on every platform.
static uint32_t s_rng = 0xC0FFEEu;
static uint8_t rnd(void)
{
    s_rng = s_rng * 1664525u + 1013904223u;
    return (uint8_t)(s_rng >> 24);
}

int main(void)
{
    // 1. Canonical check value (final, xorout applied).
    CHECK(crc16_x25((const uint8_t *)"123456789", 9) == 0x906Eu,
          "canonical: got 0x%04X, want 0x906E",
          crc16_x25((const uint8_t *)"123456789", 9));

    // 2. Empty input: raw == init, final == ~init.
    CHECK(crc16_x25_raw(NULL, 0) == 0xFFFFu, "empty raw must be 0xFFFF");
    CHECK(crc16_x25(NULL, 0) == 0x0000u, "empty final must be 0x0000");

    // 3. crc16_x25 == crc16_x25_raw ^ 0xFFFF by definition, and both are
    //    bit-exact vs the bit-serial reference over 50000 random buffers.
    int mism = 0;
    for (int t = 0; t < 50000; t++) {
        uint8_t buf[64];
        int     n = rnd() & 63;
        for (int i = 0; i < n; i++) buf[i] = rnd();
        uint16_t raw = crc16_x25_raw(buf, (size_t)n);
        if (raw != crc_ref_bitserial_raw(buf, (size_t)n) ||
            crc16_x25(buf, (size_t)n) != (uint16_t)(raw ^ 0xFFFFu)) {
            if (mism < 3) fprintf(stderr, "  mismatch at n=%d\n", n);
            mism++;
        }
    }
    CHECK(mism == 0, "%d/50000 table-vs-bitserial mismatches", mism);

    // 4. HDLC residue property — the exact receive check avlc.c performs:
    //    append the FCS low byte first (HDLC LSB-first serialisation), then
    //    the raw CRC over the whole frame is the fixed good residue 0xF0B8.
    //    Flipping any single bit must destroy it.
    {
        uint8_t msg[32];
        for (int i = 0; i < 30; i++) msg[i] = rnd();
        uint16_t fcs = crc16_x25(msg, 30);
        msg[30]      = (uint8_t)(fcs & 0xFF);
        msg[31]      = (uint8_t)(fcs >> 8);
        CHECK(crc16_x25_raw(msg, 32) == CRC16_X25_GOOD_RESIDUE,
              "residue: got 0x%04X, want 0xF0B8", crc16_x25_raw(msg, 32));
        int broken = 0;
        for (int bit = 0; bit < 32 * 8; bit++) {
            msg[bit >> 3] ^= (uint8_t)(1u << (bit & 7));
            if (crc16_x25_raw(msg, 32) != CRC16_X25_GOOD_RESIDUE) broken++;
            msg[bit >> 3] ^= (uint8_t)(1u << (bit & 7));
        }
        CHECK(broken == 32 * 8,
              "single-bit flips undetected: %d/256 caught", broken);
    }

    // 5. Guard against variant aliasing: the reflected X-25 pair and the
    //    non-reflected CCITT-FALSE sibling must NOT agree (they share the
    //    generator polynomial only).
    {
        const uint8_t *v = (const uint8_t *)"123456789";
        CHECK(crc16_x25(v, 9) != crc16_ccitt_false(v, 9),
              "x25 and ccitt_false must differ on the canonical vector");
    }

    printf("\n=== %d passed, %d failed ===\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
