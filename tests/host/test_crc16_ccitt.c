// test_crc16_ccitt.c — pins the table-driven crc16_ccitt_false (crc16.c)
// bit-exact against an independent bit-serial reference. Guards the 512-byte
// lookup table + the MSB-first lookup formula against any future corruption.
//
// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, refin=false, refout=false,
// xorout=0. Canonical check value for "123456789" is 0x29B1.

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>

#include "crc16.h" // crc16_ccitt_false — shared table-based impl (crc16.c)

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

// Independent reference: the bit-serial form the table replaced. If the two ever
// disagree, either the table or the lookup formula regressed.
static uint16_t crc_ref_bitserial(const uint8_t *data, size_t n)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
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
    // 1. Canonical check value.
    CHECK(crc16_ccitt_false((const uint8_t *)"123456789", 9) == 0x29B1u,
          "canonical: got 0x%04X, want 0x29B1", crc16_ccitt_false((const uint8_t *)"123456789", 9));

    // 2. Empty input == init value.
    CHECK(crc16_ccitt_false(NULL, 0) == 0xFFFFu, "empty input must be 0xFFFF");

    // 3. Bit-exact vs the bit-serial reference over 50000 random buffers,
    //    lengths 0..64 (covers the ~23-byte SBD payloads and beyond).
    int mism = 0;
    for (int t = 0; t < 50000; t++) {
        uint8_t buf[64];
        int     n = rnd() & 63;
        for (int i = 0; i < n; i++) buf[i] = rnd();
        if (crc16_ccitt_false(buf, (size_t)n) != crc_ref_bitserial(buf, (size_t)n)) {
            if (mism < 3)
                fprintf(stderr, "  table != bitserial at n=%d\n", n);
            mism++;
        }
    }
    CHECK(mism == 0, "%d/50000 table-vs-bitserial mismatches", mism);

    // 4. CCITT-FALSE residual property: appending the CRC (MSB first) makes the
    //    CRC over the whole message == 0. This is exactly what ida_decode relies
    //    on (da_crc_computed == 0 when the frame's CRC field is included).
    {
        uint8_t msg[26];
        for (int i = 0; i < 24; i++) msg[i] = rnd();
        uint16_t c = crc16_ccitt_false(msg, 24);
        msg[24]    = (uint8_t)(c >> 8);
        msg[25]    = (uint8_t)(c & 0xFF);
        CHECK(crc16_ccitt_false(msg, 26) == 0x0000u,
              "residual: crc(data||crc) must be 0, got 0x%04X", crc16_ccitt_false(msg, 26));
    }

    printf("\n=== %d passed, %d failed ===\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
