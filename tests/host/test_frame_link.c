// Host test for the inter-chip frame_link wire framing (#136 Phase 3).
// Exercises the PURE layer only (crc16 + encode/decode); the SPI transport
// is target-only and validated on-device via the jumpered loopback
// self-test (CONFIG_FRAME_LINK_LOOPBACK_SELFTEST).

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "frame_link.h"

static int passed = 0, failed = 0;
#define CHECK(c, ...)                                    \
    do {                                                 \
        if (!(c)) {                                      \
            fprintf(stderr, "FAIL line %d: ", __LINE__); \
            fprintf(stderr, __VA_ARGS__);                \
            fprintf(stderr, "\n");                       \
            failed++;                                    \
        } else {                                         \
            passed++;                                    \
        }                                                \
    } while (0)

static void fill_pdu(iridium_frame_pdu_t *p)
{
    memset(p, 0, sizeof(*p));
    p->timestamp_us = 0x0123456789ABCDEFull;
    p->source_id    = 0xDEADBEEFu;
    p->rel_freq_hz  = -482865;
    p->peak_snr_db  = 13.72f;
    p->peak_bin     = 1040;
    p->direction    = 1;
    p->bch_e1       = 2;
    p->bch_e2       = -1;
    p->flags        = FRAME_PDU_FLAG_CHASE;
    p->n_bits       = 382;
    for (int i = 0; i < FRAME_PDU_BITS_BYTES; i++)
        p->bits_packed[i] = (uint8_t)(i * 31 + 7);
}

int main(void)
{
    // 1) CRC-16/CCITT-FALSE canonical vector: crc("123456789") == 0x29B1.
    CHECK(frame_link_crc16((const uint8_t *)"123456789", 9) == 0x29B1,
          "crc16 check value = 0x%04X", frame_link_crc16((const uint8_t *)"123456789", 9));

    // 2) encode -> decode round-trip preserves every field.
    iridium_frame_pdu_t a;
    fill_pdu(&a);
    uint8_t buf[FRAME_LINK_FRAME_SIZE];
    CHECK(frame_link_encode(&a, buf, sizeof(buf)) == FRAME_LINK_FRAME_SIZE, "encode size");
    CHECK(buf[0] == (FRAME_LINK_MAGIC & 0xFF) && buf[1] == (FRAME_LINK_MAGIC >> 8), "magic");
    CHECK(buf[2] == FRAME_LINK_VER, "version byte");

    iridium_frame_pdu_t b;
    CHECK(frame_link_decode(buf, sizeof(buf), &b), "decode valid frame");
    CHECK(b.timestamp_us == a.timestamp_us, "ts");
    CHECK(b.source_id == a.source_id, "src");
    CHECK(b.rel_freq_hz == a.rel_freq_hz, "freq");
    CHECK(b.peak_snr_db == a.peak_snr_db, "snr");
    CHECK(b.peak_bin == a.peak_bin, "bin");
    CHECK(b.direction == a.direction, "dir");
    CHECK(b.bch_e1 == a.bch_e1 && b.bch_e2 == a.bch_e2, "bch");
    CHECK(b.flags == a.flags, "flags");
    CHECK(b.n_bits == a.n_bits, "n_bits");
    CHECK(memcmp(a.bits_packed, b.bits_packed, FRAME_PDU_BITS_BYTES) == 0, "bits");

    // 3) every single-byte corruption in the frame must be rejected by CRC
    //    (or the magic/ver checks). No silent acceptance of altered bytes.
    int undetected = 0;
    for (size_t i = 0; i < FRAME_LINK_FRAME_SIZE; i++) {
        uint8_t saved = buf[i];
        buf[i] ^= 0xFF;
        iridium_frame_pdu_t junk;
        if (frame_link_decode(buf, sizeof(buf), &junk)) undetected++;
        buf[i] = saved;
    }
    CHECK(undetected == 0, "%d single-byte corruptions slipped past CRC", undetected);

    // 4) bad magic / bad version / short buffer rejected.
    frame_link_encode(&a, buf, sizeof(buf));
    uint8_t save = buf[0];
    buf[0]       = 0x00;
    CHECK(!frame_link_decode(buf, sizeof(buf), &b), "bad magic accepted");
    buf[0] = save;
    buf[2] = FRAME_LINK_VER + 1;
    CHECK(!frame_link_decode(buf, sizeof(buf), &b), "bad version accepted");
    frame_link_encode(&a, buf, sizeof(buf));
    CHECK(!frame_link_decode(buf, FRAME_LINK_FRAME_SIZE - 1, &b), "short buffer accepted");
    CHECK(frame_link_encode(&a, buf, FRAME_LINK_FRAME_SIZE - 1) == 0, "short encode not rejected");

    // 5) PDU with oversized n_bits (> FRAME_PDU_MAX_BITS) is rejected even with
    //    valid CRC. This is the security fix for out-of-bounds read vulnerability.
    //    We manually craft a wire frame with n_bits > FRAME_PDU_MAX_BITS and recalculate CRC.
    iridium_frame_pdu_t c;
    fill_pdu(&c);
    c.n_bits = 2048; // oversized: > FRAME_PDU_MAX_BITS (512)
    uint8_t pdu_buf[FRAME_PDU_WIRE_SIZE];
    frame_pdu_pack(&c, pdu_buf, sizeof(pdu_buf));

    // Manually build the wire frame with correct CRC.
    memset(buf, 0, sizeof(buf));
    buf[0] = (uint8_t)(FRAME_LINK_MAGIC & 0xFF);
    buf[1] = (uint8_t)(FRAME_LINK_MAGIC >> 8);
    buf[2] = FRAME_LINK_VER;
    buf[3] = 0; // flags
    memcpy(buf + FRAME_LINK_HDR_BYTES, pdu_buf, FRAME_PDU_WIRE_SIZE);
    uint16_t crc     = frame_link_crc16(buf + 2, (size_t)(FRAME_LINK_HDR_BYTES - 2) + FRAME_PDU_WIRE_SIZE);
    size_t   crc_off = FRAME_LINK_HDR_BYTES + FRAME_PDU_WIRE_SIZE;
    buf[crc_off]     = (uint8_t)(crc & 0xFF);
    buf[crc_off + 1] = (uint8_t)(crc >> 8);

    // Verify CRC is valid (sanity check).
    uint16_t crc_check = frame_link_crc16(buf + 2, (size_t)(FRAME_LINK_HDR_BYTES - 2) + FRAME_PDU_WIRE_SIZE);
    CHECK(crc_check == crc, "CRC sanity check failed");

    // Now decode and verify it's rejected because n_bits > FRAME_PDU_MAX_BITS.
    iridium_frame_pdu_t oversized;
    CHECK(!frame_link_decode(buf, sizeof(buf), &oversized),
          "oversized n_bits PDU not rejected (n_bits=%u > max %u)",
          c.n_bits, FRAME_PDU_MAX_BITS);

    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed ? 1 : 0;
}
