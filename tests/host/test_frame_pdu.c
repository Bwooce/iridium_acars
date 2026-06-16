// Host round-trip test for the worker->aggregator decoded-frame PDU
// (#135). Validates pack/unpack of the wire format and the 0/1-per-byte
// <-> packed bit conversion the worker/aggregator rely on.

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "frame_pdu.h"

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

int main(void)
{
    // Deterministic 0/1 bit array (382 bits = the observed max frame len).
    uint8_t  bits01[FRAME_PDU_MAX_BITS];
    int      n_bits = 382;
    uint32_t st     = 0x1234u;
    for (int i = 0; i < n_bits; i++) {
        st        = st * 1103515245u + 12345u;
        bits01[i] = (st >> 20) & 1;
    }

    iridium_frame_pdu_t a = {0};
    a.timestamp_us        = 0x0123456789ABCDEFull;
    a.source_id           = 0xDEADBEEFu;
    a.rel_freq_hz         = -482865;
    a.peak_snr_db         = 13.72f;
    a.peak_bin            = 1040;
    a.direction           = 1;
    a.bch_e1              = 2;
    a.bch_e2              = -1;
    a.flags               = FRAME_PDU_FLAG_CHASE;
    frame_pdu_pack_bits(&a, bits01, n_bits);
    CHECK(a.n_bits == n_bits, "pack_bits n_bits %u != %d", a.n_bits, n_bits);

    uint8_t buf[FRAME_PDU_WIRE_SIZE];
    size_t  w = frame_pdu_pack(&a, buf, sizeof(buf));
    CHECK(w == FRAME_PDU_WIRE_SIZE, "pack wrote %zu != %d", w, FRAME_PDU_WIRE_SIZE);

    iridium_frame_pdu_t b;
    size_t              r = frame_pdu_unpack(buf, sizeof(buf), &b);
    CHECK(r == FRAME_PDU_WIRE_SIZE, "unpack read %zu", r);

    CHECK(b.timestamp_us == a.timestamp_us, "timestamp");
    CHECK(b.source_id == a.source_id, "source_id");
    CHECK(b.rel_freq_hz == a.rel_freq_hz, "rel_freq_hz %d", b.rel_freq_hz);
    CHECK(b.peak_snr_db == a.peak_snr_db, "peak_snr_db %f", (double)b.peak_snr_db);
    CHECK(b.peak_bin == a.peak_bin, "peak_bin %d", b.peak_bin);
    CHECK(b.n_bits == a.n_bits, "n_bits");
    CHECK(b.direction == a.direction, "direction");
    CHECK(b.bch_e1 == a.bch_e1 && b.bch_e2 == a.bch_e2, "bch %d/%d", b.bch_e1, b.bch_e2);
    CHECK(b.flags == a.flags, "flags");
    CHECK(memcmp(a.bits_packed, b.bits_packed, FRAME_PDU_BITS_BYTES) == 0, "bits_packed");

    // 0/1 bits survive the full pack -> wire -> unpack -> unpack_bits path.
    uint8_t out01[FRAME_PDU_MAX_BITS] = {0};
    frame_pdu_unpack_bits(&b, out01);
    CHECK(memcmp(bits01, out01, n_bits) == 0, "bits01 round-trip");

    // A short output buffer must be rejected (return 0), not overrun.
    CHECK(frame_pdu_pack(&a, buf, FRAME_PDU_WIRE_SIZE - 1) == 0, "short-buffer pack not rejected");
    CHECK(frame_pdu_unpack(buf, FRAME_PDU_WIRE_SIZE - 1, &b) == 0, "short-buffer unpack not rejected");

    // n_bits edge cases: 0 and clamp at MAX.
    frame_pdu_pack_bits(&a, bits01, 0);
    CHECK(a.n_bits == 0, "n_bits=0");
    frame_pdu_pack_bits(&a, bits01, FRAME_PDU_MAX_BITS + 10);
    CHECK(a.n_bits == FRAME_PDU_MAX_BITS, "n_bits clamp to MAX");

    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed ? 1 : 0;
}
