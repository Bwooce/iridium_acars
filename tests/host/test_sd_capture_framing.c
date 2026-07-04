// Host regression for the atomic burst-record framing decision (T4:
// sd_capture_record_burst_begin now checks whether the WHOLE record
// (header + IQ) fits in the stream buffer's free space BEFORE sending
// anything, and drops the whole record -- zero bytes enqueued -- if
// not, rather than risking a torn header or truncated IQ tail that
// would desync the "header + length_samples*4 bytes" framing for
// every burst after it in the file. sd_capture.c is device-only
// (FreeRTOS stream buffers / SDMMC) and doesn't build on the host, so
// this test exercises the pure size/fit arithmetic pulled out into
// sd_capture_framing.h -- the same functions
// sd_capture_record_burst_begin() calls for its size computation and
// fit decision.

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "sd_capture_framing.h"

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

// (a) Record byte size = header + length_samples * 2 * sizeof(int16_t)
// (4 bytes per complex sample: 2 B I + 2 B Q).
static void test_record_bytes_arithmetic(void)
{
    const size_t hdr = 40; // sd_capture_burst_hdr_t is fixed at 40 bytes

    struct {
        uint32_t length_samples;
    } cases[] = {
        {0},
        {1},
        {100},
        {2500},       // ~1 ms at 2.5 MSPS
        {2500 * 160}, // ~160 ms burst, a large real-world record
        {0xFFFFu},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        size_t got  = sd_capture_burst_record_bytes(hdr, cases[i].length_samples);
        size_t want = hdr + (size_t)cases[i].length_samples * 4;
        CHECK(got == want, "case %zu: record_bytes(%zu,%u)=%zu want %zu",
              i, hdr, cases[i].length_samples, got, want);
    }
}

// (b) A record that fits (free >= size) is accepted; one that doesn't
// (free < size) is rejected. Exercised across a spread of hdr sizes
// and lengths, not just the real 40-byte header, to pin down the
// general comparison rather than one magic number.
static void test_fits_accepts_and_rejects(void)
{
    struct {
        size_t   free_space;
        size_t   hdr_size;
        uint32_t length_samples;
        bool     want_fits;
    } cases[] = {
        // Comfortably fits.
        {1 << 20, 40, 2500, true},
        // Comfortably too big.
        {1024, 40, 2500, false},
        // Zero-length record (header only) fits in exactly hdr bytes.
        {40, 40, 0, true},
        // Zero-length record one byte short of the header.
        {39, 40, 0, false},
        // free_space == 0, any nonzero record rejected.
        {0, 40, 1, false},
        // free_space == 0, hdr_size == 0, zero-length record fits
        // (degenerate all-zero record: 0 >= 0).
        {0, 0, 0, true},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        bool got = sd_capture_burst_fits(cases[i].free_space, cases[i].hdr_size,
                                         cases[i].length_samples);
        CHECK(got == cases[i].want_fits,
              "case %zu: fits(free=%zu,hdr=%zu,len=%u)=%d want %d",
              i, cases[i].free_space, cases[i].hdr_size,
              cases[i].length_samples, (int)got, (int)cases[i].want_fits);
    }
}

// (c) TEETH: the atomic-or-nothing invariant at its exact boundary.
// A record that is EXACTLY the free space fits; the same record with
// free space one byte SHORT does not -- proving the comparison is a
// true "whole record or nothing" test, not an off-by-one that would
// let a record be accepted when only PART of it (e.g. just the
// header) would actually have room. This models the real bug: T4's
// predecessor checked nothing and let xStreamBufferSend's own partial-
// enqueue behaviour decide, silently tearing the header/IQ boundary.
static void test_exact_boundary_all_or_nothing(void)
{
    const size_t   hdr            = 40;
    const uint32_t length_samples = 12345;
    size_t         record_size    = sd_capture_burst_record_bytes(hdr, length_samples);

    // Exactly enough room: accepted.
    CHECK(sd_capture_burst_fits(record_size, hdr, length_samples),
          "record of exactly %zu free bytes should fit", record_size);

    // One byte short: the WHOLE record must be rejected -- there is
    // no partial-accept path in this model, matching the production
    // fix's all-or-nothing framing contract.
    CHECK(!sd_capture_burst_fits(record_size - 1, hdr, length_samples),
          "record one byte short of %zu must be rejected whole, not partially accepted",
          record_size);

    // One byte more than needed: still accepted (never a false
    // rejection of a record that does fit).
    CHECK(sd_capture_burst_fits(record_size + 1, hdr, length_samples),
          "record with one spare byte should still fit");
}

int main(void)
{
    test_record_bytes_arithmetic();
    test_fits_accepts_and_rejects();
    test_exact_boundary_all_or_nothing();

    fprintf(stderr, "sd_capture_framing: %d passed, %d failed\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
