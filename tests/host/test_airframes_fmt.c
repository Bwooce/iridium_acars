// test_airframes_fmt.c — pins the airframes.io wire-JSON formatter
// (common/acars_feed/airframes_fmt.c) BYTE-EXACT for both schemas:
//
//   1. VDL2 fixture: a real dumpvdl2 2.6.0 sample (t.sec 1785148910,
//      136.975 MHz, AVLC 390826 -> 26B117, reg .F-GCBG) — full-string
//      strcmp, the correctness anchor for the dumpvdl2-native shape.
//   2. Iridium fixture: iridium-toolkit reassembler.py -m acars -a json
//      shape — full-string strcmp, incl. the tail dot-strip asymmetry
//      (VDL2 keeps ".F-GCBG", Iridium emits "F-GCBG") and the ISO8601
//      "+0000" timestamp.
//   3. Truncation: EVERY cap <= needed must return 0 (never a torn
//      message on the wire); cap == needed+1 must reproduce the full
//      string.
//   4. JSON escaping: quote, backslash, \n, and a raw control byte
//      (0x01 -> \\u0001) in the payload text.
//   5. Omission rules: unsynced time / missing station / zero blk_id /
//      etc. are OMITTED (never emitted empty); NAK ack (0x15) -> "!";
//      Iridium label "_\x7f" -> "_d".

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "airframes_fmt.h"

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

// Byte-exact compare with a first-divergence dump — a bare strcmp fail
// on a 400-char JSON line is undebuggable otherwise.
static void check_exact(const char *got, const char *want, const char *what)
{
    if (strcmp(got, want) == 0) {
        s_pass++;
        return;
    }
    size_t i = 0;
    while (got[i] && want[i] && got[i] == want[i]) i++;
    fprintf(stderr, "  FAIL %s: diverges at byte %zu\n", what, i);
    fprintf(stderr, "    got : %s\n", got);
    fprintf(stderr, "    want: %s\n", want);
    s_fail++;
}

// Shared ACARS fields of the dumpvdl2 reference sample (also reused for
// the Iridium fixture so the reg-dot asymmetry is visible on identical
// input).
static af_msg_t vdl2_fixture(void)
{
    af_msg_t m;
    memset(&m, 0, sizeof m);
    m.band       = AF_BAND_VDL2;
    m.epoch_us   = 1785148910LL * 1000000 + 428657; // t.sec/t.usec of the sample
    m.time_valid = true;
    m.uplink     = false;
    m.crc_ok     = true;
    m.err        = false;
    m.more       = false;
    m.mode       = '2';
    strcpy(m.label, "2T");
    m.block_id    = '7';
    strcpy(m.msg_num, "M06");
    m.msg_num_seq = 'A';
    strcpy(m.flight_id, "AF0000");
    strcpy(m.reg, ".F-GCBG");
    m.ack = '!';
    m.txt = "VER/038/B747/M\r\nSCH/AFR6748/LFPG/OKBK/05JAN/1030\r\nROT";
    m.freq_hz         = 136975000;
    m.sig_level_dbfs  = -11.016439f;
    m.sig_level_valid = true;
    strcpy(m.src_addr, "390826");
    strcpy(m.dst_addr, "26B117");
    m.src_type   = "Aircraft";
    m.dst_type   = "Ground station";
    m.src_status = "Airborne";
    m.station_id = "TEST-STN-1";
    return m;
}

int main(void)
{
    char out[1024];

    // 1. VDL2 — byte-exact vs the dumpvdl2 2.6.0 native shape.
    {
        af_msg_t m = vdl2_fixture();
        size_t   n = airframes_format(out, sizeof out, &m);
        CHECK(n > 0, "vdl2 format returned 0");
        CHECK(n == strlen(out), "vdl2 return %zu != strlen %zu", n,
              strlen(out));
        const char *want =
            "{\"vdl2\":{\"app\":{\"name\":\"dumpvdl2\",\"ver\":\"2.6.0\"},"
            "\"station\":\"TEST-STN-1\","
            "\"t\":{\"sec\":1785148910,\"usec\":428657},"
            "\"freq\":136975000,"
            "\"sig_level\":-11.02,"
            "\"avlc\":{"
            "\"src\":{\"addr\":\"390826\",\"type\":\"Aircraft\","
            "\"status\":\"Airborne\"},"
            "\"dst\":{\"addr\":\"26B117\",\"type\":\"Ground station\"},"
            "\"cr\":\"Command\",\"frame_type\":\"I\","
            "\"acars\":{\"err\":false,\"crc_ok\":true,\"more\":false,"
            "\"reg\":\".F-GCBG\",\"mode\":\"2\",\"label\":\"2T\","
            "\"blk_id\":\"7\",\"ack\":\"!\",\"flight\":\"AF0000\","
            "\"msg_num\":\"M06\",\"msg_num_seq\":\"A\","
            "\"msg_text\":\"VER/038/B747/M\\r\\nSCH/AFR6748/LFPG/OKBK/"
            "05JAN/1030\\r\\nROT\"}}}}";
        check_exact(out, want, "vdl2 fixture");
    }

    // 2. Iridium — byte-exact vs the iridium-toolkit shape. Same ACARS
    //    fields; note tail "F-GCBG" (dot stripped) vs reg ".F-GCBG" kept
    //    by the VDL2 schema above, and message_number without the seq
    //    char (the Iridium schema has no msg_num_seq member).
    {
        af_msg_t m   = vdl2_fixture();
        m.band       = AF_BAND_IRIDIUM;
        m.epoch_us   = 1767609000LL * 1000000; // 2026-01-05T10:30:00 UTC
        m.txt        = "AGFSR QF0164/05 OK";
        size_t n     = airframes_format(out, sizeof out, &m);
        CHECK(n > 0, "iridium format returned 0");
        const char *want =
            "{\"app\":{\"name\":\"iridium-toolkit\",\"version\":\"0.0.1\"},"
            "\"source\":{\"transport\":\"iridium\",\"protocol\":\"acars\","
            "\"station_id\":\"TEST-STN-1\"},"
            "\"acars\":{"
            "\"timestamp\":\"2026-01-05T10:30:00+0000\","
            "\"errors\":0,"
            "\"link_direction\":\"downlink\","
            "\"block_end\":true,"
            "\"mode\":\"2\",\"tail\":\"F-GCBG\",\"flight\":\"AF0000\","
            "\"label\":\"2T\",\"block_id\":\"7\","
            "\"message_number\":\"M06\",\"ack\":\"!\","
            "\"text\":\"AGFSR QF0164/05 OK\"}}";
        check_exact(out, want, "iridium fixture");
    }

    // 3. Truncation: every cap from 0 to the exact needed length must
    //    return 0 (a torn JSON line must never reach the wire); needed+1
    //    (room for the NUL) must reproduce the full string.
    {
        af_msg_t m    = vdl2_fixture();
        size_t   need = airframes_format(out, sizeof out, &m);
        CHECK(need > 0 && need + 1 < sizeof out, "fixture too big for harness");
        char small[1024];
        int  bad = 0;
        for (size_t cap = 0; cap <= need; cap++) {
            if (airframes_format(small, cap, &m) != 0) bad++;
        }
        CHECK(bad == 0, "%d truncated caps returned nonzero", bad);
        size_t n = airframes_format(small, need + 1, &m);
        CHECK(n == need, "exact-fit cap: got %zu want %zu", n, need);
        check_exact(small, out, "exact-fit reproduction");
        CHECK(airframes_format(NULL, 64, &m) == 0, "NULL out must return 0");
        CHECK(airframes_format(out, sizeof out, NULL) == 0,
              "NULL msg must return 0");
    }

    // 4. JSON escaping in the payload: quote, \n, backslash, raw 0x01.
    {
        af_msg_t m = vdl2_fixture();
        m.band     = AF_BAND_IRIDIUM;
        // NB "\x01" "e" spliced: a single "\x01e" literal would parse as
        // hex 0x1E (C hex escapes are maximal-munch).
        m.txt      = "a\"b\nc\\d\x01" "e";
        size_t n   = airframes_format(out, sizeof out, &m);
        CHECK(n > 0, "escape-fixture format returned 0");
        CHECK(strstr(out, "\"text\":\"a\\\"b\\nc\\\\d\\u0001e\"") != NULL,
              "escaped text wrong: %s", out);
    }

    // 5. Omission + special-byte rules.
    {
        af_msg_t m        = vdl2_fixture();
        m.time_valid      = false;
        m.station_id      = NULL;
        m.freq_hz         = 0;
        m.sig_level_valid = false;
        m.block_id        = 0;
        m.msg_num_seq     = 0;
        m.txt             = NULL;
        m.src_type        = NULL;
        m.src_status      = NULL;
        m.dst_addr[0]     = '\0';
        m.ack             = 0x15; // NAK
        size_t n          = airframes_format(out, sizeof out, &m);
        CHECK(n > 0, "omission-fixture format returned 0");
        CHECK(strstr(out, "\"t\":") == NULL, "unsynced time must omit t");
        CHECK(strstr(out, "station") == NULL, "NULL station must be omitted");
        CHECK(strstr(out, "freq") == NULL, "freq 0 must be omitted");
        CHECK(strstr(out, "sig_level") == NULL,
              "invalid sig_level must be omitted");
        CHECK(strstr(out, "blk_id") == NULL, "blk_id 0 must be omitted");
        CHECK(strstr(out, "msg_num_seq") == NULL,
              "msg_num_seq 0 must be omitted");
        CHECK(strstr(out, "msg_text") == NULL, "NULL txt must be omitted");
        CHECK(strstr(out, "\"type\"") == NULL,
              "NULL src_type must be omitted");
        CHECK(strstr(out, "status") == NULL,
              "NULL src_status must be omitted");
        CHECK(strstr(out, "\"dst\"") == NULL,
              "empty dst_addr must omit dst");
        CHECK(strstr(out, "\"src\":{\"addr\":\"390826\"}") != NULL,
              "src must survive with addr only: %s", out);
        CHECK(strstr(out, "\"ack\":\"!\"") != NULL, "NAK ack must map to !");

        // Iridium side of the same rules + the "_\x7f" label mapping.
        m.band = AF_BAND_IRIDIUM;
        strcpy(m.label, "_\x7f");
        m.reg[0] = '\0'; // no reg -> no tail member at all
        n        = airframes_format(out, sizeof out, &m);
        CHECK(n > 0, "iridium omission-fixture format returned 0");
        CHECK(strstr(out, "timestamp") == NULL,
              "unsynced time must omit timestamp");
        CHECK(strstr(out, "station_id") == NULL,
              "NULL station_id must be omitted");
        CHECK(strstr(out, "tail") == NULL, "empty reg must omit tail");
        CHECK(strstr(out, "\"label\":\"_d\"") != NULL,
              "label _DEL must map to _d: %s", out);
        CHECK(strstr(out, "\"errors\":0") != NULL, "errors must be present");
        CHECK(strstr(out, "\"block_end\":true") != NULL,
              "block_end must be present");
    }

    printf("\n=== %d passed, %d failed ===\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
