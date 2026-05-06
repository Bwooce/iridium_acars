// Unit tests for common/iridium_decoder/iridium_frame.c.
//
// Synthesises bit streams matching each known header pattern, verifies
// the classifier returns the right type. No real-RF data here — the
// regression test against gr-iridium output lives in
// test_iridium_frame_corpus.c.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "iridium_frame.h"

static int passed = 0;
static int failed = 0;

#define ASSERT_EQ(a, b, fmt, ...) do {                                  \
    if ((a) != (b)) {                                                   \
        printf("  FAIL: " fmt " (got %d, expected %d)\n",               \
               ##__VA_ARGS__, (int)(a), (int)(b));                      \
        failed++;                                                       \
        return;                                                         \
    } else {                                                            \
        passed++;                                                       \
    }                                                                   \
} while(0)

#define UW_LEN 24

// Standard DL UW as produced by qpsk_demod's DQPSK + Gray-remap on the
// canonical UW symbols. We don't actually need to match this for the
// classifier — it ignores the UW — but keep it realistic in the fixture
// so the tests look like real demod output.
static const uint8_t UW_DL_BITS[UW_LEN] = {
    0,0, 1,1, 0,0, 0,0, 0,0, 1,1, 0,0, 0,0, 1,1, 1,1, 0,0, 1,1,
};

// 32-bit messaging header (BPSK 0x9669).
static const uint8_t HDR_MS[32] = {
    0,0,1,1, 0,0,1,1, 1,1,1,1, 0,0,1,1,
    0,0,1,1, 0,0,1,1, 1,1,1,1, 0,0,1,1,
};

static void test_too_short(void)
{
    printf("Test: too-short input (< UW length) returns -1\n");
    iridium_frame_t f = { 0 };
    uint8_t bits[10] = { 0 };
    int rc = iridium_frame_classify(bits, 10, IR_FRM_DIR_DOWNLINK, &f);
    ASSERT_EQ(rc, -1, "expected -1");
}

static void test_null(void)
{
    printf("Test: NULL pointers return -1\n");
    iridium_frame_t f = { 0 };
    int rc = iridium_frame_classify(NULL, 191, IR_FRM_DIR_DOWNLINK, &f);
    ASSERT_EQ(rc, -1, "NULL bits");
    rc = iridium_frame_classify((uint8_t*)"", 191, IR_FRM_DIR_DOWNLINK, NULL);
    ASSERT_EQ(rc, -1, "NULL out");
}

static void test_uw_only_unknown(void)
{
    printf("Test: only UW present (no payload) -> UNKNOWN, success\n");
    iridium_frame_t f = { 0 };
    int rc = iridium_frame_classify(UW_DL_BITS, UW_LEN,
                                    IR_FRM_DIR_DOWNLINK, &f);
    ASSERT_EQ(rc, 0, "should succeed");
    ASSERT_EQ(f.type, IR_FRAME_UNKNOWN, "unknown");
    ASSERT_EQ(f.direction, IR_FRM_DIR_DOWNLINK, "DL");
}

static void test_messaging_match(void)
{
    printf("Test: UW + 32-bit messaging header -> IR_FRAME_MS\n");
    uint8_t bits[UW_LEN + 32];
    memcpy(bits,                UW_DL_BITS, UW_LEN);
    memcpy(bits + UW_LEN,       HDR_MS,     32);
    iridium_frame_t f = { 0 };
    int rc = iridium_frame_classify(bits, sizeof(bits),
                                    IR_FRM_DIR_DOWNLINK, &f);
    ASSERT_EQ(rc, 0, "rc");
    ASSERT_EQ(f.type, IR_FRAME_MS, "type");
    ASSERT_EQ(f.payload_off, UW_LEN, "payload_off");
}

static void test_messaging_one_bit_flip_no_match(void)
{
    printf("Test: 1-bit-flipped messaging header -> NOT MS (Phase A is "
           "exact-match; ECC-corrected match is Phase B)\n");
    uint8_t bits[UW_LEN + 32];
    memcpy(bits,                UW_DL_BITS, UW_LEN);
    memcpy(bits + UW_LEN,       HDR_MS,     32);
    bits[UW_LEN + 5] ^= 1;  // flip
    iridium_frame_t f = { 0 };
    int rc = iridium_frame_classify(bits, sizeof(bits),
                                    IR_FRM_DIR_DOWNLINK, &f);
    ASSERT_EQ(rc, 0, "rc");
    ASSERT_EQ(f.type, IR_FRAME_UNKNOWN, "type");
}

static void test_time_location_match(void)
{
    printf("Test: UW + '11' + 94 zeros -> IR_FRAME_TL\n");
    uint8_t bits[UW_LEN + 96];
    memcpy(bits, UW_DL_BITS, UW_LEN);
    memset(bits + UW_LEN, 0, 96);
    bits[UW_LEN + 0] = 1;
    bits[UW_LEN + 1] = 1;
    iridium_frame_t f = { 0 };
    int rc = iridium_frame_classify(bits, sizeof(bits),
                                    IR_FRM_DIR_DOWNLINK, &f);
    ASSERT_EQ(rc, 0, "rc");
    ASSERT_EQ(f.type, IR_FRAME_TL, "type");
}

static void test_time_location_with_extra_one_no_match(void)
{
    printf("Test: TL prefix + a stray 1 in the zero region -> NOT TL\n");
    uint8_t bits[UW_LEN + 96];
    memcpy(bits, UW_DL_BITS, UW_LEN);
    memset(bits + UW_LEN, 0, 96);
    bits[UW_LEN + 0] = 1;
    bits[UW_LEN + 1] = 1;
    bits[UW_LEN + 50] = 1;   // contaminate the all-zero region
    iridium_frame_t f = { 0 };
    int rc = iridium_frame_classify(bits, sizeof(bits),
                                    IR_FRM_DIR_DOWNLINK, &f);
    ASSERT_EQ(rc, 0, "rc");
    ASSERT_EQ(f.type, IR_FRAME_UNKNOWN, "type");
}

static void test_uplink_passthrough(void)
{
    printf("Test: direction is preserved in output\n");
    uint8_t bits[UW_LEN + 32];
    memcpy(bits,          UW_DL_BITS, UW_LEN);
    memcpy(bits + UW_LEN, HDR_MS,     32);
    iridium_frame_t f = { 0 };
    int rc = iridium_frame_classify(bits, sizeof(bits),
                                    IR_FRM_DIR_UPLINK, &f);
    ASSERT_EQ(rc, 0, "rc");
    ASSERT_EQ(f.direction, IR_FRM_DIR_UPLINK, "direction");
}

static void test_type_name(void)
{
    printf("Test: type-name strings are stable\n");
    ASSERT_EQ(strcmp(iridium_frame_type_name(IR_FRAME_MS), "MS"), 0, "MS");
    ASSERT_EQ(strcmp(iridium_frame_type_name(IR_FRAME_TL), "TL"), 0, "TL");
    ASSERT_EQ(strcmp(iridium_frame_type_name(IR_FRAME_BC), "BC"), 0, "BC");
    ASSERT_EQ(strcmp(iridium_frame_type_name(IR_FRAME_LW), "LW"), 0, "LW");
    ASSERT_EQ(strcmp(iridium_frame_type_name(IR_FRAME_UNKNOWN), "??"), 0, "??");
}

int main(void)
{
    test_null();
    test_too_short();
    test_uw_only_unknown();
    test_messaging_match();
    test_messaging_one_bit_flip_no_match();
    test_time_location_match();
    test_time_location_with_extra_one_no_match();
    test_uplink_passthrough();
    test_type_name();
    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
