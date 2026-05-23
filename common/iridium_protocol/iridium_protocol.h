// Inter-chip protocol shared by P4 and C6 firmware.
// See docs/c6-companion-firmware-design.md section 9.
//
// Frame format on the wire:
//   [SOF:1=0xAA][LEN:2 LE][TYPE:1][PAYLOAD:LEN bytes][CRC16:2 LE]
//
// LEN is the payload length, not the whole frame. CRC16-CCITT is
// computed over TYPE + PAYLOAD (LEN+1 bytes). Maximum payload
// 512 bytes.
//
// Type codes < 0x80 are C6→P4 requests; P4 responds with the same
// type code and the response payload.
// Type codes ≥ 0x80 are P4→C6 push events (no response).

#pragma once

#include <stdint.h>

#define IRP_SOF            0xAA
#define IRP_MAX_PAYLOAD    512
#define IRP_FRAME_OVERHEAD 6     // SOF + LEN(2) + TYPE + CRC(2)
#define IRP_MAX_FRAME      (IRP_MAX_PAYLOAD + IRP_FRAME_OVERHEAD)

// Request types (C6 -> P4)
#define IRP_TYPE_PING            0x01
#define IRP_TYPE_NVS_GET         0x10
#define IRP_TYPE_NVS_SET         0x11
#define IRP_TYPE_SDR_RETUNE      0x20
#define IRP_TYPE_SD_LIST         0x30
#define IRP_TYPE_SD_READ         0x31
#define IRP_TYPE_SD_WRITE        0x32

// Push types (P4 -> C6, no response)
#define IRP_TYPE_BOOT_COMPLETE   0x80
#define IRP_TYPE_ACARS_MSG       0x81
#define IRP_TYPE_STATUS_SNAP     0x82

// ----- Payload structs (packed, little-endian) -----

#define IRP_NVS_KEY_LEN   16
#define IRP_NVS_NS_LEN    16
#define IRP_NVS_VAL_LEN   64

typedef struct __attribute__((packed)) {
    uint64_t uptime_ms;
} irp_ping_resp_t;

typedef struct __attribute__((packed)) {
    char ns[IRP_NVS_NS_LEN];
    char key[IRP_NVS_KEY_LEN];
} irp_nvs_get_req_t;

typedef struct __attribute__((packed)) {
    uint8_t  type;             // 0=str, 1=u8, 2=u16, 3=u32, 4=i16, 5=f32, 6=blob
    uint8_t  err;              // 0=ok, !=0 NVS error
    uint16_t actual_len;       // bytes in val[] used
    uint8_t  val[IRP_NVS_VAL_LEN];
} irp_nvs_get_resp_t;

typedef struct __attribute__((packed)) {
    char     ns[IRP_NVS_NS_LEN];
    char     key[IRP_NVS_KEY_LEN];
    uint8_t  type;
    uint16_t actual_len;
    uint8_t  val[IRP_NVS_VAL_LEN];
} irp_nvs_set_req_t;

typedef struct __attribute__((packed)) {
    uint8_t err;
} irp_nvs_set_resp_t;

typedef struct __attribute__((packed)) {
    uint32_t freq_hz;
    uint8_t  gain_mode;
    int16_t  gain_db_x10;
    uint8_t  bias_tee;
} irp_sdr_retune_req_t;

typedef struct __attribute__((packed)) {
    uint8_t err;
} irp_sdr_retune_resp_t;

#define IRP_FW_VER_LEN   16
typedef struct __attribute__((packed)) {
    char fw_ver[IRP_FW_VER_LEN];
} irp_boot_complete_t;

#define IRP_ACARS_LABEL_LEN     2
#define IRP_ACARS_FLIGHT_LEN    6
#define IRP_ACARS_MSGNUM_LEN    4
#define IRP_ACARS_TEXT_LEN    160

typedef struct __attribute__((packed)) {
    uint32_t seq;
    uint64_t ts_us;
    uint8_t  direction;        // 0=DL, 1=UL, 2=unknown
    uint8_t  mode;             // ACARS mode char (e.g. '2')
    char     label[IRP_ACARS_LABEL_LEN];
    char     flight[IRP_ACARS_FLIGHT_LEN];
    char     msg_num[IRP_ACARS_MSGNUM_LEN];
    char     text[IRP_ACARS_TEXT_LEN];
    float    snr_db;
    uint32_t freq_hz;
} irp_acars_msg_t;

typedef struct __attribute__((packed)) {
    float    rate_mb_s;
    float    dsp_cap;
    float    worker_cap;
    uint32_t drops;
    uint32_t frames;
    uint32_t processed;
    uint32_t acars_decoded;
} irp_status_snap_t;

// ----- CRC16-CCITT (polynomial 0x1021, init 0xFFFF) -----
//
// Inlined so both firmware projects pick it up via this header
// without a separate .c file. Small and rarely called.

static inline uint16_t irp_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

// Build a frame in `out` (must be at least IRP_FRAME_OVERHEAD +
// payload_len bytes). Returns total frame length on success or 0
// if payload_len > IRP_MAX_PAYLOAD.
static inline uint16_t irp_encode_frame(uint8_t *out, uint16_t out_cap,
                                         uint8_t type,
                                         const void *payload,
                                         uint16_t payload_len)
{
    uint16_t total = (uint16_t)(IRP_FRAME_OVERHEAD + payload_len);
    if (payload_len > IRP_MAX_PAYLOAD) return 0;
    if (out_cap < total) return 0;
    out[0] = IRP_SOF;
    out[1] = (uint8_t)(payload_len & 0xFF);
    out[2] = (uint8_t)((payload_len >> 8) & 0xFF);
    out[3] = type;
    if (payload && payload_len) {
        const uint8_t *p = (const uint8_t *)payload;
        for (uint16_t i = 0; i < payload_len; i++) out[4 + i] = p[i];
    }
    // CRC over TYPE + PAYLOAD.
    uint16_t crc = irp_crc16(out + 3, (uint16_t)(1 + payload_len));
    out[4 + payload_len] = (uint8_t)(crc & 0xFF);
    out[5 + payload_len] = (uint8_t)((crc >> 8) & 0xFF);
    return total;
}

// Decoder state machine. Feed bytes one-by-one via irp_feed_byte().
// On valid frame the user callback (out_handler) fires with the
// type code and payload pointer/length.
typedef struct {
    enum {
        IRP_DEC_SOF, IRP_DEC_LEN_LO, IRP_DEC_LEN_HI,
        IRP_DEC_TYPE, IRP_DEC_PAYLOAD, IRP_DEC_CRC_LO, IRP_DEC_CRC_HI,
    } state;
    uint16_t expected_len;
    uint16_t got;
    uint8_t  type;
    uint16_t crc_recv;
    uint8_t  payload[IRP_MAX_PAYLOAD];
} irp_decoder_t;

static inline void irp_decoder_reset(irp_decoder_t *d) { d->state = IRP_DEC_SOF; }

// Returns 1 when a full valid frame has been assembled (caller
// reads d->type, d->payload[0..d->expected_len-1]). Returns 0
// while accumulating, -1 on CRC failure.
static inline int irp_feed_byte(irp_decoder_t *d, uint8_t b)
{
    switch (d->state) {
    case IRP_DEC_SOF:
        if (b == IRP_SOF) d->state = IRP_DEC_LEN_LO;
        return 0;
    case IRP_DEC_LEN_LO:
        d->expected_len = b;
        d->state = IRP_DEC_LEN_HI;
        return 0;
    case IRP_DEC_LEN_HI:
        d->expected_len |= ((uint16_t)b) << 8;
        if (d->expected_len > IRP_MAX_PAYLOAD) {
            d->state = IRP_DEC_SOF;
            return -1;
        }
        d->state = IRP_DEC_TYPE;
        return 0;
    case IRP_DEC_TYPE:
        d->type = b;
        d->got = 0;
        d->state = (d->expected_len == 0) ? IRP_DEC_CRC_LO : IRP_DEC_PAYLOAD;
        return 0;
    case IRP_DEC_PAYLOAD:
        d->payload[d->got++] = b;
        if (d->got == d->expected_len) d->state = IRP_DEC_CRC_LO;
        return 0;
    case IRP_DEC_CRC_LO:
        d->crc_recv = b;
        d->state = IRP_DEC_CRC_HI;
        return 0;
    case IRP_DEC_CRC_HI:
        d->crc_recv |= ((uint16_t)b) << 8;
        d->state = IRP_DEC_SOF;
        // Recompute CRC over TYPE + PAYLOAD.
        {
            // Build [TYPE][PAYLOAD] in a tiny temp via direct indexing
            // since payload is in d already.
            uint16_t crc = 0xFFFF;
            uint8_t  hdr = d->type;
            crc ^= (uint16_t)hdr << 8;
            for (int b2 = 0; b2 < 8; b2++) {
                crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                      : (uint16_t)(crc << 1);
            }
            for (uint16_t i = 0; i < d->expected_len; i++) {
                crc ^= (uint16_t)d->payload[i] << 8;
                for (int b2 = 0; b2 < 8; b2++) {
                    crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                          : (uint16_t)(crc << 1);
                }
            }
            return (crc == d->crc_recv) ? 1 : -1;
        }
    }
    d->state = IRP_DEC_SOF;
    return -1;
}
