#include "frame_pdu.h"
#include <string.h>

// --- little-endian put/get helpers ---------------------------------------

static size_t put_u8(uint8_t *b, size_t o, uint8_t v)
{
    b[o] = v;
    return o + 1;
}
static size_t put_u16(uint8_t *b, size_t o, uint16_t v)
{
    b[o]     = (uint8_t)v;
    b[o + 1] = (uint8_t)(v >> 8);
    return o + 2;
}
static size_t put_u32(uint8_t *b, size_t o, uint32_t v)
{
    for (int i = 0; i < 4; i++)
        b[o + i] = (uint8_t)(v >> (8 * i));
    return o + 4;
}
static size_t put_u64(uint8_t *b, size_t o, uint64_t v)
{
    for (int i = 0; i < 8; i++)
        b[o + i] = (uint8_t)(v >> (8 * i));
    return o + 8;
}
static uint8_t get_u8(const uint8_t *b, size_t o)
{
    return b[o];
}
static uint16_t get_u16(const uint8_t *b, size_t o)
{
    return (uint16_t)(b[o] | (b[o + 1] << 8));
}
static uint32_t get_u32(const uint8_t *b, size_t o)
{
    uint32_t v = 0;
    for (int i = 0; i < 4; i++)
        v |= (uint32_t)b[o + i] << (8 * i);
    return v;
}
static uint64_t get_u64(const uint8_t *b, size_t o)
{
    uint64_t v = 0;
    for (int i = 0; i < 8; i++)
        v |= (uint64_t)b[o + i] << (8 * i);
    return v;
}

// --- pack/unpack ---------------------------------------------------------

size_t frame_pdu_pack(const iridium_frame_pdu_t *pdu, uint8_t *buf, size_t buflen)
{
    if (!pdu || !buf || buflen < FRAME_PDU_WIRE_SIZE) return 0;
    uint32_t snr_bits;
    memcpy(&snr_bits, &pdu->peak_snr_db, 4); // float bit-pattern
    size_t o = 0;
    o        = put_u64(buf, o, pdu->timestamp_us);
    o        = put_u32(buf, o, pdu->source_id);
    o        = put_u32(buf, o, (uint32_t)pdu->rel_freq_hz);
    o        = put_u32(buf, o, snr_bits);
    o        = put_u16(buf, o, (uint16_t)pdu->peak_bin);
    o        = put_u16(buf, o, pdu->n_bits);
    o        = put_u8(buf, o, pdu->direction);
    o        = put_u8(buf, o, (uint8_t)pdu->bch_e1);
    o        = put_u8(buf, o, (uint8_t)pdu->bch_e2);
    o        = put_u8(buf, o, pdu->flags);
    memcpy(buf + o, pdu->bits_packed, FRAME_PDU_BITS_BYTES);
    o += FRAME_PDU_BITS_BYTES;
    return o;
}

size_t frame_pdu_unpack(const uint8_t *buf, size_t buflen, iridium_frame_pdu_t *pdu)
{
    if (!buf || !pdu || buflen < FRAME_PDU_WIRE_SIZE) return 0;
    memset(pdu, 0, sizeof(*pdu));
    size_t o          = 0;
    pdu->timestamp_us = get_u64(buf, o);
    o += 8;
    pdu->source_id = get_u32(buf, o);
    o += 4;
    pdu->rel_freq_hz = (int32_t)get_u32(buf, o);
    o += 4;
    uint32_t snr_bits = get_u32(buf, o);
    o += 4;
    memcpy(&pdu->peak_snr_db, &snr_bits, 4);
    pdu->peak_bin = (int16_t)get_u16(buf, o);
    o += 2;
    pdu->n_bits = get_u16(buf, o);
    o += 2;
    pdu->direction = get_u8(buf, o);
    o += 1;
    pdu->bch_e1 = (int8_t)get_u8(buf, o);
    o += 1;
    pdu->bch_e2 = (int8_t)get_u8(buf, o);
    o += 1;
    pdu->flags = get_u8(buf, o);
    o += 1;
    memcpy(pdu->bits_packed, buf + o, FRAME_PDU_BITS_BYTES);
    o += FRAME_PDU_BITS_BYTES;
    return o;
}

void frame_pdu_pack_bits(iridium_frame_pdu_t *pdu, const uint8_t *bits01, int n_bits)
{
    if (n_bits < 0) n_bits = 0;
    if (n_bits > FRAME_PDU_MAX_BITS) n_bits = FRAME_PDU_MAX_BITS;
    memset(pdu->bits_packed, 0, FRAME_PDU_BITS_BYTES);
    for (int i = 0; i < n_bits; i++) {
        if (bits01[i] & 1) pdu->bits_packed[i >> 3] |= (uint8_t)(0x80u >> (i & 7));
    }
    pdu->n_bits = (uint16_t)n_bits;
}

void frame_pdu_unpack_bits(const iridium_frame_pdu_t *pdu, uint8_t *bits01_out)
{
    int n = pdu->n_bits;
    if (n > FRAME_PDU_MAX_BITS) n = FRAME_PDU_MAX_BITS;
    for (int i = 0; i < n; i++) {
        bits01_out[i] = (pdu->bits_packed[i >> 3] >> (7 - (i & 7))) & 1;
    }
}

// --- worker output queue (target only) -----------------------------------

#ifdef ESP_PLATFORM
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/idf_additions.h"
#include "esp_mac.h"
#include "esp_log.h"

#define FRAME_PDU_QUEUE_DEPTH 32

static const char      *TAG       = "FRAME_PDU";
static QueueHandle_t    s_q       = NULL;
static uint32_t         s_src_id  = 0;
static _Atomic uint32_t s_dropped = 0;

esp_err_t frame_pdu_queue_init(void)
{
    if (s_q) return ESP_OK;
    // Queue in PSRAM: 32 × ~92 B ≈ 3 KB; 1-PDU-per-real-frame traffic, no
    // need to spend internal/DMA SRAM on it.
    s_q = xQueueCreateWithCaps(FRAME_PDU_QUEUE_DEPTH, sizeof(iridium_frame_pdu_t),
                               MALLOC_CAP_SPIRAM);
    if (!s_q) {
        ESP_LOGE(TAG, "frame PDU queue alloc failed");
        return ESP_ERR_NO_MEM;
    }
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    s_src_id = ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) |
               ((uint32_t)mac[4] << 8) | (uint32_t)mac[5];
    ESP_LOGI(TAG, "frame PDU queue ready (depth %d), source_id=0x%08lx",
             FRAME_PDU_QUEUE_DEPTH, (unsigned long)s_src_id);
    return ESP_OK;
}

uint32_t frame_pdu_source_id(void)
{
    return s_src_id;
}

bool frame_pdu_queue_push(const iridium_frame_pdu_t *pdu)
{
    if (!s_q) return false;
    if (xQueueSend(s_q, pdu, 0) != pdTRUE) {
        atomic_fetch_add_explicit(&s_dropped, 1, memory_order_relaxed);
        return false;
    }
    return true;
}

bool frame_pdu_queue_pop(iridium_frame_pdu_t *pdu, uint32_t timeout_ms)
{
    if (!s_q) return false;
    return xQueueReceive(s_q, pdu, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

uint32_t frame_pdu_queue_dropped(void)
{
    return atomic_load_explicit(&s_dropped, memory_order_relaxed);
}
#endif // ESP_PLATFORM
