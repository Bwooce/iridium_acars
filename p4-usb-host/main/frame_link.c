#include "frame_link.h"
#include <string.h>

#ifdef ESP_PLATFORM
#include <stdatomic.h>
#endif

// ============================================================================
// Pure wire framing (host-testable; no ESP/RTOS dependencies)
// ============================================================================

uint16_t frame_link_crc16(const uint8_t *data, size_t len)
{
    // CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflect, no xorout.
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

size_t frame_link_encode(const iridium_frame_pdu_t *pdu, uint8_t *buf, size_t buflen)
{
    if (!pdu || !buf || buflen < FRAME_LINK_FRAME_SIZE) return 0;
    buf[0]   = (uint8_t)(FRAME_LINK_MAGIC & 0xFF);
    buf[1]   = (uint8_t)(FRAME_LINK_MAGIC >> 8);
    buf[2]   = FRAME_LINK_VER;
    buf[3]   = 0; // flags (reserved)
    size_t w = frame_pdu_pack(pdu, buf + FRAME_LINK_HDR_BYTES,
                              buflen - FRAME_LINK_HDR_BYTES);
    if (w != FRAME_PDU_WIRE_SIZE) return 0;
    // CRC over ver..payload (everything between magic and crc).
    uint16_t crc = frame_link_crc16(buf + 2, (size_t)(FRAME_LINK_HDR_BYTES - 2) + w);
    size_t   o   = FRAME_LINK_HDR_BYTES + w;
    buf[o]       = (uint8_t)(crc & 0xFF);
    buf[o + 1]   = (uint8_t)(crc >> 8);
    return FRAME_LINK_FRAME_SIZE;
}

bool frame_link_decode(const uint8_t *buf, size_t buflen, iridium_frame_pdu_t *pdu)
{
    if (!buf || !pdu || buflen < FRAME_LINK_FRAME_SIZE) return false;
    if (buf[0] != (uint8_t)(FRAME_LINK_MAGIC & 0xFF) ||
        buf[1] != (uint8_t)(FRAME_LINK_MAGIC >> 8)) {
        return false;
    }
    if (buf[2] != FRAME_LINK_VER) return false;
    size_t   crc_off = FRAME_LINK_HDR_BYTES + FRAME_PDU_WIRE_SIZE;
    uint16_t want    = (uint16_t)(buf[crc_off] | (buf[crc_off + 1] << 8));
    uint16_t got     = frame_link_crc16(buf + 2, (size_t)(FRAME_LINK_HDR_BYTES - 2) + FRAME_PDU_WIRE_SIZE);
    if (want != got) return false;
    if (frame_pdu_unpack(buf + FRAME_LINK_HDR_BYTES, FRAME_PDU_WIRE_SIZE, pdu) !=
        FRAME_PDU_WIRE_SIZE) {
        return false;
    }
    // Reject any PDU with n_bits > FRAME_PDU_MAX_BITS to prevent out-of-bounds
    // read in downstream unpacking (aggregator_ingest.c unpacks to a 512-byte buffer).
    if (pdu->n_bits > FRAME_PDU_MAX_BITS) {
        return false;
    }
    return true;
}

// ============================================================================
// SPI transport (target only)
// ============================================================================
#ifdef ESP_PLATFORM
#include "driver/spi_master.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

// --- pin config (Kconfig; illustrative fallbacks so the file always builds
//     — VERIFY against your board's free header GPIOs before wiring) -------
#ifndef CONFIG_FRAME_LINK_SCLK_GPIO
#define CONFIG_FRAME_LINK_SCLK_GPIO 20
#endif
#ifndef CONFIG_FRAME_LINK_MOSI_GPIO
#define CONFIG_FRAME_LINK_MOSI_GPIO 21
#endif
#ifndef CONFIG_FRAME_LINK_MISO_GPIO
#define CONFIG_FRAME_LINK_MISO_GPIO 22
#endif
#ifndef CONFIG_FRAME_LINK_CS_GPIO
#define CONFIG_FRAME_LINK_CS_GPIO 23
#endif
#ifndef CONFIG_FRAME_LINK_HANDSHAKE_GPIO
#define CONFIG_FRAME_LINK_HANDSHAKE_GPIO 7
#endif
#ifndef CONFIG_FRAME_LINK_CLOCK_HZ
#define CONFIG_FRAME_LINK_CLOCK_HZ 1000000 // 1 MHz: conservative for jumpers
#endif

// Loopback self-test slave-side pins (must be a SECOND, jumpered set).
#ifndef CONFIG_FRAME_LINK_LB_SCLK_GPIO
#define CONFIG_FRAME_LINK_LB_SCLK_GPIO 8
#endif
#ifndef CONFIG_FRAME_LINK_LB_MOSI_GPIO
#define CONFIG_FRAME_LINK_LB_MOSI_GPIO 9
#endif
#ifndef CONFIG_FRAME_LINK_LB_MISO_GPIO
#define CONFIG_FRAME_LINK_LB_MISO_GPIO 10
#endif
#ifndef CONFIG_FRAME_LINK_LB_CS_GPIO
#define CONFIG_FRAME_LINK_LB_CS_GPIO 11
#endif
#ifndef CONFIG_FRAME_LINK_LB_HANDSHAKE_GPIO
#define CONFIG_FRAME_LINK_LB_HANDSHAKE_GPIO 13
#endif

#define MASTER_HOST SPI2_HOST
#define SLAVE_HOST SPI3_HOST

static const char *TAG = "FRAME_LINK";

static atomic_uint_least32_t s_frames_tx;
static atomic_uint_least32_t s_frames_rx;
static atomic_uint_least32_t s_crc_errors;
static atomic_uint_least32_t s_bus_errors;
static atomic_uint_least32_t s_queue_full;

static TaskHandle_t      s_slave_task  = NULL;
static TaskHandle_t      s_master_task = NULL;
static SemaphoreHandle_t s_master_ready_sem; // given by handshake ISR

// --- slave handshake: assert when a transaction is loaded, deassert after.
static void IRAM_ATTR slave_post_setup(spi_slave_transaction_t *t)
{
    (void)t;
    gpio_set_level(CONFIG_FRAME_LINK_HANDSHAKE_GPIO, 1);
}
static void IRAM_ATTR slave_post_trans(spi_slave_transaction_t *t)
{
    (void)t;
    gpio_set_level(CONFIG_FRAME_LINK_HANDSHAKE_GPIO, 0);
}

// --- master handshake input ISR: data-ready edge -> wake the master task.
static void IRAM_ATTR master_handshake_isr(void *arg)
{
    BaseType_t hp = pdFALSE;
    xSemaphoreGiveFromISR(s_master_ready_sem, &hp);
    if (hp) portYIELD_FROM_ISR();
}

static esp_err_t slave_bus_init(spi_host_device_t host, int sclk, int mosi, int miso,
                                int cs, int handshake)
{
    gpio_config_t hs = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << handshake,
    };
    gpio_config(&hs);
    gpio_set_level(handshake, 0);

    spi_bus_config_t buscfg = {
        .mosi_io_num     = mosi,
        .miso_io_num     = miso,
        .sclk_io_num     = sclk,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = FRAME_LINK_FRAME_SIZE,
    };
    spi_slave_interface_config_t slvcfg = {
        .mode          = 0,
        .spics_io_num  = cs,
        .queue_size    = 2,
        .flags         = 0,
        .post_setup_cb = slave_post_setup,
        .post_trans_cb = slave_post_trans,
    };
    return spi_slave_initialize(host, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
}

static esp_err_t master_bus_init(spi_host_device_t host, int sclk, int mosi, int miso,
                                 int cs, int handshake, spi_device_handle_t *out_dev)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num     = mosi,
        .miso_io_num     = miso,
        .sclk_io_num     = sclk,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = FRAME_LINK_FRAME_SIZE,
    };
    esp_err_t rc = spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO);
    if (rc != ESP_OK) return rc;

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz   = CONFIG_FRAME_LINK_CLOCK_HZ,
        .mode             = 0,
        .spics_io_num     = cs,
        .queue_size       = 2,
        .cs_ena_posttrans = 2,
    };
    rc = spi_bus_add_device(host, &devcfg, out_dev);
    if (rc != ESP_OK) return rc;

    // Handshake input: rising edge = slave has a frame ready.
    gpio_config_t hs = {
        .intr_type    = GPIO_INTR_POSEDGE,
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << handshake,
        .pull_down_en = 1,
    };
    gpio_config(&hs);
    return ESP_OK;
}

// --- worker slave task: drain frame_pdu output queue -> SPI ----------------
static void slave_task(void *arg)
{
    (void)arg;
    uint8_t *txbuf = heap_caps_malloc(FRAME_LINK_FRAME_SIZE, MALLOC_CAP_DMA);
    uint8_t *rxbuf = heap_caps_malloc(FRAME_LINK_FRAME_SIZE, MALLOC_CAP_DMA);
    if (!txbuf || !rxbuf) {
        ESP_LOGE(TAG, "slave DMA buf alloc failed");
        free(txbuf);
        free(rxbuf);
        vTaskDelete(NULL);
        return;
    }
    iridium_frame_pdu_t pdu;
    while (1) {
        if (!frame_pdu_queue_pop(&pdu, 1000)) continue;
        if (frame_link_encode(&pdu, txbuf, FRAME_LINK_FRAME_SIZE) != FRAME_LINK_FRAME_SIZE) {
            continue;
        }
        spi_slave_transaction_t t = {
            .length    = FRAME_LINK_FRAME_SIZE * 8,
            .tx_buffer = txbuf,
            .rx_buffer = rxbuf,
        };
        // Blocks until the master clocks the frame (post_setup_cb raised the
        // handshake; post_trans_cb lowers it).
        esp_err_t rc = spi_slave_transmit(SLAVE_HOST, &t, portMAX_DELAY);
        if (rc != ESP_OK) {
            atomic_fetch_add(&s_bus_errors, 1);
            continue;
        }
        atomic_fetch_add(&s_frames_tx, 1);
    }
}

// --- aggregator master task: clock frames -> frame_pdu input queue ---------
static spi_device_handle_t s_master_dev;

static void master_task(void *arg)
{
    (void)arg;
    uint8_t *rxbuf = heap_caps_malloc(FRAME_LINK_FRAME_SIZE, MALLOC_CAP_DMA);
    uint8_t *txbuf = heap_caps_calloc(1, FRAME_LINK_FRAME_SIZE, MALLOC_CAP_DMA);
    if (!rxbuf || !txbuf) {
        ESP_LOGE(TAG, "master DMA buf alloc failed");
        free(rxbuf);
        free(txbuf);
        vTaskDelete(NULL);
        return;
    }
    while (1) {
        // Wait for the slave's data-ready edge (1 s fallback poll so a missed
        // edge can't wedge the link forever).
        xSemaphoreTake(s_master_ready_sem, pdMS_TO_TICKS(1000));
        if (gpio_get_level(CONFIG_FRAME_LINK_HANDSHAKE_GPIO) == 0) continue;

        spi_transaction_t t = {
            .length    = FRAME_LINK_FRAME_SIZE * 8,
            .tx_buffer = txbuf,
            .rx_buffer = rxbuf,
        };
        esp_err_t rc = spi_device_transmit(s_master_dev, &t);
        if (rc != ESP_OK) {
            atomic_fetch_add(&s_bus_errors, 1);
            continue;
        }
        iridium_frame_pdu_t pdu;
        if (!frame_link_decode(rxbuf, FRAME_LINK_FRAME_SIZE, &pdu)) {
            atomic_fetch_add(&s_crc_errors, 1);
            continue;
        }
        atomic_fetch_add(&s_frames_rx, 1);
        if (!frame_pdu_queue_push(&pdu)) {
            atomic_fetch_add(&s_queue_full, 1);
        }
    }
}

esp_err_t frame_link_slave_start(void)
{
    if (s_slave_task) return ESP_OK;
    esp_err_t rc = slave_bus_init(SLAVE_HOST, CONFIG_FRAME_LINK_SCLK_GPIO,
                                  CONFIG_FRAME_LINK_MOSI_GPIO, CONFIG_FRAME_LINK_MISO_GPIO,
                                  CONFIG_FRAME_LINK_CS_GPIO, CONFIG_FRAME_LINK_HANDSHAKE_GPIO);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "slave init failed: %s", esp_err_to_name(rc));
        return rc;
    }
    if (xTaskCreatePinnedToCore(slave_task, "flink_slave", 4096, NULL, 6, &s_slave_task, 1) !=
        pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "SPI slave up (SPI3: sclk=%d mosi=%d miso=%d cs=%d hs=%d)",
             CONFIG_FRAME_LINK_SCLK_GPIO, CONFIG_FRAME_LINK_MOSI_GPIO,
             CONFIG_FRAME_LINK_MISO_GPIO, CONFIG_FRAME_LINK_CS_GPIO,
             CONFIG_FRAME_LINK_HANDSHAKE_GPIO);
    return ESP_OK;
}

esp_err_t frame_link_master_start(void)
{
    if (s_master_task) return ESP_OK;
    if (!s_master_ready_sem) s_master_ready_sem = xSemaphoreCreateBinary();
    if (!s_master_ready_sem) return ESP_ERR_NO_MEM;

    esp_err_t rc = master_bus_init(MASTER_HOST, CONFIG_FRAME_LINK_SCLK_GPIO,
                                   CONFIG_FRAME_LINK_MOSI_GPIO, CONFIG_FRAME_LINK_MISO_GPIO,
                                   CONFIG_FRAME_LINK_CS_GPIO, CONFIG_FRAME_LINK_HANDSHAKE_GPIO,
                                   &s_master_dev);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "master init failed: %s", esp_err_to_name(rc));
        return rc;
    }
    // gpio_install_isr_service may already be installed elsewhere; ignore
    // ESP_ERR_INVALID_STATE.
    esp_err_t isr_rc = gpio_install_isr_service(0);
    if (isr_rc != ESP_OK && isr_rc != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "gpio isr service: %s", esp_err_to_name(isr_rc));
    }
    gpio_isr_handler_add(CONFIG_FRAME_LINK_HANDSHAKE_GPIO, master_handshake_isr, NULL);

    if (xTaskCreatePinnedToCore(master_task, "flink_master", 4096, NULL, 6, &s_master_task, 1) !=
        pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "SPI master up (SPI2: sclk=%d mosi=%d miso=%d cs=%d hs=%d @ %d Hz)",
             CONFIG_FRAME_LINK_SCLK_GPIO, CONFIG_FRAME_LINK_MOSI_GPIO,
             CONFIG_FRAME_LINK_MISO_GPIO, CONFIG_FRAME_LINK_CS_GPIO,
             CONFIG_FRAME_LINK_HANDSHAKE_GPIO, CONFIG_FRAME_LINK_CLOCK_HZ);
    return ESP_OK;
}

void frame_link_get_stats(frame_link_stats_t *out)
{
    if (!out) return;
    out->frames_tx  = atomic_load(&s_frames_tx);
    out->frames_rx  = atomic_load(&s_frames_rx);
    out->crc_errors = atomic_load(&s_crc_errors);
    out->bus_errors = atomic_load(&s_bus_errors);
    out->queue_full = atomic_load(&s_queue_full);
}

// --- one-board jumpered loopback self-test ---------------------------------
esp_err_t frame_link_loopback_selftest(void)
{
    // Master on SPI2 (primary pins), slave on SPI3 (LB pins) — jumper:
    //   SCLK<->LB_SCLK, MOSI<->LB_MOSI, MISO<->LB_MISO, CS<->LB_CS, HS<->LB_HS.
    esp_err_t rc = slave_bus_init(SLAVE_HOST, CONFIG_FRAME_LINK_LB_SCLK_GPIO,
                                  CONFIG_FRAME_LINK_LB_MOSI_GPIO, CONFIG_FRAME_LINK_LB_MISO_GPIO,
                                  CONFIG_FRAME_LINK_LB_CS_GPIO,
                                  CONFIG_FRAME_LINK_HANDSHAKE_GPIO);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "loopback: slave init %s", esp_err_to_name(rc));
        return rc;
    }
    spi_device_handle_t dev;
    rc = master_bus_init(MASTER_HOST, CONFIG_FRAME_LINK_SCLK_GPIO, CONFIG_FRAME_LINK_MOSI_GPIO,
                         CONFIG_FRAME_LINK_MISO_GPIO, CONFIG_FRAME_LINK_CS_GPIO,
                         CONFIG_FRAME_LINK_HANDSHAKE_GPIO, &dev);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "loopback: master init %s", esp_err_to_name(rc));
        return rc;
    }

    uint8_t *tx = heap_caps_malloc(FRAME_LINK_FRAME_SIZE, MALLOC_CAP_DMA);
    uint8_t *rx = heap_caps_calloc(1, FRAME_LINK_FRAME_SIZE, MALLOC_CAP_DMA);
    uint8_t *st = heap_caps_malloc(FRAME_LINK_FRAME_SIZE, MALLOC_CAP_DMA);
    if (!tx || !rx || !st) {
        free(tx);
        free(rx);
        free(st);
        return ESP_ERR_NO_MEM;
    }

    iridium_frame_pdu_t pdu = {0};
    pdu.timestamp_us        = 0x0011223344556677ull;
    pdu.source_id           = 0xA5A51234u;
    pdu.rel_freq_hz         = -123456;
    pdu.peak_snr_db         = 17.5f;
    pdu.peak_bin            = 1024;
    pdu.direction           = 1;
    pdu.bch_e1              = 1;
    pdu.bch_e2              = 0;
    pdu.n_bits              = 382;
    for (int i = 0; i < FRAME_PDU_BITS_BYTES; i++)
        pdu.bits_packed[i] = (uint8_t)(i * 7 + 3);
    frame_link_encode(&pdu, st, FRAME_LINK_FRAME_SIZE);

    // Queue the slave's outgoing frame, then clock it from the master.
    static spi_slave_transaction_t lb_trans; // static: outlives the call window
    memset(&lb_trans, 0, sizeof(lb_trans));
    memcpy(tx, st, FRAME_LINK_FRAME_SIZE);
    lb_trans.length    = FRAME_LINK_FRAME_SIZE * 8;
    lb_trans.tx_buffer = tx;
    rc                 = spi_slave_queue_trans(SLAVE_HOST, &lb_trans, pdMS_TO_TICKS(100));
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "loopback: slave queue %s", esp_err_to_name(rc));
        goto done;
    }
    vTaskDelay(pdMS_TO_TICKS(5)); // let post_setup raise the handshake

    spi_transaction_t mt = {
        .length    = FRAME_LINK_FRAME_SIZE * 8,
        .rx_buffer = rx,
    };
    rc = spi_device_transmit(dev, &mt);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "loopback: master xfer %s", esp_err_to_name(rc));
        goto done;
    }
    spi_slave_transaction_t *done_trans;
    spi_slave_get_trans_result(SLAVE_HOST, &done_trans, pdMS_TO_TICKS(100));

    iridium_frame_pdu_t got;
    if (!frame_link_decode(rx, FRAME_LINK_FRAME_SIZE, &got)) {
        ESP_LOGE(TAG, "loopback: decode/CRC FAILED");
        rc = ESP_FAIL;
        goto done;
    }
    if (memcmp(&got, &pdu, sizeof(pdu)) != 0) {
        ESP_LOGE(TAG, "loopback: PDU mismatch after round-trip");
        rc = ESP_FAIL;
        goto done;
    }
    ESP_LOGI(TAG, "loopback: SELFTEST PASS (round-trip byte-exact, CRC ok)");
    rc = ESP_OK;

done:
    free(tx);
    free(rx);
    free(st);
    return rc;
}
#endif // ESP_PLATFORM
