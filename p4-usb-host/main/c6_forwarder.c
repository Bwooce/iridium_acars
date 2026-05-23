// See c6_forwarder.h. UART driver + frame encoder + low-priority
// forwarder task on Core 1.
//
// GPIO/UART numbers are placeholders pending Waveshare Nano
// schematic confirmation (see design doc section 12, open
// question 1). Currently using P4 UART1 with conservative TX/RX
// pins from the docs-suggested set. Easy to retune via the
// CONFIG_C6FWD_* defines below when the schematic is read.

#include "c6_forwarder.h"
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "iridium_protocol.h"

static const char *TAG = "C6FWD";

// Placeholder pin assignments. Update from Waveshare schematic.
#define C6FWD_UART_NUM      UART_NUM_1
#define C6FWD_UART_TX_GPIO  12
#define C6FWD_UART_RX_GPIO  13
#define C6FWD_BAUD          921600
#define C6FWD_QUEUE_DEPTH   8
#define C6FWD_TX_BUF        2048
#define C6FWD_RX_BUF        2048

// Single outbound item. Payload stored inline so the producer
// doesn't need to allocate. Keeps queue elements predictable size.
typedef struct {
    uint8_t  type;
    uint16_t payload_len;
    uint8_t  payload[IRP_MAX_PAYLOAD];
} outgoing_t;

static QueueHandle_t s_tx_q   = NULL;
static TaskHandle_t  s_task   = NULL;
static bool          s_inited = false;

static void forwarder_task(void *arg)
{
    (void)arg;
    static uint8_t framebuf[IRP_MAX_FRAME];
    outgoing_t item;
    while (1) {
        if (xQueueReceive(s_tx_q, &item, portMAX_DELAY) != pdTRUE) continue;
        uint16_t framelen = irp_encode_frame(framebuf, sizeof(framebuf),
                                              item.type,
                                              item.payload,
                                              item.payload_len);
        if (framelen == 0) {
            ESP_LOGW(TAG, "frame encode failed (type=0x%02x len=%u)",
                     item.type, item.payload_len);
            continue;
        }
        int wrote = uart_write_bytes(C6FWD_UART_NUM,
                                      (const char *)framebuf, framelen);
        if (wrote != framelen) {
            ESP_LOGW(TAG, "uart_write_bytes short (%d != %u)",
                     wrote, framelen);
        }
    }
}

esp_err_t c6_forwarder_init(void)
{
    if (s_inited) return ESP_OK;

    uart_config_t cfg = {
        .baud_rate  = C6FWD_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t r = uart_driver_install(C6FWD_UART_NUM,
                                       C6FWD_RX_BUF, C6FWD_TX_BUF,
                                       0, NULL, 0);
    if (r != ESP_OK) { ESP_LOGE(TAG, "uart_driver_install: %s", esp_err_to_name(r)); return r; }
    r = uart_param_config(C6FWD_UART_NUM, &cfg);
    if (r != ESP_OK) { ESP_LOGE(TAG, "uart_param_config: %s", esp_err_to_name(r)); return r; }
    r = uart_set_pin(C6FWD_UART_NUM, C6FWD_UART_TX_GPIO, C6FWD_UART_RX_GPIO,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (r != ESP_OK) { ESP_LOGE(TAG, "uart_set_pin: %s", esp_err_to_name(r)); return r; }

    s_tx_q = xQueueCreate(C6FWD_QUEUE_DEPTH, sizeof(outgoing_t));
    if (!s_tx_q) return ESP_ERR_NO_MEM;

    BaseType_t ok = xTaskCreatePinnedToCore(
        forwarder_task, "c6fwd",
        /*stack=*/ 4096, NULL,
        /*prio=*/ 1,        // low; backpressure drops, never blocks DSP
        &s_task,
        /*core=*/ 1);
    if (ok != pdPASS) {
        vQueueDelete(s_tx_q);
        s_tx_q = NULL;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "C6 forwarder up: UART%d TX=%d RX=%d @%d baud",
             C6FWD_UART_NUM, C6FWD_UART_TX_GPIO, C6FWD_UART_RX_GPIO,
             C6FWD_BAUD);
    s_inited = true;
    return ESP_OK;
}

static esp_err_t queue_outgoing(uint8_t type, const void *payload, uint16_t len)
{
    if (!s_inited || !s_tx_q) return ESP_ERR_INVALID_STATE;
    if (len > IRP_MAX_PAYLOAD) return ESP_ERR_INVALID_SIZE;
    outgoing_t item;
    item.type = type;
    item.payload_len = len;
    if (payload && len) memcpy(item.payload, payload, len);
    // Non-blocking — if queue full, drop. DSP path stays smooth.
    if (xQueueSend(s_tx_q, &item, 0) != pdTRUE) return ESP_FAIL;
    return ESP_OK;
}

esp_err_t c6_forwarder_post_status(const irp_status_snap_t *snap)
{
    if (!snap) return ESP_ERR_INVALID_ARG;
    return queue_outgoing(IRP_TYPE_STATUS_SNAP, snap, sizeof(*snap));
}

esp_err_t c6_forwarder_post_acars(const irp_acars_msg_t *msg)
{
    if (!msg) return ESP_ERR_INVALID_ARG;
    return queue_outgoing(IRP_TYPE_ACARS_MSG, msg, sizeof(*msg));
}

esp_err_t c6_forwarder_post_boot_complete(const char *fw_ver)
{
    irp_boot_complete_t b = {0};
    if (fw_ver) {
        strncpy(b.fw_ver, fw_ver, IRP_FW_VER_LEN - 1);
        b.fw_ver[IRP_FW_VER_LEN - 1] = '\0';
    }
    return queue_outgoing(IRP_TYPE_BOOT_COMPLETE, &b, sizeof(b));
}
