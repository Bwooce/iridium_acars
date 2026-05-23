// See uart_link.h. UART receiver task that decodes inter-chip
// protocol frames and pushes them into the message ring buffer.

#include "uart_link.h"
#include "msg_ring.h"
#include "iridium_protocol.h"

#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "UART_LINK";

// Placeholder pin assignments — must match P4's c6_forwarder TX/RX.
// Update from Waveshare schematic. C6 UART1 with TX=21, RX=20 are
// common defaults on small modules; confirm at integration time.
#define LINK_UART_NUM    UART_NUM_1
#define LINK_TX_GPIO     21
#define LINK_RX_GPIO     20
#define LINK_BAUD        921600
#define LINK_RX_BUF      4096

static volatile bool s_p4_ready = false;

bool uart_link_p4_ready(void) { return s_p4_ready; }

static void dispatch(const irp_decoder_t *d)
{
    switch (d->type) {
    case IRP_TYPE_BOOT_COMPLETE: {
        if (d->expected_len >= sizeof(irp_boot_complete_t)) {
            const irp_boot_complete_t *b = (const irp_boot_complete_t *)d->payload;
            ESP_LOGI(TAG, "P4 boot_complete fw=%.16s", b->fw_ver);
            s_p4_ready = true;
        }
        break;
    }
    case IRP_TYPE_ACARS_MSG: {
        if (d->expected_len >= sizeof(irp_acars_msg_t)) {
            const irp_acars_msg_t *m = (const irp_acars_msg_t *)d->payload;
            ESP_LOGI(TAG, "ACARS seq=%u dir=%s label=%.2s flight=%.6s text=%.40s%s",
                     (unsigned)m->seq,
                     m->direction == 1 ? "UL" : "DL",
                     m->label, m->flight, m->text,
                     strnlen(m->text, IRP_ACARS_TEXT_LEN) > 40 ? "..." : "");
            msg_ring_push_acars(m);
        }
        break;
    }
    case IRP_TYPE_STATUS_SNAP: {
        if (d->expected_len >= sizeof(irp_status_snap_t)) {
            const irp_status_snap_t *s = (const irp_status_snap_t *)d->payload;
            msg_ring_push_status(s);
            // Don't log every status (too chatty); web UI shows it.
        }
        break;
    }
    default:
        ESP_LOGW(TAG, "unknown frame type 0x%02x len=%u",
                 d->type, d->expected_len);
        break;
    }
}

static void rx_task(void *arg)
{
    (void)arg;
    static irp_decoder_t dec;
    irp_decoder_reset(&dec);
    uint8_t buf[256];
    while (1) {
        int n = uart_read_bytes(LINK_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(100));
        if (n <= 0) continue;
        for (int i = 0; i < n; i++) {
            int r = irp_feed_byte(&dec, buf[i]);
            if (r == 1) {
                dispatch(&dec);
            } else if (r < 0) {
                // CRC mismatch / oversize / malformed — silently
                // resync on next SOF.
            }
        }
    }
}

esp_err_t uart_link_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = LINK_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t r;
    r = uart_driver_install(LINK_UART_NUM, LINK_RX_BUF, 0, 0, NULL, 0);
    if (r != ESP_OK) { ESP_LOGE(TAG, "uart_driver_install: %s", esp_err_to_name(r)); return r; }
    r = uart_param_config(LINK_UART_NUM, &cfg);
    if (r != ESP_OK) { ESP_LOGE(TAG, "uart_param_config: %s", esp_err_to_name(r)); return r; }
    r = uart_set_pin(LINK_UART_NUM, LINK_TX_GPIO, LINK_RX_GPIO,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (r != ESP_OK) { ESP_LOGE(TAG, "uart_set_pin: %s", esp_err_to_name(r)); return r; }

    BaseType_t ok = xTaskCreate(rx_task, "uart_rx", 4096, NULL, 5, NULL);
    if (ok != pdPASS) return ESP_FAIL;

    ESP_LOGI(TAG, "UART link up: UART%d TX=%d RX=%d @%d baud",
             LINK_UART_NUM, LINK_TX_GPIO, LINK_RX_GPIO, LINK_BAUD);
    return ESP_OK;
}
