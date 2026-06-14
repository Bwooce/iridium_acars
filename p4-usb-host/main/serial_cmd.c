#include "serial_cmd.h"
#include "app_config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

#define TAG "SCMD"
#define CMD_UART UART_NUM_0
#define CMD_LINE_MAX 256
#define TASK_STACK 4096

static void uart_puts(const char *s)
{
    uart_write_bytes(CMD_UART, s, strlen(s));
}

static void cmd_config(void)
{
    app_config_t c;
    app_config_snapshot(&c);
    char buf[160];

    snprintf(buf, sizeof(buf), "wifi_ssid=%s\r\n", c.wifi_ssid);
    uart_puts(buf);
    snprintf(buf, sizeof(buf), "wifi_psk=%s\r\n", c.wifi_psk);
    uart_puts(buf);
    snprintf(buf, sizeof(buf), "out_host=%s\r\n", c.out_host);
    uart_puts(buf);
    snprintf(buf, sizeof(buf), "out_port=%u\r\n", (unsigned)c.out_port);
    uart_puts(buf);
    snprintf(buf, sizeof(buf), "station_id=%s\r\n", c.station_id);
    uart_puts(buf);
    snprintf(buf, sizeof(buf), "lo_hz=%lu\r\n", (unsigned long)c.lo_freq_hz);
    uart_puts(buf);
    snprintf(buf, sizeof(buf), "rate_hz=%lu\r\n", (unsigned long)c.sample_rate_hz);
    uart_puts(buf);
    snprintf(buf, sizeof(buf), "gain_mode=%d\r\n", (int)c.gain_mode);
    uart_puts(buf);
    snprintf(buf, sizeof(buf), "gain_dbx10=%d\r\n", (int)c.gain_db_x10);
    uart_puts(buf);
    snprintf(buf, sizeof(buf), "bias_tee=%d\r\n", (int)c.bias_tee);
    uart_puts(buf);
    snprintf(buf, sizeof(buf), "tag_thr=%.2f\r\n", (double)c.tagger_threshold_db);
    uart_puts(buf);
    snprintf(buf, sizeof(buf), "ota_url=%s\r\n", c.ota_url);
    uart_puts(buf);
}

static void cmd_get(const char *key)
{
    app_config_t c;
    app_config_snapshot(&c);
    char buf[160];
    if (strcmp(key, "wifi_ssid") == 0)
        snprintf(buf, sizeof(buf), "%s\r\n", c.wifi_ssid);
    else if (strcmp(key, "wifi_psk") == 0)
        snprintf(buf, sizeof(buf), "%s\r\n", c.wifi_psk);
    else if (strcmp(key, "out_host") == 0)
        snprintf(buf, sizeof(buf), "%s\r\n", c.out_host);
    else if (strcmp(key, "out_port") == 0)
        snprintf(buf, sizeof(buf), "%u\r\n", (unsigned)c.out_port);
    else if (strcmp(key, "station_id") == 0)
        snprintf(buf, sizeof(buf), "%s\r\n", c.station_id);
    else if (strcmp(key, "lo_hz") == 0)
        snprintf(buf, sizeof(buf), "%lu\r\n", (unsigned long)c.lo_freq_hz);
    else if (strcmp(key, "rate_hz") == 0)
        snprintf(buf, sizeof(buf), "%lu\r\n", (unsigned long)c.sample_rate_hz);
    else if (strcmp(key, "gain_mode") == 0)
        snprintf(buf, sizeof(buf), "%d\r\n", (int)c.gain_mode);
    else if (strcmp(key, "gain_dbx10") == 0)
        snprintf(buf, sizeof(buf), "%d\r\n", (int)c.gain_db_x10);
    else if (strcmp(key, "bias_tee") == 0)
        snprintf(buf, sizeof(buf), "%d\r\n", (int)c.bias_tee);
    else if (strcmp(key, "tag_thr") == 0)
        snprintf(buf, sizeof(buf), "%.2f\r\n", (double)c.tagger_threshold_db);
    else if (strcmp(key, "ota_url") == 0)
        snprintf(buf, sizeof(buf), "%s\r\n", c.ota_url);
    else {
        uart_puts("ERR unknown key\r\n");
        return;
    }
    uart_puts(buf);
}

static void cmd_set(const char *key, const char *val)
{
    esp_err_t rc = ESP_OK;
    if (strcmp(key, "wifi_ssid") == 0)
        rc = app_config_set_wifi_ssid(val);
    else if (strcmp(key, "wifi_psk") == 0)
        rc = app_config_set_wifi_psk(val);
    else if (strcmp(key, "out_host") == 0)
        rc = app_config_set_out_host(val);
    else if (strcmp(key, "out_port") == 0)
        rc = app_config_set_out_port((uint16_t)atoi(val));
    else if (strcmp(key, "station_id") == 0)
        rc = app_config_set_station_id(val);
    else if (strcmp(key, "lo_hz") == 0)
        rc = app_config_set_lo_freq_hz((uint32_t)atol(val));
    else if (strcmp(key, "rate_hz") == 0)
        rc = app_config_set_sample_rate_hz((uint32_t)atol(val));
    else if (strcmp(key, "gain_mode") == 0)
        rc = app_config_set_gain_mode((gain_mode_t)atoi(val));
    else if (strcmp(key, "gain_dbx10") == 0)
        rc = app_config_set_gain_db_x10((int16_t)atoi(val));
    else if (strcmp(key, "bias_tee") == 0)
        rc = app_config_set_bias_tee(atoi(val) != 0);
    else if (strcmp(key, "tag_thr") == 0)
        rc = app_config_set_tagger_threshold_db((float)atof(val));
    else if (strcmp(key, "ota_url") == 0)
        rc = app_config_set_ota_url(val);
    else {
        uart_puts("ERR unknown key\r\n");
        return;
    }

    if (rc == ESP_OK) {
        uart_puts("OK\r\n");
    } else {
        char buf[48];
        snprintf(buf, sizeof(buf), "ERR nvs 0x%x\r\n", (unsigned)rc);
        uart_puts(buf);
    }
}

static void dispatch(char *line)
{
    // Trim trailing whitespace
    int len = (int)strlen(line);
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n' || line[len - 1] == ' '))
        line[--len] = '\0';
    if (len == 0) return;

    char *cmd = strtok(line, " \t");
    if (!cmd) return;

    if (strcmp(cmd, "reboot") == 0) {
        uart_puts("OK rebooting\r\n");
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
        return;
    }
    if (strcmp(cmd, "config") == 0) {
        cmd_config();
        return;
    }

    char *key = strtok(NULL, " \t");
    if (!key) {
        uart_puts("ERR missing key\r\n");
        return;
    }

    if (strcmp(cmd, "get") == 0) {
        cmd_get(key);
        return;
    }
    if (strcmp(cmd, "set") == 0) {
        char *val = strtok(NULL, "");
        if (!val) val = "";
        while (*val == ' ' || *val == '\t')
            val++;
        cmd_set(key, val);
        return;
    }

    uart_puts("ERR unknown command (set/get/config/reboot)\r\n");
}

static void serial_cmd_task(void *arg)
{
    (void)arg;
    char line[CMD_LINE_MAX];
    int  pos = 0;

    uart_puts("\r\n[SCMD] ready — set <key> <val>  get <key>  config  reboot\r\n");

    while (1) {
        uint8_t c;
        if (uart_read_bytes(CMD_UART, &c, 1, portMAX_DELAY) != 1) continue;

        if (c == '\n' || c == '\r') {
            if (pos > 0) {
                line[pos] = '\0';
                dispatch(line);
                pos = 0;
            }
        } else if (c == '\b' || c == 127) {
            if (pos > 0) pos--;
        } else if (pos < CMD_LINE_MAX - 1) {
            line[pos++] = (char)c;
        }
    }
}

void serial_cmd_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(CMD_UART, &cfg);
    // tx_buffer_size=0: TX writes block until the FIFO drains (fine for
    // low-rate command responses).  RX buffer 512 bytes.
    uart_driver_install(CMD_UART, 512, 0, 0, NULL, 0);

    xTaskCreate(serial_cmd_task, "serial_cmd", TASK_STACK, NULL, 3, NULL);
    ESP_LOGI(TAG, "serial command task started");
}
