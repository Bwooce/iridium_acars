#include "serial_cmd.h"
#include "app_config.h"
#include "scanner.h"
#include "wifi_link.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h" // xTaskCreatePinnedToCoreWithCaps
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include <errno.h>
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

// Diagnostic: probe whether multicast egress works over esp_hosted/C6.
// Sends 5 UNICAST datagrams (control — should always reach the host) and
// 5 MULTICAST datagrams to 239.255.1.100:4210, logging each sendto() rc +
// errno. Interpretation, paired with a host-side tcpdump:
//   unicast arrives + mcast sendto OK but no mcast on wire => esp_hosted
//     silently drops group-addressed TX (the iot_log blocker).
//   mcast sendto < 0 with errno => host lwip/netif rejects mcast routing.
static void cmd_nettest(const char *unicast_ip)
{
    char buf[112];
    int  s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) {
        uart_puts("ERR socket\r\n");
        return;
    }
    uint8_t ttl = 1;
    setsockopt(s, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));

    const char *payload = "P4NETTEST";
    size_t      plen    = strlen(payload);

    if (unicast_ip && unicast_ip[0]) {
        struct sockaddr_in u = {0};
        u.sin_family         = AF_INET;
        u.sin_port           = htons(9999);
        u.sin_addr.s_addr    = inet_addr(unicast_ip);
        for (int i = 0; i < 5; i++) {
            errno = 0;
            int r = sendto(s, payload, plen, 0, (struct sockaddr *)&u, sizeof(u));
            snprintf(buf, sizeof(buf), "unicast %s:9999  sendto=%d errno=%d\r\n",
                     unicast_ip, r, errno);
            uart_puts(buf);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    struct sockaddr_in m = {0};
    m.sin_family         = AF_INET;
    m.sin_port           = htons(4210);
    m.sin_addr.s_addr    = inet_addr("239.255.1.100");

    // Batch A: multicast with DEFAULT egress (no IP_MULTICAST_IF) — same as
    // iot_log does today.
    for (int i = 0; i < 5; i++) {
        errno = 0;
        int r = sendto(s, payload, plen, 0, (struct sockaddr *)&m, sizeof(m));
        snprintf(buf, sizeof(buf), "mcastA(default) 239.255.1.100:4210  sendto=%d errno=%d\r\n",
                 r, errno);
        uart_puts(buf);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Batch B: multicast with IP_MULTICAST_IF pinned to the STA address.
    // If A drops but B arrives => egress-interface selection bug (fixable
    // with setsockopt), NOT an esp_hosted datapath drop.
    uint32_t sta_ip = wifi_link_ip_u32(); // network byte order
    if (sta_ip) {
        struct in_addr ifa = {.s_addr = sta_ip};
        int            sr  = setsockopt(s, IPPROTO_IP, IP_MULTICAST_IF, &ifa, sizeof(ifa));
        snprintf(buf, sizeof(buf), "set IP_MULTICAST_IF=0x%08lx rc=%d errno=%d\r\n",
                 (unsigned long)sta_ip, sr, errno);
        uart_puts(buf);
        for (int i = 0; i < 5; i++) {
            errno = 0;
            int r = sendto(s, payload, plen, 0, (struct sockaddr *)&m, sizeof(m));
            snprintf(buf, sizeof(buf), "mcastB(IF=STA) 239.255.1.100:4210  sendto=%d errno=%d\r\n",
                     r, errno);
            uart_puts(buf);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    } else {
        uart_puts("mcastB skipped (no STA IP)\r\n");
    }
    close(s);
    uart_puts("nettest done\r\n");
}

static void cmd_hop(char *args)
{
    char *hz_s = strtok(args, " \t");
    char *save = strtok(NULL, " \t");
    if (!hz_s) {
        uart_puts("ERR usage: hop <hz> [save]\r\n");
        return;
    }
    uint32_t  hz      = (uint32_t)strtoul(hz_s, NULL, 10);
    bool      persist = (save && strcmp(save, "save") == 0);
    esp_err_t e       = scanner_hop(hz, persist);
    if (e == ESP_OK)
        uart_puts("OK hopped (settling ~0.5s)\r\n");
    else if (e == ESP_ERR_INVALID_STATE)
        uart_puts("ERR SDR not streaming yet\r\n");
    else
        uart_puts("ERR retune failed\r\n");
}

static void cmd_scan(char *args)
{
    uint32_t start = SCAN_START_HZ, stop = SCAN_STOP_HZ, step = SCAN_STEP_HZ, dwell = SCAN_DWELL_MS;
    char    *a;
    if ((a = strtok(args, " \t"))) start = (uint32_t)strtoul(a, NULL, 10);
    if ((a = strtok(NULL, " \t"))) stop = (uint32_t)strtoul(a, NULL, 10);
    if ((a = strtok(NULL, " \t"))) step = (uint32_t)strtoul(a, NULL, 10);
    if ((a = strtok(NULL, " \t"))) dwell = (uint32_t)strtoul(a, NULL, 10);
    uart_puts("OK scanning (see log for map)\r\n");
    scanner_scan(start, stop, step, dwell);
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
    if (strcmp(cmd, "nettest") == 0) {
        char *ip = strtok(NULL, " \t");
        cmd_nettest(ip ? ip : "");
        return;
    }
    if (strcmp(cmd, "hop") == 0) {
        cmd_hop(strtok(NULL, ""));
        return;
    }
    if (strcmp(cmd, "scan") == 0) {
        cmd_scan(strtok(NULL, ""));
        return;
    }
    if (strcmp(cmd, "map") == 0) {
        scanner_print_last_map();
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

    uart_puts("ERR unknown command (set/get/config/reboot/hop/scan/map/nettest)\r\n");
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
    esp_err_t err = uart_param_config(CMD_UART, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        return;
    }

    // tx_buffer_size=0: TX writes block until the FIFO drains (fine for
    // low-rate command responses).  RX buffer 512 bytes.
    err = uart_driver_install(CMD_UART, 512, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return;
    }

    // PSRAM stack (P3-4, DMA-INT reclaim) -- this task only does UART
    // line I/O and NVS/scanner command dispatch, no DMA-internal-SRAM-only
    // work (contrast ota_runner.c's task, which MUST stay internal-SRAM
    // because esp_https_ota's flash writes disable PSRAM cache access).
    // Same pattern as wifi_link.c's health_wdt_task / sd_log.c / frame_decoder.c.
    BaseType_t ok = xTaskCreatePinnedToCoreWithCaps(serial_cmd_task, "serial_cmd",
                                                    TASK_STACK, NULL, 3, NULL,
                                                    tskNO_AFFINITY, MALLOC_CAP_SPIRAM);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "serial_cmd task create failed");
        return;
    }
    ESP_LOGI(TAG, "serial command task started");
}
