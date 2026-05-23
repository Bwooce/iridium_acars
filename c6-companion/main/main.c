// ESP32-C6 companion firmware — application entry point.
// See docs/c6-companion-firmware-design.md.

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "msg_ring.h"
#include "uart_link.h"
#include "wifi.h"
#include "web.h"

static const char *TAG = "C6_MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "C6 companion firmware booting");
    msg_ring_init();
    uart_link_init();
    wifi_init();
    web_init();
    ESP_LOGI(TAG, "C6 companion ready");
}
