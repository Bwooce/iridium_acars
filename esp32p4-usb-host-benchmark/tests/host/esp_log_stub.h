#ifndef ESP_LOG_STUB_H
#define ESP_LOG_STUB_H

// Host-side replacement for esp_log.h. The production code calls
// ESP_LOGI/D/E/W/V; on the host we drop them. (If a test needs to assert
// against a log line, swap to printf in that test only.)

#define ESP_LOGE(tag, fmt, ...) ((void)0)
#define ESP_LOGW(tag, fmt, ...) ((void)0)
#define ESP_LOGI(tag, fmt, ...) ((void)0)
#define ESP_LOGD(tag, fmt, ...) ((void)0)
#define ESP_LOGV(tag, fmt, ...) ((void)0)

#endif
