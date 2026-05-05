#include <string.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "signal_buffer.h"

static const char *TAG = "SIG_BUF";

// Store as bytes for easy circular wrapping, but manage as int16
static int16_t *circular_buf = NULL;
static uint32_t head = 0; // In complex samples (I+Q)

esp_err_t signal_buffer_init()
{
    ESP_LOGI(TAG, "Allocating 4MB Signal Buffer in PSRAM...");
    circular_buf = heap_caps_malloc(SIGNAL_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!circular_buf) return ESP_ERR_NO_MEM;
    
    memset(circular_buf, 0, SIGNAL_BUF_SIZE);
    head = 0;
    return ESP_OK;
}

void signal_buffer_push(const int16_t *samples, size_t n_samples)
{
    // n_samples is complex samples
    // circular_buf capacity is SIGNAL_BUF_SIZE / sizeof(int16_t) / 2
    uint32_t total_cap_samples = SIGNAL_BUF_SIZE / 4;
    
    for (size_t i = 0; i < n_samples; i++) {
        circular_buf[head * 2 + 0] = samples[i * 2 + 0];
        circular_buf[head * 2 + 1] = samples[i * 2 + 1];
        head = (head + 1) % total_cap_samples;
    }
}

void signal_buffer_extract(uint32_t start_idx, uint32_t length, int16_t *dest)
{
    uint32_t total_cap_samples = SIGNAL_BUF_SIZE / 4;
    uint32_t actual_start = start_idx % total_cap_samples;
    
    for (uint32_t i = 0; i < length; i++) {
        uint32_t idx = (actual_start + i) % total_cap_samples;
        dest[i * 2 + 0] = circular_buf[idx * 2 + 0];
        dest[i * 2 + 1] = circular_buf[idx * 2 + 1];
    }
}
