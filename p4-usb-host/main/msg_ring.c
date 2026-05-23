#include "msg_ring.h"

#include <string.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "MSGR";

static acars_msg_t *s_ring = NULL;     // PSRAM, MSG_RING_CAPACITY slots
static uint32_t     s_head = 0;        // next slot to write (index)
static uint64_t     s_last_id = 0;
static SemaphoreHandle_t s_mutex;

void msg_ring_init(void)
{
    if (s_ring) return;

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        ESP_LOGE(TAG, "mutex alloc failed");
        return;
    }

    // PSRAM is fine — entries are large and the access rate is low
    // (one push per decoded ACARS message, a snapshot at HTTP /messages
    // request rate).
    s_ring = (acars_msg_t *)heap_caps_calloc(
        MSG_RING_CAPACITY, sizeof(acars_msg_t), MALLOC_CAP_SPIRAM);
    if (!s_ring) {
        ESP_LOGE(TAG, "ring alloc failed (%u B in PSRAM)",
                 (unsigned)(MSG_RING_CAPACITY * sizeof(acars_msg_t)));
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
        return;
    }
    ESP_LOGI(TAG, "ACARS message ring: %d slots × %u B = %u B in PSRAM",
             MSG_RING_CAPACITY, (unsigned)sizeof(acars_msg_t),
             (unsigned)(MSG_RING_CAPACITY * sizeof(acars_msg_t)));
}

void msg_ring_push(const acars_msg_t *m)
{
    if (!s_ring || !m) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_last_id += 1;
    acars_msg_t *slot = &s_ring[s_head];
    *slot = *m;
    slot->id = s_last_id;
    s_head = (s_head + 1) % MSG_RING_CAPACITY;
    xSemaphoreGive(s_mutex);
}

size_t msg_ring_snapshot(uint64_t since_id, acars_msg_t *out, size_t cap)
{
    if (!s_ring || !out || cap == 0) return 0;
    size_t n = 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    // Walk from oldest to newest. Oldest is at s_head (next-to-write).
    for (uint32_t k = 0; k < MSG_RING_CAPACITY && n < cap; k++) {
        uint32_t idx = (s_head + k) % MSG_RING_CAPACITY;
        const acars_msg_t *e = &s_ring[idx];
        if (e->id == 0 || e->id <= since_id) continue;
        out[n++] = *e;
    }
    xSemaphoreGive(s_mutex);
    return n;
}

uint64_t msg_ring_total(void)
{
    return s_last_id;
}
