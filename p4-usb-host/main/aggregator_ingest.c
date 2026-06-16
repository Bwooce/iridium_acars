#include "aggregator_ingest.h"
#include "frame_pdu.h"
#include "frame_decoder.h"
#include "qpsk_demod.h" // ir_direction_t, DIR_*

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"

#include <stdatomic.h>

static const char *TAG = "AGG_INGEST";

static TaskHandle_t     s_task     = NULL;
static _Atomic uint32_t s_ingested = 0;

// Per-source liveness table (#138). Written only by the ingest task,
// read by other tasks (HTTP /status) under s_src_lock.
static portMUX_TYPE             s_src_lock = portMUX_INITIALIZER_UNLOCKED;
static aggregator_source_stat_t s_sources[AGG_MAX_SOURCES];
static uint32_t                 s_n_sources = 0;

// Bits scratch: 0/1-per-byte, sized to the PDU max. Task-local (on the
// task stack would need ~512 B; keep it static to the single consumer).
static uint8_t s_bits01[FRAME_PDU_MAX_BITS];

// Record a PDU sighting from pdu->source_id. Single-writer (ingest task);
// the spinlock only serialises against /status readers.
static void note_source(uint32_t source_id, uint64_t now_us)
{
    portENTER_CRITICAL(&s_src_lock);
    uint32_t i;
    for (i = 0; i < s_n_sources; i++) {
        if (s_sources[i].source_id == source_id) break;
    }
    if (i == s_n_sources && s_n_sources < AGG_MAX_SOURCES) {
        s_sources[i].source_id = source_id;
        s_sources[i].count     = 0;
        s_n_sources++;
    }
    if (i < AGG_MAX_SOURCES) {
        s_sources[i].count++;
        s_sources[i].last_seen_us = now_us;
    }
    portEXIT_CRITICAL(&s_src_lock);
}

static void aggregator_ingest_task(void *arg)
{
    (void)arg;
    // Long pop timeout so the task is mostly blocked; it does no busy work.
    // Not WDT-subscribed: a wedged decoder upstream must not panic-reboot
    // the aggregator (see feedback on TASK_WDT + portMAX_DELAY waits).
    iridium_frame_pdu_t pdu;
    while (1) {
        if (!frame_pdu_queue_pop(&pdu, 1000)) {
            continue;
        }
        frame_pdu_unpack_bits(&pdu, s_bits01);
        // PDU direction: 0 = DL, 1 = UL -> ir_direction_t (DIR_DOWNLINK=0,
        // DIR_UPLINK=1). freq_hz is unused by the classifier (peak_bin +
        // snr carry the per-burst metadata); pass 0 as the STANDALONE
        // worker_core1 path does.
        ir_direction_t dir = (pdu.direction != 0) ? DIR_UPLINK : DIR_DOWNLINK;
        frame_decoder_push(s_bits01, pdu.n_bits, dir, 0u,
                           (int)pdu.peak_bin, pdu.peak_snr_db);
        atomic_fetch_add_explicit(&s_ingested, 1, memory_order_relaxed);
        note_source(pdu.source_id, (uint64_t)esp_timer_get_time());
    }
}

esp_err_t aggregator_ingest_init(void)
{
    if (s_task) return ESP_OK;
    // Core 1, priority 4: same tier as frame_decoder, below worker (5) and
    // ingest (8). 4 KB stack is ample — the task only pops + forwards.
    BaseType_t ok = xTaskCreatePinnedToCore(aggregator_ingest_task,
                                            "agg_ingest", 4096, NULL, 4,
                                            &s_task, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "task create failed");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "aggregator ingest ready (PDU queue -> frame_decoder)");
    return ESP_OK;
}

uint32_t aggregator_ingest_count(void)
{
    return atomic_load_explicit(&s_ingested, memory_order_relaxed);
}

void aggregator_ingest_get_stats(aggregator_ingest_stats_t *out)
{
    if (!out) return;
    out->forwarded       = atomic_load_explicit(&s_ingested, memory_order_relaxed);
    out->pdu_queue_depth = frame_pdu_queue_count();
    out->pdu_dropped     = frame_pdu_queue_dropped();
    portENTER_CRITICAL(&s_src_lock);
    out->n_sources = s_n_sources;
    for (uint32_t i = 0; i < AGG_MAX_SOURCES; i++) {
        out->sources[i] = s_sources[i];
    }
    portEXIT_CRITICAL(&s_src_lock);
}
