// frame_decoder — consumer task for the worker -> higher-layer decode
// pipeline. See frame_decoder.h for the architecture rationale.
//
// Phase D (current): pop frames, classify with iridium_frame_classify,
// log type + LW subtype, count occurrences. SBD reassembly + libacars
// dispatch land in subsequent commits.

#include <stdio.h>
#include <stdatomic.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "frame_decoder.h"
#include "frame_queue.h"
#include "iridium_frame.h"

static const char *TAG = "FRMDEC";

#define FRAME_QUEUE_SLOTS  64        // 64 × 432 B ≈ 27 KB in PSRAM
#define DECODER_STACK      6144
#define DECODER_PRIO       4         // < worker (5), < ingest (8), > logger (1)
#define DECODER_CORE       1

static frame_queue_t  *s_queue       = NULL;
static TaskHandle_t    s_task        = NULL;
static volatile bool   s_initialised = false;

static _Atomic uint64_t s_class_unknown  = 0;
static _Atomic uint64_t s_class_ms       = 0;
static _Atomic uint64_t s_class_tl       = 0;
static _Atomic uint64_t s_class_bc       = 0;
static _Atomic uint64_t s_class_lw_da    = 0;
static _Atomic uint64_t s_class_lw_other = 0;

static void process_one(const frame_queue_item_t *it)
{
    iridium_frame_t classified = { 0 };
    ir_frame_direction_t dir = (it->direction == DIR_DOWNLINK)
                               ? IR_FRM_DIR_DOWNLINK
                               : IR_FRM_DIR_UPLINK;
    int rc = iridium_frame_classify(it->bits, it->n_bits, dir, &classified);
    if (rc != 0) {
        ESP_LOGW(TAG, "classify rc=%d (n_bits=%u dir=%u)",
                 rc, it->n_bits, it->direction);
        atomic_fetch_add_explicit(&s_class_unknown, 1, memory_order_relaxed);
        return;
    }

    switch (classified.type) {
    case IR_FRAME_MS:
        atomic_fetch_add_explicit(&s_class_ms, 1, memory_order_relaxed);
        ESP_LOGI(TAG, "FRAME: MS bin=%ld snr=%.1f freq=%lu",
                 (long)it->peak_bin, (double)it->snr_db,
                 (unsigned long)it->freq_hz);
        break;
    case IR_FRAME_TL:
        atomic_fetch_add_explicit(&s_class_tl, 1, memory_order_relaxed);
        ESP_LOGI(TAG, "FRAME: TL bin=%ld snr=%.1f freq=%lu",
                 (long)it->peak_bin, (double)it->snr_db,
                 (unsigned long)it->freq_hz);
        break;
    case IR_FRAME_BC:
        atomic_fetch_add_explicit(&s_class_bc, 1, memory_order_relaxed);
        ESP_LOGI(TAG, "FRAME: BC bin=%ld snr=%.1f freq=%lu",
                 (long)it->peak_bin, (double)it->snr_db,
                 (unsigned long)it->freq_hz);
        break;
    case IR_FRAME_LW:
        if (classified.lw_subtype == IR_LW_DA) {
            atomic_fetch_add_explicit(&s_class_lw_da, 1, memory_order_relaxed);
        } else {
            atomic_fetch_add_explicit(&s_class_lw_other, 1, memory_order_relaxed);
        }
        ESP_LOGI(TAG, "FRAME: LW.%s bin=%ld snr=%.1f freq=%lu",
                 iridium_lw_subtype_name(classified.lw_subtype),
                 (long)it->peak_bin, (double)it->snr_db,
                 (unsigned long)it->freq_hz);
        break;
    case IR_FRAME_UNKNOWN:
    default:
        atomic_fetch_add_explicit(&s_class_unknown, 1, memory_order_relaxed);
        ESP_LOGD(TAG, "FRAME: ?? bin=%ld snr=%.1f", (long)it->peak_bin,
                 (double)it->snr_db);
        break;
    }
}

static void decoder_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Decoder task started on Core %d", xPortGetCoreID());

    // Register with the task watchdog so the decoder participates in
    // panic-on-stuck behaviour like the other long-running tasks.
    esp_err_t wdt_rc = esp_task_wdt_add(NULL);
    if (wdt_rc != ESP_OK && wdt_rc != ESP_ERR_INVALID_ARG) {
        ESP_LOGW(TAG, "esp_task_wdt_add returned %d (%s)",
                 wdt_rc, esp_err_to_name(wdt_rc));
    }

    frame_queue_item_t item;
    while (1) {
        bool got = frame_queue_pop(s_queue, &item);
        if (got) {
            process_one(&item);
            // Yield once after each item so IDLE1 / lower-prio tasks
            // (status_logger) get a slice even if bursts arrive
            // back-to-back. taskYIELD here would also work but a
            // 1-tick delay is more predictable.
            vTaskDelay(1);
        } else {
            // Empty — sleep one tick (10 ms at 100 Hz tick rate). Note:
            // pdMS_TO_TICKS(2) rounds to 0 ticks at the default 100 Hz
            // and won't yield to IDLE1, so the WDT trips on IDLE1
            // starvation. Using `1` directly forces at least one tick.
            vTaskDelay(1);
        }
        esp_task_wdt_reset();
    }
}

esp_err_t frame_decoder_init(void)
{
    if (s_initialised) return ESP_OK;

    s_queue = frame_queue_create(FRAME_QUEUE_SLOTS);
    if (!s_queue) {
        ESP_LOGE(TAG, "frame_queue_create(%d) failed (PSRAM exhausted?)",
                 FRAME_QUEUE_SLOTS);
        return ESP_ERR_NO_MEM;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(decoder_task, "frame_decoder",
                                            DECODER_STACK, NULL,
                                            DECODER_PRIO, &s_task,
                                            DECODER_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreatePinnedToCore failed");
        frame_queue_destroy(s_queue);
        s_queue = NULL;
        return ESP_FAIL;
    }

    s_initialised = true;
    ESP_LOGI(TAG, "frame_decoder ready: %d-slot queue (~%u KB PSRAM), "
             "task on Core %d prio %d",
             FRAME_QUEUE_SLOTS,
             (unsigned)(FRAME_QUEUE_SLOTS * sizeof(frame_queue_item_t) / 1024),
             DECODER_CORE, DECODER_PRIO);
    return ESP_OK;
}

bool frame_decoder_push(const uint8_t *bits, size_t n_bits,
                        ir_direction_t direction,
                        uint32_t freq_hz, int peak_bin, float snr_db)
{
    if (!s_initialised || !bits) return false;
    if (n_bits == 0 || n_bits > FRAME_QUEUE_MAX_BITS) return false;

    frame_queue_item_t item;
    item.timestamp_us = (uint32_t)esp_timer_get_time();
    item.freq_hz      = freq_hz;
    item.peak_bin     = peak_bin;
    item.snr_db       = snr_db;
    item.n_bits       = (uint16_t)n_bits;
    item.direction    = (uint8_t)((direction == DIR_DOWNLINK) ? 0 : 1);
    item.pad          = 0;
    memcpy(item.bits, bits, n_bits);
    if (n_bits < FRAME_QUEUE_MAX_BITS) {
        memset(item.bits + n_bits, 0, FRAME_QUEUE_MAX_BITS - n_bits);
    }
    return frame_queue_push(s_queue, &item);
}

uint64_t frame_decoder_pushed(void)
{
    return s_queue ? frame_queue_pushed(s_queue) : 0;
}

uint64_t frame_decoder_popped(void)
{
    return s_queue ? frame_queue_popped(s_queue) : 0;
}

uint64_t frame_decoder_dropped(void)
{
    return s_queue ? frame_queue_dropped(s_queue) : 0;
}

size_t frame_decoder_queue_count(void)
{
    return s_queue ? frame_queue_count(s_queue) : 0;
}

void frame_decoder_get_class_counts(frame_decoder_class_counts_t *out)
{
    if (!out) return;
    out->unknown  = atomic_load_explicit(&s_class_unknown,  memory_order_relaxed);
    out->ms       = atomic_load_explicit(&s_class_ms,       memory_order_relaxed);
    out->tl       = atomic_load_explicit(&s_class_tl,       memory_order_relaxed);
    out->bc       = atomic_load_explicit(&s_class_bc,       memory_order_relaxed);
    out->lw_da    = atomic_load_explicit(&s_class_lw_da,    memory_order_relaxed);
    out->lw_other = atomic_load_explicit(&s_class_lw_other, memory_order_relaxed);
}
