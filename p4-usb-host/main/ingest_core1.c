#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "ingest_core1.h"
#include "signal_buffer.h"
#include "resample_256_to_250.h"

static const char *TAG = "INGEST";

// Per-slot state. Two slots alternated per consumer cycle.
static uint8_t *s_raw[INGEST_NUM_SLOTS];     // raw uint8 USB ingress, internal SRAM, DMA-aligned
static int16_t *s_conv[INGEST_NUM_SLOTS];    // converted int16 Q15 @ 2.56 MSPS (scratch, internal SRAM)
static int16_t *s_resamp[INGEST_NUM_SLOTS];  // resampled int16 Q15 @ 2.5 MSPS (downstream feed, internal SRAM)
static size_t   s_resamp_n_int16[INGEST_NUM_SLOTS];  // int16 element count in s_resamp

// 125/128 polyphase rational resampler (2.56 → 2.5 MSPS). gri's
// wideband fft_burst_tagger expects 2.5 MSPS exactly; the entire
// downstream tagger / direct_if_decim / burst_pipeline tuning is
// against gri's defaults at that rate. Resample once here so
// signal_buffer and dsp_processor both see the gri-aligned rate.
//
// Resampler instance is shared across slots (its delay line is
// stateful — each call advances the polyphase phase counter). At
// 8192 complex input per call the output is ~8000 complex, varying
// by ±1 with phase tracking.
static resample_256_to_250_t s_rs;

// Semaphores per slot. ready[i]: given by ingest task when conversion +
// signal_buffer_push for slot i are complete; taken by class_driver before
// running DSP feed. free[i]: given by class_driver after DSP feed; taken
// by ingest task before reusing the slot.
static SemaphoreHandle_t s_ready[INGEST_NUM_SLOTS];
static SemaphoreHandle_t s_free[INGEST_NUM_SLOTS];

// Dispatch queue: class_driver posts (slot, bytes) tuples; ingest task
// receives them. Length 4 — enough for the 2 slots in flight plus a small
// runway, but small enough that a stuck ingest blocks dispatch quickly.
typedef struct {
    int    slot;
    size_t bytes;
} dispatch_msg_t;
static QueueHandle_t s_dispatch;

// Consumer-side raw acquisition state. The class_driver alternates between
// slots; we track the next slot to allocate so we can wait on s_free[i]
// before handing the raw pointer back.
static int s_next_acquire_slot = 0;

// Diagnostic accumulators (reset by ingest_core1_get_stats).
static volatile uint64_t s_acc_convert_us = 0;
static volatile uint64_t s_acc_push_us = 0;
static volatile uint32_t s_acc_dispatches = 0;
static volatile uint32_t s_acc_slot_wait_us = 0;
static volatile uint32_t s_acc_consumer_waits = 0;

static void ingest_task(void *arg)
{
    ESP_LOGI(TAG, "Ingest task started on Core %d", xPortGetCoreID());
    dispatch_msg_t msg;

    while (1) {
        if (!xQueueReceive(s_dispatch, &msg, portMAX_DELAY)) continue;

        // Slot ownership protocol:
        //   - class_driver took s_free[slot] in acquire_raw → it owns the
        //     slot exclusively until it gives s_free back
        //   - class_driver fills raw and dispatches
        //   - ingest task (here) processes raw → conv, signals s_ready
        //   - class_driver takes s_ready, runs DSP on converted, gives
        //     s_free back
        // We do NOT take s_free here — that would deadlock since
        // class_driver already holds it.

        // 1. Convert raw uint8 -> int16 Q15.
        //   out[i] = (int16_t)((b[i] << 8) ^ 0x8000)
        // Same formula as the in-class_driver path before this offload.
        // 4x unrolled with 32-bit input loads. Buffers are 64-byte aligned.
        int64_t t0 = esp_timer_get_time();
        const uint8_t *__restrict src = s_raw[msg.slot];
        int16_t *__restrict dst = s_conv[msg.slot];
        size_t n = msg.bytes;
        size_t n4 = n & ~(size_t)3;
        size_t i = 0;
        for (; i < n4; i += 4) {
            uint32_t b4 = *(const uint32_t *)(src + i);
            dst[i + 0] = (int16_t)((((b4 >>  0) & 0xff) << 8) ^ 0x8000);
            dst[i + 1] = (int16_t)((((b4 >>  8) & 0xff) << 8) ^ 0x8000);
            dst[i + 2] = (int16_t)((((b4 >> 16) & 0xff) << 8) ^ 0x8000);
            dst[i + 3] = (int16_t)((((b4 >> 24) & 0xff) << 8) ^ 0x8000);
        }
        for (; i < n; i++) {
            dst[i] = (int16_t)(((src[i] << 8) ^ 0x8000));
        }
        int64_t t1 = esp_timer_get_time();
        s_acc_convert_us += (uint64_t)(t1 - t0);

        // 2. Resample 2.56 → 2.5 MSPS (125/128 polyphase). gri's
        // wideband tagger / direct_if_decim are tuned at 2.5 MSPS;
        // performing this once here keeps the entire downstream chain
        // (signal_buffer, dsp_processor, worker) at the gri-aligned
        // rate. Input is n/2 complex samples (interleaved IQ in
        // s_conv); output is ~n/2 × 125/128 complex into s_resamp.
        int n_in_complex = (int)(n / 2);
        int n_out_complex = resample_256_to_250_process(&s_rs,
                                                         dst,
                                                         n_in_complex,
                                                         s_resamp[msg.slot]);
        int n_out_int16 = n_out_complex * 2;
        s_resamp_n_int16[msg.slot] = (size_t)n_out_int16;

        // 3. Push resampled samples into the PSRAM circular buffer.
        // signal_buffer_push is itself async (AXI-GDMA from Step 2) so the
        // CPU cost here is just descriptor programming + cache flush.
        signal_buffer_push(s_resamp[msg.slot], (size_t)n_out_complex);
        int64_t t2 = esp_timer_get_time();
        s_acc_push_us += (uint64_t)(t2 - t1);

        s_acc_dispatches++;
        xSemaphoreGive(s_ready[msg.slot]);
    }
}

esp_err_t ingest_core1_init(void)
{
    // Allocate ping-pong buffers.
    //
    // s_raw[]: USB DWC OTG DMA target. MUST be in DMA-capable internal
    //   SRAM because CONFIG_USB_HOST_DWC_DMA_CAP_MEMORY_IN_PSRAM=n.
    //
    // s_conv[]: CPU-only path -- written by the uint8->int16 convert
    //   loop (above this fn), read by resample_256_to_250_process.
    //   No DMA engine touches it. Previously flagged
    //   MALLOC_CAP_INTERNAL|MALLOC_CAP_DMA "for safety" but that
    //   over-restriction starved the USB transfer pool of internal
    //   DMA SRAM (esp_libusb's bulk-IN allocations failed at
    //   start_stream with ESP_ERR_NO_MEM). Moved to PSRAM; freed
    //   64 KB DMA-internal (2 slots * 32 KB). Cost: CPU writes go
    //   to PSRAM @ ~100 MB/s vs ~700 MB/s internal SRAM -> ~160 us
    //   extra per 16 ms slot, ~1% real-time overhead.
    //
    // s_resamp[]: AXI-GDMA reads from here into signal_buffer (also
    //   PSRAM). AXI handles PSRAM sources fine. CPU touch is one
    //   linear write pass per cycle. Already in PSRAM.
    for (int i = 0; i < INGEST_NUM_SLOTS; i++) {
        s_raw[i] = heap_caps_aligned_alloc(64, 16 * 1024,
                                           MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        s_conv[i] = heap_caps_aligned_alloc(64, INGEST_SLOT_ELEMS * sizeof(int16_t),
                                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        s_resamp[i] = heap_caps_aligned_alloc(64, INGEST_SLOT_ELEMS * sizeof(int16_t),
                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        if (!s_raw[i] || !s_conv[i] || !s_resamp[i]) {
            ESP_LOGE(TAG, "Slot %d alloc failed (raw=%p conv=%p resamp=%p)",
                     i, s_raw[i], s_conv[i], s_resamp[i]);
            return ESP_ERR_NO_MEM;
        }
        s_resamp_n_int16[i] = 0;

        s_ready[i] = xSemaphoreCreateBinary();
        s_free[i]  = xSemaphoreCreateBinary();
        if (!s_ready[i] || !s_free[i]) return ESP_ERR_NO_MEM;
        // Initially: free=1 (slot available), ready=0 (no data yet).
        xSemaphoreGive(s_free[i]);
    }

    // Shared 125/128 resampler (state persists across slots — see
    // module-level comment).
    resample_256_to_250_init(&s_rs);

    s_dispatch = xQueueCreate(4, sizeof(dispatch_msg_t));
    if (!s_dispatch) return ESP_ERR_NO_MEM;

    s_next_acquire_slot = 0;

    // Spawn at higher priority than worker_core1 (5) so a busy worker
    // doesn't stall USB ingest. Watch for inversion if both cores hit
    // PSRAM hard.
    BaseType_t ok = xTaskCreatePinnedToCore(ingest_task, "ingest_core1",
                                            8192, NULL, 8, NULL, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreatePinnedToCore(ingest_core1) failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Ingest pipeline initialised (2 slots × 32 KB)");
    return ESP_OK;
}

uint8_t *ingest_core1_acquire_raw(int *out_slot)
{
    int slot = s_next_acquire_slot;
    // The slot must be free before the consumer overwrites the raw buffer.
    // In steady state this is immediate (ingest typically runs faster than
    // DSP feed). Time the wait so we can see contention if it shows up.
    int64_t t_wait = esp_timer_get_time();
    xSemaphoreTake(s_free[slot], portMAX_DELAY);
    uint32_t waited = (uint32_t)(esp_timer_get_time() - t_wait);
    if (waited > 100) {  // skip noise; only count meaningful waits
        s_acc_slot_wait_us += waited;
        s_acc_consumer_waits++;
    }
    *out_slot = slot;
    s_next_acquire_slot = (slot + 1) % INGEST_NUM_SLOTS;
    return s_raw[slot];
}

void ingest_core1_dispatch(int slot, size_t bytes_filled)
{
    dispatch_msg_t msg = { .slot = slot, .bytes = bytes_filled };
    // Non-blocking send. The queue depth (4) exceeds the slot count (2),
    // so a successful acquire always implies space available here.
    xQueueSend(s_dispatch, &msg, 0);
}

int16_t *ingest_core1_take_converted(int slot, size_t *out_n_int16)
{
    xSemaphoreTake(s_ready[slot], portMAX_DELAY);
    // Returns the RESAMPLED buffer (2.5 MSPS) and its int16 count —
    // post-Phase 3.6.M cutover this is what downstream DSP wants.
    if (out_n_int16) *out_n_int16 = s_resamp_n_int16[slot];
    return s_resamp[slot];
}

void ingest_core1_release(int slot)
{
    xSemaphoreGive(s_free[slot]);
}

void ingest_core1_get_stats(ingest_stats_t *out)
{
    out->convert_us_total   = s_acc_convert_us;
    out->push_us_total      = s_acc_push_us;
    out->dispatches         = s_acc_dispatches;
    out->slot_wait_total_us = s_acc_slot_wait_us;
    out->consumer_waits     = s_acc_consumer_waits;
    s_acc_convert_us = 0;
    s_acc_push_us = 0;
    s_acc_dispatches = 0;
    s_acc_slot_wait_us = 0;
    s_acc_consumer_waits = 0;
}
