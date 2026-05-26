#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_async_memcpy.h"
#include "esp_cache.h"
#include "sdkconfig.h"
#include "signal_buffer.h"

static const char *TAG = "SIG_BUF";

// ESP32-P4 v1.3 silicon errata MSPI-750 guardrail. The PSRAM DMA path
// requires byte-aligned bursts; our PSRAM writes here use 4-byte
// alignment (n_samples * 4 bytes per transfer, head_bytes = head * 4
// for the destination offset) which is enough as long as IDF GDMA
// keeps weighted-arbitration OFF. If a future build turns on
// CONFIG_GDMA_ENABLE_WEIGHTED_ARBITRATION, GDMA will raise the
// required alignment to dma_burst_size (= 64 below) and our 4-byte-
// multiple lengths could fail validation, silently dropping into a
// slow fallback or returning an error. Catch that at compile time.
#ifdef CONFIG_GDMA_ENABLE_WEIGHTED_ARBITRATION
_Static_assert(0,
    "GDMA weighted arbitration changes alignment requirements; review "
    "signal_buffer_push's 4-byte multiples vs dma_burst_size=64 before "
    "enabling. See memory/project_p4_errata_status.md (MSPI-750).");
#endif

// 4 MB circular buffer in PSRAM. int16 IQ pairs:
//   circular_buf[head*2 + 0] = I
//   circular_buf[head*2 + 1] = Q
static int16_t *circular_buf = NULL;
static uint32_t head = 0;        // in complex samples

// AXI-GDMA async memcpy. signal_buffer_push fires PSRAM writes off to this
// channel so Core 0 doesn't block on the ~800 us PSRAM transfer per cycle.
// AXI master is the right choice here: USB DWC OTG-HS sits on AHB; using
// a separate AXI channel for PSRAM avoids cross-traffic on the AHB master.
static async_memcpy_handle_t s_dma = NULL;

// Binary semaphore given by the completion ISR. Pre-given at init so the
// very first push doesn't block. Each push takes the semaphore (waits for
// previous DMA done) before submitting; the second segment of a wrap
// transfer carries the give-back callback.
static SemaphoreHandle_t s_dma_done = NULL;

// Deadlock-guard diagnostics (#106). dma_submit_errors: esp_async_memcpy
// returned non-OK (its completion callback then never fires). dma_timeouts:
// the s_dma_done wait timed out (a previous DMA's give never came). Either,
// untreated, would hang signal_buffer_push forever -> ingest never signals
// s_ready -> class deadlocks in take_converted.
static volatile uint32_t s_dma_submit_errors = 0;
static volatile uint32_t s_dma_timeouts      = 0;

uint32_t signal_buffer_dma_submit_errors(void) { return s_dma_submit_errors; }
uint32_t signal_buffer_dma_timeouts(void)      { return s_dma_timeouts; }

static IRAM_ATTR bool dma_done_cb(async_memcpy_handle_t mcp,
                                  async_memcpy_event_t *evt, void *arg)
{
    BaseType_t hp_woken = pdFALSE;
    xSemaphoreGiveFromISR(s_dma_done, &hp_woken);
    return hp_woken == pdTRUE;
}

esp_err_t signal_buffer_init()
{
    ESP_LOGI(TAG, "Allocating 4MB Signal Buffer in PSRAM (DMA-aligned)...");
    // 64-byte cache-line alignment for the DMA destination.
    circular_buf = heap_caps_aligned_alloc(64, SIGNAL_BUF_SIZE,
                                           MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!circular_buf) return ESP_ERR_NO_MEM;
    memset(circular_buf, 0, SIGNAL_BUF_SIZE);
    head = 0;

    async_memcpy_config_t cfg = ASYNC_MEMCPY_DEFAULT_CONFIG();
    cfg.backlog = 4;          // up to 4 outstanding transfers
    cfg.dma_burst_size = 64;  // match L2 cache line for efficient bursts
    esp_err_t r = esp_async_memcpy_install_gdma_axi(&cfg, &s_dma);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "esp_async_memcpy_install_gdma_axi failed: 0x%x (%s)",
                 r, esp_err_to_name(r));
        return r;
    }

    s_dma_done = xSemaphoreCreateBinary();
    if (!s_dma_done) return ESP_ERR_NO_MEM;
    xSemaphoreGive(s_dma_done);  // first push doesn't wait

    ESP_LOGI(TAG, "Signal buffer + AXI-GDMA installed");
    return ESP_OK;
}

void signal_buffer_push(const int16_t *samples, size_t n_samples)
{
    if (!circular_buf || !s_dma) return;

    const uint32_t total_cap = SIGNAL_BUF_SIZE / 4;  // complex samples

    // Wait for the previous DMA to finish before reusing circular_buf at
    // the (potentially old) head pointer or before the caller's `samples`
    // gets overwritten by the next class_driver loop iteration.
    //
    // TIMEOUT, not portMAX_DELAY: if a previous DMA's completion callback
    // never fired (a failed/lost async_memcpy submit, or a driver wedge), the
    // semaphore would never be returned and this take would hang FOREVER —
    // ingest then never signals s_ready and class deadlocks in
    // ingest_core1_take_converted (#106). A real DMA completes in ~microsec-
    // onds, so a 250 ms wait means the previous DMA is dead; proceed (this
    // push's own callback re-arms the semaphore on the next cycle).
    if (xSemaphoreTake(s_dma_done, pdMS_TO_TICKS(250)) != pdTRUE) {
        s_dma_timeouts++;
    }

    size_t bytes = n_samples * 4;
    uint8_t *dst_base = (uint8_t *)circular_buf;
    uint32_t head_bytes = head * 4;
    size_t bytes_to_end = (uint32_t)SIGNAL_BUF_SIZE - head_bytes;

    esp_err_t r;
    if (bytes <= bytes_to_end) {
        // Common path: single contiguous write.
        r = esp_async_memcpy(s_dma, dst_base + head_bytes, (void *)samples, bytes,
                             dma_done_cb, NULL);
    } else {
        // Wrap: two writes. Only the second carries the completion callback
        // so the semaphore is given exactly once.
        (void)esp_async_memcpy(s_dma, dst_base + head_bytes, (void *)samples,
                               bytes_to_end, NULL, NULL);
        size_t remainder = bytes - bytes_to_end;
        r = esp_async_memcpy(s_dma, dst_base, (uint8_t *)samples + bytes_to_end,
                             remainder, dma_done_cb, NULL);
    }
    if (r != ESP_OK) {
        // The callback-carrying submit failed → dma_done_cb will never fire →
        // s_dma_done would never be returned and the NEXT push would deadlock
        // (#106). Give it back ourselves so the pipeline keeps moving; this
        // chunk is dropped (head not advanced). Log it — this is the failure
        // message that was previously swallowed by the unchecked return.
        s_dma_submit_errors++;
        if ((s_dma_submit_errors & 0x3f) == 1) {   // rate-limit to ~1/64
            ESP_LOGW(TAG, "esp_async_memcpy submit failed: %s (n=%u) — chunk dropped, sem restored",
                     esp_err_to_name(r), (unsigned)n_samples);
        }
        xSemaphoreGive(s_dma_done);
        return;   // do not advance head: this chunk was not (fully) written
    }

    head = (head + (uint32_t)n_samples) % total_cap;
}

uint32_t signal_buffer_head(void)
{
    // Single 32-bit read of a producer-updated counter. The worker
    // reading this can see at worst a slightly-stale value, which only
    // biases stale-burst checks toward false-positive (skip a burst
    // that was actually fine). That's acceptable; we never get a false
    // negative (think a burst is valid when it's not).
    return head;
}

bool signal_buffer_burst_valid(uint32_t start_idx, uint32_t length)
{
    const uint32_t total_cap = SIGNAL_BUF_SIZE / 4;
    if (length == 0 || length >= total_cap) return false;

    // Distance from the burst's END (in producer order) to the current
    // head, modulo wrap. If this exceeds (total_cap - length), the
    // producer has lapped onto the burst's window — data is gone.
    uint32_t end       = (start_idx + length) % total_cap;
    uint32_t since_end = (head - end + total_cap) % total_cap;
    return since_end <= (total_cap - length);
}

void signal_buffer_extract(uint32_t start_idx, uint32_t length, int16_t *dest)
{
    if (!circular_buf) return;

    // The DMA writes into PSRAM bypass any CPU caches on Core 0. Core 1's
    // CPU caches may hold stale lines for the region we just wrote, so
    // invalidate before the worker reads. M2C + INVALIDATE drops cached
    // lines so the next reads pull fresh data from PSRAM.
    const uint32_t total_cap = SIGNAL_BUF_SIZE / 4;
    uint32_t actual_start = start_idx % total_cap;
    size_t bytes_to_read = length * 4;
    uint8_t *base = (uint8_t *)circular_buf;
    uint32_t start_bytes = actual_start * 4;
    size_t to_end_bytes = (uint32_t)SIGNAL_BUF_SIZE - start_bytes;

    if (bytes_to_read <= to_end_bytes) {
        esp_cache_msync(base + start_bytes, bytes_to_read,
                        ESP_CACHE_MSYNC_FLAG_DIR_M2C |
                        ESP_CACHE_MSYNC_FLAG_INVALIDATE);
    } else {
        esp_cache_msync(base + start_bytes, to_end_bytes,
                        ESP_CACHE_MSYNC_FLAG_DIR_M2C |
                        ESP_CACHE_MSYNC_FLAG_INVALIDATE);
        size_t rem = bytes_to_read - to_end_bytes;
        esp_cache_msync(base, rem,
                        ESP_CACHE_MSYNC_FLAG_DIR_M2C |
                        ESP_CACHE_MSYNC_FLAG_INVALIDATE);
    }

    // Per-element copy across the wrap. Runs once per detected burst (not
    // per sample on the hot path), so the loop overhead is fine relative
    // to the rest of the worker pipeline.
    for (uint32_t i = 0; i < length; i++) {
        uint32_t idx = (actual_start + i) % total_cap;
        dest[i * 2 + 0] = circular_buf[idx * 2 + 0];
        dest[i * 2 + 1] = circular_buf[idx * 2 + 1];
    }
}

void signal_buffer_invalidate_range(uint32_t start_idx, uint32_t length)
{
    if (!circular_buf) return;
    const uint32_t total_cap = SIGNAL_BUF_SIZE / 4;
    uint32_t actual_start = start_idx % total_cap;
    size_t bytes_to_read = length * 4;
    uint8_t *base = (uint8_t *)circular_buf;
    uint32_t start_bytes = actual_start * 4;
    size_t to_end_bytes = (uint32_t)SIGNAL_BUF_SIZE - start_bytes;

    if (bytes_to_read <= to_end_bytes) {
        esp_cache_msync(base + start_bytes, bytes_to_read,
                        ESP_CACHE_MSYNC_FLAG_DIR_M2C |
                        ESP_CACHE_MSYNC_FLAG_INVALIDATE);
    } else {
        esp_cache_msync(base + start_bytes, to_end_bytes,
                        ESP_CACHE_MSYNC_FLAG_DIR_M2C |
                        ESP_CACHE_MSYNC_FLAG_INVALIDATE);
        size_t rem = bytes_to_read - to_end_bytes;
        esp_cache_msync(base, rem,
                        ESP_CACHE_MSYNC_FLAG_DIR_M2C |
                        ESP_CACHE_MSYNC_FLAG_INVALIDATE);
    }
}

void signal_buffer_read_chunk(uint32_t start_idx, uint32_t length, int16_t *dest)
{
    if (!circular_buf) return;
    const uint32_t total_cap = SIGNAL_BUF_SIZE / 4;
    uint32_t actual_start = start_idx % total_cap;
    // Fast path: chunk is fully contiguous (no wrap). memcpy beats the
    // per-element loop -- it can do 16-byte burst PSRAM reads via the
    // L2 cache prefetcher.
    if (actual_start + length <= total_cap) {
        memcpy(dest, &circular_buf[actual_start * 2],
               (size_t)length * 4);
        return;
    }
    // Wrap: two memcpys.
    uint32_t to_end = total_cap - actual_start;
    memcpy(dest, &circular_buf[actual_start * 2], (size_t)to_end * 4);
    memcpy(dest + to_end * 2, &circular_buf[0],
           (size_t)(length - to_end) * 4);
}
