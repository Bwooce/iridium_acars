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

// 64-byte cache-line alignment infrastructure for the DMA path (#125).
// Pre-quick-wins, signal_buffer_push passed raw n_complex×4 byte lengths
// straight to esp_async_memcpy. The destination offset (head_bytes) is
// 64-aligned at allocation, but advanced by n_complex×4 each push — so
// any push with n_complex not a multiple of 16 left head_bytes mis-
// aligned for the NEXT push. GDMA then fell into the cache-alignment
// split path on every push, which itself allocates a "stash buffer" per
// transfer from DMA-INT — at ~300/min that path overran fragmentation
// and logged ~300 errors/min "no mem for stash buffer". The recovery in
// #106/#107 absorbed the visible ESP_ERR_NO_MEM hits, but ~0.16% of
// chunks were silently dropped each minute.
//
// Fix: round each push down to a 16-complex (64-byte) multiple and
// carry the 0..15-complex tail forward into the next push. All DMA
// submits now use 64-byte-aligned src offset (start of scratch),
// 64-byte-aligned dest offset (head_bytes always advances by multiples
// of 64), and 64-byte-multiple length. No more split path triggered.
//
// Scratch must be DMA-readable and 64-aligned. PSRAM (cap-DMA) is fine
// for source; sized to fit one max push (INGEST_SLOT_ELEMS complex =
// 32 KB).
#define ALIGN_COMPLEX           16   // 16 complex × 4 B = 64 B = one cache line
#define ALIGN_BYTES             (ALIGN_COMPLEX * 4)
#define ALIGN_SCRATCH_MAX_BYTES (16 * 1024 * 4)   // 16 K complex × 4 B = 64 KB
static int16_t *s_align_scratch = NULL;
// Carry: 0..15 complex samples = at most 60 bytes of "left over" from
// the previous push, prepended to the next push so no samples are lost.
static int16_t  s_carry[ALIGN_COMPLEX * 2];   // 16 complex × 2 int16 = 64 B
static uint8_t  s_carry_n_complex = 0;

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
    // Note: IDF v6.1's async_memcpy_config_t doesn't expose
    // psram_trans_align / sram_trans_align (those were earlier-IDF
    // fields). We can't tell GDMA "skip the split-RX path because
    // our buffers are guaranteed 64-aligned" — even though the
    // 64-aligned scratch + 64-aligned head invariant from #125 makes
    // that true, the driver re-validates and triggers the split path
    // anyway. Result: residual ~110 split-RX errors/min vs the ~300/min
    // pre-#125 baseline (63% reduction). Going further would need
    // either an IDF source patch or a different memcpy mechanism
    // (e.g. direct gdma_link API).
    esp_err_t r = esp_async_memcpy_install_gdma_axi(&cfg, &s_dma);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "esp_async_memcpy_install_gdma_axi failed: 0x%x (%s)",
                 r, esp_err_to_name(r));
        return r;
    }

    // 64-aligned scratch in PSRAM (cap-DMA). One max-push worth of complex
    // samples; the carry tail (<= 15 complex) prepended to each push lives
    // in BSS s_carry and gets copied to the head of this scratch. See the
    // long comment above (#125).
    s_align_scratch = heap_caps_aligned_alloc(64, ALIGN_SCRATCH_MAX_BYTES,
                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!s_align_scratch) {
        ESP_LOGE(TAG, "alignment scratch alloc (%d B) failed", (int)ALIGN_SCRATCH_MAX_BYTES);
        return ESP_ERR_NO_MEM;
    }
    s_carry_n_complex = 0;

    s_dma_done = xSemaphoreCreateBinary();
    if (!s_dma_done) return ESP_ERR_NO_MEM;
    xSemaphoreGive(s_dma_done);  // first push doesn't wait

    ESP_LOGI(TAG, "Signal buffer + AXI-GDMA installed (align scratch %d B PSRAM)",
             (int)ALIGN_SCRATCH_MAX_BYTES);
    return ESP_OK;
}

void signal_buffer_push(const int16_t *samples, size_t n_samples)
{
    if (!circular_buf || !s_dma) return;

    const uint32_t total_cap = SIGNAL_BUF_SIZE / 4;  // complex samples

    // Cache-line alignment via carry-forward (#125). Available = previous
    // carry + this push. If less than ALIGN_COMPLEX (16), accumulate in
    // carry and return — no DMA this cycle, no head advance.
    size_t total_avail = (size_t)s_carry_n_complex + n_samples;
    if (total_avail < ALIGN_COMPLEX) {
        if (n_samples > 0) {
            memcpy(s_carry + (size_t)s_carry_n_complex * 2, samples, n_samples * 4);
            s_carry_n_complex = (uint8_t)total_avail;
        }
        return;
    }

    // Round down to 16-complex granularity. The remaining 0..15 complex
    // samples become the new carry for the next push (no sample loss).
    size_t aligned_count    = total_avail & ~((size_t)(ALIGN_COMPLEX - 1));
    size_t new_carry_count  = total_avail - aligned_count;
    size_t aligned_bytes    = aligned_count * 4;
    // Sanity: scratch is sized for one max push; very large overruns are
    // a caller error. Clamp defensively rather than overflow.
    if (aligned_bytes > (size_t)ALIGN_SCRATCH_MAX_BYTES) {
        ESP_LOGW(TAG, "push %u complex exceeds scratch (%d B max) — clamping",
                 (unsigned)aligned_count, (int)ALIGN_SCRATCH_MAX_BYTES);
        aligned_bytes  = ALIGN_SCRATCH_MAX_BYTES & ~((size_t)63);
        aligned_count  = aligned_bytes / 4;
        new_carry_count = total_avail - aligned_count;
    }

    // Wait for the previous DMA BEFORE we overwrite s_align_scratch (which
    // the previous DMA may still be reading).
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

    // Build contiguous aligned source in scratch:
    //   [s_carry_n_complex carry samples] then [aligned_count - s_carry_n
    //    samples from caller's src]. Both copies into 64-aligned scratch,
    //    cumulative length = aligned_bytes (multiple of 64).
    size_t carry_bytes = (size_t)s_carry_n_complex * 4;
    size_t src_take    = aligned_count - s_carry_n_complex;
    if (carry_bytes) {
        memcpy(s_align_scratch, s_carry, carry_bytes);
    }
    memcpy(((uint8_t *)s_align_scratch) + carry_bytes, samples, src_take * 4);

    uint8_t *dst_base = (uint8_t *)circular_buf;
    uint32_t head_bytes = head * 4;   // 64-aligned by invariant (#125)
    size_t bytes_to_end = (uint32_t)SIGNAL_BUF_SIZE - head_bytes;

    esp_err_t r;
    if (aligned_bytes <= bytes_to_end) {
        // Common path: single contiguous write. All three (src, dst, len)
        // are 64-aligned, so no cache-split-RX path triggered.
        r = esp_async_memcpy(s_dma, dst_base + head_bytes, s_align_scratch,
                             aligned_bytes, dma_done_cb, NULL);
    } else {
        // Wrap: two writes. SIGNAL_BUF_SIZE is 64-multiple and head_bytes
        // is 64-aligned, so bytes_to_end is 64-aligned. aligned_bytes is
        // 64-multiple. Both submits are 64-aligned in src offset, dst
        // offset, and length. (#107 wrap-failure handling preserved.)
        r = esp_async_memcpy(s_dma, dst_base + head_bytes, s_align_scratch,
                             bytes_to_end, NULL, NULL);
        if (r == ESP_OK) {
            size_t remainder = aligned_bytes - bytes_to_end;
            r = esp_async_memcpy(s_dma, dst_base,
                                 ((uint8_t *)s_align_scratch) + bytes_to_end,
                                 remainder, dma_done_cb, NULL);
        }
    }
    if (r != ESP_OK) {
        // The callback-carrying submit failed → dma_done_cb will never fire →
        // s_dma_done would never be returned and the NEXT push would deadlock
        // (#106). Give it back ourselves so the pipeline keeps moving; this
        // chunk is dropped (head not advanced) — the carry is also preserved
        // unchanged so no sample is lost on retry.
        s_dma_submit_errors++;
        if ((s_dma_submit_errors & 0x3f) == 1) {   // rate-limit to ~1/64
            ESP_LOGW(TAG, "esp_async_memcpy submit failed: %s (n=%u) — chunk dropped, sem restored",
                     esp_err_to_name(r), (unsigned)aligned_count);
        }
        xSemaphoreGive(s_dma_done);
        return;   // do not advance head: this chunk was not (fully) written
    }

    head = (head + (uint32_t)aligned_count) % total_cap;

    // Update carry with the tail of THIS push's source (samples we deferred).
    if (new_carry_count) {
        size_t tail_offset_complex = n_samples - new_carry_count;
        memcpy(s_carry, samples + tail_offset_complex * 2, new_carry_count * 4);
    }
    s_carry_n_complex = (uint8_t)new_carry_count;
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
