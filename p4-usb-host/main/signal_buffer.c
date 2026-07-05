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
#include "signal_buffer_ring.h"
#include "fault_inject.h"

static const char *TAG = "SIG_BUF";

// AXI-GDMA alignment guardrail. (NB: this is NOT an MSPI-750 guard, as
// an earlier comment claimed — MSPI-750 is a v3.0-only erratum and does
// not affect our v1.x silicon; it's also about USB/SDMMC unaligned reads,
// not GDMA writes.) The real concern is purely GDMA arbitration mode: our
// PSRAM writes use 4-byte alignment (n_samples * 4 bytes per transfer,
// head_bytes = head * 4 for the destination offset), which GDMA accepts
// while weighted-arbitration is OFF. If a future build turns on
// CONFIG_GDMA_ENABLE_WEIGHTED_ARBITRATION, GDMA raises the required
// alignment to dma_burst_size (= 64 below) and our 4-byte-multiple
// lengths could fail validation, silently dropping into a slow fallback
// or returning an error. Catch that at compile time.
#ifdef CONFIG_GDMA_ENABLE_WEIGHTED_ARBITRATION
_Static_assert(0,
               "GDMA weighted arbitration raises alignment to dma_burst_size; review "
               "signal_buffer_push's 4-byte multiples vs dma_burst_size=64 before "
               "enabling.");
#endif

// 4 MB circular buffer in PSRAM. int16 IQ pairs:
//   circular_buf[head*2 + 0] = I
//   circular_buf[head*2 + 1] = Q
static int16_t *circular_buf = NULL;
static uint32_t head         = 0; // in complex samples

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
#define ALIGN_COMPLEX 16 // 16 complex × 4 B = 64 B = one cache line
#define ALIGN_BYTES (ALIGN_COMPLEX * 4)
#define ALIGN_SCRATCH_MAX_BYTES (16 * 1024 * 4) // 16 K complex × 4 B = 64 KB
static int16_t *s_align_scratch = NULL;
// Carry: 0..15 complex samples = at most 60 bytes of "left over" from
// the previous push, prepended to the next push so no samples are lost.
static int16_t s_carry[ALIGN_COMPLEX * 2]; // 16 complex × 2 int16 = 64 B
static uint8_t s_carry_n_complex = 0;

// Binary semaphore given by the completion ISR. Pre-given at init so the
// very first push doesn't block. Each push takes the semaphore (waits for
// previous DMA done) before submitting; the second segment of a wrap
// transfer carries the give-back callback.
static SemaphoreHandle_t s_dma_done = NULL;

// Deadlock-guard diagnostics (#106). Named to match what they actually
// measure (was dma_submit_errors / dma_cpu_fallbacks until 2026-05-31):
//
//   stash_alloc_fails  — IDF dma_utils failed to allocate the 128 B
//     cache-line stash buffer; esp_async_memcpy returns ESP_ERR_NO_MEM.
//     This IS the count of "no mem for stash buffer" events that
//     LOG_VERSION_2 + esp_log_level_set silence in the UART log.
//   stash_alloc_recoveries — submit fails that we recovered via CPU
//     memcpy: the simple path (#126E) and the wrap path when neither
//     segment reached GDMA. stash_alloc_fails - stash_alloc_recoveries
//     is now just the wrap sub-case where the first segment's GDMA was
//     already in flight when the second failed to submit — the ring
//     gets a garbage window there, but head still advances (no more
//     permanent index desync; see signal_buffer_push's wrap-fail
//     handling).
//   dma_timeouts — s_dma_done wait timed out (a previous DMA's give
//     never came). Untreated would hang signal_buffer_push forever ->
//     ingest never signals s_ready -> class deadlocks in
//     take_converted. Has never fired in production.
static volatile uint32_t s_stash_alloc_fails      = 0;
static volatile uint32_t s_stash_alloc_recoveries = 0;
static volatile uint32_t s_dma_timeouts           = 0;
// Complex samples dropped by the oversized-push clamp in
// signal_buffer_push (caller contract violation; should stay 0).
static volatile uint32_t s_clamp_dropped_complex = 0;

uint32_t signal_buffer_stash_alloc_fails(void)
{
    return s_stash_alloc_fails;
}
uint32_t signal_buffer_stash_alloc_recoveries(void)
{
    return s_stash_alloc_recoveries;
}
uint32_t signal_buffer_dma_timeouts(void)
{
    return s_dma_timeouts;
}

static IRAM_ATTR bool dma_done_cb(async_memcpy_handle_t mcp,
                                  async_memcpy_event_t *evt, void *arg)
{
    BaseType_t hp_woken = pdFALSE;
    xSemaphoreGiveFromISR(s_dma_done, &hp_woken);
    return hp_woken == pdTRUE;
}

// T44 (partial): every submit-failure recovery branch in signal_buffer_push
// gives s_dma_done back synchronously (see the big comment at that call
// site for why). If the xSemaphoreTake() above timed out first (counted by
// s_dma_timeouts, which "has never fired in production" per that field's
// comment), the take never actually consumed a previous give -- so a STALE
// DMA's completion callback can still be pending and could fire after this
// synchronous give, making the next xSemaphoreGive() here a double-give.
// FreeRTOS binary semaphores already tolerate this safely (a Give at
// count==1 just returns pdFALSE; no overflow, no corruption), so this was
// never a crash risk -- only a silently-absorbed one, which is why it has
// "never fired" by the only signal anyone was watching (s_dma_timeouts).
// Make the redundant-give case observable instead of silently swallowed.
static volatile uint32_t s_dma_give_races = 0;

static inline void give_dma_done_once(const char *why)
{
    if (uxSemaphoreGetCount(s_dma_done) != 0) {
        s_dma_give_races++;
        if ((s_dma_give_races & 0x3f) == 1) { // rate-limit to ~1/64
            ESP_LOGW(TAG, "dma_done semaphore already given (race #%u, %s) — "
                          "skipping redundant give",
                     (unsigned)s_dma_give_races, why);
        }
        return;
    }
    xSemaphoreGive(s_dma_done);
}

esp_err_t signal_buffer_init()
{
    // Suppress the IDF GDMA noise that fires once per failed cache-
    // aligned stash allocation (~100/min on production hardware):
    //   E dma_utils: esp_dma_split_rx_buffer_to_cache_aligned(54):
    //                no mem for stash buffer
    //   E async_mcp.gdma: mcp_gdma_memcpy(411): failed to split RX
    //                buffer into aligned ones
    // These are the same events we already count via s_stash_alloc_fails
    // and recover from via #126E's CPU memcpy fallback (100% recovery
    // observed in 33 min soak — zero audio dropped). Per-event ESP_LOGE
    // formatting + UART output burned a measurable slice of Core 0 for
    // information we already track at our layer. Bumping the tag to
    // ERROR-only would still print these; need NONE to silence.
    esp_log_level_set("dma_utils", ESP_LOG_NONE);
    esp_log_level_set("async_mcp.gdma", ESP_LOG_NONE);

    ESP_LOGI(TAG, "Allocating 4MB Signal Buffer in PSRAM (DMA-aligned)...");
    // 64-byte cache-line alignment for the DMA destination.
    circular_buf = heap_caps_aligned_alloc(64, SIGNAL_BUF_SIZE,
                                           MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!circular_buf) return ESP_ERR_NO_MEM;
    memset(circular_buf, 0, SIGNAL_BUF_SIZE);
    head = 0;

    async_memcpy_config_t cfg = ASYNC_MEMCPY_DEFAULT_CONFIG();
    cfg.backlog               = 4;  // up to 4 outstanding transfers
    cfg.dma_burst_size        = 64; // match L2 cache line for efficient bursts
    // Note: IDF v6.1's async_memcpy_config_t only has backlog/weight/
    // dma_burst_size/flags (esp_async_memcpy.h) — no psram_trans_align /
    // sram_trans_align field (those were earlier-IDF fields), and `flags`
    // is dead in this version (grep finds no reader of config->flags in
    // async_memcpy_gdma.c or esp_dma_utils.c). There is no config knob
    // that can suppress the split-RX stash allocation.
    //
    // T54 root cause (traced into esp-idf/components/esp_driver_dma,
    // checkout v6.1-dev-4427-gc00874869b): mcp_gdma_memcpy() in
    // async_memcpy_gdma.c unconditionally frees the transaction's
    // previous stash_buffer and sets it to NULL (lines 356-359) on
    // EVERY call, before it knows whether this transfer needs one. It
    // then calls esp_dma_split_rx_buffer_to_cache_aligned() (line 411),
    // which — because *ret_stash_buffer is NULL — unconditionally
    // heap_caps_calloc()s a fresh 2×cache-line stash from
    // MALLOC_CAP_DMA|MALLOC_CAP_INTERNAL (esp_dma_utils.c:50-54) BEFORE
    // it computes head_overflow_len/tail_overflow_len and decides
    // whether the split is actually needed (esp_dma_utils.c:67-99). For
    // our transfers head_overflow_len and tail_overflow_len both work
    // out to 0 (64-aligned src/dst/len per #125's invariant), so the
    // stash is allocated, never written to (head/tail segment lengths
    // are 0), and immediately eligible to be freed on the next push —
    // pure churn of a ~128 B DMA-INT allocation on every single
    // esp_async_memcpy() call to PSRAM, aligned or not. This is a
    // worst-case-provisioning pattern baked into the driver, not
    // something the alignment check gates; there is no public API to
    // hand the driver a persistent stash buffer (the low-level
    // esp_dma_split_rx_buffer_to_cache_aligned() supports reuse via a
    // non-NULL *ret_stash_buffer, but mcp_gdma_memcpy never exercises
    // that path). Avoiding it structurally would mean bypassing
    // esp_async_memcpy entirely and driving the low-level gdma_link_list
    // API ourselves (own channel setup, link-list construction, cache
    // sync) — a materially larger and riskier change than this
    // DSP-path file warrants; not attempted here. The T2 CPU-memcpy
    // recovery (#106/#107/#126E, this function's `r != ESP_OK` branch
    // below) remains the correct mitigation: it's cheap (~50 us/16 KB),
    // 100% effective, and infrequent (fires only when DMA-INT is
    // transiently exhausted by this same churn from other allocators).
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
    xSemaphoreGive(s_dma_done); // first push doesn't wait

    ESP_LOGI(TAG, "Signal buffer + AXI-GDMA installed (align scratch %d B PSRAM)",
             (int)ALIGN_SCRATCH_MAX_BYTES);
    return ESP_OK;
}

void signal_buffer_push(const int16_t *samples, size_t n_samples)
{
    if (!circular_buf || !s_dma) return;

    const uint32_t total_cap = SIGNAL_BUF_SIZE / 4; // complex samples

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
    size_t aligned_count   = total_avail & ~((size_t)(ALIGN_COMPLEX - 1));
    size_t new_carry_count = total_avail - aligned_count;
    size_t aligned_bytes   = aligned_count * 4;
    // Sanity: scratch is sized for one max push; very large overruns are
    // a caller error. Clamp defensively rather than overflow.
    if (aligned_bytes > (size_t)ALIGN_SCRATCH_MAX_BYTES) {
        ESP_LOGW(TAG, "push %u complex exceeds scratch (%d B max) — clamping",
                 (unsigned)aligned_count, (int)ALIGN_SCRATCH_MAX_BYTES);
        aligned_bytes   = ALIGN_SCRATCH_MAX_BYTES & ~((size_t)63);
        aligned_count   = aligned_bytes / 4;
        new_carry_count = total_avail - aligned_count;
        // The carry buffer holds at most ALIGN_COMPLEX-1 samples and its
        // count is a uint8_t. An oversized push would overflow both —
        // corrupting BSS in exactly the contract-violation case this
        // clamp exists to catch. Drop the excess instead and count it.
        if (new_carry_count > (size_t)(ALIGN_COMPLEX - 1)) {
            s_clamp_dropped_complex += new_carry_count - (ALIGN_COMPLEX - 1);
            new_carry_count = ALIGN_COMPLEX - 1;
        }
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

    uint8_t *dst_base     = (uint8_t *)circular_buf;
    uint32_t head_bytes   = head * 4; // 64-aligned by invariant (#125)
    size_t   bytes_to_end = (uint32_t)SIGNAL_BUF_SIZE - head_bytes;

    esp_err_t r;
    bool      wrap = (aligned_bytes > bytes_to_end);
    // Set only on the wrap path, true once the FIRST segment's GDMA submit
    // has succeeded (i.e. it may be in flight reading s_align_scratch /
    // writing the ring tail even though the overall push goes on to fail).
    // Distinguishes the two wrap failure sub-cases below.
    bool wrap_first_submitted = false;
    if (!wrap) {
        // Common path: single contiguous write. All three (src, dst, len)
        // are 64-aligned, so no cache-split-RX path triggered.
        if (fault_inject_should_fail(FI_SITE_DMA_SUBMIT)) {
            r = ESP_ERR_NO_MEM; // synthetic (#122): exercise CPU-memcpy fallback
        } else {
            r = esp_async_memcpy(s_dma, dst_base + head_bytes, s_align_scratch,
                                 aligned_bytes, dma_done_cb, NULL);
        }
    } else {
        // Wrap: two writes. SIGNAL_BUF_SIZE is 64-multiple and head_bytes
        // is 64-aligned, so bytes_to_end is 64-aligned. aligned_bytes is
        // 64-multiple. Both submits are 64-aligned in src offset, dst
        // offset, and length.
        if (fault_inject_should_fail(FI_SITE_DMA_SUBMIT_WRAP)) {
            r = ESP_ERR_NO_MEM; // synthetic (#122): exercise wrap-path first-segment failure
        } else {
            r = esp_async_memcpy(s_dma, dst_base + head_bytes, s_align_scratch,
                                 bytes_to_end, NULL, NULL);
        }
        if (r == ESP_OK) {
            wrap_first_submitted = true;
            size_t remainder     = aligned_bytes - bytes_to_end;
            r                    = esp_async_memcpy(s_dma, dst_base,
                                                    ((uint8_t *)s_align_scratch) + bytes_to_end,
                                                    remainder, dma_done_cb, NULL);
        }
    }
    if (r != ESP_OK) {
        // Submit failed — dma_done_cb won't fire and s_dma_done would
        // never be returned (#106 deadlock class), so every branch below
        // gives it back synchronously. Recovery strategy depends on wrap
        // state AND, on the wrap path, on which of the two segments
        // failed:
        //
        //   Simple path (no wrap, #126E): CPU memcpy fallback. Nothing
        //   was enqueued. Both src+dst are 64-aligned (#125) so it's a
        //   plain block copy; ~50 us per 16 KB on Core 0. No audio
        //   dropped; head advances normally.
        //
        //   Wrap path, neither segment submitted (first failed): nothing
        //   is in flight touching s_align_scratch or the ring, so it's
        //   safe to CPU-recover both segments the same way, in two
        //   pieces (end-of-ring then start-of-ring).
        //
        //   Wrap path, first segment submitted but second failed: the
        //   first GDMA transaction may still be in flight reading
        //   s_align_scratch / writing the ring tail, and it has a NULL
        //   completion callback (#125's design — only the second
        //   segment signals completion), so there is no way to wait for
        //   it before touching either buffer. We do NOT CPU-recover this
        //   case; we accept a single garbage window in the ring.
        //
        // In ALL wrap sub-cases head still advances by aligned_count
        // (see below) instead of being left behind (the old #107
        // behaviour). head is the base for every cumulative sample index
        // -> ring offset mapping (signal_buffer_read_chunk, burst_valid,
        // and the worker's start_idx % total_cap): leaving it unadvanced
        // after the producer's own sample count has already moved on
        // permanently desyncs that mapping for every future burst, not
        // just this one. One bad decode window is far cheaper than a
        // permanent skew.
        s_stash_alloc_fails++;
        if ((s_stash_alloc_fails & 0x3f) == 1) { // rate-limit to ~1/64
            // Log contains both 'stash_alloc_fail' (new canonical name)
            // and 'submit failed' (legacy phrase) so old grep filters
            // and any external dashboards keep matching.
            ESP_LOGW(TAG, "stash_alloc_fail (esp_async_memcpy submit failed): %s "
                          "(n=%u, wrap=%d, wrap_first_submitted=%d) — %s",
                     esp_err_to_name(r), (unsigned)aligned_count, (int)wrap,
                     (int)wrap_first_submitted,
                     !wrap ? "CPU memcpy fallback"
                           : (wrap_first_submitted ? "garbage window, head still advanced"
                                                   : "CPU memcpy fallback (both segments)"));
        }
        if (!wrap) {
            memcpy(dst_base + head_bytes, s_align_scratch, aligned_bytes);
            // The memcpy went through the write-back L2 cache; the GDMA
            // path lands data in PSRAM directly. Write the dirty lines
            // back NOW — the worker's extract/invalidate_range does a
            // discarding M2C+INVALIDATE over burst regions, which would
            // silently replace still-dirty lines with stale PSRAM
            // contents. Address and length are 64-aligned (#125), so
            // this is a clean call.
            esp_cache_msync(dst_base + head_bytes, aligned_bytes,
                            ESP_CACHE_MSYNC_FLAG_DIR_C2M);
            s_stash_alloc_recoveries++;
            give_dma_done_once("simple-path recovery");
            // fall through to head + carry update — data IS in ring
        } else if (!wrap_first_submitted) {
            // Neither segment reached GDMA: nothing is in flight, so it
            // is safe to CPU-recover both pieces of the wrap, mirroring
            // the simple path but split at the ring end.
            size_t remainder = aligned_bytes - bytes_to_end;
            memcpy(dst_base + head_bytes, s_align_scratch, bytes_to_end);
            esp_cache_msync(dst_base + head_bytes, bytes_to_end,
                            ESP_CACHE_MSYNC_FLAG_DIR_C2M);
            memcpy(dst_base, ((uint8_t *)s_align_scratch) + bytes_to_end, remainder);
            esp_cache_msync(dst_base, remainder, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
            s_stash_alloc_recoveries++;
            give_dma_done_once("wrap-path recovery, both segments");
            // fall through to head + carry update — data IS in ring
        } else {
            // First segment's GDMA may still be in flight; cannot safely
            // touch s_align_scratch or the ring tail from the CPU. Give
            // the semaphore back (nothing will complete it otherwise) and
            // fall through WITHOUT a CPU recovery — this chunk's ring
            // contents may be stale/garbage for the second segment's
            // span, but head still advances (see block comment above).
            give_dma_done_once("wrap-path recovery, first segment in flight");
            // fall through to head + carry update — audio in this window
            // may be wrong; the index invariant is preserved regardless
        }
    }

    head = signal_buffer_next_head(head, (uint32_t)aligned_count, total_cap);

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

    // T44 (assessed, not fixed here): both `head` and `start_idx` are
    // ring-relative positions modulo total_cap (~1M complex samples,
    // ~420 ms of producer time at 2.5 Msps). The check below can only
    // measure "distance since end" modulo one full lap -- if the
    // producer has actually lapped the ring an extra whole total_cap
    // (or more) since start_idx was captured, that extra distance is
    // invisible to this arithmetic and a truly stale burst can alias
    // back to a small, apparently-fresh `since_end`. Closing this needs
    // a monotonically increasing 64-bit sample counter threaded through
    // the callers that hand start_idx to this function (the tagger and
    // worker_core1.c), not just this file, so start_idx itself carries
    // enough range to disambiguate laps -- a wider cross-file API
    // change outside this task's file scope (only signal_buffer.c) and
    // too risky to improvise without touching those call sites in
    // lockstep. Left as-is; no behavior change here, this is the
    // maximally-correct check obtainable from ring-relative-only
    // inputs. In practice this needs a full extra lap of worker stall
    // (~420 ms) to misfire, far beyond normal tagger→worker latency.
    //
    // Distance from the burst's END (in producer order) to the current
    // head, modulo wrap. If this exceeds (total_cap - length), the
    // producer has lapped onto the burst's window — data is gone.
    uint32_t end       = (start_idx + length) % total_cap;
    uint32_t since_end = (head - end + total_cap) % total_cap;
    return since_end <= (total_cap - length);
}

// Invalidate one contiguous byte range of the ring, expanded outward to
// 64-byte cache-line boundaries. esp_cache_msync REJECTS unaligned M2C
// invalidates (ESP_ERR_INVALID_ARG) without touching the cache — and
// burst start/length from the tagger are NOT guaranteed multiples of 16
// complex samples, so the unexpanded call could silently no-op and leave
// the worker reading stale lines. Expanding is safe: the whole ring is
// owned by this module and never holds dirty lines (GDMA writes bypass
// the cache; the CPU-fallback path writes back immediately).
static void invalidate_ring_segment(uint8_t *base, size_t off, size_t len)
{
    size_t a_off = off & ~(size_t)63;
    size_t a_end = (off + len + 63) & ~(size_t)63;
    if (a_end > (size_t)SIGNAL_BUF_SIZE) a_end = SIGNAL_BUF_SIZE;
    esp_err_t r = esp_cache_msync(base + a_off, a_end - a_off,
                                  ESP_CACHE_MSYNC_FLAG_DIR_M2C |
                                      ESP_CACHE_MSYNC_FLAG_INVALIDATE);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "ring invalidate failed: %s (off=%zu len=%zu)",
                 esp_err_to_name(r), a_off, a_end - a_off);
    }
}

void signal_buffer_extract(uint32_t start_idx, uint32_t length, int16_t *dest)
{
    if (!circular_buf) return;

    // The DMA writes into PSRAM bypass any CPU caches on Core 0. Core 1's
    // CPU caches may hold stale lines for the region we just wrote, so
    // invalidate before the worker reads. M2C + INVALIDATE drops cached
    // lines so the next reads pull fresh data from PSRAM.
    const uint32_t total_cap     = SIGNAL_BUF_SIZE / 4;
    uint32_t       actual_start  = start_idx % total_cap;
    size_t         bytes_to_read = length * 4;
    uint8_t       *base          = (uint8_t *)circular_buf;
    uint32_t       start_bytes   = actual_start * 4;
    size_t         to_end_bytes  = (uint32_t)SIGNAL_BUF_SIZE - start_bytes;

    if (bytes_to_read <= to_end_bytes) {
        invalidate_ring_segment(base, start_bytes, bytes_to_read);
    } else {
        invalidate_ring_segment(base, start_bytes, to_end_bytes);
        invalidate_ring_segment(base, 0, bytes_to_read - to_end_bytes);
    }

    // Per-element copy across the wrap. Runs once per detected burst (not
    // per sample on the hot path), so the loop overhead is fine relative
    // to the rest of the worker pipeline.
    for (uint32_t i = 0; i < length; i++) {
        uint32_t idx    = (actual_start + i) % total_cap;
        dest[i * 2 + 0] = circular_buf[idx * 2 + 0];
        dest[i * 2 + 1] = circular_buf[idx * 2 + 1];
    }
}

void signal_buffer_invalidate_range(uint32_t start_idx, uint32_t length)
{
    if (!circular_buf) return;
    const uint32_t total_cap     = SIGNAL_BUF_SIZE / 4;
    uint32_t       actual_start  = start_idx % total_cap;
    size_t         bytes_to_read = length * 4;
    uint8_t       *base          = (uint8_t *)circular_buf;
    uint32_t       start_bytes   = actual_start * 4;
    size_t         to_end_bytes  = (uint32_t)SIGNAL_BUF_SIZE - start_bytes;

    if (bytes_to_read <= to_end_bytes) {
        invalidate_ring_segment(base, start_bytes, bytes_to_read);
    } else {
        invalidate_ring_segment(base, start_bytes, to_end_bytes);
        invalidate_ring_segment(base, 0, bytes_to_read - to_end_bytes);
    }
}

void signal_buffer_read_chunk(uint32_t start_idx, uint32_t length, int16_t *dest)
{
    if (!circular_buf) return;
    const uint32_t total_cap    = SIGNAL_BUF_SIZE / 4;
    uint32_t       actual_start = start_idx % total_cap;
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
