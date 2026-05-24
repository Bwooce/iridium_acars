// Raw IQ capture to SD — see sd_capture.h for the architecture.

#include "sd_capture.h"
#include "sd_log.h"      // for sd_log_force_mount()

#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "freertos/semphr.h"
#include "freertos/idf_additions.h"

static const char *TAG = "SDCAP";

// Stream buffer sizing: 4 MB of PSRAM holds ~900 ms of the current
// ~4.5 MB/s sustained USB rate. Sized to absorb SDMMC block-erase
// stalls (cards routinely pause 100-300 ms while reordering
// writes; cheap SDHC can stall 500 ms+). PSRAM is plentiful
// (32 MB on the P4-NANO), so spending 4 MB on this buffer is
// cheap insurance against producer-side drops.
//
// Was 128 KB originally — only ~28 ms of buffering, which produced
// worker_dropped tail noise in early capture runs.
#define STREAM_BUFFER_BYTES   (4 * 1024 * 1024)

// Trigger size on the stream buffer — the writer task wakes when at
// least this many bytes are available. 4 KB = 8 SDMMC sectors,
// matches typical FAT optimal write granularity.
#define STREAM_BUFFER_TRIG     4096

// Writer task params. Pinned to Core 1 (see init below for the
// why). Priority 5 = above worker_core1 (3) so the writer drains
// the PSRAM stream buffer to SD even when the tagger is firing
// 145 bursts/sec and worker is monopolising Core 1. Below ingest
// (8) so USB ingest never stalls. Sharing the prio-5 slot with
// http_server (also 5) is fine — both are mostly idle.
//
// Trade-off when capture is active: worker may drop bursts it
// can't process in time. That's the right back-pressure direction
// (lose some marginal burst decodes to keep the IQ recording
// intact) since the WHOLE POINT of capture is offline analysis.
#define WRITER_STACK            6144
#define WRITER_PRIO             5

// Per-receive scratch — small enough to live on the writer task
// stack, big enough for an efficient single fwrite. Tuned to match
// the FILE-level setvbuf below; one FILE-buffer chunk per dequeue.
#define WRITER_RECV_CHUNK       4096

// FATFS FILE buffer size: 64 KB. Each fwrite to a buffered FILE
// just memcpy's into here; the actual SDMMC write happens when
// this fills. Large buffer = fewer SDMMC sector writes = lower
// per-byte overhead.
#define FILE_BUFFER_BYTES       (64 * 1024)

typedef enum {
    CAP_STATE_IDLE = 0,
    CAP_STATE_ACTIVE,
    CAP_STATE_STOPPING,
} cap_state_t;

static StreamBufferHandle_t s_stream   = NULL;
static volatile cap_state_t s_state    = CAP_STATE_IDLE;
static FILE                *s_fp       = NULL;
static SemaphoreHandle_t    s_stats_mu = NULL;
static sd_capture_stats_t   s_stats    = {0};

static void update_bytes_written(size_t n)
{
    if (!s_stats_mu) return;
    xSemaphoreTake(s_stats_mu, portMAX_DELAY);
    s_stats.bytes_captured += n;
    xSemaphoreGive(s_stats_mu);
}

static void update_bytes_dropped(size_t n)
{
    if (!s_stats_mu) return;
    xSemaphoreTake(s_stats_mu, portMAX_DELAY);
    s_stats.bytes_dropped += n;
    xSemaphoreGive(s_stats_mu);
}

static void update_write_error(void)
{
    if (!s_stats_mu) return;
    xSemaphoreTake(s_stats_mu, portMAX_DELAY);
    s_stats.write_errors++;
    xSemaphoreGive(s_stats_mu);
}

static void writer_task(void *arg)
{
    (void)arg;
    // fwrite buffer MUST be in DMA-capable internal SRAM. SDMMC
    // DMA reads from this on the way to the card; if it lives in
    // PSRAM (which is the default for the writer's stack since we
    // set MALLOC_CAP_SPIRAM for the task), the IDF DMA layer tries
    // to allocate a "stash" copy in DMA-INT SRAM and fails with
    // `no mem for stash buffer` when DMA-INT is already booked by
    // the USB transfer pool. Outcome was fwrite returning EIO
    // after ~76 KB of capture. Heap-alloc the buffer once here.
    uint8_t *buf = heap_caps_aligned_alloc(64, WRITER_RECV_CHUNK,
                                            MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (!buf) {
        ESP_LOGE(TAG, "writer_task: DMA-INT buf alloc failed — capture disabled");
        vTaskDelete(NULL);
        return;
    }
    int64_t last_flush = 0;

    while (1) {
        if (s_state == CAP_STATE_IDLE || !s_stream) {
            // No capture in flight — sleep until one starts. 200 ms
            // tick is fine; sd_capture_start is a low-frequency
            // operator action.
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        // Drain whatever the producer queued. Short timeout so we
        // notice STOPPING quickly even on a quiet capture.
        size_t n = xStreamBufferReceive(s_stream, buf, sizeof(buf),
                                         pdMS_TO_TICKS(100));
        if (n > 0 && s_fp) {
            // Single fwrite of the whole received chunk. We tried
            // chunking to 512 bytes with vTaskDelay(0) between
            // each — that round-robined to httpd (same prio 5)
            // and cut effective throughput to ~700 B/s. Writer
            // is now Core-1, prio 5: above worker (3) and below
            // ingest (8), so it doesn't starve Core 0 (class_
            // driver is on a different core) and doesn't need
            // per-slice yields.
            size_t total_wr = fwrite(buf, 1, n, s_fp);
            if (total_wr == n) {
                update_bytes_written(n);
                // Stop on target reached. Producer also gates on
                // bytes_target so this is belt-and-suspenders.
                uint64_t cap = 0, tgt = 0;
                xSemaphoreTake(s_stats_mu, portMAX_DELAY);
                cap = s_stats.bytes_captured;
                tgt = s_stats.bytes_target;
                xSemaphoreGive(s_stats_mu);
                if (tgt > 0 && cap >= tgt) {
                    ESP_LOGI(TAG, "target reached (%llu bytes) — stopping",
                             (unsigned long long)cap);
                    s_state = CAP_STATE_STOPPING;
                }
            } else {
                ESP_LOGW(TAG, "fwrite short %u/%u (errno=%d)",
                         (unsigned)total_wr, (unsigned)n, errno);
                update_write_error();
            }
        }

        // Periodic flush — limits crash-loss to ~1 s of capture.
        int64_t now = esp_timer_get_time();
        if (s_fp && (now - last_flush) >= 1000000) {
            fflush(s_fp);
            last_flush = now;
        }

        // Handle stop: drain whatever's still in the buffer, then
        // close. Producer is already gated by s_state != ACTIVE so
        // nothing new arrives.
        if (s_state == CAP_STATE_STOPPING) {
            // Drain residual in non-blocking dequeues until empty.
            for (;;) {
                size_t rest = xStreamBufferReceive(s_stream, buf,
                                                    sizeof(buf), 0);
                if (rest == 0) break;
                if (s_fp) {
                    size_t wr = fwrite(buf, 1, rest, s_fp);
                    if (wr == rest) update_bytes_written(rest);
                    else            update_write_error();
                }
            }
            if (s_fp) {
                fflush(s_fp);
                fclose(s_fp);
                s_fp = NULL;
            }
            // Mark closed in stats; keep path/bytes for /capture/status.
            xSemaphoreTake(s_stats_mu, portMAX_DELAY);
            s_stats.active    = false;
            s_stats.file_open = false;
            xSemaphoreGive(s_stats_mu);
            ESP_LOGI(TAG, "capture closed");
            s_state = CAP_STATE_IDLE;
        }
    }
}

esp_err_t sd_capture_init(void)
{
    if (s_stats_mu) return ESP_OK;       // idempotent

    s_stats_mu = xSemaphoreCreateMutex();
    if (!s_stats_mu) return ESP_ERR_NO_MEM;

    // Writer task on Core 1 — moved off Core 0 (where class_driver
    // lives) because sustained SD writes were starving class_driver
    // past its 5 s WDT. Stays at prio 2 so frame_decoder (4) and
    // worker_core1 (3) and ingest_core1 (8) all preempt; the writer
    // gets Core 1 cycles only when those are blocked. PSRAM stack
    // is fine — fwrite is latency-tolerant.
    BaseType_t ok = xTaskCreatePinnedToCoreWithCaps(writer_task, "sd_capture",
                                                     WRITER_STACK, NULL,
                                                     WRITER_PRIO, NULL, 1,
                                                     MALLOC_CAP_SPIRAM);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "writer task create failed");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "SD capture writer ready on Core 1 (prio %d, stack in PSRAM) "
                  "— lazy stream buffer on start", WRITER_PRIO);
    return ESP_OK;
}

esp_err_t sd_capture_start(uint64_t target_bytes)
{
    if (!s_stats_mu) return ESP_ERR_INVALID_STATE;
    if (s_state != CAP_STATE_IDLE) return ESP_ERR_INVALID_STATE;

    // Ensure SD is mounted (shared with sd_log).
    esp_err_t r = sd_log_force_mount();
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "SD not mounted (%s) — capture refused",
                 esp_err_to_name(r));
        return r;
    }

    // Lazy-allocate the stream buffer in PSRAM. Recreating on every
    // start is fine; the cost is one PSRAM alloc (~µs) and the
    // delete-on-stop matches.
    s_stream = xStreamBufferCreateWithCaps(STREAM_BUFFER_BYTES,
                                            STREAM_BUFFER_TRIG,
                                            MALLOC_CAP_SPIRAM);
    if (!s_stream) {
        ESP_LOGE(TAG, "stream buffer alloc failed");
        return ESP_ERR_NO_MEM;
    }

    // One file per capture session; same /sdcard/acars/ dir as the
    // ACARS log uses. .u8 extension hints at the raw uint8 IQ format.
    char path[64];
    int64_t t0 = esp_timer_get_time();
    snprintf(path, sizeof(path), "/sdcard/acars/iq-%lld.u8", (long long)t0);

    FILE *fp = fopen(path, "wb");
    if (!fp) {
        ESP_LOGE(TAG, "fopen(%s) failed errno=%d", path, errno);
        vStreamBufferDeleteWithCaps(s_stream);
        s_stream = NULL;
        return ESP_FAIL;
    }
    setvbuf(fp, NULL, _IOFBF, FILE_BUFFER_BYTES);

    // Preallocate via ftruncate when a target is known — extends
    // the FAT cluster chain up front so mid-stream writes don't
    // pay the cluster-allocation cost. Non-fatal if FATFS doesn't
    // support it on this card.
    if (target_bytes > 0) {
        if (ftruncate(fileno(fp), (off_t)target_bytes) != 0) {
            ESP_LOGW(TAG, "ftruncate(%llu) failed errno=%d — "
                          "FAT extents will be allocated on demand",
                     (unsigned long long)target_bytes, errno);
        } else {
            // Rewind to start; ftruncate may have moved the file pointer.
            fseek(fp, 0, SEEK_SET);
        }
    }

    s_fp = fp;
    xSemaphoreTake(s_stats_mu, portMAX_DELAY);
    s_stats.active         = true;
    s_stats.file_open      = true;
    s_stats.bytes_captured = 0;
    s_stats.bytes_target   = target_bytes;
    s_stats.bytes_dropped  = 0;
    s_stats.write_errors   = 0;
    s_stats.start_us       = t0;
    strlcpy(s_stats.path, path, sizeof(s_stats.path));
    xSemaphoreGive(s_stats_mu);

    s_state = CAP_STATE_ACTIVE;
    ESP_LOGI(TAG, "capture started: %s (target=%llu bytes)",
             path, (unsigned long long)target_bytes);
    return ESP_OK;
}

esp_err_t sd_capture_stop(void)
{
    if (s_state != CAP_STATE_ACTIVE) return ESP_ERR_INVALID_STATE;
    ESP_LOGI(TAG, "capture stop requested");
    // Tell the writer to drain + close. It transitions to IDLE on
    // its own and we tear down the stream buffer there.
    s_state = CAP_STATE_STOPPING;

    // Wait briefly (up to ~1 s) for the writer to reach IDLE. Stop
    // is a low-frequency action, so the busy-wait poll is fine.
    for (int i = 0; i < 50; i++) {
        if (s_state == CAP_STATE_IDLE) break;
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Release stream buffer regardless of whether the writer fully
    // drained — anything still in there got dropped by the timeout.
    if (s_stream) {
        vStreamBufferDeleteWithCaps(s_stream);
        s_stream = NULL;
    }
    return ESP_OK;
}

void sd_capture_write(const uint8_t *data, size_t n)
{
    if (s_state != CAP_STATE_ACTIVE || !s_stream || !data || n == 0) return;

    // Target gate — stop accepting bytes once we've queued past the
    // target. The writer also checks and transitions to STOPPING
    // when its own counter passes the target; this just avoids
    // queueing extra bytes that would get truncated.
    uint64_t cap = 0, tgt = 0;
    if (s_stats_mu) {
        xSemaphoreTake(s_stats_mu, portMAX_DELAY);
        cap = s_stats.bytes_captured;
        tgt = s_stats.bytes_target;
        xSemaphoreGive(s_stats_mu);
    }
    if (tgt > 0 && cap + n > tgt) {
        // Trim to remaining target.
        if (cap >= tgt) return;
        n = tgt - cap;
    }

    size_t sent = xStreamBufferSend(s_stream, data, n, 0);
    if (sent < n) {
        update_bytes_dropped(n - sent);
    }
}

void sd_capture_get_stats(sd_capture_stats_t *out)
{
    if (!out) return;
    if (!s_stats_mu) {
        memset(out, 0, sizeof(*out));
        return;
    }
    xSemaphoreTake(s_stats_mu, portMAX_DELAY);
    *out = s_stats;
    xSemaphoreGive(s_stats_mu);
}

FILE *sd_capture_open_for_read(const char *name)
{
    if (!name || !*name) return NULL;
    // Reject anything with '/' or ".." to keep the request scoped
    // to /sdcard/acars/. The basename-only contract is enough; full
    // chroot is overkill on a per-device LAN-only firmware.
    if (strchr(name, '/') || strstr(name, "..")) return NULL;

    // Make sure the capture isn't still writing this file: if a
    // capture is active and its current path ends with the
    // requested name, refuse the open so the file's FATFS state is
    // self-consistent. The caller (HTTP handler) can return 409.
    if (s_stats_mu) {
        bool busy = false;
        xSemaphoreTake(s_stats_mu, portMAX_DELAY);
        if (s_stats.active && s_stats.path[0]) {
            const char *base = strrchr(s_stats.path, '/');
            base = base ? base + 1 : s_stats.path;
            if (strcmp(base, name) == 0) busy = true;
        }
        xSemaphoreGive(s_stats_mu);
        if (busy) {
            errno = EBUSY;
            return NULL;
        }
    }

    char path[96];
    snprintf(path, sizeof(path), "/sdcard/acars/%s", name);
    return fopen(path, "rb");
}
