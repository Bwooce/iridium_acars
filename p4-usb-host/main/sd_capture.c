// Raw IQ capture to SD — see sd_capture.h for the architecture.

#include "sd_capture.h"
#include "sd_log.h" // for sd_log_force_mount()

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
#define STREAM_BUFFER_BYTES (4 * 1024 * 1024)

// Trigger size on the stream buffer — the writer task wakes when at
// least this many bytes are available. 4 KB = 8 SDMMC sectors,
// matches typical FAT optimal write granularity.
#define STREAM_BUFFER_TRIG 4096

// Writer task params. Pinned to Core 1. Priority 5 = above
// worker_core1 (4 — deliberate placement, see the priority history
// note in worker_core1.c) so the writer can drain the PSRAM stream
// buffer even while the tagger fires bursts continuously. The
// blocking-in-fwrite case is harmless: SDMMC waits on an interrupt
// semaphore, so the writer task is suspended and worker / ingest
// keep running on their own. Below ingest (8) so USB ingest itself
// is never preempted.
#define WRITER_STACK 6144
#define WRITER_PRIO 5

// Per-receive scratch. Must be DMA-capable internal SRAM (SDMMC
// driver can't DMA from PSRAM; SDMMC_HOST_FLAG_ALLOC_ALIGNED_BUF
// is SDIO-only). After tagger (66 KB) and USB pool (~56 KB
// actually used — pool tries 64 KB but typically gets 7 of 8
// transfers), only ~8-16 KB contiguous DMA-INT is left.
// 8 KB is the largest that reliably fits AFTER USB pool init.
//
// 8 KB writes give ~300-500 KB/s on Class 4 cards. Burst-mode
// peak rate is ~10 bursts/sec × 80 KB avg = 0.8 MB/s — close to
// the limit but workable. If the producer outruns the writer,
// stream-buffer back-pressure drops bytes (bytes_dropped ticks
// up) rather than losing whole bursts.
#define WRITER_RECV_CHUNK (8 * 1024)

// FATFS FILE buffer size: 64 KB. Each fwrite to a buffered FILE
// just memcpy's into here; the actual SDMMC write happens when
// this fills. Large buffer = fewer SDMMC sector writes = lower
// per-byte overhead.
#define FILE_BUFFER_BYTES (64 * 1024)

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

// Capture mode: 0 = continuous (sd_capture_write tap in
// class_driver), 1 = burst-only (sd_capture_record_burst_*
// from worker_core1). The two modes share the writer task and
// stream buffer; only one is producing at any time. Set by
// sd_capture_start / sd_capture_start_bursts.
static volatile bool     s_burst_mode = false;
static volatile uint32_t s_burst_seq  = 0;

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

static uint8_t *s_writer_buf = NULL; // DMA-INT, alloc'd in action_start_stream

static void writer_task(void *arg)
{
    (void)arg;
    // fwrite buf is allocated via sd_capture_alloc_writer_buf() from
    // action_start_stream() AFTER tagger init. Until then we idle —
    // s_stream stays NULL anyway because sd_capture_start refuses if
    // s_writer_buf hasn't landed yet.
    int64_t last_flush = 0;

    // Throttle the fwrite-failure log spam — when SDMMC returns EIO
    // continuously (card bus error / unrecoverable controller state),
    // we don't want a wall of warnings. One line per WARN_THROTTLE_NS
    // and one summary on transition back to success.
    int64_t  last_warn_us = 0;
    uint32_t warn_skipped = 0;
#define WARN_THROTTLE_US 500000 // half a second between warn lines

// Hard cap on consecutive write failures — if SDMMC won't accept a
// write this many times in a row, the controller is wedged. Bail
// out of capture rather than burn CPU on a dead card. Operator can
// POST /sd/format + /capture/start to retry.
#define CONSEC_FAIL_LIMIT 64
    uint32_t consec_fail = 0;

// Per-write sector alignment — every fwrite size MUST be a
// multiple of 512 (SD sector size). Without this, FATFS detects
// a partial-sector tail, advances the user buf by `tail` bytes,
// and hands the SDMMC driver a misaligned pointer (e.g.
// buf+472). SDMMC's `is_aligned` check fails, it falls back to
// allocate_dma_buf which fails under DMA-INT pressure with
// ESP_ERR_NO_MEM. We saw this as deterministic 165032-byte (then
// 900 KB at 20 MHz) cliffs followed by every subsequent write
// returning EIO. Holding the unaligned tail in `stash` and
// prepending it to the next read keeps every fwrite aligned and
// every user-buf-offset aligned.
#define SECTOR_BYTES 512
    size_t stash_len = 0; // bytes held over from previous read

    while (1) {
        uint8_t *buf = s_writer_buf;
        if (!buf || s_state == CAP_STATE_IDLE || !s_stream) {
            // No capture in flight — sleep until one starts. 200 ms
            // tick is fine; sd_capture_start is a low-frequency
            // operator action.
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        // Drain whatever the producer queued, leaving room at the
        // front of `buf` for the previous iteration's unaligned tail.
        size_t n = xStreamBufferReceive(s_stream, buf + stash_len,
                                        WRITER_RECV_CHUNK - stash_len,
                                        pdMS_TO_TICKS(100));
        // Combine stash + new data, then round down to a sector
        // multiple. The leftover bytes become the next iteration's
        // stash.
        size_t total    = stash_len + n;
        size_t aligned  = total & ~(SECTOR_BYTES - 1);
        size_t leftover = total - aligned;
        n               = aligned;
        if (n > 0 && s_fp) {
            // Single fwrite of the whole received chunk. We tried
            // chunking to 512 bytes with vTaskDelay(0) between
            // each — that round-robined to httpd (same prio 5)
            // and cut effective throughput to ~700 B/s. Writer
            // is now Core-1, prio 5: above worker (4) and below
            // ingest (8), so it doesn't starve Core 0 (class_
            // driver is on a different core) and doesn't need
            // per-slice yields.
            size_t total_wr = fwrite(buf, 1, n, s_fp);
            if (total_wr == n) {
                update_bytes_written(n);
                if (consec_fail > 0) {
                    ESP_LOGI(TAG, "fwrite recovered after %u failures "
                                  "(skipped %u warn lines)",
                             (unsigned)consec_fail, (unsigned)warn_skipped);
                    consec_fail  = 0;
                    warn_skipped = 0;
                }
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
                int64_t now = esp_timer_get_time();
                if (now - last_warn_us >= WARN_THROTTLE_US) {
                    ESP_LOGW(TAG, "fwrite short %u/%u (errno=%d) "
                                  "consec_fail=%u (skipped %u since last)",
                             (unsigned)total_wr, (unsigned)n, errno,
                             (unsigned)consec_fail + 1, (unsigned)warn_skipped);
                    last_warn_us = now;
                    warn_skipped = 0;
                } else {
                    warn_skipped++;
                }
                update_write_error();
                consec_fail++;
                if (consec_fail >= CONSEC_FAIL_LIMIT) {
                    ESP_LOGE(TAG, "fwrite failed %u consecutive times — "
                                  "SDMMC controller likely wedged. Aborting "
                                  "capture; POST /sd/format and try again.",
                             (unsigned)consec_fail);
                    s_state     = CAP_STATE_STOPPING;
                    consec_fail = 0;
                }
            }
        }
        // Move the unaligned tail to the front of buf for the next
        // iteration's stash. Always do this after computing aligned/
        // leftover so the rotation happens whether or not we wrote.
        if (leftover > 0) {
            memmove(buf, buf + aligned, leftover);
        }
        stash_len = leftover;

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
            // Drain residual in non-blocking dequeues. Bounded so a
            // wedged SDMMC can't keep us looping on a 4 MB buffer.
            // Same sector-alignment dance as the main loop: round
            // each fwrite down to a 512-byte multiple, carry the
            // leftover into the next iteration.
            int drain_iters = 512;
            for (; drain_iters > 0; drain_iters--) {
                size_t rest = xStreamBufferReceive(s_stream, buf + stash_len,
                                                   WRITER_RECV_CHUNK - stash_len, 0);
                if (rest == 0 && stash_len == 0) break;
                size_t total    = stash_len + rest;
                size_t aligned  = total & ~(SECTOR_BYTES - 1);
                size_t leftover = total - aligned;
                if (aligned > 0 && s_fp) {
                    size_t wr = fwrite(buf, 1, aligned, s_fp);
                    if (wr == aligned)
                        update_bytes_written(aligned);
                    else
                        update_write_error();
                }
                if (leftover > 0) memmove(buf, buf + aligned, leftover);
                stash_len = leftover;
                if (rest == 0) break; // nothing new and we just flushed
            }
            if (drain_iters == 0) {
                ESP_LOGW(TAG, "drain hit iteration cap — discarding rest");
            }
            // Final partial-sector flush — fwrites the trailing
            // `stash_len` (< 512) bytes. FATFS goes through its WIN
            // cache for this single misaligned write; one such write
            // is OK because the file is being closed immediately after.
            if (stash_len > 0 && s_fp) {
                size_t wr = fwrite(buf, 1, stash_len, s_fp);
                if (wr == stash_len)
                    update_bytes_written(stash_len);
                else
                    update_write_error();
                stash_len = 0;
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
            // Clear burst mode here so any path that reaches STOPPING
            // (target-hit, fwrite circuit-breaker) leaves us with a
            // clean default for the next /capture/start. sd_capture_stop
            // also clears this, but writer-initiated stops don't go
            // through it.
            s_burst_mode = false;
            s_state      = CAP_STATE_IDLE;
        }
    }
}

esp_err_t sd_capture_init(void)
{
    if (s_stats_mu) return ESP_OK; // idempotent

    s_stats_mu = xSemaphoreCreateMutex();
    if (!s_stats_mu) return ESP_ERR_NO_MEM;

    // Eager-allocate the 8 KB DMA-INT fwrite scratch HERE, at the
    // earliest possible moment (sd_capture_init runs in app_main
    // before USB stack init). At this point DMA-INT has 139 KB
    // largest contiguous; an 8 KB slice leaves 131 KB contiguous
    // for tagger's later 66 KB allocation. Any later (after USB
    // pool init) only ~2-3 KB largest is left — too small.
    //
    // SDMMC requires DMA-capable internal SRAM for the fwrite
    // source. PSRAM source produces ENOSPC (the ALLOC_ALIGNED_BUF
    // host flag is SDIO-only and does not help SD card writes).
    s_writer_buf = heap_caps_aligned_alloc(64, WRITER_RECV_CHUNK,
                                           MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (!s_writer_buf) {
        ESP_LOGE(TAG, "writer-buf %u-byte DMA-INT alloc failed (largest=%u)",
                 (unsigned)WRITER_RECV_CHUNK,
                 (unsigned)heap_caps_get_largest_free_block(
                     MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
        return ESP_ERR_NO_MEM;
    }

    // Allocate the PSRAM stream buffer ONCE at boot; never free it.
    // Previously we created/destroyed it per capture session, which
    // raced with the writer task: vStreamBufferDeleteWithCaps would
    // run while xStreamBufferReceive was still holding the handle,
    // tripping a tlsf double-free assert during sd_capture_stop.
    // 4 MB of PSRAM is rounding error on a 32 MB device.
    s_stream = xStreamBufferCreateWithCaps(STREAM_BUFFER_BYTES,
                                           STREAM_BUFFER_TRIG,
                                           MALLOC_CAP_SPIRAM);
    if (!s_stream) {
        ESP_LOGE(TAG, "stream buffer alloc failed (need %u B PSRAM)",
                 (unsigned)STREAM_BUFFER_BYTES);
        heap_caps_free(s_writer_buf);
        s_writer_buf = NULL;
        return ESP_ERR_NO_MEM;
    }

    BaseType_t ok = xTaskCreatePinnedToCoreWithCaps(writer_task, "sd_capture",
                                                    WRITER_STACK, NULL,
                                                    WRITER_PRIO, NULL, 1,
                                                    MALLOC_CAP_SPIRAM);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "writer task create failed");
        heap_caps_free(s_writer_buf);
        s_writer_buf = NULL;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "SD capture ready on Core 1 (prio %d, stack PSRAM, "
                  "%u-byte DMA-INT writer-buf)",
             WRITER_PRIO, (unsigned)WRITER_RECV_CHUNK);
    return ESP_OK;
}

esp_err_t sd_capture_alloc_writer_buf(void)
{
    // Now a no-op — writer-buf is eager-allocated in sd_capture_init.
    // Kept for ABI continuity with action_start_stream.
    return s_writer_buf ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t sd_capture_start(uint64_t target_bytes)
{
    if (!s_stats_mu) return ESP_ERR_INVALID_STATE;
    if (s_state != CAP_STATE_IDLE) return ESP_ERR_INVALID_STATE;
    if (!s_writer_buf) {
        ESP_LOGE(TAG, "writer-buf not allocated — sd_capture_alloc_writer_buf() "
                      "never ran (USB device not enumerated?)");
        return ESP_ERR_INVALID_STATE;
    }

    // Ensure SD is mounted (shared with sd_log).
    esp_err_t r = sd_log_force_mount();
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "SD not mounted (%s) — capture refused",
                 esp_err_to_name(r));
        return r;
    }

    // Reset any leftover content from a prior capture. Stream buffer
    // itself is permanent (allocated in sd_capture_init) — see
    // double-free note there.
    if (!s_stream) return ESP_ERR_INVALID_STATE;
    xStreamBufferReset(s_stream);

    // One file per capture session; same /sdcard/acars/ dir as the
    // ACARS log uses. .u8 extension hints at the raw uint8 IQ format.
    char    path[64];
    int64_t t0 = esp_timer_get_time();
    snprintf(path, sizeof(path), "/sdcard/acars/iq-%lld.u8", (long long)t0);

    FILE *fp = fopen(path, "wb");
    if (!fp) {
        ESP_LOGE(TAG, "fopen(%s) failed errno=%d", path, errno);
        // Don't free s_stream — it's permanent now. Just leave the
        // capture state at IDLE so a retry can re-arm.
        return ESP_FAIL;
    }
    // Disable libc buffering — the writer task already feeds us
    // 64 KB chunks straight from the stream buffer, so any libc-
    // layer buffering just adds memcpy overhead and obscures the
    // SD write timing. _IONBF makes each fwrite go straight to
    // FATFS f_write. Also verify the call succeeded so we don't
    // silently fall back to the default 1024 B line buffer.
    if (setvbuf(fp, NULL, _IONBF, 0) != 0) {
        ESP_LOGW(TAG, "setvbuf(_IONBF) failed — proceeding with libc default");
    }

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

    // Wait BRIEFLY for the writer to reach IDLE. A full 4 MB stream-
    // buffer backlog at ~500 KB/s takes ~10 s to drain, but this
    // function runs on the single esp_http_server serve task — blocking
    // here made the WHOLE HTTP server unreachable for up to 60 s (the
    // previous loop bound). Cap at 2 s: short drains finish inline and
    // return ESP_OK; longer drains continue asynchronously in the
    // writer task (its STOPPING branch always runs to fclose on its
    // own, with a 512-iter safety cap) and we return ESP_ERR_TIMEOUT
    // so the HTTP handler can report 202 stop-in-progress. Since the
    // stream buffer is permanent (alloc'd once in sd_capture_init)
    // there is nothing to tear down here — returning early is safe.
    for (int i = 0; i < 100; i++) { // 2 s @ 20 ms tick
        if (s_state == CAP_STATE_IDLE) break;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    s_burst_mode = false;
    if (s_state != CAP_STATE_IDLE) {
        ESP_LOGI(TAG, "writer still draining after 2 s — close continues "
                      "asynchronously (poll /capture/status)");
        return ESP_ERR_TIMEOUT; // stop accepted, drain in progress
    }

    // Stream buffer is permanent (allocated once in sd_capture_init)
    // so we don't free it here. It gets reset on the next start.
    return ESP_OK;
}

void sd_capture_write(const uint8_t *data, size_t n)
{
    // Burst-mode mutually excludes the continuous tap.
    if (s_burst_mode) return;
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

// ---- Burst-mode capture (per-burst IQ records) -------------------

esp_err_t sd_capture_start_bursts(void)
{
    // Pre-arm burst mode BEFORE sd_capture_start transitions s_state
    // to ACTIVE. sd_capture_write gates on (state == ACTIVE && !burst_mode);
    // if we set the flag AFTER state goes ACTIVE the brief window lets
    // raw USB bytes leak into the new file. At 4.5 MB/s a ~1 ms context
    // switch produces a 4-5 KB junk prefix that shifts every BRST header
    // off byte 0 and breaks naive decoders. Observed live 2026-05-25 in
    // iq-4188149994.u8 (first BRST at offset 4992).
    s_burst_mode = true;
    s_burst_seq  = 0;
    esp_err_t r  = sd_capture_start(0);
    if (r != ESP_OK) {
        s_burst_mode = false; // back out so a continuous /capture/start can succeed
        return r;
    }
    ESP_LOGI(TAG, "burst-mode capture armed");
    return ESP_OK;
}

void sd_capture_record_burst_begin(uint32_t length_samples,
                                   float    rel_freq_hz,
                                   float    peak_snr_db,
                                   float    magnitude_db,
                                   float    noise_db)
{
    if (!s_burst_mode || s_state != CAP_STATE_ACTIVE || !s_stream) return;

    sd_capture_burst_hdr_t hdr = {
        .magic          = SD_CAPTURE_BURST_MAGIC,
        .seq            = s_burst_seq++,
        .t_us           = (uint64_t)esp_timer_get_time(),
        .length_samples = length_samples,
        .rel_freq_hz    = rel_freq_hz,
        .peak_snr_db    = peak_snr_db,
        .magnitude_db   = magnitude_db,
        .noise_db       = noise_db,
    };
    size_t sent = xStreamBufferSend(s_stream, &hdr, sizeof(hdr), 0);
    if (sent < sizeof(hdr)) {
        update_bytes_dropped(sizeof(hdr) - sent);
    }
}

void sd_capture_record_burst_chunk(const int16_t *iq, size_t n_complex)
{
    if (!s_burst_mode || s_state != CAP_STATE_ACTIVE || !s_stream) return;
    if (!iq || n_complex == 0) return;

    size_t n_bytes = n_complex * 2 * sizeof(int16_t);
    size_t sent    = xStreamBufferSend(s_stream, iq, n_bytes, 0);
    if (sent < n_bytes) {
        update_bytes_dropped(n_bytes - sent);
    }
}

void sd_capture_record_burst_end(void)
{
    // No trailer in the current format — burst boundaries are
    // implicit (parser reads length_samples * 4 bytes after each
    // header). Reserved for future use (e.g. CRC of payload).
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
            base             = base ? base + 1 : s_stats.path;
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
