// SD card log writer. Mounts SDMMC slot 0 (P4 GPIO 39-44), opens an
// NDJSON file per boot, and appends one line per decoded ACARS message.
//
// Pinout (per docs/p4-nano-board-schematic-summary.md, line 29):
//   CLK=GPIO43  CMD=GPIO44  D0=GPIO39 D1=GPIO40 D2=GPIO41 D3=GPIO42
//   SD_VDD gated by GPIO45 (P-FET, drive LOW to enable card power).
//
// Threading: producer is frame_decoder (Core 1) which calls sd_log_emit
// non-blockingly. A dedicated writer task on Core 0 (where the USB host
// already lives but is mostly idle on its read loop) drains the queue
// and writes to FATFS. Pinning to Core 0 keeps the heavier Core 1 DSP
// path unaffected.

#include "sd_log.h"

#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <errno.h>
#include <sys/stat.h>

#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "esp_task_wdt.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/idf_additions.h"     // xQueueCreateWithCaps, xTaskCreatePinnedToCoreWithCaps

#include "esp_heap_caps.h"
#include "esp_timer.h"

static const char *TAG = "SDLOG";

#define MOUNT_POINT     "/sdcard"
#define LOG_DIR         "/sdcard/acars"

// Pin assignments — see header above.
#define PIN_CLK  43
#define PIN_CMD  44
#define PIN_D0   39
#define PIN_D1   40
#define PIN_D2   41
#define PIN_D3   42
#define PIN_PWR  45        // GPIO45 LOW = SD_VDD on (P-FET)

// SD1_VDD on the Waveshare P4-NANO is sourced from ESP_LDO_VO4 (P4
// internal LDO #4), gated by Q1 / GPIO45 (see
// docs/p4-nano-board-schematic-summary.md §1). Without explicitly
// turning on LDO #4, GPIO45 just gates a powerless rail — the card
// never sees voltage and OCR (ACMD41) times out. Hours of "card
// won't respond" on the bench traced back to this.
#define SDMMC_PWR_LDO_CHANNEL  4

#define EMIT_QUEUE_DEPTH    32
#define WRITER_STACK        6144
#define WRITER_PRIO         3

static sdmmc_card_t   *s_card           = NULL;
static FILE           *s_log            = NULL;
static QueueHandle_t   s_q              = NULL;
// 64 KB DMA-INT buffer the SDMMC driver uses for read/write to
// PSRAM-resident user buffers. Allocated eagerly in sd_log_init
// (early in app_main, before USB + tagger fragment DMA-INT) and
// attached to s_card->host.dma_aligned_buffer at mount time.
static void           *s_sdmmc_stash    = NULL;
static SemaphoreHandle_t s_stats_mu     = NULL;
static sd_log_stats_t  s_stats          = {0};
static volatile bool   s_mount_attempted = false;    // lazy-mount flag

static void update_stats_ok(size_t bytes_added)
{
    if (!s_stats_mu) return;
    xSemaphoreTake(s_stats_mu, portMAX_DELAY);
    s_stats.messages_written++;
    s_stats.bytes_written += bytes_added;
    xSemaphoreGive(s_stats_mu);
}

static void update_stats_err(void)
{
    if (!s_stats_mu) return;
    xSemaphoreTake(s_stats_mu, portMAX_DELAY);
    s_stats.write_errors++;
    xSemaphoreGive(s_stats_mu);
}

// Same single-line JSON shape as GET /messages and acars_push, plus a
// trailing newline so the file is true NDJSON. Returns bytes written
// (excluding the NUL).
static size_t format_msg_line(char *out, size_t cap, const acars_msg_t *m)
{
    // Tiny JSON escape for the text field — copied from acars_push.c
    // structure rather than shared because pulling them into a common
    // helper would touch three call sites for marginal benefit.
    char esc[2 * MSG_RING_TXT_MAX + 8];
    size_t w = 0;
    for (const char *p = m->txt; *p && w + 7 < sizeof(esc); p++) {
        unsigned char c = (unsigned char)*p;
        switch (c) {
        case '"':  esc[w++] = '\\'; esc[w++] = '"';  break;
        case '\\': esc[w++] = '\\'; esc[w++] = '\\'; break;
        case '\n': esc[w++] = '\\'; esc[w++] = 'n';  break;
        case '\r': esc[w++] = '\\'; esc[w++] = 'r';  break;
        case '\t': esc[w++] = '\\'; esc[w++] = 't';  break;
        default:
            if (c < 0x20) {
                static const char hex[] = "0123456789abcdef";
                esc[w++] = '\\'; esc[w++] = 'u';
                esc[w++] = '0'; esc[w++] = '0';
                esc[w++] = hex[(c >> 4) & 0xf];
                esc[w++] = hex[c & 0xf];
            } else {
                esc[w++] = (char)c;
            }
        }
    }
    esc[w] = '\0';

    int n = snprintf(out, cap,
        "{\"id\":%llu,\"t_us\":%llu,\"dir\":\"%s\","
        "\"mode\":\"%c\",\"label\":\"%.2s\",\"block\":\"%c\","
        "\"msg_num\":\"%s\",\"flight\":\"%s\","
        "\"crc\":%s,\"peak_bin\":%ld,\"snr_db\":%.1f,"
        "\"txt\":\"%s\"}\n",
        (unsigned long long)m->id,
        (unsigned long long)m->timestamp_us,
        m->uplink ? "UL" : "DL",
        m->mode,
        m->label,
        m->block_id,
        m->msg_num,
        m->flight_id,
        m->crc_ok ? "true" : "false",
        (long)m->peak_bin,
        (double)m->snr_db,
        esc);
    if (n < 0) return 0;
    if ((size_t)n >= cap) return cap - 1;
    return (size_t)n;
}

// Forward decls — lazy-mount triggers these.
static esp_err_t mount_sd(void);
static esp_err_t open_log_file(void);

// Try to bring up SD + open the log file. Caller (the writer task) sets
// s_mount_attempted so we only try once per "session". Returns ESP_OK on
// success. On failure, leaves s_log NULL — subsequent messages get
// dropped silently until force_mount() is called.
static esp_err_t try_mount_and_open(void)
{
    s_mount_attempted = true;
    esp_err_t r = mount_sd();
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "SD mount failed (%s) — logging stays disabled",
                 esp_err_to_name(r));
        return r;
    }
    if (open_log_file() != ESP_OK) {
        ESP_LOGW(TAG, "SD mounted but log file open failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void writer_task(void *arg)
{
    (void)arg;
    acars_msg_t m;
    char        line[2048];        // worst case ~600 B; 2K is generous
    int64_t     last_flush = esp_timer_get_time();

    while (1) {
        BaseType_t got = xQueueReceive(s_q, &m, pdMS_TO_TICKS(1000));
        if (got == pdTRUE) {
            // Lazy mount: first message triggers the SDMMC + FATFS
            // bring-up. Subsequent failures stay failed until the
            // user POSTs /sd/mount.
            if (!s_mount_attempted) {
                ESP_LOGI(TAG, "first message — attempting lazy SD mount");
                try_mount_and_open();
            }
            if (s_log) {
                size_t len = format_msg_line(line, sizeof(line), &m);
                if (len > 0) {
                    size_t wr = fwrite(line, 1, len, s_log);
                    if (wr == len) {
                        update_stats_ok(len);
                    } else {
                        ESP_LOGW(TAG, "fwrite short: %u/%u — disk full?",
                                 (unsigned)wr, (unsigned)len);
                        update_stats_err();
                    }
                }
            }
        }

        // Flush at least every 1 s so an abrupt power loss doesn't lose
        // more than a second of decodes. FATFS fsync is fairly cheap
        // when the SDMMC DMA isn't backed up.
        int64_t now = esp_timer_get_time();
        if (s_log && (now - last_flush) >= 1000000) {
            fflush(s_log);
            last_flush = now;
        }
    }
}

static void enable_card_power(void)
{
    // GPIO45 is the P-FET gate. Drive LOW to turn the card ON.
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << PIN_PWR,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(PIN_PWR, 0);     // 0 = card power ON
    // Cards need ~1 ms to come up after power-on; some 4 GB / older
    // cards need much longer for their internal controller to be
    // ready for OCR (CMD1/ACMD41) commands. 100 ms is the SD-spec
    // "power-up to first command" upper bound and costs nothing on
    // the mount path.
    vTaskDelay(pdMS_TO_TICKS(100));
}

// esp_hosted owns the SDMMC host controller (single controller on P4,
// claimed first at boot for SDIO slot 1 → C6). esp_vfs_fat_sdmmc_mount
// would otherwise try to claim it again for slot 0 and fail with
// ESP_ERR_NOT_FOUND. Override host.init/deinit with no-ops so we
// reuse the already-initialised controller and just add the slot.
// Reference: managed_components/espressif__esp_hosted/examples/
//            host_sdcard_with_hosted/main/sd_card_functions.c
static esp_err_t sdmmc_init_noop(void)   { return ESP_OK; }
static esp_err_t sdmmc_deinit_noop(void) { return ESP_OK; }

static esp_err_t mount_sd(void)
{
    enable_card_power();

    // Enable LDO #4 so SD1_VDD actually has a voltage source. The
    // P-FET on GPIO45 only gates this rail — without the LDO, the
    // card power pin is floating regardless of GPIO45's state.
    sd_pwr_ctrl_ldo_config_t ldo_cfg = { .ldo_chan_id = SDMMC_PWR_LDO_CHANNEL };
    sd_pwr_ctrl_handle_t ldo_handle = NULL;
    esp_err_t pr = sd_pwr_ctrl_new_on_chip_ldo(&ldo_cfg, &ldo_handle);
    if (pr != ESP_OK) {
        ESP_LOGW(TAG, "sd_pwr_ctrl_new_on_chip_ldo(ch=%d) failed: %s",
                 SDMMC_PWR_LDO_CHANNEL, esp_err_to_name(pr));
    }

    // Verbose SDMMC logs — diagnosing the deterministic ~165 KB
    // EIO cliff in burst-mode capture. Reveals card status bits,
    // CMD25/CMD12 errors, and wait_for_idle timeouts.
    esp_log_level_set("sdmmc_cmd", ESP_LOG_DEBUG);
    esp_log_level_set("sdmmc_common", ESP_LOG_DEBUG);
    esp_log_level_set("sdmmc_io", ESP_LOG_DEBUG);

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    // 20 MHz (SDMMC_FREQ_DEFAULT). Tried 40 MHz (HIGHSPEED) first —
    // each burst's first multi-block write succeeded, then subsequent
    // writes returned EIO until the writer's circuit breaker tripped
    // at 64 consecutive failures. Symptom matches an overclocked card
    // wedging under sustained load. 20 MHz halves theoretical
    // throughput but burst-mode peak (~0.8 MB/s) is well within reach.
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;
    // Slot 0 (P4-NANO's SD slot per schematic, distinct from slot 1
    // which is C6 esp_hosted).
    host.slot = SDMMC_HOST_SLOT_0;
    host.init   = sdmmc_init_noop;
    host.deinit = sdmmc_deinit_noop;
    host.pwr_ctrl_handle = ldo_handle;
    // (Tried SDMMC_HOST_FLAG_ALLOC_ALIGNED_BUF here to support PSRAM
    // source buffers — that flag is documented as SDIO-only and
    // produces ENOSPC on SD card writes. Producers must keep their
    // fwrite source in DMA-capable internal SRAM. See sd_capture.c.)

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    // 4-bit mode. Board wires D0..D3 per schematic §1 (GPIO39..42).
    // Was 1 briefly during LDO bring-up debugging (when card
    // wasn't responding at all) — fallback no longer needed now
    // that LDO #4 is properly enabled. 4-bit at SDMMC_FREQ_HIGHSPEED
    // (40 MHz) = 20 MB/s peak, ~5-10 MB/s sustained on a decent
    // card. Bench card was ~800 B/s under 1-bit so 4-bit should
    // matter even more here (bus width is rarely THE bottleneck,
    // but worth trying before blaming the card).
    slot.width = 4;
    slot.clk = PIN_CLK;
    slot.cmd = PIN_CMD;
    slot.d0  = PIN_D0;
    slot.d1  = PIN_D1;
    slot.d2  = PIN_D2;
    slot.d3  = PIN_D3;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_vfs_fat_mount_config_t mount = {
        // Format the card to FAT32 if it's blank or has a non-FAT
        // filesystem. The card is exclusively for the device's own
        // use (ACARS log + IQ capture) so destroying any pre-
        // existing data is acceptable. The WDT bump below makes
        // this safe — without it, f_mkfs on a 4 GB card runs
        // synchronously long enough to starve frame_decoder past
        // its 5 s watchdog.
        .format_if_mount_failed = true,
        .max_files              = 4,
        .allocation_unit_size   = 16 * 1024,
    };

    // Mount may trigger a synchronous f_mkfs (format) that takes
    // tens of seconds on a multi-GB card and hammers the SDMMC bus.
    // The watched class_driver / frame_decoder tasks can starve
    // past their 5 s WDT timeout during this. Bump the global TWDT
    // timeout to 60 s for the duration of the mount, restore after.
    esp_task_wdt_config_t wdt_save = {
        .timeout_ms     = CONFIG_ESP_TASK_WDT_TIMEOUT_S * 1000U,
        .idle_core_mask = 0,    // restore: leave as configured
        .trigger_panic  = true,
    };
    // Format of a 4 GB card empirically took ~96 s in one bench
    // run; 180 s gives margin for slower cards / fragmentation.
    esp_task_wdt_config_t wdt_mount = {
        .timeout_ms     = 180000,
        .idle_core_mask = 0,
        .trigger_panic  = true,
    };
    esp_err_t wdt_r = esp_task_wdt_reconfigure(&wdt_mount);
    ESP_LOGI(TAG, "wdt reconfigure to %u ms -> %s",
             (unsigned)wdt_mount.timeout_ms, esp_err_to_name(wdt_r));

    esp_err_t r = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot, &mount, &s_card);

    (void)esp_task_wdt_reconfigure(&wdt_save);
    if (r != ESP_OK) {
        snprintf(s_stats.mount_error, sizeof(s_stats.mount_error),
                 "%s", esp_err_to_name(r));
        // Power the card rail back off so a missing-card socket isn't
        // sitting with VDD asserted.
        gpio_set_level(PIN_PWR, 1);     // 1 = card power OFF
        // Tear down what esp_vfs_fat_sdmmc_mount partially set up so
        // a retry (after the user inserts a card and POSTs /sd/mount
        // or /capture/start) can re-add the slot cleanly. Without
        // this, the SDMMC driver returns "slot is not available"
        // forever — even after the user fixes the underlying problem
        // — and the only recovery is a hard reboot.
        //
        // The slot is what stays allocated after a failed mount;
        // esp_vfs_fat_sdmmc_mount already cleans up its own VFS
        // registration on failure. Errors from this teardown call
        // are expected when init didn't reach slot allocation;
        // ignored.
        s_card = NULL;
        (void)sdmmc_host_deinit_slot(SDMMC_HOST_SLOT_0);
        if (ldo_handle) (void)sd_pwr_ctrl_del_on_chip_ldo(ldo_handle);
        return r;
    }

    s_stats.mounted = true;
    s_stats.mount_error[0] = '\0';
    sdmmc_card_print_info(stdout, s_card);

    // Attach the SDMMC stash buffer that sd_log_init pre-allocated.
    // SDMMC reuses host.dma_aligned_buffer for every PSRAM-sourced
    // transaction (reads AND writes) instead of calling
    // allocate_dma_buf per-call. Allocating it here (mount-time) is
    // too late — DMA-INT is fragmented after USB pool + tagger + the
    // sd_capture writer take their slices. Allocating in
    // sd_log_init (app_main pre-USB) catches DMA-INT with 139 KB
    // largest contiguous free.
    if (!s_card->host.dma_aligned_buffer && s_sdmmc_stash) {
        s_card->host.dma_aligned_buffer = s_sdmmc_stash;
        ESP_LOGI(TAG, "attached pre-allocated SDMMC stash @ %p", s_sdmmc_stash);
    } else if (!s_sdmmc_stash) {
        ESP_LOGW(TAG, "no SDMMC stash — PSRAM-sourced SDMMC ops will "
                      "allocate per-transaction and may fail under "
                      "DMA-INT pressure");
    }
    return ESP_OK;
}

static esp_err_t open_log_file(void)
{
    struct stat st;
    if (stat(LOG_DIR, &st) != 0) {
        if (mkdir(LOG_DIR, 0775) != 0) {
            ESP_LOGW(TAG, "mkdir(%s) failed errno=%d", LOG_DIR, errno);
            return ESP_FAIL;
        }
    }

    // One file per boot keyed by boot timestamp (microseconds since
    // epoch isn't available; use esp_timer_get_time which is microseconds
    // since boot — really we want a wall-clock once NTP is integrated;
    // for now, the boot tick distinguishes runs).
    char path[64];
    int64_t t0 = esp_timer_get_time();
    snprintf(path, sizeof(path), LOG_DIR "/log-%lld.ndjson", (long long)t0);

    s_log = fopen(path, "a");
    if (!s_log) {
        ESP_LOGE(TAG, "fopen(%s) failed errno=%d", path, errno);
        return ESP_FAIL;
    }
    setvbuf(s_log, NULL, _IOFBF, 4096);    // 4 KB FILE buffer — coalesce writes
    strlcpy(s_stats.log_path, path, sizeof(s_stats.log_path));
    s_stats.log_open = true;
    ESP_LOGI(TAG, "ACARS log open: %s", path);
    return ESP_OK;
}

esp_err_t sd_log_init(void)
{
    if (s_q) return ESP_OK;        // idempotent

    s_stats_mu = xSemaphoreCreateMutex();
    if (!s_stats_mu) return ESP_ERR_NO_MEM;

    // Pre-allocate the SDMMC stash buffer here, BEFORE USB pool +
    // tagger fragment DMA-INT. Mount happens later (lazy on first
    // log message OR explicit POST /sd/mount) and would otherwise
    // find DMA-INT too tight for a contiguous block. Without the
    // stash, every PSRAM-sourced SDMMC op (sd_log writes;
    // /capture/file reads into PSRAM dst) returns EIO under load.
    //
    // Sized to SDMMC's actual per-chunk cap: the host descriptor's
    // unaligned_multi_block_rw_max_chunk_size is 16 sectors = 8 KB.
    // 16 KB = 2x margin. Was 64 KB initially, which left only 47 KB
    // largest DMA-INT contiguous after USB pool init — too small
    // for the 66 KB tagger struct → fft_burst_tagger_init failed and
    // the device sat at 0 bursts/sec forever.
    s_sdmmc_stash = heap_caps_aligned_alloc(64, 16 * 1024,
                                              MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (s_sdmmc_stash) {
        ESP_LOGI(TAG, "pre-allocated 16 KB SDMMC stash @ %p (DMA-INT largest now %u)",
                 s_sdmmc_stash,
                 (unsigned)heap_caps_get_largest_free_block(
                     MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
    } else {
        ESP_LOGW(TAG, "SDMMC stash early-alloc failed — PSRAM-sourced "
                      "ops will allocate per-transaction (may fail)");
    }

    // Queue in PSRAM (NOT DMA-capable internal SRAM, the default for
    // xQueueCreate). 32 × ~304-byte items = ~10 KB; if it lands in
    // internal SRAM it steals from the USB transfer pool and kills
    // live-SDR throughput. PSRAM is plenty fast for the per-message
    // copy (~600 ns at 200 MHz).
    s_q = xQueueCreateWithCaps(EMIT_QUEUE_DEPTH, sizeof(acars_msg_t),
                                MALLOC_CAP_SPIRAM);
    if (!s_q) return ESP_ERR_NO_MEM;

    // Writer task pinned to Core 0 — Core 1 is already heavily loaded
    // by the DSP/worker/ingest pipeline. Stack in PSRAM for the same
    // reason as the queue (default xTaskCreate puts stacks in internal
    // SRAM). 6 KB stack rate is fine on PSRAM — the writer task is at
    // 1 Hz flush + ~few-Hz queue dequeue, latency-tolerant.
    BaseType_t ok = xTaskCreatePinnedToCoreWithCaps(writer_task, "sd_log",
                                                     WRITER_STACK, NULL,
                                                     WRITER_PRIO, NULL, 0,
                                                     MALLOC_CAP_SPIRAM);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "writer task create failed");
        return ESP_ERR_NO_MEM;
    }
    // Lazy mount: the writer task will attempt the SDMMC + FATFS bring-up
    // the first time it receives an ACARS message. No internal SRAM is
    // consumed by the SD subsystem until then — important on no-card
    // boots and during the gap between boot and the first decode.
    ESP_LOGI(TAG, "SD log writer ready on Core 0 (prio %d, queue %d, "
                  "queue+stack in PSRAM) — lazy mount on first message",
             WRITER_PRIO, EMIT_QUEUE_DEPTH);
    return ESP_OK;
}

esp_err_t sd_log_force_mount(void)
{
    if (s_log) return ESP_OK;             // already up
    // Reset the latched "tried + failed" flag so the next attempt isn't
    // short-circuited by the writer task.
    s_mount_attempted = false;
    return try_mount_and_open();
}

esp_err_t sd_log_force_format(void)
{
    // Close the log file first so f_mkfs doesn't trip over an open
    // handle. Subsequent acars writes will lazy-reopen the log.
    if (s_log) {
        fflush(s_log);
        fclose(s_log);
        s_log = NULL;
        xSemaphoreTake(s_stats_mu, portMAX_DELAY);
        s_stats.log_open = false;
        s_stats.log_path[0] = '\0';
        xSemaphoreGive(s_stats_mu);
    }
    // f_mkfs takes up to ~3 min on a 4 GB card; bump the WDT so the
    // task that called us (httpd) doesn't trip during the format.
    esp_task_wdt_reconfigure(&(esp_task_wdt_config_t){
        .timeout_ms = 240000, .idle_core_mask = 0, .trigger_panic = true });

    esp_err_t r = ESP_OK;
    if (s_card) {
        ESP_LOGW(TAG, "force_format: erasing card");
        r = esp_vfs_fat_sdcard_format(MOUNT_POINT, s_card);
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "esp_vfs_fat_sdcard_format failed: %s",
                     esp_err_to_name(r));
        } else {
            ESP_LOGI(TAG, "force_format: complete — card reformatted FAT32");
        }
    } else {
        ESP_LOGW(TAG, "force_format: no card mounted");
        r = ESP_ERR_INVALID_STATE;
    }

    // Restore WDT to its normal value.
    esp_task_wdt_reconfigure(&(esp_task_wdt_config_t){
        .timeout_ms = 5000, .idle_core_mask = 0, .trigger_panic = true });

    // Recreate /sdcard/acars/ for the log + capture files, then
    // re-open a fresh log file. esp_vfs_fat_sdcard_format leaves the
    // VFS mount in place, so sd_log_force_mount would short-circuit on
    // the stale s_log==NULL path and try to re-register the same mount
    // point (which fails with INVALID_STATE). Re-opening the log here
    // brings s_log back so subsequent /capture/start / sd_log_emit
    // calls see a healthy logger.
    if (r == ESP_OK) {
        mkdir(MOUNT_POINT "/acars", 0775);
        (void)open_log_file();
    }
    return r;
}

void sd_log_emit(const acars_msg_t *m)
{
    if (!s_q || !m) return;
    // Non-blocking: drop on queue-full. Producer (frame_decoder) must
    // never block waiting on SD.
    (void)xQueueSend(s_q, m, 0);
}

void sd_log_get_stats(sd_log_stats_t *out)
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
