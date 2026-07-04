/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "usb/usb_host.h"
#include "sdkconfig.h"
#include "app_config.h"
#include "agc.h"
#include "wifi_link.h"
#include "http_server.h"
#include "captive_dns.h"
#include "acars_push.h"
#include "ota_runner.h"
#include "sd_log.h"
#include "sd_capture.h"
#include "serial_cmd.h"
#include "frame_pdu.h"
#include "frame_decoder.h"
#include "bch_decoder.h"
#include "aggregator_ingest.h"
#include "frame_link.h"
#include "esp_iot_log.h"
#include "esp_heap_caps.h"

// One-line snapshot of internal-DMA-capable heap (the pool the USB
// transfer ring competes for). Temporary diagnostic for the SD-link
// regression hunt.
static void log_dma_int_heap(const char *tag)
{
    size_t f = heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    size_t l = heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    ESP_LOGW("HEAPDIAG", "%-30s DMA-INT free=%u largest=%u",
             tag, (unsigned)f, (unsigned)l);
}

#if CONFIG_SMOKE_TEST_MODE
#include "smoke_test.h"
#endif

#define DAEMON_TASK_PRIORITY 4
// T48 (docs/perf-decoupling-design-2026-07-04.md §T48): class_driver_task
// is now "usb_pump" — it only drains USB URB completions into the
// usbring (T49a) and handles enumeration/recovery/the 1 Hz snapshot. The
// actual ring-drain + DSP feed (the "USB consumer" the comment below used
// to describe) moved into a separate task, dsp_feed, spawned internally
// by class_driver.c at priority (this priority - 1) — see
// action_start_stream()'s comment for the derivation. Bumped 6 -> 7 so
// that dsp_feed (6) still outranks httpd (prio 5, http_server.c): a
// multi-MB /capture/file download must not be able to starve ring drain
// and reintroduce the rb_full_drops task #91 fixed. usb_pump itself
// stays event-driven (usb_host_client_handle_events' 100 ms cap — 10
// TICKS at the 100 Hz tick), so raising its priority further above
// dsp_feed does not starve anything below it. See task #91 / #101.
#define CLASS_TASK_PRIORITY 7

extern void class_driver_task(void *arg);

static const char *TAG = "DAEMON";

// Note on VBUS for ESP32-P4-Pico (Waveshare):
// The native USB OTG (Picoblade P1) pin 1 is hardwired to VCC_5V — no GPIO enable.
// Earlier code drove GPIO 45 / 54 thinking they were VBUS_EN; per the schematic
// netlist, GPIO 45 controls SD-card power (Q1 SI2301CDS), and GPIO 54 is just a
// breakout pin (header GP00). Neither has anything to do with USB VBUS.
//
// v3.x compatibility note: GPIO 54 is reassigned from NC to VDD_HP_1 (HP digital
// power rail) on ESP32-P4 rev v3.x silicon. If/when this firmware targets v3.x
// chips, GPIO 54 must NOT be driven as a generic IO — it's a power rail.
//
// APM-560 runbook (ESP32-P4 v1.3, see memory/project_p4_errata_status.md):
// We currently run with the default (permissive) APM policy and a single
// AHB master targeting PSRAM (AXI-GDMA via esp_async_memcpy in
// signal_buffer.c). USB DWC-OTG-HS DMA writes into INTERNAL SRAM only
// (the URB transfer pool in esp_libusb.c — ASYNC_TRANSFER_COUNT ×
// ASYNC_TRANSFER_SIZE). This avoids the APM-560 concurrency window; the
// usbring PSRAM stream ring (T49a, usbring.c) is filled by a plain CPU
// memcpy in the URB completion callback, not by the DMA engine, so it
// doesn't change this invariant. Before adding ANY of (SDMMC, GMAC,
// USB-OTGFS, APM enforcement policy) that touches PSRAM concurrently
// with the signal-buffer DMA, audit the PSRAM access ordering — APM-560
// recovery is system-reset-only on v1.3.

void host_lib_daemon_task(void *arg)
{
    SemaphoreHandle_t signaling_sem = (SemaphoreHandle_t)arg;

    ESP_LOGI(TAG, "Installing USB Host Library");
    // root_port_unpowered=true: the root port starts powered OFF, so we must
    // explicitly power it ON below. This guarantees a fresh USB attach
    // sequence on every boot — important because on the Nano VBUS to the
    // RTL-SDR is hardwired-on (U2 EN held active), so the device otherwise
    // never sees a power cycle across ESP32 resets and can get stuck after
    // an unclean shutdown. Toggling root port power forces SOFs to stop and
    // re-evaluates attach, which recovers most stuck states without a physical
    // unplug.
    usb_host_config_t host_config = {
        .skip_phy_setup      = false,
        .root_port_unpowered = true,
        .intr_flags          = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    ESP_LOGI(TAG, "USB Host Library installed; powering root port ON");
    esp_err_t pwr_r = usb_host_lib_set_root_port_power(true);
    if (pwr_r != ESP_OK) {
        ESP_LOGW(TAG, "set_root_port_power(true) returned 0x%x (%s)",
                 pwr_r, esp_err_to_name(pwr_r));
    }
    ESP_LOGI(TAG, "Root port powered, waiting for device events...");

    // Signal to the class driver task that the host library is installed
    xSemaphoreGive(signaling_sem);
    vTaskDelay(10); // Short delay to let client task spin up

    bool    has_clients   = true;
    bool    has_devices   = true;
    int64_t last_idle_log = esp_timer_get_time();
    while (has_clients || has_devices) {
        uint32_t  event_flags = 0;
        esp_err_t r           = usb_host_lib_handle_events(pdMS_TO_TICKS(2000), &event_flags);
        if (r != ESP_OK && r != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "usb_host_lib_handle_events returned 0x%x (%s)", r, esp_err_to_name(r));
        }
        if (event_flags) {
            ESP_LOGI(TAG, "Host lib event_flags=0x%08lx", (unsigned long)event_flags);
        }
        int64_t now = esp_timer_get_time();
        if (now - last_idle_log >= 5 * 1000000) {
            ESP_LOGI(TAG, "Host lib idle (no NEW_DEV yet — check D+/D- polarity on Picoblade pigtail, VBUS at device)");
            last_idle_log = now;
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            has_clients = false;
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            has_devices = false;
        }
    }
    ESP_LOGI(TAG, "No more clients and devices");

    // Uninstall the USB Host Library
    ESP_ERROR_CHECK(usb_host_uninstall());
    // Wait to be deleted
    xSemaphoreGive(signaling_sem);
    vTaskSuspend(NULL);
}

void app_main(void)
{
    // Log the reset reason on every boot so we can see at a glance
    // whether the previous run ended via a panic, brownout, watchdog,
    // external reset, etc. Especially useful when the smoke test
    // appears to reboot mid-Phase-2 with no panic dump on the serial
    // line — distinguishes brownout (RST_BROWNOUT) from USB-CDC line-
    // state reset (RST_EXT or RST_USB) from a silent abort.
    {
        esp_reset_reason_t r    = esp_reset_reason();
        const char        *name = "?";
        switch (r) {
        case ESP_RST_POWERON:
            name = "POWERON";
            break;
        case ESP_RST_EXT:
            name = "EXT";
            break;
        case ESP_RST_SW:
            name = "SW";
            break;
        case ESP_RST_PANIC:
            name = "PANIC";
            break;
        case ESP_RST_INT_WDT:
            name = "INT_WDT";
            break;
        case ESP_RST_TASK_WDT:
            name = "TASK_WDT";
            break;
        case ESP_RST_WDT:
            name = "OTHER_WDT";
            break;
        case ESP_RST_DEEPSLEEP:
            name = "DEEPSLEEP";
            break;
        case ESP_RST_BROWNOUT:
            name = "BROWNOUT";
            break;
        case ESP_RST_SDIO:
            name = "SDIO";
            break;
        case ESP_RST_USB:
            name = "USB";
            break;
        case ESP_RST_JTAG:
            name = "JTAG";
            break;
        case ESP_RST_UNKNOWN:
            name = "UNKNOWN";
            break;
        default:
            name = "(other)";
            break;
        }
        ESP_LOGW("BOOT", "reset reason: %s (%d)", name, (int)r);
    }

    log_dma_int_heap("app_main entry");

#if CONFIG_FRAME_LINK_LOOPBACK_SELFTEST
    // One-board SPI loopback bring-up test (#136): requires the master pins
    // jumpered to the FRAME_LINK_LB_* slave pins. Runs before anything else
    // claims the SPI buses, logs PASS/FAIL, then idles.
    {
        esp_err_t lb = frame_link_loopback_selftest();
        ESP_LOGW("BOOT", "frame_link loopback self-test: %s",
                 lb == ESP_OK ? "PASS" : "FAIL");
        return;
    }
#endif

    // Load runtime config from NVS (D18). Defaults are applied for any
    // missing keys — system stays operational with no NVS data.
    app_config_init();
    app_config_log();

    // Serial NVS command interface — always active so config can be
    // recovered even when WiFi is down or credentials are lost.
    serial_cmd_init();

    // D16 software AGC. Idle unless gain_mode==SOFTWARE_AGC.
    agc_init();

    log_dma_int_heap("after app_config + agc");

    // D17 Wi-Fi via the C6 esp_hosted slave. STA if SSID in NVS, else
    // open SoftAP for first-time config. The smoke-mode build skips
    // Wi-Fi entirely since the SDIO link to the C6 isn't useful
    // during the synthetic-fixture regression.
#if !CONFIG_SMOKE_TEST_MODE
    wifi_link_start();
    {
        app_config_t snap;
        app_config_snapshot(&snap);
        iot_log_config_t iot_cfg = IOT_LOG_CONFIG_DEFAULT();
        if (snap.station_id[0]) {
            iot_cfg.device_name = snap.station_id;
        }
        iot_log_init(&iot_cfg);
    }
    http_server_start();
    // Only in AP-fallback mode: hijack DNS so phones auto-open the
    // config form via captive-portal detection. STA mode leaves DNS
    // alone (the user has a router that does it properly).
    if (wifi_link_is_ap_mode()) {
        captive_dns_start();
    }
    // ACARS UDP push (D17) — task is always created; emits only when
    // out_host/out_port are set in NVS.
    acars_push_init();
    log_dma_int_heap("before sd_log_init");
    // SD card log (#63) — mounts the card if present; no-op if absent.
    sd_log_init();
    // SD card raw IQ capture (#63) — spawns the writer task; stream
    // buffer and SD mount happen lazily on POST /capture/start.
    sd_capture_init();
    log_dma_int_heap("after sd_log_init");
    // D19 OTA: if we got here without crashing, the current image is
    // healthy — cancel any pending rollback the bootloader was tracking.
    ota_runner_mark_valid();
#endif

#if CONFIG_SMOKE_TEST_MODE
    // Smoke test mode: bypass the USB stack entirely and run the
    // synthetic-IQ regression test on a single Core 0 task. The smoke
    // test never returns (parks the CPU after logging the result).
    // 32 KB stack: the smoke task drives the entire ingest+DSP path
    // and accumulates frames from libm calls deep inside
    // fft_burst_tagger_step / dsp_processor_feed. 24 KB worked at
    // threshold=10 dB but overflowed again at 14 dB (different
    // callback-fire pattern → different stack depth). 32 KB is the
    // generous setting that has plenty of headroom for any iteration.
    // The production class_driver task uses 8 KB because it doesn't
    // drive DSP itself.
    xTaskCreatePinnedToCore((TaskFunction_t)smoke_test_run,
                            "smoke", 32768, NULL, 5, NULL, 0);
    return;
#endif

#if CONFIG_DEVICE_ROLE_WORKER || CONFIG_DEVICE_ROLE_COMBINED_LOOPBACK
    // Worker output ring for decoded-frame PDUs (#135). Must exist before
    // the worker task starts emitting frames.
    frame_pdu_queue_init();
#endif

#if CONFIG_DEVICE_ROLE_WORKER
    // Worker ships PDUs to the aggregator over SPI (#136). COMBINED skips
    // this — it drains the queue in-process via aggregator_ingest.
    if (frame_link_slave_start() != ESP_OK) {
        ESP_LOGE(TAG, "WORKER: frame_link_slave_start failed");
    }
#endif

#if CONFIG_DEVICE_ROLE_AGGREGATOR
    // Aggregator role (#119/#134): no SDR/USB front end. It receives
    // decoded-frame PDUs from N worker P4s over SPI and runs the
    // classifier + outputs. The shared services above (wifi_link,
    // http_server, acars_push, sd_log) are already up. Bring up the
    // decode chain + PDU consumer here (#137); the SPI slave that feeds
    // the PDU queue lands in Phase 3. Until then the queue stays empty
    // and the ingest task idles.
    frame_pdu_queue_init();
    bch_decoder_init();
    if (frame_decoder_init() != ESP_OK) {
        ESP_LOGE(TAG, "AGGREGATOR: frame_decoder_init failed");
    }
    if (aggregator_ingest_init() != ESP_OK) {
        ESP_LOGE(TAG, "AGGREGATOR: aggregator_ingest_init failed");
    }
    // SPI master ingest (#136): clocks PDUs from the worker(s) into the
    // local frame_pdu queue, which aggregator_ingest drains.
    if (frame_link_master_start() != ESP_OK) {
        ESP_LOGE(TAG, "AGGREGATOR: frame_link_master_start failed");
    }
    ESP_LOGW(TAG, "DEVICE_ROLE=AGGREGATOR: front-end disabled; PDU consumer + "
                  "SPI master ready, awaiting worker PDUs");
    return;
#endif

    // WORKER / COMBINED_LOOPBACK: full front-end DSP pipeline (today's
    // single-P4 behaviour). COMBINED also consumes its own PDU queue
    // in-process (added in later phases).
    SemaphoreHandle_t signaling_sem = xSemaphoreCreateBinary();

    TaskHandle_t daemon_task_hdl;
    TaskHandle_t class_driver_task_hdl;
    // Daemon pinned to Core 1. usb_host_install runs from this task and
    // registers the USB DWC OTG ISR on whichever core executed it — putting
    // it on Core 1 means transfer-complete interrupts (~300/sec) fire on
    // Core 1 (which is mostly idle while no bursts are active) rather than
    // preempting Core 0's hot DSP loop. Saves the ISR-induced jitter on
    // every consumer cycle. Reverts to Core 0 if cross-core overhead with
    // the class_driver client (still on Core 0) ends up worse.
    xTaskCreatePinnedToCore(host_lib_daemon_task,
                            "daemon",
                            4096,
                            (void *)signaling_sem,
                            DAEMON_TASK_PRIORITY,
                            &daemon_task_hdl,
                            1);
    // usb_pump (class_driver_task) stays on Core 0. It spawns its own
    // dsp_feed sibling task internally (also Core 0, T48) once streaming
    // starts — see class_driver.c:action_start_stream().
    xTaskCreatePinnedToCore(class_driver_task,
                            "usb_pump",
                            4096,
                            (void *)signaling_sem,
                            CLASS_TASK_PRIORITY,
                            &class_driver_task_hdl,
                            0);

    vTaskDelay(10); // Add a short delay to let the tasks run

    // Wait for the tasks to complete
    for (int i = 0; i < 2; i++) {
        xSemaphoreTake(signaling_sem, portMAX_DELAY);
    }

    // Delete the tasks
    vTaskDelete(class_driver_task_hdl);
    vTaskDelete(daemon_task_hdl);
}