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

#if CONFIG_SMOKE_TEST_MODE
#include "smoke_test.h"
#endif

#define DAEMON_TASK_PRIORITY 4
#define CLASS_TASK_PRIORITY 3

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
// (s_raw[], s_conv[] in ingest_core1.c). This avoids the APM-560
// concurrency window. Before adding ANY of (SDMMC, GMAC, USB-OTGFS,
// APM enforcement policy) that touches PSRAM concurrently with the
// signal-buffer DMA, audit the PSRAM access ordering — APM-560
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
        .skip_phy_setup = false,
        .root_port_unpowered = true,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
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

    bool has_clients = true;
    bool has_devices = true;
    int64_t last_idle_log = esp_timer_get_time();
    while (has_clients || has_devices)
    {
        uint32_t event_flags = 0;
        esp_err_t r = usb_host_lib_handle_events(pdMS_TO_TICKS(2000), &event_flags);
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
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
        {
            has_clients = false;
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE)
        {
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
        esp_reset_reason_t r = esp_reset_reason();
        const char *name = "?";
        switch (r) {
            case ESP_RST_POWERON:  name = "POWERON";        break;
            case ESP_RST_EXT:      name = "EXT";            break;
            case ESP_RST_SW:       name = "SW";             break;
            case ESP_RST_PANIC:    name = "PANIC";          break;
            case ESP_RST_INT_WDT:  name = "INT_WDT";        break;
            case ESP_RST_TASK_WDT: name = "TASK_WDT";       break;
            case ESP_RST_WDT:      name = "OTHER_WDT";      break;
            case ESP_RST_DEEPSLEEP:name = "DEEPSLEEP";      break;
            case ESP_RST_BROWNOUT: name = "BROWNOUT";       break;
            case ESP_RST_SDIO:     name = "SDIO";           break;
            case ESP_RST_USB:      name = "USB";            break;
            case ESP_RST_JTAG:     name = "JTAG";           break;
            case ESP_RST_UNKNOWN:  name = "UNKNOWN";        break;
            default:               name = "(other)";        break;
        }
        ESP_LOGW("BOOT", "reset reason: %s (%d)", name, (int)r);
    }

    // Load runtime config from NVS (D18). Defaults are applied for any
    // missing keys — system stays operational with no NVS data.
    app_config_init();
    app_config_log();

    // D16 software AGC. Idle unless gain_mode==SOFTWARE_AGC.
    agc_init();

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
    // class_driver stays on Core 0 — that's where the DSP feed runs.
    xTaskCreatePinnedToCore(class_driver_task,
                            "class",
                            4096,
                            (void *)signaling_sem,
                            CLASS_TASK_PRIORITY,
                            &class_driver_task_hdl,
                            0);

    vTaskDelay(10); // Add a short delay to let the tasks run

    // Wait for the tasks to complete
    for (int i = 0; i < 2; i++)
    {
        xSemaphoreTake(signaling_sem, portMAX_DELAY);
    }

    // Delete the tasks
    vTaskDelete(class_driver_task_hdl);
    vTaskDelete(daemon_task_hdl);
}