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
#include "esp_timer.h"
#include "usb/usb_host.h"

#define DAEMON_TASK_PRIORITY 4
#define CLASS_TASK_PRIORITY 3

extern void class_driver_task(void *arg);

static const char *TAG = "DAEMON";

// Note on VBUS for ESP32-P4-Pico (Waveshare):
// The native USB OTG (Picoblade P1) pin 1 is hardwired to VCC_5V — no GPIO enable.
// Earlier code drove GPIO 45 / 54 thinking they were VBUS_EN; per the schematic
// netlist, GPIO 45 controls SD-card power (Q1 SI2301CDS), and GPIO 54 is just a
// breakout pin (header GP00). Neither has anything to do with USB VBUS.

static void host_lib_daemon_task(void *arg)
{
    SemaphoreHandle_t signaling_sem = (SemaphoreHandle_t)arg;

    ESP_LOGI(TAG, "Installing USB Host Library");
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    ESP_LOGI(TAG, "USB Host Library installed; waiting for device events...");

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
    SemaphoreHandle_t signaling_sem = xSemaphoreCreateBinary();

    TaskHandle_t daemon_task_hdl;
    TaskHandle_t class_driver_task_hdl;
    // Create daemon task
    xTaskCreatePinnedToCore(host_lib_daemon_task,
                            "daemon",
                            4096,
                            (void *)signaling_sem,
                            DAEMON_TASK_PRIORITY,
                            &daemon_task_hdl,
                            0);
    // Create the class driver task
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