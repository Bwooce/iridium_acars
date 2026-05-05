/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "usb/usb_host.h"
#include "esp_libusb.h"
#include "dsp_processor.h"
#include "signal_buffer.h"
#include "worker_core1.h"
#include "bch_decoder.h"
#include "rtl-sdr.h"

#define CLIENT_NUM_EVENT_MSG 5

#define ACTION_OPEN_DEV 0x01
#define ACTION_GET_DEV_INFO 0x02
#define ACTION_GET_DEV_DESC 0x04
#define ACTION_GET_CONFIG_DESC 0x08
#define ACTION_GET_STR_DESC 0x10
#define ACTION_CLOSE_DEV 0x20
#define ACTION_EXIT 0x40
#define ACTION_START_STREAM 0x80

static const char *TAG = "CLASS";
static rtlsdr_dev_t *rtldev = NULL;
static class_driver_t s_driver_obj = {0};

static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    class_driver_t *driver_obj = &s_driver_obj;
    switch (event_msg->event)
    {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        ESP_LOGI(TAG, "New USB device connected at address %d", event_msg->new_dev.address);
        if (driver_obj->dev_addr == 0)
        {
            driver_obj->dev_addr = event_msg->new_dev.address;
            driver_obj->actions |= ACTION_OPEN_DEV;
        }
        break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        ESP_LOGI(TAG, "USB device gone");
        if (driver_obj->dev_hdl != NULL)
        {
            driver_obj->actions |= ACTION_CLOSE_DEV;
        }
        break;
    default:
        ESP_LOGI(TAG, "Unknown USB client event: %d", event_msg->event);
        break;
    }
}

static void action_open_dev(class_driver_t *driver_obj)
{
    ESP_LOGI(TAG, "Opening device at address %d", driver_obj->dev_addr);
    esp_err_t err = rtlsdr_open(&rtldev, driver_obj->dev_addr, driver_obj->client_hdl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open RTL-SDR: %d", err);
        return;
    }
    
    driver_obj->dev_hdl = esp_libusb_get_dev_hdl();
    ESP_LOGI(TAG, "Device opened, handle: %p", driver_obj->dev_hdl);
    
    driver_obj->actions &= ~ACTION_OPEN_DEV;
    driver_obj->actions |= ACTION_START_STREAM;
}

static void action_start_stream(class_driver_t *driver_obj)
{
    ESP_LOGI(TAG, "Configuring RTL-SDR...");
    rtlsdr_set_sample_rate(rtldev, 2560000);
    rtlsdr_set_center_freq(rtldev, 1626000000);
    rtlsdr_set_tuner_gain_mode(rtldev, 0);
    rtlsdr_reset_buffer(rtldev);

    ESP_LOGI(TAG, "Initializing System Buffers...");
    signal_buffer_init();
    worker_core1_init();
    bch_decoder_init();

    ESP_LOGI(TAG, "Initializing DSP...");
    dsp_processor_init(worker_core1_push_burst);

    ESP_LOGI(TAG, "Starting Async Stream...");
    esp_libusb_start_stream(driver_obj, 0x81);
    
    driver_obj->actions &= ~ACTION_START_STREAM;
}

static void action_close_dev(class_driver_t *driver_obj)
{
    ESP_LOGI(TAG, "Closing device");
    if (rtldev) {
        rtlsdr_close(rtldev);
        rtldev = NULL;
    }
    driver_obj->dev_hdl = NULL;
    driver_obj->dev_addr = 0;
    driver_obj->actions &= ~ACTION_CLOSE_DEV;
    driver_obj->actions |= ACTION_EXIT;
}

void class_driver_task(void *arg)
{
    SemaphoreHandle_t signaling_sem = (SemaphoreHandle_t)arg;
    memset(&s_driver_obj, 0, sizeof(class_driver_t));

    xSemaphoreTake(signaling_sem, portMAX_DELAY);

    ESP_LOGI(TAG, "Registering Client");
    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = CLIENT_NUM_EVENT_MSG,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg = (void *)&s_driver_obj,
        },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &s_driver_obj.client_hdl));

    uint32_t out_block_size = 16 * 1024;
    uint8_t *buffer = malloc(out_block_size);
    int16_t *convert_buf = malloc(out_block_size * sizeof(int16_t));
    
    uint64_t total_bytes = 0;
    int64_t start_time = esp_timer_get_time();
    int64_t last_report = start_time;

    uint64_t dsp_total_time_us = 0;
    uint32_t dsp_frame_count = 0;
    size_t min_free_rb = 512 * 1024;

    while (1)
    {
        usb_host_client_handle_events(s_driver_obj.client_hdl, 0);

        if (s_driver_obj.actions & ACTION_OPEN_DEV) action_open_dev(&s_driver_obj);
        if (s_driver_obj.actions & ACTION_START_STREAM) action_start_stream(&s_driver_obj);
        if (s_driver_obj.actions & ACTION_CLOSE_DEV) action_close_dev(&s_driver_obj);
        if (s_driver_obj.actions & ACTION_EXIT) break;

        // If streaming, process data
        size_t n_read = 0;
        if (esp_libusb_read_stream(buffer, out_block_size, &n_read, 0) == 0) {
            total_bytes += n_read;
            for (int i = 0; i < n_read; i++) {
                convert_buf[i] = ((int16_t)buffer[i] - 128) << 8;
            }
            
            signal_buffer_push(convert_buf, n_read / 2);

            int64_t dsp_start = esp_timer_get_time();
            dsp_processor_feed(convert_buf, n_read / 2);
            dsp_total_time_us += (esp_timer_get_time() - dsp_start);
            dsp_frame_count += (n_read / 2) / 2048; 

            size_t free_rb, total_rb;
            esp_libusb_get_ringbuffer_info(&free_rb, &total_rb);
            if (free_rb < min_free_rb) min_free_rb = free_rb;
        } else {
            // No data read, yield to prevent tight spin starvation
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        int64_t now = esp_timer_get_time();
        if (now - last_report >= 1000000) {
            double elapsed = (now - start_time) / 1000000.0;
            double rate = (total_bytes / (1024.0 * 1024.0)) / elapsed;
            
            float avg_dsp_us = (dsp_frame_count > 0) ? (float)dsp_total_time_us / dsp_frame_count : 0;
            float rb_usage = 100.0f * (1.0f - (float)min_free_rb / (512 * 1024));

            ESP_LOGI(TAG, "Rate:%.2fMB/s | DSP:%.0fus | RB-HWM:%.1f%% | PSRAM:%d", 
                     rate, avg_dsp_us, rb_usage, heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
            
            last_report = now;
            dsp_total_time_us = 0;
            dsp_frame_count = 0;
            min_free_rb = 512 * 1024;
        }
    }

    ESP_LOGI(TAG, "Deregistering Client");
    usb_host_client_deregister(s_driver_obj.client_hdl);
    xSemaphoreGive(signaling_sem);
    vTaskDelete(NULL);
}
