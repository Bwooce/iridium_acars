#include "usb/usb_host.h"
#include "esp_log.h"
#include "esp_libusb.h"

static class_adsb_dev *adsbdev;

// Streaming bulk-IN diagnostic counters. Read & reset by
// esp_libusb_get_stream_stats(). volatile because they're written from the
// USB host task (transfer callback) and read from the class_driver task.
static volatile uint32_t s_xfer_completed = 0;
static volatile uint32_t s_xfer_status_errors = 0;
static volatile uint32_t s_xfer_resubmit_errors = 0;
static volatile uint32_t s_xfer_rb_full_drops = 0;
static volatile uint32_t s_xfer_short = 0;
static volatile uint64_t s_xfer_actual_bytes = 0;
static volatile uint64_t s_xfer_requested_bytes = 0;
static volatile uint8_t  s_xfer_last_error = 0;
// Cumulative-since-boot counters that never reset. The above are
// consumed by status_logger every second (read-and-reset); these
// parallel counters let /status JSON expose lifetime totals so an
// external monitor can compute deltas across capture cycles.
static volatile uint64_t s_total_completed       = 0;
static volatile uint64_t s_total_rb_full_drops   = 0;
static volatile uint64_t s_total_status_errors   = 0;
static volatile uint64_t s_total_short_xfers     = 0;
// Producer-side ringbuffer fill tracking. The class_driver consumer measures
// HWM after each read which biases towards 0; these are sampled in the USB
// callback (the producer) so we capture the actual peak fills.
static volatile size_t   s_producer_rb_max_used = 0;
static volatile size_t   s_producer_rb_used_at_drop = 0;
static volatile uint32_t s_producer_samples = 0;

void init_adsb_dev()
{
    adsbdev = calloc(1, sizeof(class_adsb_dev));
    adsbdev->is_adsb = true;
}

void bulk_transfer_read_cb(usb_transfer_t *transfer)
{
    for (int i = 0; i < transfer->actual_num_bytes; i++)
    {
        adsbdev->response_buf[i] = transfer->data_buffer[i];
    }
    adsbdev->is_done = true;
    adsbdev->is_success = transfer->status == 0;
    adsbdev->bytes_transferred = transfer->num_bytes;
    if (adsbdev->dev_hdl == NULL) adsbdev->dev_hdl = transfer->device_handle;
}

void transfer_read_cb(usb_transfer_t *transfer)
{
    for (int i = 0; i < transfer->actual_num_bytes; i++)
    {
        adsbdev->response_buf[i] = transfer->data_buffer[i];
    }
    adsbdev->is_done = true;
    adsbdev->is_success = transfer->status == 0;
    adsbdev->bytes_transferred = transfer->actual_num_bytes - sizeof(usb_setup_packet_t);
    if (adsbdev->dev_hdl == NULL) adsbdev->dev_hdl = transfer->device_handle;
}

int esp_libusb_bulk_transfer(class_driver_t *driver_obj, unsigned char endpoint, unsigned char *data, int length, int *transferred, unsigned int timeout)
{
    assert(driver_obj->client_hdl != NULL);
    usb_device_handle_t dev_hdl = driver_obj->dev_hdl ? driver_obj->dev_hdl : adsbdev->dev_hdl;
    
    size_t sizePacket = usb_round_up_to_mps(length, 64);
    usb_transfer_t *transfer = NULL;
    usb_host_transfer_alloc(sizePacket, 0, &transfer);
    
    transfer->num_bytes = sizePacket;
    transfer->device_handle = dev_hdl;
    transfer->bEndpointAddress = endpoint;
    transfer->callback = bulk_transfer_read_cb;
    transfer->context = (void *)&driver_obj;
    transfer->timeout_ms = timeout;
    adsbdev->is_done = false;
    adsbdev->response_buf = calloc(sizePacket, sizeof(uint8_t));

    esp_err_t r = usb_host_transfer_submit(transfer);
    if (r != ESP_OK)
    {
        free(adsbdev->response_buf);
        usb_host_transfer_free(transfer);
        return -1;
    }
    while (!adsbdev->is_done)
    {
        usb_host_client_handle_events(driver_obj->client_hdl, portMAX_DELAY);
    }
    
    if (!adsbdev->is_success)
    {
        free(adsbdev->response_buf);
        usb_host_transfer_free(transfer);
        return -1;
    }
    
    ESP_ERROR_CHECK(usb_host_endpoint_clear(dev_hdl, endpoint));
    for (int i = 0; i < length; i++)
    {
        data[i] = adsbdev->response_buf[i];
    }
    *transferred = adsbdev->bytes_transferred;
    free(adsbdev->response_buf);
    adsbdev->response_buf = NULL;
    usb_host_transfer_free(transfer);
    return 0;
}

int esp_libusb_control_transfer(class_driver_t *driver_obj, uint8_t bm_req_type, uint8_t b_request, uint16_t wValue, uint16_t wIndex, unsigned char *data, uint16_t wLength, unsigned int timeout)
{
    if (adsbdev->transfer) {
        usb_host_transfer_free(adsbdev->transfer);
        adsbdev->transfer = NULL;
    }
    if (adsbdev->response_buf) {
        free(adsbdev->response_buf);
        adsbdev->response_buf = NULL;
    }
    
    size_t sizePacket = sizeof(usb_setup_packet_t) + wLength;
    usb_host_transfer_alloc(sizePacket, 0, &adsbdev->transfer);
    USB_SETUP_PACKET_INIT_CONTROL((usb_setup_packet_t *)adsbdev->transfer->data_buffer, bm_req_type, b_request, wValue, wIndex, wLength);
    adsbdev->transfer->num_bytes = sizePacket;
    adsbdev->transfer->device_handle = driver_obj->dev_hdl ? driver_obj->dev_hdl : adsbdev->dev_hdl;
    adsbdev->transfer->timeout_ms = timeout;
    adsbdev->transfer->context = (void *)&driver_obj;
    adsbdev->transfer->callback = transfer_read_cb;
    adsbdev->is_done = false;
    adsbdev->response_buf = calloc(sizePacket, sizeof(uint8_t));

    if (bm_req_type == CTRL_OUT)
    {
        for (uint8_t i = 0; i < wLength; i++)
        {
            adsbdev->transfer->data_buffer[sizeof(usb_setup_packet_t) + i] = data[i];
        }
    }
    esp_err_t r = usb_host_transfer_submit_control(driver_obj->client_hdl, adsbdev->transfer);
    if (r != ESP_OK)
    {
        return -1;
    }

    while (!adsbdev->is_done)
    {
        usb_host_client_handle_events(driver_obj->client_hdl, portMAX_DELAY);
    }
    
    if (!adsbdev->is_success)
    {
        return -1;
    }
    
    for (uint8_t i = 0; i < wLength; i++)
    {
        data[i] = adsbdev->response_buf[sizeof(usb_setup_packet_t) + i];
    }
    return adsbdev->bytes_transferred;
}

void stream_transfer_cb(usb_transfer_t *transfer)
{
    class_adsb_dev *dev = adsbdev;
    if (!dev->streaming) {
        usb_host_transfer_free(transfer);
        return;
    }

    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED) {
        s_xfer_completed++;
        s_total_completed++;
        s_xfer_actual_bytes    += (uint32_t)transfer->actual_num_bytes;
        s_xfer_requested_bytes += (uint32_t)transfer->num_bytes;
        if (transfer->actual_num_bytes < transfer->num_bytes) {
            s_xfer_short++;
            s_total_short_xfers++;
        }
        if (transfer->actual_num_bytes > 0) {
            // Sample fill BEFORE the send. vRingbufferGetInfo's last arg is
            // uxItemsWaiting which IS the used-byte count for a byte buffer
            // (not free bytes — earlier code had this inverted).
            UBaseType_t items_waiting = 0;
            vRingbufferGetInfo(dev->ringbuf, NULL, NULL, NULL, NULL, &items_waiting);
            size_t used_bytes = (size_t)items_waiting;
            if (used_bytes > s_producer_rb_max_used) s_producer_rb_max_used = used_bytes;
            s_producer_samples++;

            BaseType_t ok = xRingbufferSend(dev->ringbuf, transfer->data_buffer,
                                            transfer->actual_num_bytes, 0);
            if (ok != pdTRUE) {
                s_xfer_rb_full_drops++;
                s_total_rb_full_drops++;
                s_producer_rb_used_at_drop = used_bytes;
            }
        }
    } else {
        s_xfer_status_errors++;
        s_total_status_errors++;
        s_xfer_last_error = (uint8_t)transfer->status;
    }

    if (usb_host_transfer_submit(transfer) != ESP_OK) {
        s_xfer_resubmit_errors++;
    }
}

void esp_libusb_get_stream_stats(usb_stream_stats_t *out)
{
    out->completed             = s_xfer_completed;
    out->status_errors         = s_xfer_status_errors;
    out->resubmit_errors       = s_xfer_resubmit_errors;
    out->rb_full_drops         = s_xfer_rb_full_drops;
    out->short_xfers           = s_xfer_short;
    out->total_actual_bytes    = s_xfer_actual_bytes;
    out->total_requested_bytes = s_xfer_requested_bytes;
    out->last_error_status     = s_xfer_last_error;
    out->producer_rb_max_used  = s_producer_rb_max_used;
    out->producer_rb_used_at_drop = s_producer_rb_used_at_drop;
    out->producer_samples      = s_producer_samples;
    s_xfer_completed = 0;
    s_xfer_status_errors = 0;
    s_xfer_resubmit_errors = 0;
    s_xfer_rb_full_drops = 0;
    s_xfer_short = 0;
    s_xfer_actual_bytes = 0;
    s_xfer_requested_bytes = 0;
    s_xfer_last_error = 0;
    s_producer_rb_max_used = 0;
    s_producer_rb_used_at_drop = 0;
    s_producer_samples = 0;
}

void esp_libusb_get_stream_totals(usb_stream_totals_t *out)
{
    if (!out) return;
    out->completed     = s_total_completed;
    out->rb_full_drops = s_total_rb_full_drops;
    out->status_errors = s_total_status_errors;
    out->short_xfers   = s_total_short_xfers;
}

void esp_libusb_set_dev_hdl(usb_device_handle_t hdl)
{
    if (adsbdev) {
        adsbdev->dev_hdl = hdl;
        ESP_LOGI("LIBUSB", "Permanent device handle set: %p", hdl);
    }
}

int esp_libusb_start_stream(class_driver_t *driver_obj, unsigned char endpoint)
{
    class_adsb_dev *dev = adsbdev;
    if (!dev) return -1;

    usb_device_handle_t dev_hdl = driver_obj->dev_hdl ? driver_obj->dev_hdl : dev->dev_hdl;
    if (!dev_hdl) {
        ESP_LOGE("LIBUSB", "Cannot start stream: NULL device handle");
        return -1;
    }

    // 4 MB PSRAM ringbuffer ≈ 900 ms of buffering at 4.5 MB/s
    // sustained USB ingest. Sized to absorb the worst consumer-side
    // stalls we see:
    //  - sd_capture writer fwrite blocking 100-300 ms during SDMMC
    //    multi-sector writes
    //  - SD card block-erase pauses up to 500 ms+ on cheap SDHC
    //  - tagger spikes when bursts arrive in clusters (back-to-back
    //    sub-frames within a single satellite pass)
    // Previously 512 KB (~110 ms); we measured rb_full_drops of
    // 40-50/sec during sustained capture, ≈ 700 KB/s of dropped
    // samples corrupting downstream burst data. 4 MB is well within
    // the 32 MB PSRAM budget and turns the consumer-stall window
    // into a true elastic queue.
    dev->ringbuf = xRingbufferCreateWithCaps(4 * 1024 * 1024,
                                              RINGBUF_TYPE_BYTEBUF, MALLOC_CAP_SPIRAM);
    if (dev->ringbuf == NULL) {
        ESP_LOGE("LIBUSB", "Failed to create stream ringbuffer in PSRAM");
        return -1;
    }

    dev->streaming = true;
    // Diagnostic for the DMA-pool exhaustion symptom that previously
    // showed up as "Failed to alloc async transfer 0" with no further
    // detail. usb_host_transfer_alloc requires DMA-capable internal
    // SRAM because CONFIG_USB_HOST_DWC_DMA_CAP_MEMORY_IN_PSRAM is off
    // (silicon errata hardening, see sdkconfig.defaults). Print heap
    // state so a future failure points immediately at the right
    // budget knob.
    size_t internal_free_at_start = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    size_t internal_largest_at_start = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    ESP_LOGI("LIBUSB", "Pre-stream DMA-internal heap: free=%u KB, largest=%u KB; "
                       "need %d × %d KB = %d KB",
             (unsigned)(internal_free_at_start / 1024),
             (unsigned)(internal_largest_at_start / 1024),
             ASYNC_TRANSFER_COUNT, ASYNC_TRANSFER_SIZE / 1024,
             ASYNC_TRANSFER_COUNT * ASYNC_TRANSFER_SIZE / 1024);
    for (int i = 0; i < ASYNC_TRANSFER_COUNT; i++) {
        esp_err_t r = usb_host_transfer_alloc(ASYNC_TRANSFER_SIZE, 0, &dev->transfers[i]);
        if (r != ESP_OK || dev->transfers[i] == NULL) {
            size_t free_now = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
            size_t largest_now = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
            ESP_LOGE("LIBUSB", "transfer_alloc #%d failed: r=0x%x (%s), "
                              "DMA-internal heap free=%u KB largest=%u KB "
                              "(needed %d KB). Bump CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL "
                              "or shrink ASYNC_TRANSFER_COUNT/_SIZE.",
                     i, r, esp_err_to_name(r),
                     (unsigned)(free_now / 1024),
                     (unsigned)(largest_now / 1024),
                     ASYNC_TRANSFER_SIZE / 1024);
            return -1;
        }
        dev->transfers[i]->device_handle = dev_hdl;
        dev->transfers[i]->bEndpointAddress = endpoint;
        dev->transfers[i]->callback = stream_transfer_cb;
        dev->transfers[i]->context = (void *)driver_obj;
        dev->transfers[i]->num_bytes = ASYNC_TRANSFER_SIZE;

        r = usb_host_transfer_submit(dev->transfers[i]);
        if (r != ESP_OK) {
            ESP_LOGE("LIBUSB", "Failed to submit async transfer %d: %d", i, r);
            return -1;
        }
    }
    ESP_LOGI("LIBUSB", "Started async stream on handle %p", dev_hdl);
    return 0;
}

int esp_libusb_read_stream(uint8_t *buffer, size_t length, size_t *received, TickType_t timeout)
{
    class_adsb_dev *dev = adsbdev;
    if (!dev || !dev->ringbuf) {
        *received = 0;
        return -1;
    }
    
    size_t item_size;
    uint8_t *item = xRingbufferReceiveUpTo(dev->ringbuf, &item_size, timeout, length);
    if (item != NULL) {
        memcpy(buffer, item, item_size);
        vRingbufferReturnItem(dev->ringbuf, item);
        *received = item_size;
        return 0;
    }
    *received = 0;
    return -1;
}

void esp_libusb_get_ringbuffer_info(size_t *used, size_t *capacity)
{
    if (adsbdev && adsbdev->ringbuf) {
        // vRingbufferGetInfo's last arg is uxItemsWaiting which for a
        // RINGBUF_TYPE_BYTEBUF is the number of pending bytes (= used).
        // Earlier code passed this into a variable called `free` and then
        // computed (1 - free/total) as "usage", which was inverted.
        UBaseType_t items_waiting = 0;
        vRingbufferGetInfo(adsbdev->ringbuf, NULL, NULL, NULL, NULL, &items_waiting);
        *used = (size_t)items_waiting;
        *capacity = 512 * 1024;
    } else {
        *used = 0;
        *capacity = 0;
    }
}

usb_device_handle_t esp_libusb_get_dev_hdl()
{
    if (adsbdev) return adsbdev->dev_hdl;
    return NULL;
}

void esp_libusb_get_string_descriptor_ascii(const usb_str_desc_t *str_desc, char *str)
{
    if (str_desc == NULL) return;
    for (int i = 0; i < str_desc->bLength / 2; i++) {
        str[i] = (char)str_desc->wData[i];
    }
}
