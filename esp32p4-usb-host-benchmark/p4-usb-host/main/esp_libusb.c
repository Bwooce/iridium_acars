#include "usb/usb_host.h"
#include "esp_log.h"
#include "esp_libusb.h"

static class_adsb_dev *adsbdev;

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
        if (transfer->actual_num_bytes > 0) {
            xRingbufferSend(dev->ringbuf, transfer->data_buffer, transfer->actual_num_bytes, 0);
        }
        usb_host_transfer_submit(transfer);
    } else {
        usb_host_transfer_submit(transfer);
    }
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

    dev->ringbuf = xRingbufferCreateWithCaps(512 * 1024, RINGBUF_TYPE_BYTEBUF, MALLOC_CAP_SPIRAM);
    if (dev->ringbuf == NULL) {
        ESP_LOGE("LIBUSB", "Failed to create stream ringbuffer in PSRAM");
        return -1;
    }

    dev->streaming = true;
    for (int i = 0; i < ASYNC_TRANSFER_COUNT; i++) {
        esp_err_t r = usb_host_transfer_alloc(ASYNC_TRANSFER_SIZE, 0, &dev->transfers[i]);
        if (r != ESP_OK || dev->transfers[i] == NULL) {
            ESP_LOGE("LIBUSB", "Failed to alloc async transfer %d", i);
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

void esp_libusb_get_ringbuffer_info(size_t *free, size_t *max_free)
{
    if (adsbdev && adsbdev->ringbuf) {
        vRingbufferGetInfo(adsbdev->ringbuf, NULL, NULL, NULL, NULL, free);
        *max_free = 512 * 1024; 
    } else {
        *free = 0;
        *max_free = 0;
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
