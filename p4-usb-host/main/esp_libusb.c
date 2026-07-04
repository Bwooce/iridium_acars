#include "usb/usb_host.h"
#include "esp_log.h"
#include "esp_libusb.h"
#include "esp_timer.h"
#include "fault_inject.h"
#include "usbring.h"

static class_adsb_dev *adsbdev;

// Streaming bulk-IN diagnostic counters. Read & reset by
// esp_libusb_get_stream_stats(). volatile because they're written from the
// USB host task (transfer callback) and read from the class_driver task.
static volatile uint32_t s_xfer_completed       = 0;
static volatile uint32_t s_xfer_status_errors   = 0;
static volatile uint32_t s_xfer_resubmit_errors = 0;
static volatile uint32_t s_xfer_pool_lost       = 0; // URBs whose 3-attempt resubmit retry exhausted; pool size shrinks (#124)
uint32_t                 esp_libusb_xfer_pool_lost(void)
{
    return s_xfer_pool_lost;
}
static volatile uint32_t s_xfer_rb_full_drops   = 0;
static volatile uint32_t s_xfer_short           = 0;
static volatile uint64_t s_xfer_actual_bytes    = 0;
static volatile uint64_t s_xfer_requested_bytes = 0;
static volatile uint8_t  s_xfer_last_error      = 0;
// Cumulative-since-boot counters that never reset. The above are
// consumed by status_logger every second (read-and-reset); these
// parallel counters let /status JSON expose lifetime totals so an
// external monitor can compute deltas across capture cycles.
//
// Startup grace period: USB streaming has a settling transient
// (pool transfer churn, DMA-INT competition, EMA priming on tagger
// side) that produces ~5000 rb_full_drops in the first few seconds
// of every boot. The per-second stats above still record it (useful
// for diagnosing startup regressions), but the lifetime totals below
// only start incrementing AFTER the grace window so external
// monitors don't get a misleading 0.86% lifetime drop rate driven
// entirely by boot transients.
#define STREAM_STATS_GRACE_US (5 * 1000 * 1000)
static volatile int64_t  s_stream_start_us     = 0;
static volatile uint64_t s_total_completed     = 0;
static volatile uint64_t s_total_rb_full_drops = 0;
static volatile uint64_t s_total_status_errors = 0;
static volatile uint64_t s_total_short_xfers   = 0;
// Producer-side ringbuffer fill tracking. The class_driver consumer measures
// HWM after each read which biases towards 0; these are sampled in the USB
// callback (the producer) so we capture the actual peak fills.
static volatile size_t   s_producer_rb_max_used     = 0;
static volatile size_t   s_producer_rb_used_at_drop = 0;
static volatile uint32_t s_producer_samples         = 0;

// Mutex-take timeout for the control/bulk transfer critical section (#T8).
// Generous relative to a single USB control transfer (CTRL_TIMEOUT=300 ms in
// librtlsdr.c) since worst case is waiting out one other in-flight transfer,
// not an unbounded block.
#define XFER_MUTEX_TIMEOUT_MS 1000

void init_adsb_dev()
{
    adsbdev = calloc(1, sizeof(class_adsb_dev));
    assert(adsbdev != NULL); // boot-time, tiny: failure means heap is gone
    adsbdev->is_adsb    = true;
    adsbdev->xfer_mutex = xSemaphoreCreateMutex();
    if (adsbdev->xfer_mutex == NULL) {
        // Non-fatal here: esp_libusb_control_transfer/esp_libusb_bulk_transfer
        // both check for NULL and fail the transfer rather than dereference it.
        ESP_LOGE("LIBUSB", "Failed to create xfer_mutex — control/bulk transfers will fail");
    }
}

void bulk_transfer_read_cb(usb_transfer_t *transfer)
{
    for (int i = 0; i < transfer->actual_num_bytes; i++) {
        adsbdev->response_buf[i] = transfer->data_buffer[i];
    }
    adsbdev->is_done           = true;
    adsbdev->is_success        = transfer->status == 0;
    adsbdev->bytes_transferred = transfer->num_bytes;
    if (adsbdev->dev_hdl == NULL) adsbdev->dev_hdl = transfer->device_handle;
}

void transfer_read_cb(usb_transfer_t *transfer)
{
    for (int i = 0; i < transfer->actual_num_bytes; i++) {
        adsbdev->response_buf[i] = transfer->data_buffer[i];
    }
    adsbdev->is_done           = true;
    adsbdev->is_success        = transfer->status == 0;
    adsbdev->bytes_transferred = transfer->actual_num_bytes - sizeof(usb_setup_packet_t);
    if (adsbdev->dev_hdl == NULL) adsbdev->dev_hdl = transfer->device_handle;
}

int esp_libusb_bulk_transfer(class_driver_t *driver_obj, unsigned char endpoint, unsigned char *data, int length, int *transferred, unsigned int timeout)
{
    assert(driver_obj->client_hdl != NULL);
    usb_device_handle_t dev_hdl = driver_obj->dev_hdl ? driver_obj->dev_hdl : adsbdev->dev_hdl;

    // Serialise against esp_libusb_control_transfer(): both share
    // adsbdev->response_buf/is_done/is_success/bytes_transferred (#T8).
    if (adsbdev->xfer_mutex == NULL) {
        ESP_LOGE("LIBUSB", "bulk_transfer: xfer_mutex not initialised");
        return -1;
    }
    if (xSemaphoreTake(adsbdev->xfer_mutex, pdMS_TO_TICKS(XFER_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE("LIBUSB", "bulk_transfer: timed out waiting for xfer_mutex");
        return -1;
    }

    int             ret = -1;
    size_t          sizePacket;
    usb_transfer_t *transfer = NULL;
    esp_err_t       r;

    sizePacket = usb_round_up_to_mps(length, 64);
    if (usb_host_transfer_alloc(sizePacket, 0, &transfer) != ESP_OK ||
        transfer == NULL) {
        goto done;
    }

    transfer->num_bytes        = sizePacket;
    transfer->device_handle    = dev_hdl;
    transfer->bEndpointAddress = endpoint;
    transfer->callback         = bulk_transfer_read_cb;
    // driver_obj, not &driver_obj: the latter was the address of this
    // function's PARAMETER — dangling the moment we return. (No current
    // callback reads context; fixed so the next one that does can.)
    transfer->context     = (void *)driver_obj;
    transfer->timeout_ms  = timeout;
    adsbdev->is_done      = false;
    adsbdev->response_buf = calloc(sizePacket, sizeof(uint8_t));
    if (!adsbdev->response_buf) {
        usb_host_transfer_free(transfer);
        goto done;
    }

    r = usb_host_transfer_submit(transfer);
    if (r != ESP_OK) {
        free(adsbdev->response_buf);
        adsbdev->response_buf = NULL;
        usb_host_transfer_free(transfer);
        goto done;
    }
    while (!adsbdev->is_done) {
        usb_host_client_handle_events(driver_obj->client_hdl, portMAX_DELAY);
    }

    if (!adsbdev->is_success) {
        free(adsbdev->response_buf);
        adsbdev->response_buf = NULL;
        usb_host_transfer_free(transfer);
        goto done;
    }

    ESP_ERROR_CHECK(usb_host_endpoint_clear(dev_hdl, endpoint));
    for (int i = 0; i < length; i++) {
        data[i] = adsbdev->response_buf[i];
    }
    *transferred = adsbdev->bytes_transferred;
    free(adsbdev->response_buf);
    adsbdev->response_buf = NULL;
    usb_host_transfer_free(transfer);
    ret = 0;

done:
    xSemaphoreGive(adsbdev->xfer_mutex);
    return ret;
}

int esp_libusb_control_transfer(class_driver_t *driver_obj, uint8_t bm_req_type, uint8_t b_request, uint16_t wValue, uint16_t wIndex, unsigned char *data, uint16_t wLength, unsigned int timeout)
{
    // Serialise: adsbdev->transfer/response_buf/is_done are shared with
    // esp_libusb_bulk_transfer() and with any concurrent caller of this
    // function (e.g. the AGC task's multi-register gain sequence in
    // agc.c/librtlsdr.c racing the class_driver task's own control
    // transfers). Without this lock, one caller can free() a transfer or
    // response_buf that another caller's in-flight completion callback is
    // still about to write into (#T8).
    if (adsbdev->xfer_mutex == NULL) {
        ESP_LOGE("LIBUSB", "control_transfer: xfer_mutex not initialised");
        return -1;
    }
    if (xSemaphoreTake(adsbdev->xfer_mutex, pdMS_TO_TICKS(XFER_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE("LIBUSB", "control_transfer: timed out waiting for xfer_mutex");
        return -1;
    }

    int       ret = -1;
    size_t    sizePacket;
    esp_err_t r;

    if (adsbdev->transfer) {
        usb_host_transfer_free(adsbdev->transfer);
        adsbdev->transfer = NULL;
    }
    if (adsbdev->response_buf) {
        free(adsbdev->response_buf);
        adsbdev->response_buf = NULL;
    }

    sizePacket = sizeof(usb_setup_packet_t) + wLength;
    if (usb_host_transfer_alloc(sizePacket, 0, &adsbdev->transfer) != ESP_OK ||
        adsbdev->transfer == NULL) {
        goto done;
    }
    USB_SETUP_PACKET_INIT_CONTROL((usb_setup_packet_t *)adsbdev->transfer->data_buffer, bm_req_type, b_request, wValue, wIndex, wLength);
    adsbdev->transfer->num_bytes     = sizePacket;
    adsbdev->transfer->device_handle = driver_obj->dev_hdl ? driver_obj->dev_hdl : adsbdev->dev_hdl;
    adsbdev->transfer->timeout_ms    = timeout;
    adsbdev->transfer->context       = (void *)driver_obj; // was &param: dangling
    adsbdev->transfer->callback      = transfer_read_cb;
    adsbdev->is_done                 = false;
    adsbdev->response_buf            = calloc(sizePacket, sizeof(uint8_t));

    if (bm_req_type == CTRL_OUT) {
        for (uint8_t i = 0; i < wLength; i++) {
            adsbdev->transfer->data_buffer[sizeof(usb_setup_packet_t) + i] = data[i];
        }
    }
    r = usb_host_transfer_submit_control(driver_obj->client_hdl, adsbdev->transfer);
    if (r != ESP_OK) {
        goto done;
    }

    while (!adsbdev->is_done) {
        usb_host_client_handle_events(driver_obj->client_hdl, portMAX_DELAY);
    }

    if (!adsbdev->is_success) {
        goto done;
    }

    for (uint8_t i = 0; i < wLength; i++) {
        data[i] = adsbdev->response_buf[sizeof(usb_setup_packet_t) + i];
    }
    ret = adsbdev->bytes_transferred;

done:
    xSemaphoreGive(adsbdev->xfer_mutex);
    return ret;
}

void stream_transfer_cb(usb_transfer_t *transfer)
{
    class_adsb_dev *dev = adsbdev;
    if (!dev->streaming) {
        usb_host_transfer_free(transfer);
        return;
    }

    // Lifetime totals only count post-grace. Per-second stats still
    // capture the boot transient so status_logger can show it.
    bool post_grace = (s_stream_start_us != 0) &&
                      (esp_timer_get_time() - s_stream_start_us >= STREAM_STATS_GRACE_US);

    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED) {
        s_xfer_completed++;
        if (post_grace) s_total_completed++;
        s_xfer_actual_bytes += (uint32_t)transfer->actual_num_bytes;
        s_xfer_requested_bytes += (uint32_t)transfer->num_bytes;
        if (transfer->actual_num_bytes < transfer->num_bytes) {
            s_xfer_short++;
            if (post_grace) s_total_short_xfers++;
        }
        if (transfer->actual_num_bytes > 0) {
            // Sample fill BEFORE the write (usbring_get_info reads head/tail).
            size_t used_bytes = 0;
            usbring_get_info(&used_bytes, NULL);
            if (used_bytes > s_producer_rb_max_used) s_producer_rb_max_used = used_bytes;
            s_producer_samples++;

            bool ok = usbring_write(transfer->data_buffer, (uint32_t)transfer->actual_num_bytes);
            if (!ok) {
                s_xfer_rb_full_drops++;
                if (post_grace) s_total_rb_full_drops++;
                s_producer_rb_used_at_drop = used_bytes;
            }
        }
    } else {
        s_xfer_status_errors++;
        if (post_grace) s_total_status_errors++;
        s_xfer_last_error = (uint8_t)transfer->status;
    }

    // Resubmit this URB to keep it in the in-flight ring. A failure drops it
    // permanently from the ring (one fewer of ASYNC_TRANSFER_COUNT); if they
    // accumulate, the ring empties → no more completions → the stream stalls
    // (eventually caught by the stall watchdog). Retry a bounded few times to
    // ride out transient submit failures rather than leaking the URB.
    bool      submitted = false;
    esp_err_t last_err  = ESP_OK;
    // #122: FI_SITE_URB_SUBMIT simulates a URB whose 3 resubmit attempts all
    // failed, to exercise the pool-lost accounting + loud log below. It is
    // NON-destructive: the URB is still resubmitted in the !submitted block
    // so the in-flight pool depth is preserved and the test is repeatable.
    bool fi_urb = fault_inject_should_fail(FI_SITE_URB_SUBMIT);
    if (fi_urb) {
        last_err = ESP_ERR_NO_MEM;
        s_xfer_resubmit_errors += 3;
    } else {
        for (int attempt = 0; attempt < 3; attempt++) {
            last_err = usb_host_transfer_submit(transfer);
            if (last_err == ESP_OK) {
                submitted = true;
                break;
            }
            s_xfer_resubmit_errors++;
        }
    }
    if (!submitted) {
        // All 3 attempts exhausted — this URB is permanently retired from
        // the in-flight pool. Pool shrinks 8 → 7 → … silently until stream
        // throttle is caught by health_wdt (~30-90 s). Make it loud so a
        // post-mortem doesn't have to infer the cause from a downstream
        // STATUS-ERR line (#124). Rate-limited; with ASYNC_TRANSFER_COUNT=8
        // even 8 occurrences here means the entire pool is gone.
        s_xfer_pool_lost++;
        ESP_LOGE("LIBUSB", "URB resubmit exhausted 3 attempts: %s — pool shrunk (lost=%u)",
                 esp_err_to_name(last_err), (unsigned)s_xfer_pool_lost);
        if (fi_urb) {
            // #122 non-destructive: actually keep the URB in flight.
            usb_host_transfer_submit(transfer);
        }
    }
}

void esp_libusb_get_stream_stats(usb_stream_stats_t *out)
{
    out->completed                = s_xfer_completed;
    out->status_errors            = s_xfer_status_errors;
    out->resubmit_errors          = s_xfer_resubmit_errors;
    out->rb_full_drops            = s_xfer_rb_full_drops;
    out->short_xfers              = s_xfer_short;
    out->total_actual_bytes       = s_xfer_actual_bytes;
    out->total_requested_bytes    = s_xfer_requested_bytes;
    out->last_error_status        = s_xfer_last_error;
    out->producer_rb_max_used     = s_producer_rb_max_used;
    out->producer_rb_used_at_drop = s_producer_rb_used_at_drop;
    out->producer_samples         = s_producer_samples;
    s_xfer_completed              = 0;
    s_xfer_status_errors          = 0;
    s_xfer_resubmit_errors        = 0;
    s_xfer_rb_full_drops          = 0;
    s_xfer_short                  = 0;
    s_xfer_actual_bytes           = 0;
    s_xfer_requested_bytes        = 0;
    s_xfer_last_error             = 0;
    s_producer_rb_max_used        = 0;
    s_producer_rb_used_at_drop    = 0;
    s_producer_samples            = 0;
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
    esp_err_t ring_rc = usbring_init(STREAM_RINGBUF_BYTES);
    if (ring_rc != ESP_OK) {
        ESP_LOGE("LIBUSB", "Failed to create stream ring in PSRAM: %s",
                 esp_err_to_name(ring_rc));
        return -1;
    }

    dev->streaming = true;
    // Mark when streaming began so the transfer callback can decide
    // whether the lifetime counters should accumulate yet. See the
    // STREAM_STATS_GRACE_US block above.
    s_stream_start_us = esp_timer_get_time();
    // Diagnostic for the DMA-pool exhaustion symptom that previously
    // showed up as "Failed to alloc async transfer 0" with no further
    // detail. usb_host_transfer_alloc requires DMA-capable internal
    // SRAM because CONFIG_USB_HOST_DWC_DMA_CAP_MEMORY_IN_PSRAM is off
    // (silicon errata hardening, see sdkconfig.defaults). Print heap
    // state so a future failure points immediately at the right
    // budget knob.
    size_t internal_free_at_start    = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
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
            size_t free_now    = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
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
        dev->transfers[i]->device_handle    = dev_hdl;
        dev->transfers[i]->bEndpointAddress = endpoint;
        dev->transfers[i]->callback         = stream_transfer_cb;
        dev->transfers[i]->context          = (void *)driver_obj;
        dev->transfers[i]->num_bytes        = ASYNC_TRANSFER_SIZE;

        r = usb_host_transfer_submit(dev->transfers[i]);
        if (r != ESP_OK) {
            // KNOWN LEAK on this fatal path: the ringbuffer and the
            // already-submitted URBs are NOT reclaimed — earlier
            // transfers are in flight, and freeing them (or the ring
            // their callback writes into) without an endpoint
            // halt+flush would be a use-after-free. Start-stream
            // failure leaves the device unusable anyway; the recovery
            // path is a reboot. Proper unwind = halt+flush+free, only
            // worth doing if this ever needs to be retryable.
            ESP_LOGE("LIBUSB", "Failed to submit async transfer %d: %d", i, r);
            return -1;
        }
    }
    ESP_LOGI("LIBUSB", "Started async stream on handle %p", dev_hdl);
    return 0;
}

int esp_libusb_read_stream(const uint8_t **out_ptr, size_t max_length, size_t *received)
{
    if (!adsbdev) {
        *received = 0;
        return -1;
    }

    uint32_t       n_contig = 0;
    const uint8_t *ptr      = usbring_peek(&n_contig);
    if (ptr == NULL || n_contig == 0) {
        *received = 0;
        return -1;
    }
    size_t n  = (size_t)n_contig < max_length ? (size_t)n_contig : max_length;
    *out_ptr  = ptr;
    *received = n;
    return 0;
}

void esp_libusb_consume_stream(size_t n)
{
    usbring_consume((uint32_t)n);
}

void esp_libusb_get_ringbuffer_info(size_t *used, size_t *capacity)
{
    usbring_get_info(used, capacity);
}

usb_device_handle_t esp_libusb_get_dev_hdl()
{
    if (adsbdev) return adsbdev->dev_hdl;
    return NULL;
}

void esp_libusb_get_string_descriptor_ascii(const usb_str_desc_t *str_desc, char *str)
{
    if (str_desc == NULL) return;
    // bLength includes the 2-byte descriptor header, so the payload is
    // (bLength - 2) / 2 UTF-16 units — the old bLength/2 count read one
    // unit past wData. Also NUL-terminate; callers treat str as a C
    // string.
    int n = (str_desc->bLength - 2) / 2;
    if (n < 0) n = 0;
    for (int i = 0; i < n; i++) {
        str[i] = (char)str_desc->wData[i];
    }
    str[n] = '\0';
}
