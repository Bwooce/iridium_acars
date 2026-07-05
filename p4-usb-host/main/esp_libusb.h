#ifndef ESP_LIBUSB_H
#define ESP_LIBUSB_H

#include "usb/usb_host.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifndef portMAX_DELAY
#define portMAX_DELAY (TickType_t)0xffffffffUL
#endif

#define CTRL_OUT (USB_BM_REQUEST_TYPE_TYPE_VENDOR | USB_BM_REQUEST_TYPE_DIR_OUT)
#define CTRL_IN (USB_BM_REQUEST_TYPE_TYPE_VENDOR | USB_BM_REQUEST_TYPE_DIR_IN)

#define USB_SETUP_PACKET_INIT_CONTROL(setup_pkt_ptr, bm_reqtype, b_request, w_value, w_index, w_length) ({ \
    (setup_pkt_ptr)->bmRequestType = bm_reqtype;                                                           \
    (setup_pkt_ptr)->bRequest      = b_request;                                                            \
    (setup_pkt_ptr)->wValue        = w_value;                                                              \
    (setup_pkt_ptr)->wIndex        = w_index;                                                              \
    (setup_pkt_ptr)->wLength       = w_length;                                                             \
})

typedef struct
{
    usb_host_client_handle_t client_hdl;
    uint8_t                  dev_addr;
    usb_device_handle_t      dev_hdl;
    uint32_t                 actions;
} class_driver_t;

// USB bulk-IN transfer pool. usb_host_transfer_alloc returns DMA-
// capable internal SRAM (CONFIG_USB_HOST_DWC_DMA_CAP_MEMORY_IN_PSRAM
// is disabled for errata hardening -- see sdkconfig.defaults).
//
// Total budget here = COUNT * SIZE. The post-s_conv-move DMA-internal
// pool has ~70 KB free pre-stream; 8 * 8 KB = 64 KB fits with margin.
// More transfers = less back-pressure on the SDR (smaller pool causes
// rb_full_drops once the consumer falls behind for even a few ms).
//
// History:
//  - 8 x 16 KB (128 KB) original. Fails ESP_ERR_NO_MEM after the
//    wideband C front end's static allocations (~96 KB DMA-internal
//    for ingest_core1 raw+conv) consume the budget.
//  - 4 x 8 KB (32 KB) fits in the constrained pool but throttles
//    the SDR to ~0.85 MB/s vs 2.5 MB/s needed -> rb_full_drops.
//  - 8 x 8 KB (64 KB) is the post-s_conv-move setting. Verified the
//    DMA-internal heap accommodates this comfortably.
#define ASYNC_TRANSFER_COUNT 8
#define ASYNC_TRANSFER_SIZE (8 * 1024)

// PSRAM stream ring size. Single source of truth for both the
// usbring_init() call and the /status capacity metric — these were
// once separate literals and drifted 8× apart after the 512 KB → 4 MB
// upgrade (#109). MUST stay a power of two (usbring_init() rejects
// anything else) — 4 MiB already is.
#define STREAM_RINGBUF_BYTES (4 * 1024 * 1024)

typedef struct
{
    bool                is_adsb;
    uint8_t            *response_buf;
    bool                is_done;
    bool                is_success;
    int                 bytes_transferred;
    usb_transfer_t     *transfer;
    usb_device_handle_t dev_hdl; // Permanent handle
    // Serialises esp_libusb_control_transfer() callers: they read-modify-write
    // the transfer/response_buf/is_done fields above and share one
    // usb_host_client event loop. Without this, the AGC task
    // (agc.c, multi control-transfer gain sequence) can race the class_driver
    // task's own control transfers, freeing a transfer the other side still
    // has in flight (#T8).
    SemaphoreHandle_t xfer_mutex;

    // Async streaming. The stream byte ring itself is a module-level
    // singleton owned by usbring.c (T49a) — mirrors signal_buffer.c's
    // circular_buf pattern — so there's no handle to store here anymore.
    usb_transfer_t *transfers[ASYNC_TRANSFER_COUNT];
    bool            streaming;
} class_adsb_dev;

void init_adsb_dev();
void stream_transfer_cb(usb_transfer_t *transfer);
void transfer_read_cb(usb_transfer_t *transfer);
int  esp_libusb_control_transfer(class_driver_t *driver_obj, uint8_t bm_req_type, uint8_t b_request, uint16_t wValue, uint16_t wIndex, unsigned char *data, uint16_t wLength, unsigned int timeout);
int  esp_libusb_start_stream(class_driver_t *driver_obj, unsigned char endpoint);
// Zero-copy stream read (T49a): peeks the usbring instead of memcpy'ing
// into a caller buffer. On success (return 0), *out_ptr points directly
// into the ring's PSRAM backing store and *received is the contiguous
// byte count available there, clamped to max_length (which may be LESS
// than max_length near a physical wrap boundary — callers must already
// tolerate short reads, same as before). Always non-blocking (the old
// `timeout` parameter is gone — every call site used 0). The caller
// MUST eventually pass the same *received count to
// esp_libusb_consume_stream() once it — and anything it handed the
// pointer to — is done reading, before peeking again (see usbring.h's
// "single outstanding span" note).
int esp_libusb_read_stream(const uint8_t **out_ptr, size_t max_length, size_t *received);
// Reclaim `n` bytes at the ring's tail. Must match a length previously
// returned by esp_libusb_read_stream() that hasn't already been
// (partially) consumed.
void                esp_libusb_consume_stream(size_t n);
void                esp_libusb_get_ringbuffer_info(size_t *used, size_t *capacity);
usb_device_handle_t esp_libusb_get_dev_hdl();
void                esp_libusb_set_dev_hdl(usb_device_handle_t hdl);
void                esp_libusb_get_string_descriptor_ascii(const usb_str_desc_t *str_desc, char *str);

// Diagnostic stats for the streaming bulk-IN endpoint. Read & reset by the
// caller. Useful for distinguishing between:
//   - device sending short packets (actual_bytes < requested_bytes)
//   - host stack throttling (rb_full_drops > 0)
//   - underlying USB errors (status_errors > 0, broken down by status code)
typedef struct {
    uint32_t completed;             // transfers that completed normally (status==COMPLETED)
    uint32_t status_errors;         // transfers with non-COMPLETED status
    uint32_t resubmit_errors;       // failed to resubmit transfer
    uint32_t rb_full_drops;         // transfer payload couldn't fit in ringbuffer
    uint32_t short_xfers;           // completed but actual_num_bytes < num_bytes
    uint64_t total_actual_bytes;    // sum of actual_num_bytes from completed transfers
    uint64_t total_requested_bytes; // sum of num_bytes from completed transfers
    uint8_t  last_error_status;     // last non-COMPLETED status seen
    // Producer-side ringbuffer fill tracking — sampled inside the USB callback
    // so we see the *peak* fill, not the post-drain residual the consumer sees.
    size_t   producer_rb_max_used;     // peak bytes-in-ringbuffer observed at send time
    size_t   producer_rb_used_at_drop; // bytes-used when xRingbufferSend failed
    uint32_t producer_samples;         // number of producer-side fill samples taken
} usb_stream_stats_t;

void esp_libusb_get_stream_stats(usb_stream_stats_t *out);

// Lifetime totals — never reset. Parallel counters maintained
// alongside the per-second stats above. Lets external monitors
// compute deltas across an arbitrary window without racing
// status_logger's reset-on-read.
typedef struct {
    uint64_t completed;     // total successful transfers since boot
    uint64_t rb_full_drops; // total transfers dropped at ringbuf-send
    uint64_t status_errors; // total transfers with non-COMPLETED status
    uint64_t short_xfers;   // total transfers where actual_bytes < requested
} usb_stream_totals_t;

void esp_libusb_get_stream_totals(usb_stream_totals_t *out);

// #124: URBs whose 3-attempt resubmit retry exhausted (pool size
// shrinks). Has never fired in production; surfaced here so
// /diag/recovery_counters can prove it.
uint32_t esp_libusb_xfer_pool_lost(void);

#endif // ESP_LIBUSB_H
