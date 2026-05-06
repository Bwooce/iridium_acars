#include "usb/usb_host.h"

#include "freertos/ringbuf.h"

#ifndef portMAX_DELAY
#define portMAX_DELAY (TickType_t)0xffffffffUL
#endif

#define CTRL_OUT (USB_BM_REQUEST_TYPE_TYPE_VENDOR | USB_BM_REQUEST_TYPE_DIR_OUT)
#define CTRL_IN (USB_BM_REQUEST_TYPE_TYPE_VENDOR | USB_BM_REQUEST_TYPE_DIR_IN)

#define USB_SETUP_PACKET_INIT_CONTROL(setup_pkt_ptr, bm_reqtype, b_request, w_value, w_index, w_length) ({ \
    (setup_pkt_ptr)->bmRequestType = bm_reqtype;                                                           \
    (setup_pkt_ptr)->bRequest = b_request;                                                                 \
    (setup_pkt_ptr)->wValue = w_value;                                                                     \
    (setup_pkt_ptr)->wIndex = w_index;                                                                     \
    (setup_pkt_ptr)->wLength = w_length;                                                                   \
})

typedef struct
{
    usb_host_client_handle_t client_hdl;
    uint8_t dev_addr;
    usb_device_handle_t dev_hdl;
    uint32_t actions;
} class_driver_t;

#define ASYNC_TRANSFER_COUNT 8
#define ASYNC_TRANSFER_SIZE (16 * 1024)

typedef struct
{
    bool is_adsb;
    uint8_t *response_buf;
    bool is_done;
    bool is_success;
    int bytes_transferred;
    usb_transfer_t *transfer;
    usb_device_handle_t dev_hdl; // Permanent handle
    
    // Async streaming
    RingbufHandle_t ringbuf;
    usb_transfer_t *transfers[ASYNC_TRANSFER_COUNT];
    bool streaming;
} class_adsb_dev;

void init_adsb_dev();
void bulk_transfer_read_cb(usb_transfer_t *transfer);
void stream_transfer_cb(usb_transfer_t *transfer);
void transfer_read_cb(usb_transfer_t *transfer);
int esp_libusb_bulk_transfer(class_driver_t *driver_obj, unsigned char endpoint, unsigned char *data, int length, int *transferred, unsigned int timeout);
int esp_libusb_control_transfer(class_driver_t *driver_obj, uint8_t bm_req_type, uint8_t b_request, uint16_t wValue, uint16_t wIndex, unsigned char *data, uint16_t wLength, unsigned int timeout);
int esp_libusb_start_stream(class_driver_t *driver_obj, unsigned char endpoint);
int esp_libusb_read_stream(uint8_t *buffer, size_t length, size_t *received, TickType_t timeout);
void esp_libusb_get_ringbuffer_info(size_t *free, size_t *max_free);
usb_device_handle_t esp_libusb_get_dev_hdl();
void esp_libusb_set_dev_hdl(usb_device_handle_t hdl);
void esp_libusb_get_string_descriptor_ascii(const usb_str_desc_t *str_desc, char *str);

// Diagnostic stats for the streaming bulk-IN endpoint. Read & reset by the
// caller. Useful for distinguishing between:
//   - device sending short packets (actual_bytes < requested_bytes)
//   - host stack throttling (rb_full_drops > 0)
//   - underlying USB errors (status_errors > 0, broken down by status code)
typedef struct {
    uint32_t completed;           // transfers that completed normally (status==COMPLETED)
    uint32_t status_errors;       // transfers with non-COMPLETED status
    uint32_t resubmit_errors;     // failed to resubmit transfer
    uint32_t rb_full_drops;       // transfer payload couldn't fit in ringbuffer
    uint32_t short_xfers;         // completed but actual_num_bytes < num_bytes
    uint64_t total_actual_bytes;  // sum of actual_num_bytes from completed transfers
    uint64_t total_requested_bytes; // sum of num_bytes from completed transfers
    uint8_t  last_error_status;   // last non-COMPLETED status seen
    // Producer-side ringbuffer fill tracking — sampled inside the USB callback
    // so we see the *peak* fill, not the post-drain residual the consumer sees.
    size_t   producer_rb_max_used; // peak bytes-in-ringbuffer observed at send time
    size_t   producer_rb_used_at_drop; // bytes-used when xRingbufferSend failed
    uint32_t producer_samples;     // number of producer-side fill samples taken
} usb_stream_stats_t;

void esp_libusb_get_stream_stats(usb_stream_stats_t *out);