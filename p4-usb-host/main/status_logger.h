#ifndef STATUS_LOGGER_H
#define STATUS_LOGGER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_libusb.h"
#include "dsp_processor.h"
#include "ingest_core1.h"
#include "worker_core1.h"

// Per-second snapshot of all the diagnostic counters that the production
// build emits as a status block. Owned by class_driver, posted to a queue
// once per second; consumed by the dedicated logger_task on Core 1, which
// does all the printf / ESP_LOGI / UART work off Core 0's hot path.
//
// Why this matters: when status formatting ran inline on Core 0, the ~6
// ESP_LOGI calls/sec produced enough consumer-loop pause (~5-10 ms) to
// fill the 512 KB USB ringbuffer past 480 KB and drop ~7 transfers/sec.
// Moving the formatting to Core 1 (which is otherwise idle outside the
// short ingest_core1 bursts) brings drops to 0.
typedef struct {
    int64_t  window_us;               // wall-time of this 1 s window
    int64_t  elapsed_us;              // wall-time since boot
    uint64_t bytes_window;            // USB bytes received in this window
    uint64_t total_bytes;             // USB bytes received since boot
    uint32_t feed_calls_window;       // dsp_processor_feed calls in this window
    uint64_t dsp_total_time_us;       // sum of feed wall time
    uint32_t dsp_frame_count;         // FFT steps (2048-sample blocks) fed
                                      // through the tagger this window.
                                      // Proportional to USB byte rate by
                                      // construction — NOT a burst/frame
                                      // rate. Printed as "steps=" in the
                                      // STATUS line; see dsp.gone_bursts
                                      // (printed as "bursts=") for the
                                      // actual tagger burst-dispatch rate.
    uint64_t cycle_read_us;           // sum of esp_libusb_read_stream wall time
    uint64_t cycle_handle_events_us;  // sum of usb_host_client_handle_events wall time
    uint64_t cycle_take_converted_us; // sum of ingest_core1_take_converted wall time
    uint32_t cycle_iterations;        // count of class_driver loop iterations
    int      psram_free_bytes;        // heap_caps_get_free_size(MALLOC_CAP_SPIRAM)

    usb_stream_stats_t us;
    dsp_stage_stats_t  dsp;
    ingest_stats_t     ingest;
    worker_stats_t     ws;
} status_snapshot_t;

// Bring up the logger task + queue. Call once at startup. Task pinned to
// Core 1 at low priority so it never preempts ingest_core1 (prio 8).
esp_err_t status_logger_init(void);

// Non-blocking post. Returns true if the snapshot was queued, false if
// the queue was full (in which case this snapshot is silently dropped —
// the next one will land instead). Intended to be called once per second
// from class_driver_task on Core 0.
bool status_logger_post(const status_snapshot_t *snap);

// Copy the most recently emitted snapshot into *out. Returns false if no
// snapshot has been emitted yet (nothing to show). Non-resetting, purely a
// read of the last logged values — the HTTP /status page uses this to
// surface the same fields the STATUS log line prints WITHOUT calling the
// resetting worker/dsp getters (which would steal counts from the logger's
// own 1 s window). The read is unlocked and cross-core (http on Core 0 vs
// logger on Core 1); a torn field is harmless for a diagnostic display.
bool status_logger_get_last(status_snapshot_t *out);

// Capacity telemetry accumulated since boot, for remote monitoring of a
// headless deployment. Peak vs mean answers "is the burst load transient or
// sustained" — i.e. whether a backlog buffer would drain. Same unlocked
// cross-core read caveat as status_logger_get_last().
typedef struct {
    uint32_t windows;         // sampled 1 s windows since boot
    uint32_t peak_bursts;     // max tagger bursts dispatched in any window
    float    mean_bursts;     // mean tagger bursts/window
    float    peak_worker_cap; // max Core-1 worker capacity %
    float    peak_dsp_cap;    // max Core-0 DSP/tagger capacity %
    float    worker_ge90_pct; // % of windows with worker cap >= 90 (headroom gauge)
    uint32_t peak_processed;      // max accepted+serviced bursts/window (demod throughput)
    uint32_t peak_queue_drops;    // max bursts dropped at the SNR queue/window (backlog salvage target)
    float    prefilter_accept_pct;// % of pre-filtered bursts that passed (drops real vs junk)
} status_capacity_t;
void status_logger_get_capacity(status_capacity_t *out);

#endif
