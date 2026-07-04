#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "ingest_core1.h"
#include "signal_buffer.h"
#include "resample_256_to_250.h"
#include "fault_inject.h"

static const char *TAG = "INGEST";

// Per-slot state. Two slots alternated per consumer cycle.
// T49a: raw USB bytes no longer have a slot-owned buffer -- they live
// in the usbring PSRAM ring (esp_libusb.c/usbring.c) and are handed to
// ingest_task as a (ptr, bytes) pair per dispatch. See dispatch_msg_t.
static int16_t *s_conv[INGEST_NUM_SLOTS];           // PSRAM heap-alloc'd
static int16_t *s_resamp[INGEST_NUM_SLOTS];         // resampled int16 Q15 @ 2.5 MSPS (downstream feed, internal SRAM)
static size_t   s_resamp_n_int16[INGEST_NUM_SLOTS]; // int16 element count in s_resamp

// 125/128 polyphase rational resampler state. Caller-managed —
// process_explicit() reads/updates these in place each dispatch.
// Separate file-scope statics instead of a single struct so the
// resampler API doesn't tie us to the legacy resample_256_to_250_t
// wrapper.
//
// delay buffers are 32 int16 — slots [0..15] are the live ring,
// [16..31] mirror the same values to make the 9-tap MAC window
// readable contiguously at any wpos. See resample_256_to_250.c for
// the layout rationale (task #58 double-mirror).
static int16_t s_persist_delay_i[32] __attribute__((aligned(16)));
static int16_t s_persist_delay_q[32] __attribute__((aligned(16)));
static int     s_persist_start_pos = 0;
static int     s_persist_wpos      = 0;

// Fraction (0–50%) of each dispatch's input handed to Worker A on
// Core 0. 0 = single-thread inline path on Core 1 (default).
// Tunable at runtime via ingest_core1_set_split_pct().
static volatile uint8_t s_split_pct = 0; /* default OFF — Worker A on Core 0 still slows FFT even with pipelined tagger (sweep 2026-05-23). See docs/split-resample-sweep-2026-05-22.md. Re-confirmed 2026-05-24 after task #58 wpos change: FFT 258→412 µs (+60%) at split=25; lighter wrapper doesn't change the L2 contention. */

// 125/128 polyphase: number of outputs emitted by processing `k`
// inputs starting from phase counter `S0`. Derivation: pre_(i+1) =
// (pre_i + 3) mod 128 always (emit branch +128-125 = +3; no-emit
// branch -125 ≡ +3 mod 128). No-emit fires iff pre_i ∈ {125,126,127}.
// Across 128 consecutive iters, exactly 3 are no-emit, 125 emit. The
// remainder r ≤ 127 is counted scalar (under 1 µs).
static inline int rs_n_emits(int S0, int k)
{
    int m       = k / 128;
    int r       = k - m * 128;
    int n_emits = 125 * m;
    for (int i = 0; i < r; i++) {
        int pre = (S0 + 3 * i) & 127;
        if (pre < 125) n_emits++;
    }
    return n_emits;
}

// Resample worker scratch (used by the parallel split path; unused at
// split=0 but the tasks exist at boot so we can ramp up dynamically).
//
// delay buffers carry the double-mirror layout — see s_persist_delay_*
// comment above.
typedef struct {
    int16_t        delay_i[32] __attribute__((aligned(16)));
    int16_t        delay_q[32] __attribute__((aligned(16)));
    int            start_pos;
    int            wpos;
    const int16_t *in_iq;
    int            n_in_complex;
    int16_t       *out_iq;
    int            max_out;
    int            n_out;
    TaskHandle_t   task;
    TaskHandle_t   coord_task;
} resample_worker_t;
static resample_worker_t s_worker_a;
static resample_worker_t s_worker_b;

// L2-CONTENTION ISOLATION TEST (2026-05-22): redirect Worker A's
// output writes to a static internal-SRAM scratch instead of the
// PSRAM out_iq. If FFT cost on Core 0 stays flat at split>0 with
// this enabled, s_resamp PSRAM writes were the L2 evictor and the
// fix is clear (move s_resamp to internal too, or partial). If FFT
// still slows down, the contention is elsewhere (PIE shared state,
// cache coherency between cores, etc.) and not easily addressable.
//
// Disabled by default. Decode is GARBAGE with this on — we only
// care about the FFT timing data, not the decode result.
// Set to 1 to redirect Worker A's output writes from PSRAM s_resamp
// to an internal-SRAM scratch — used to isolate whether the FFT
// slowdown on Core 0 at split>0 was caused by Worker A's PSRAM
// output writes evicting tagger L2 cache lines. Result (2026-05-22):
// FFT cost UNCHANGED at split=25 even with this fix (536 µs vs
// 443 µs with PSRAM writes — slightly worse). So PSRAM writes are
// NOT the evictor; the contention is something more fundamental
// (Worker A's PSRAM reads of s_conv input, shared L2 churn between
// cores writing to different regions, or PIE-internal state).
// Disabled. Documented for posterity.
#define WORKER_OUT_TO_INTERNAL_SCRATCH 0

#if WORKER_OUT_TO_INTERNAL_SCRATCH
// Heap-alloc'd at init in MALLOC_CAP_INTERNAL — sized for half a
// slot (8 KB int16) which is more than Worker A produces at any
// split ratio. Static .bss broke the DMA pool reserve at 32 KB so
// we runtime-alloc instead.
#define WORKER_OUT_SCRATCH_INT16 4096
static int16_t *s_worker_out_scratch = NULL;
#endif

static void resample_worker_task(void *arg)
{
    resample_worker_t *w = (resample_worker_t *)arg;
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#if WORKER_OUT_TO_INTERNAL_SCRATCH
        int16_t *out_target     = s_worker_out_scratch;
        int      max_out_target = WORKER_OUT_SCRATCH_INT16 / 2;
#else
        int16_t *out_target     = w->out_iq;
        int      max_out_target = w->max_out;
#endif
        // Workers run on PSRAM-backed stacks, so a stack-local
        // batch scratch would live in PSRAM and the memcpy from
        // PSRAM→PSRAM would be net negative vs direct writes.
        // Pass NULL to keep direct PSRAM writes for the worker path.
        w->n_out = resample_256_to_250_process_explicit(
            w->delay_i, w->delay_q, &w->wpos, &w->start_pos,
            w->in_iq, w->n_in_complex,
            out_target, max_out_target,
            /*batch_scratch=*/NULL);
        // Ensure output buffer writes are globally visible before
        // the coord (possibly on another core) reads them / kicks
        // signal_buffer_push (AXI-GDMA reading from PSRAM).
        __sync_synchronize();
        xTaskNotifyGive(w->coord_task);
    }
}

// Semaphores per slot. ready[i]: given by ingest task when conversion +
// signal_buffer_push for slot i are complete; taken by class_driver before
// running DSP feed. free[i]: given by class_driver after DSP feed; taken
// by ingest task before reusing the slot.
static SemaphoreHandle_t s_ready[INGEST_NUM_SLOTS];
static SemaphoreHandle_t s_free[INGEST_NUM_SLOTS];
// raw_done[i]: given by ingest task right after the convert step reads
// the LAST byte of this dispatch's ring region (before resample/push).
// Taken by class_driver (ingest_core1_wait_raw_done) before it
// usbring_consume()s that region and peeks the ring for the next
// dispatch. New for T49a — see ingest_core1.h's doc comment. Does not
// change the existing s_free/s_ready protocol's shape; it is an
// additional, earlier signal for a different resource (the ring
// region) than s_ready (which covers convert+resample+push).
static SemaphoreHandle_t s_raw_done[INGEST_NUM_SLOTS];

// Dispatch queue: class_driver posts (slot, ptr, bytes) tuples; ingest
// task receives them. `ptr` points into the usbring PSRAM ring (T49a);
// it is only valid until ingest_task's convert step finishes reading it
// (see s_raw_done above). Length 4 — enough for the 2 slots in flight
// plus a small runway, but small enough that a stuck ingest blocks
// dispatch quickly.
typedef struct {
    int            slot;
    const uint8_t *ptr;
    size_t         bytes;
} dispatch_msg_t;
static QueueHandle_t s_dispatch;

// Consumer-side slot acquisition state. The class_driver alternates
// between slots; we track the next slot to allocate so we can wait on
// s_free[i] before handing the slot back.
static int s_next_acquire_slot = 0;

// Diagnostic accumulators (reset by ingest_core1_get_stats).
static volatile uint64_t s_acc_convert_us = 0;

// D16 AGC: peak deviation of input uint8 from mid-point (127). The
// AGC task reads + resets this every ~1 s to decide if the tuner is
// saturating. peak_dev=127 means a sample was at 0 or 255 (full
// saturation). peak_dev > ~100 generally means the gain is set
// too high for the current signal.
static volatile uint8_t  s_agc_peak_dev   = 0;
static volatile uint32_t s_agc_dispatches = 0;

void ingest_core1_agc_sample(uint8_t *out_peak_dev, uint32_t *out_dispatches)
{
    if (out_peak_dev) *out_peak_dev = s_agc_peak_dev;
    if (out_dispatches) *out_dispatches = s_agc_dispatches;
    s_agc_peak_dev   = 0;
    s_agc_dispatches = 0;
}
static volatile uint64_t s_acc_push_us        = 0; // == resample + signal_buffer_push total
static volatile uint64_t s_acc_resample_us    = 0; // resample step only
static volatile uint64_t s_acc_sbpush_us      = 0; // signal_buffer_push (AXI DMA wait) only
static volatile uint32_t s_acc_dispatches     = 0;
static volatile uint32_t s_acc_slot_wait_us   = 0;
static volatile uint32_t s_acc_consumer_waits = 0;

static void ingest_task(void *arg)
{
    ESP_LOGI(TAG, "Ingest task started on Core %d", xPortGetCoreID());
    dispatch_msg_t msg;

    while (1) {
        if (!xQueueReceive(s_dispatch, &msg, portMAX_DELAY)) continue;

        // Slot ownership protocol:
        //   - class_driver took s_free[slot] in acquire_slot → it owns
        //     the slot exclusively until it gives s_free back
        //   - class_driver dispatches a ring region (ptr, bytes) for
        //     this slot
        //   - ingest task (here) converts ptr -> conv, gives raw_done
        //     (the ring region is no longer read after this point),
        //     then resamples + pushes, and signals s_ready
        //   - class_driver takes s_ready, runs DSP on converted, gives
        //     s_free back
        // We do NOT take s_free here — that would deadlock since
        // class_driver already holds it.

        // 1. Convert raw uint8 -> int16 Q15.
        //   out[i] = (int16_t)((b[i] << 8) ^ 0x8000)
        // Same formula as the in-class_driver path before this offload.
        // 4x unrolled, 32-bit-at-a-time. `src` used to be s_raw[slot], a
        // dedicated 64-byte-aligned buffer; it is now a pointer straight
        // into the usbring PSRAM ring (T49a), whose physical offset can
        // land on ANY byte alignment (e.g. after a short USB transfer
        // leaves an odd byte count at the ring's wrap point) -- so the
        // 4-byte group is read via memcpy into a local, not a `uint32_t*`
        // reinterpret-cast, to avoid an unaligned-access fault. `dst` is
        // still the dedicated 64-byte-aligned s_conv[slot] PSRAM buffer.
        int64_t t0                    = esp_timer_get_time();
        const uint8_t *__restrict src = msg.ptr;
        int16_t *__restrict dst       = s_conv[msg.slot];
        size_t n                      = msg.bytes;

        // AGC sampling (D16): scan the first 256 bytes for the
        // maximum deviation from the mid-point (uint8 127). One
        // pass through a tiny prefix, cheap, gives a peak signal
        // estimate for the AGC task to decide whether the tuner
        // is saturating. Doesn't change the convert math — just
        // observes.
        size_t  agc_n   = n < 256 ? n : 256;
        uint8_t agc_max = 0;
        for (size_t k = 0; k < agc_n; k++) {
            int d = (int)src[k] - 127;
            if (d < 0) d = -d;
            if (d > (int)agc_max) agc_max = (uint8_t)d;
        }
        if (agc_max > s_agc_peak_dev) {
            s_agc_peak_dev = agc_max;
        }
        s_agc_dispatches++;

        size_t n4 = n & ~(size_t)3;
        size_t i  = 0;
        for (; i < n4; i += 4) {
            uint32_t b4;
            memcpy(&b4, src + i, sizeof(b4)); // safe unaligned load, see comment above
            dst[i + 0] = (int16_t)((((b4 >> 0) & 0xff) << 8) ^ 0x8000);
            dst[i + 1] = (int16_t)((((b4 >> 8) & 0xff) << 8) ^ 0x8000);
            dst[i + 2] = (int16_t)((((b4 >> 16) & 0xff) << 8) ^ 0x8000);
            dst[i + 3] = (int16_t)((((b4 >> 24) & 0xff) << 8) ^ 0x8000);
        }
        for (; i < n; i++) {
            dst[i] = (int16_t)(((src[i] << 8) ^ 0x8000));
        }
        int64_t t1 = esp_timer_get_time();
        s_acc_convert_us += (uint64_t)(t1 - t0);

        // The convert loop above is the LAST read of msg.ptr (the ring
        // region). Signal raw_done now so class_driver can
        // usbring_consume() it and peek the ring for the next dispatch
        // — this must happen before resample/push (which don't touch
        // the ring) so the ring reclaim isn't gated on the slower
        // resample+signal_buffer_push stage. See ingest_core1.h's
        // ingest_core1_wait_raw_done() doc comment.
        xSemaphoreGive(s_raw_done[msg.slot]);

        // 2. Resample 2.56 → 2.5 MSPS (125/128 polyphase). gri's
        // wideband tagger / direct_if_decim are tuned at 2.5 MSPS;
        // performing this once here keeps the entire downstream chain
        // (signal_buffer, dsp_processor, worker) at the gri-aligned
        // rate. Input is n/2 complex samples (interleaved IQ in
        // s_conv); output is ~n/2 × 125/128 complex into s_resamp.
        int n_in_complex = (int)(n / 2);
        int split_pct    = s_split_pct;
        int mid          = (n_in_complex * split_pct) / 100;
        if (mid < 9) mid = 0; // need ≥ 9 samples to seed Worker B's delay
        int16_t *out = s_resamp[msg.slot];
        int      n_out_complex;

        if (mid == 0) {
            // Inline single-thread path on Core 1 (split=0 default).
            // ingest_task's stack lives in internal SRAM (default
            // xTaskCreatePinnedToCore), so the stack-local batch
            // scratch lands in internal SRAM and the PIE MAC sh's
            // are fast — the function then flushes 32-byte bursts
            // into the PSRAM out buffer. See _process_explicit doc
            // for the cost rationale.
            int16_t batch_scratch[RS25_BATCH_COMPLEX * 2]
                __attribute__((aligned(16)));
            n_out_complex = resample_256_to_250_process_explicit(
                s_persist_delay_i, s_persist_delay_q,
                &s_persist_wpos, &s_persist_start_pos,
                dst, n_in_complex,
                out, n_in_complex,
                batch_scratch);
        } else {
            // Split path — Worker A on Core 0 takes the first `mid`
            // inputs; Worker B on Core 1 takes the rest. Both run
            // concurrently. Worker B's initial (delay, wpos,
            // start_pos) is computed from the closed-form polyphase
            // walk so it can start immediately, no serial pre-pass.
            memcpy(s_worker_a.delay_i, s_persist_delay_i, sizeof(s_persist_delay_i));
            memcpy(s_worker_a.delay_q, s_persist_delay_q, sizeof(s_persist_delay_q));
            s_worker_a.wpos         = s_persist_wpos;
            s_worker_a.start_pos    = s_persist_start_pos;
            s_worker_a.in_iq        = dst;
            s_worker_a.n_in_complex = mid;
            s_worker_a.out_iq       = out;
            s_worker_a.max_out      = mid;
            s_worker_a.coord_task   = xTaskGetCurrentTaskHandle();

            // Seed Worker B's delay buffer with the 9 newest input
            // samples (mid-1, mid-2, ..., mid-9) in newest-first
            // order at delay[0..8], with mirror copies at
            // delay[16..24] so the first MAC iteration sees them at
            // delay[wpos+1..wpos+8] after the wpos decrement.
            int n_emits_a = rs_n_emits(s_persist_start_pos, mid);
            for (int k = 0; k < 9; k++) {
                int     src_idx            = mid - 1 - k;
                int16_t i_s                = dst[2 * src_idx + 0];
                int16_t q_s                = dst[2 * src_idx + 1];
                s_worker_b.delay_i[k]      = i_s;
                s_worker_b.delay_i[k + 16] = i_s;
                s_worker_b.delay_q[k]      = q_s;
                s_worker_b.delay_q[k + 16] = q_s;
            }
            for (int k = 9; k < 16; k++) {
                s_worker_b.delay_i[k]      = 0;
                s_worker_b.delay_i[k + 16] = 0;
                s_worker_b.delay_q[k]      = 0;
                s_worker_b.delay_q[k + 16] = 0;
            }
            s_worker_b.wpos         = 0;
            s_worker_b.start_pos    = s_persist_start_pos + 128 * n_emits_a - 125 * mid;
            s_worker_b.in_iq        = dst + 2 * mid;
            s_worker_b.n_in_complex = n_in_complex - mid;
            s_worker_b.out_iq       = out + 2 * n_emits_a;
            s_worker_b.max_out      = n_in_complex - mid;
            s_worker_b.coord_task   = xTaskGetCurrentTaskHandle();

            xTaskNotifyGive(s_worker_a.task);
            xTaskNotifyGive(s_worker_b.task);
            // pdFALSE = decrement-on-exit (counting-semaphore semantics),
            // so two back-to-back gives don't collapse into one take.
            ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
            ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

            n_out_complex = s_worker_a.n_out + s_worker_b.n_out;

            // Persist Worker B's end state for the next chunk.
            memcpy(s_persist_delay_i, s_worker_b.delay_i, sizeof(s_persist_delay_i));
            memcpy(s_persist_delay_q, s_worker_b.delay_q, sizeof(s_persist_delay_q));
            s_persist_start_pos = s_worker_b.start_pos;
            s_persist_wpos      = s_worker_b.wpos;
        }
        int n_out_int16            = n_out_complex * 2;
        s_resamp_n_int16[msg.slot] = (size_t)n_out_int16;
        int64_t t1b                = esp_timer_get_time();
        s_acc_resample_us += (uint64_t)(t1b - t1);

        // 3. Push resampled samples into the PSRAM circular buffer.
        // signal_buffer_push is itself async (AXI-GDMA from Step 2) so the
        // CPU cost here is just descriptor programming + cache flush --
        // the visible wait inside signal_buffer_push is the semaphore
        // take for the PREVIOUS DMA to complete.
        signal_buffer_push(s_resamp[msg.slot], (size_t)n_out_complex);
        int64_t t2 = esp_timer_get_time();
        s_acc_sbpush_us += (uint64_t)(t2 - t1b);
        s_acc_push_us += (uint64_t)(t2 - t1);

        s_acc_dispatches++;
        xSemaphoreGive(s_ready[msg.slot]);
    }
}

esp_err_t ingest_core1_init(void)
{
    // Allocate ping-pong buffers.
    //
    // T49a: there is no more s_raw[] here. It used to be a 2 * 16 KB
    // DMA-capable internal-SRAM buffer the convert loop read from; the
    // raw USB bytes now live directly in the usbring PSRAM ring
    // (esp_libusb.c/usbring.c) and are handed to ingest_task as a
    // (ptr, bytes) pair per dispatch (see dispatch_msg_t / ingest_task's
    // convert step). Removing it frees ~32 KB of DMA-internal heap,
    // directly funding the USB transfer pool's pre-stream budget (see
    // esp_libusb.c's "Pre-stream DMA-internal heap: free=" log).
    //
    // s_conv[]: CPU-only path -- written by the uint8->int16 convert
    //   loop (above this fn), read by resample_256_to_250_process.
    //   No DMA engine touches it. Previously flagged
    //   MALLOC_CAP_INTERNAL|MALLOC_CAP_DMA "for safety" but that
    //   over-restriction starved the USB transfer pool of internal
    //   DMA SRAM (esp_libusb's bulk-IN allocations failed at
    //   start_stream with ESP_ERR_NO_MEM). Moved to PSRAM; freed
    //   64 KB DMA-internal (2 slots * 32 KB). Cost: CPU writes go
    //   to PSRAM @ ~100 MB/s vs ~700 MB/s internal SRAM -> ~160 us
    //   extra per 16 ms slot, ~1% real-time overhead.
    //
    // s_resamp[]: AXI-GDMA reads from here into signal_buffer (also
    //   PSRAM). AXI handles PSRAM sources fine. CPU touch is one
    //   linear write pass per cycle. Already in PSRAM.
    for (int i = 0; i < INGEST_NUM_SLOTS; i++) {
        ESP_LOGW("HEAP", "slot %d pre-alloc  INT free=%zu largest=%zu  DMA-INT free=%zu largest=%zu",
                 i,
                 heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                 heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
                 heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
        // s_conv lives in PSRAM. We explored moving it to internal
        // SRAM (L2-cache contention bypass for Worker A) and that
        // required reducing L2 cache from 256→128 KB to free heap
        // headroom — measured cost: -13% LIVE_SDR throughput (4.57
        // → 3.97 MB/s), +160% USB handle_events latency, +62%
        // sbpush. Net regression. Sticking with PSRAM s_conv.
        s_conv[i]   = heap_caps_aligned_alloc(64, INGEST_SLOT_ELEMS * sizeof(int16_t),
                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        s_resamp[i] = heap_caps_aligned_alloc(64, INGEST_SLOT_ELEMS * sizeof(int16_t),
                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        if (!s_conv[i] || !s_resamp[i]) {
            ESP_LOGE(TAG, "Slot %d alloc failed (conv=%p resamp=%p)",
                     i, s_conv[i], s_resamp[i]);
            goto slot_cleanup;
        }
        s_resamp_n_int16[i] = 0;

        s_ready[i]    = xSemaphoreCreateBinary();
        s_free[i]     = xSemaphoreCreateBinary();
        s_raw_done[i] = xSemaphoreCreateBinary();
        if (!s_ready[i] || !s_free[i] || !s_raw_done[i]) goto slot_cleanup;
        // Initially: free=1 (slot available), ready=0 (no data yet),
        // raw_done=0 (nothing dispatched yet).
        xSemaphoreGive(s_free[i]);
        continue;

    slot_cleanup:
        // Free everything allocated so far (this slot and earlier ones)
        // and null the pointers, so the caller sees a clean failure
        // instead of half-built slots with NULL semaphores. (Deeper
        // failure paths below — queue/task creation — don't unwind:
        // the caller treats any failure as fatal-for-streaming and the
        // system is headed for an operator/watchdog reboot anyway.)
        for (int j = 0; j <= i; j++) {
            free(s_conv[j]);
            free(s_resamp[j]);
            s_conv[j]   = NULL;
            s_resamp[j] = NULL;
            if (s_ready[j]) vSemaphoreDelete(s_ready[j]);
            if (s_free[j]) vSemaphoreDelete(s_free[j]);
            if (s_raw_done[j]) vSemaphoreDelete(s_raw_done[j]);
            s_ready[j]    = NULL;
            s_free[j]     = NULL;
            s_raw_done[j] = NULL;
        }
        return ESP_ERR_NO_MEM;
    }

    // Initialise the 125/128 resampler coeffs table (singleton
    // s_coeffs_pp inside resample_256_to_250.c). Heap-alloc'd
    // scratch struct since stack-allocating would near-overflow
    // main_task's 3584-byte stack (struct is ~2.5 KB).
    memset(s_persist_delay_i, 0, sizeof(s_persist_delay_i));
    memset(s_persist_delay_q, 0, sizeof(s_persist_delay_q));
    s_persist_start_pos = 0;
    s_persist_wpos      = 0;
#if WORKER_OUT_TO_INTERNAL_SCRATCH
    s_worker_out_scratch = heap_caps_aligned_alloc(64,
                                                   WORKER_OUT_SCRATCH_INT16 * sizeof(int16_t), MALLOC_CAP_INTERNAL);
    if (!s_worker_out_scratch) {
        ESP_LOGE(TAG, "iso: worker_out_scratch alloc failed");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGW("ISO", "worker_out_scratch at %p (8 KB INTERNAL)",
             s_worker_out_scratch);
#endif
    {
        resample_256_to_250_t *tmp = (resample_256_to_250_t *)
            heap_caps_calloc(1, sizeof(*tmp),
                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!tmp) {
            ESP_LOGE(TAG, "init: resample tmp alloc failed");
            return ESP_ERR_NO_MEM;
        }
        resample_256_to_250_init(tmp);
        free(tmp);
    }

    s_dispatch = xQueueCreate(4, sizeof(dispatch_msg_t));
    if (!s_dispatch) return ESP_ERR_NO_MEM;

    s_next_acquire_slot = 0;

    // Spawn at higher priority than worker_core1 (5) so a busy worker
    // doesn't stall USB ingest. Watch for inversion if both cores hit
    // PSRAM hard.
    BaseType_t ok = xTaskCreatePinnedToCore(ingest_task, "ingest_core1",
                                            8192, NULL, 8, NULL, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreatePinnedToCore(ingest_core1) failed");
        return ESP_FAIL;
    }

    // Diagnostic: spawn 2 trivial DUMMY tasks (not resample workers).
    // If decode regresses, the bug is purely about adding tasks to
    // the system. If decode preserves, the bug is specifically in
    // the resample_worker_task code.
    extern void dummy_idle_task(void *arg);
    // Spawn the two resample workers with PSRAM-allocated stacks.
    //
    // ROOT CAUSE: default xTaskCreate puts stacks in MALLOC_CAP_INTERNAL
    // (internal SRAM). Adding 8 KB of internal-SRAM stacks fragments
    // the 167 KB main internal-SRAM pool such that a later allocation
    // (somewhere in dsp_processor_init / fft_burst_tagger) can't get
    // the contiguous block it needs. Result: tagger misses bursts,
    // decode collapses (matched 61 → 44, recall 93.8 → 67.7%). Empirically
    // bisected to here: 2× 4 KB internal stacks broke decode; the same
    // 2× 4 KB stacks in PSRAM preserve it (heap_caps_get_free_size
    // MALLOC_CAP_INTERNAL goes from ~107 KB to ~115 KB after this swap).
    //
    // Resample worker hot path runs ~125 dispatches/s × 2.5 ms = ~30%
    // duty cycle. Stack accesses dominated by leaf calls (process_explicit
    // local vars), so PSRAM-backed stack adds maybe 50-100 µs per
    // dispatch through L2 cache misses — negligible vs the 2.5 ms
    // resample cost.
    ok = xTaskCreatePinnedToCoreWithCaps(resample_worker_task, "rs_worker_a",
                                         4096, &s_worker_a, 7, &s_worker_a.task,
                                         /*core=*/0,
                                         MALLOC_CAP_SPIRAM);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "rs_worker_a spawn failed");
        return ESP_FAIL;
    }
    ok = xTaskCreatePinnedToCoreWithCaps(resample_worker_task, "rs_worker_b",
                                         4096, &s_worker_b, 7, &s_worker_b.task,
                                         /*core=*/1,
                                         MALLOC_CAP_SPIRAM);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "rs_worker_b spawn failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Ingest pipeline initialised (coord + 2 PSRAM-stack workers, internal=%uKB)",
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
    return ESP_OK;
}

void ingest_core1_acquire_slot(int *out_slot)
{
    int slot = s_next_acquire_slot;
    // The slot must be free before the consumer starts a new dispatch
    // cycle for it. Wait unbounded — a previous bounded-timeout attempt
    // caused subtle data loss (when the timeout fired, the slot
    // wasn't released and the corresponding ingest output went
    // unconsumed; DSP throughput halved). The class TASK_WDT
    // concern is now mitigated upstream by the SDMMC stash buffer
    // (sd_log.c) preventing the SDMMC EIO cascade that originally
    // jammed worker → ingest → here.
    int64_t t_wait = esp_timer_get_time();
    xSemaphoreTake(s_free[slot], portMAX_DELAY);
    uint32_t waited = (uint32_t)(esp_timer_get_time() - t_wait);
    if (waited > 100) { // skip noise; only count meaningful waits
        s_acc_slot_wait_us += waited;
        s_acc_consumer_waits++;
    }
    *out_slot           = slot;
    s_next_acquire_slot = (slot + 1) % INGEST_NUM_SLOTS;
}

static volatile uint32_t s_dispatch_drops = 0;
uint32_t                 ingest_core1_dispatch_drops(void)
{
    return s_dispatch_drops;
}

// Count of take_converted iterations that exceeded one 500 ms tick
// without s_ready being given — i.e. ingest_task hasn't yet processed
// the dispatch corresponding to this slot. A handful per hour is FINE
// (just a slow ingest cycle); a sustained climb means ingest is
// wedged and health_wdt will reboot once class can't progress (#110).
static volatile uint32_t s_take_converted_slow_waits = 0;
uint32_t                 ingest_core1_take_converted_slow_waits(void)
{
    return s_take_converted_slow_waits;
}

void ingest_core1_dispatch(int slot, const uint8_t *ptr, size_t bytes_filled)
{
    dispatch_msg_t msg = {.slot = slot, .ptr = ptr, .bytes = bytes_filled};
    // Non-blocking send. The queue depth (4) exceeds the slot count (2),
    // so a successful acquire always implies space — this should never fail.
    // But if it ever did, ingest would never process this slot and never give
    // s_ready[slot], so the next ingest_core1_take_converted(slot) would hang
    // FOREVER → class deadlocks (same class as #106). Recover: mark the slot
    // zero-length and signal it ready, so take_converted returns immediately
    // (dsp_feed gets 0 samples) and the slot recycles instead of stranding.
    // ALSO give raw_done here (T49a) — ingest_task will never process this
    // message, so it will never give raw_done either; without this,
    // class_driver's ingest_core1_wait_raw_done(slot) would hang FOREVER
    // (same #106 class, new resource). The dropped ring region is still
    // reclaimed correctly: class_driver set raw_bytes[slot] and
    // prev_dsp_slot=slot unconditionally, so next cycle's CS_RAW_DONE
    // calls esp_libusb_consume_stream() for it once this synthetic
    // raw_done is taken — no ring-space leak. This branch should never
    // fire in practice anyway (queue depth > outstanding slots) and the
    // alternative is a deadlock.
    // #122: FI_SITE_DISPATCH_QUEUE short-circuits the send to exercise the
    // drop-recovery below without an actual full queue.
    if (fault_inject_should_fail(FI_SITE_DISPATCH_QUEUE) ||
        xQueueSend(s_dispatch, &msg, 0) != pdTRUE) {
        s_dispatch_drops++;
        ESP_LOGW(TAG, "dispatch queue full — slot %d dropped (recovered, no deadlock)", slot);
        s_resamp_n_int16[slot] = 0;
        xSemaphoreGive(s_raw_done[slot]);
        xSemaphoreGive(s_ready[slot]);
    }
}

// Count of 500 ms ticks ingest_core1_wait_raw_done waited without
// raw_done being given (T49a). See ingest_core1.h's doc comment.
static volatile uint32_t s_raw_done_slow_waits = 0;
uint32_t                 ingest_core1_raw_done_slow_waits(void)
{
    return s_raw_done_slow_waits;
}

void ingest_core1_wait_raw_done(int slot)
{
    // Same diagnostic-poll-loop shape as ingest_core1_take_converted()
    // below (#110-class visibility): the wait is unbounded in effect,
    // but logs if it's taking suspiciously long. ingest_task ALWAYS
    // gives raw_done unconditionally right after its convert step (or
    // via the dispatch-drop recovery above) — it never waits on
    // class_driver to do anything first — so this cannot deadlock
    // against anything class_driver holds.
    int waited_ms = 0;
    while (xSemaphoreTake(s_raw_done[slot], pdMS_TO_TICKS(500)) != pdTRUE) {
        waited_ms += 500;
        s_raw_done_slow_waits++;
        if (waited_ms == 500 || (waited_ms % 10000) == 0) {
            ESP_LOGW(TAG, "wait_raw_done slot=%d waited %dms — ingest "
                          "may be wedged before convert; health_wdt will "
                          "reboot if class stops progressing",
                     slot, waited_ms);
        }
    }
}

int16_t *ingest_core1_take_converted(int slot, size_t *out_n_int16)
{
    // Diagnostic poll loop (#110 — #106-class deadlock sibling). The wait
    // remains unbounded in effect (we keep retrying until s_ready is
    // given) — a bounded timeout with local recovery was tried for the
    // analogous s_free take at line 506 and caused subtle data loss
    // when the timeout fired without proper slot-ownership handoff.
    // External recovery: if ingest_task is truly wedged, take_converted
    // never returns, class can't advance usb.completed, and health_wdt's
    // stream-stall watchdog reboots in ~30-90 s (wifi_link.c).
    //
    // What this loop ADDS over a plain portMAX_DELAY: visibility. A
    // ~ms-scale wait is normal; anything beyond 500 ms is suspicious
    // and worth logging before the watchdog acts. We log the first
    // slow wait and then every ~10 s while still waiting.
    int waited_ms = 0;
    // #122: FI_SITE_TAKE_CONVERTED short-circuits the take (returns true,
    // decrements) to drive the slow-wait path N times before the real take.
    while (fault_inject_should_fail(FI_SITE_TAKE_CONVERTED) ||
           xSemaphoreTake(s_ready[slot], pdMS_TO_TICKS(500)) != pdTRUE) {
        waited_ms += 500;
        s_take_converted_slow_waits++;
        if (waited_ms == 500 || (waited_ms % 10000) == 0) {
            ESP_LOGW(TAG, "take_converted slot=%d waited %dms — ingest "
                          "may be wedged; health_wdt will reboot if class "
                          "stops progressing",
                     slot, waited_ms);
        }
    }
    if (out_n_int16) *out_n_int16 = s_resamp_n_int16[slot];
    return s_resamp[slot];
}

void ingest_core1_release(int slot)
{
    xSemaphoreGive(s_free[slot]);
}

void ingest_core1_get_stats(ingest_stats_t *out)
{
    out->convert_us_total   = s_acc_convert_us;
    out->push_us_total      = s_acc_push_us;
    out->resample_us_total  = s_acc_resample_us;
    out->sbpush_us_total    = s_acc_sbpush_us;
    out->dispatches         = s_acc_dispatches;
    out->slot_wait_total_us = s_acc_slot_wait_us;
    out->consumer_waits     = s_acc_consumer_waits;
    s_acc_convert_us        = 0;
    s_acc_push_us           = 0;
    s_acc_resample_us       = 0;
    s_acc_sbpush_us         = 0;
    s_acc_dispatches        = 0;
    s_acc_slot_wait_us      = 0;
    s_acc_consumer_waits    = 0;
}
