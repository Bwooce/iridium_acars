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

static const char *TAG = "INGEST";

// Per-slot state. Two slots alternated per consumer cycle.
static uint8_t *s_raw[INGEST_NUM_SLOTS];     // raw uint8 USB ingress, internal SRAM, DMA-aligned
static int16_t *s_conv[INGEST_NUM_SLOTS];    // converted int16 Q15 @ 2.56 MSPS (scratch)
static int16_t *s_resamp[INGEST_NUM_SLOTS];  // resampled int16 Q15 @ 2.5 MSPS (signal_buffer feed)
static size_t   s_resamp_n_int16[INGEST_NUM_SLOTS];

// Split-resample architecture (within-chunk parallelism, see
// optimization-opportunities-2026-05-22.md).
//
// One coordinator task on Core 1 owns the dispatch handshake +
// USB-uint8→int16 convert + signal_buffer_push. The polyphase
// resample is split across TWO unpinned worker tasks: Worker A
// processes the first half of input samples, Worker B the second
// half. Each writes to its own slice of the output buffer.
//
// The resampler's only stateful pieces are:
//   - delay_i[16], delay_q[16]: 16 int16 each (9 live, 7 zero-pad)
//   - start_pos:                int32 phase counter
//
// Worker B's initial state at the chunk midpoint is computed without
// a serial pre-pass:
//   - delay_i/q[0..8] = input[mid-1 .. mid-9] (newest at [0])
//   - start_pos       = start_pos_init + 128*n_emits_A - 125*midpoint
//     where n_emits_A = floor((125*midpoint + 124 - start_pos_init)/128)
//     (closed-form output count for the 125/128 polyphase walk)
//
// Worker A's final state is the persistent resampler state to carry
// over to the next chunk — Worker B's final state is irrelevant
// (B inherits from A's pre-pass position; the next chunk starts
// from A's final, which is the chunk's true end state).
//
// Wait, that's wrong — for the NEXT chunk, both A and B will start
// from the previous chunk's true END state, which is what Worker B
// finishes the current chunk at. So we keep Worker B's final state
// (= state at end-of-chunk), and the next chunk's Worker A inherits
// from there.

// Persistent (cross-chunk) resampler state. Belongs to the
// coordinator — workers read it at dispatch time and the coordinator
// updates it from Worker B's final state after the join.
static int16_t s_persist_delay_i[16] __attribute__((aligned(16)));
static int16_t s_persist_delay_q[16] __attribute__((aligned(16)));
static int     s_persist_start_pos = 0;

// Per-worker scratch. Each worker owns its delay line + phase
// counter for the duration of one dispatch. After the join, Worker
// B's final state is copied into s_persist_* for the next chunk.
typedef struct {
    int16_t        delay_i[16] __attribute__((aligned(16)));
    int16_t        delay_q[16] __attribute__((aligned(16)));
    int            start_pos;
    const int16_t *in_iq;        // input slice base (interleaved IQ)
    int            n_in_complex; // input samples to process
    int16_t       *out_iq;       // output slice base
    int            max_out;      // upper bound on outputs to emit
    int            n_out;        // outputs actually written (set by worker)
    TaskHandle_t   task;
    TaskHandle_t   coord_task;   // who to notify on completion
} resample_worker_t;

static resample_worker_t s_worker_a;
static resample_worker_t s_worker_b;

// Semaphores per slot.
static SemaphoreHandle_t s_ready[INGEST_NUM_SLOTS];
static SemaphoreHandle_t s_free[INGEST_NUM_SLOTS];

typedef struct {
    int    slot;
    size_t bytes;
} dispatch_msg_t;
static QueueHandle_t s_dispatch;

static int s_next_acquire_slot = 0;

// Diagnostic accumulators.
static volatile uint64_t s_acc_convert_us = 0;
static volatile uint64_t s_acc_push_us = 0;
static volatile uint64_t s_acc_resample_us = 0;
static volatile uint64_t s_acc_sbpush_us = 0;
static volatile uint32_t s_acc_dispatches = 0;
static volatile uint32_t s_acc_slot_wait_us = 0;
static volatile uint32_t s_acc_consumer_waits = 0;

// 125/128 polyphase: number of outputs emitted by processing `k`
// inputs starting from phase counter `S0`.
//
// Derivation: at each iter, `start_pos` either advances by +3 (emit
// branch: +128 then -125) or by -125 (no-emit branch). Modulo 128
// these are identical (both are +3 mod 128), so the sequence
// pre_i = (S0 + 3*i) mod 128 is deterministic. The no_emit branch
// fires iff pre_i ∈ {125, 126, 127} — that's the only range in
// [0, 127] where pre >= 125. Across any 128 consecutive iters
// (since gcd(3, 128) = 1 → 3i mod 128 visits each residue once),
// exactly 3 land in {125, 126, 127} and 125 emit. The remainder (r
// iters after m full cycles of 128) is counted scalar — r ≤ 127, so
// the inner loop is bounded and fast (< 1 µs total).
static inline int rs_n_emits(int S0, int k)
{
    int m = k / 128;
    int r = k - m * 128;
    int n_emits = 125 * m;
    for (int i = 0; i < r; i++) {
        int pre = (S0 + 3 * i) & 127;     // mod 128 via mask (128 = 2^7)
        if (pre < 125) n_emits++;
    }
    return n_emits;
}

// Worker task: blocks on TaskNotifyTake, runs its slice, notifies
// coordinator back. Unpinned so FreeRTOS schedules onto whichever
// core has cycles.
static void resample_worker_task(void *arg)
{
    resample_worker_t *w = (resample_worker_t *)arg;
    ESP_LOGI(TAG, "resample_worker started on Core %d", xPortGetCoreID());
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        w->n_out = resample_256_to_250_process_explicit(
            w->delay_i, w->delay_q, &w->start_pos,
            w->in_iq, w->n_in_complex,
            w->out_iq, w->max_out);
        xTaskNotifyGive(w->coord_task);
    }
}

static void ingest_task(void *arg)
{
    ESP_LOGI(TAG, "Ingest coordinator started on Core %d", xPortGetCoreID());
    TaskHandle_t self = xTaskGetCurrentTaskHandle();
    s_worker_a.coord_task = self;
    s_worker_b.coord_task = self;

    dispatch_msg_t msg;
    while (1) {
        if (!xQueueReceive(s_dispatch, &msg, portMAX_DELAY)) continue;

        // 1. Convert raw uint8 → int16 Q15.
        int64_t t0 = esp_timer_get_time();
        const uint8_t *__restrict src = s_raw[msg.slot];
        int16_t *__restrict dst = s_conv[msg.slot];
        size_t n = msg.bytes;
        size_t n4 = n & ~(size_t)3;
        size_t i = 0;
        for (; i < n4; i += 4) {
            uint32_t b4 = *(const uint32_t *)(src + i);
            dst[i + 0] = (int16_t)((((b4 >>  0) & 0xff) << 8) ^ 0x8000);
            dst[i + 1] = (int16_t)((((b4 >>  8) & 0xff) << 8) ^ 0x8000);
            dst[i + 2] = (int16_t)((((b4 >> 16) & 0xff) << 8) ^ 0x8000);
            dst[i + 3] = (int16_t)((((b4 >> 24) & 0xff) << 8) ^ 0x8000);
        }
        for (; i < n; i++) {
            dst[i] = (int16_t)(((src[i] << 8) ^ 0x8000));
        }
        int64_t t1 = esp_timer_get_time();
        s_acc_convert_us += (uint64_t)(t1 - t0);

        // 2. Split-resample: dispatch two workers in parallel.
        int n_in_complex = (int)(n / 2);
        int mid = n_in_complex / 2;          // input split point

        int16_t *out = s_resamp[msg.slot];

        // Worker A: input[0..mid-1] → out[0..n_emits_a-1]
        memcpy(s_worker_a.delay_i, s_persist_delay_i, sizeof(s_persist_delay_i));
        memcpy(s_worker_a.delay_q, s_persist_delay_q, sizeof(s_persist_delay_q));
        s_worker_a.start_pos    = s_persist_start_pos;
        s_worker_a.in_iq        = dst;
        s_worker_a.n_in_complex = mid;
        s_worker_a.out_iq       = out;
        s_worker_a.max_out      = mid;       // upper bound (always >= 125*mid/128)

        // Worker B: input[mid..end-1] → out[n_emits_a..n_total-1]
        // Compute B's initial state from the closed-form polyphase walk.
        int n_emits_a = rs_n_emits(s_persist_start_pos, mid);
        // delay_b[k] = newest-to-oldest input I/Q at positions
        // mid-1, mid-2, ..., mid-9. Newest at [0].
        for (int k = 0; k < 9; k++) {
            int src_idx = mid - 1 - k;       // mid >= 9 in practice (mid≈4K)
            s_worker_b.delay_i[k] = (src_idx >= 0) ? dst[2 * src_idx + 0] : 0;
            s_worker_b.delay_q[k] = (src_idx >= 0) ? dst[2 * src_idx + 1] : 0;
        }
        for (int k = 9; k < 16; k++) {
            s_worker_b.delay_i[k] = 0;
            s_worker_b.delay_q[k] = 0;
        }
        s_worker_b.start_pos    = s_persist_start_pos + 128 * n_emits_a - 125 * mid;
        s_worker_b.in_iq        = dst + 2 * mid;
        s_worker_b.n_in_complex = n_in_complex - mid;
        s_worker_b.out_iq       = out + 2 * n_emits_a;
        s_worker_b.max_out      = (n_in_complex - mid);

        // Kick both workers. They run concurrently — FreeRTOS will
        // schedule them on whatever cores are free.
        xTaskNotifyGive(s_worker_a.task);
        xTaskNotifyGive(s_worker_b.task);

        // Wait for both to complete (one take per worker).
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int n_out_complex = s_worker_a.n_out + s_worker_b.n_out;
        int n_out_int16   = n_out_complex * 2;
        s_resamp_n_int16[msg.slot] = (size_t)n_out_int16;

        // Persist Worker B's end state as next chunk's starting state.
        memcpy(s_persist_delay_i, s_worker_b.delay_i, sizeof(s_persist_delay_i));
        memcpy(s_persist_delay_q, s_worker_b.delay_q, sizeof(s_persist_delay_q));
        s_persist_start_pos = s_worker_b.start_pos;

        int64_t t1b = esp_timer_get_time();
        s_acc_resample_us += (uint64_t)(t1b - t1);

        // 3. Push resampled samples into the PSRAM circular buffer.
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
    for (int i = 0; i < INGEST_NUM_SLOTS; i++) {
        s_raw[i] = heap_caps_aligned_alloc(64, 16 * 1024,
                                           MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        s_conv[i] = heap_caps_aligned_alloc(64, INGEST_SLOT_ELEMS * sizeof(int16_t),
                                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        s_resamp[i] = heap_caps_aligned_alloc(64, INGEST_SLOT_ELEMS * sizeof(int16_t),
                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        if (!s_raw[i] || !s_conv[i] || !s_resamp[i]) {
            ESP_LOGE(TAG, "Slot %d alloc failed (raw=%p conv=%p resamp=%p)",
                     i, s_raw[i], s_conv[i], s_resamp[i]);
            return ESP_ERR_NO_MEM;
        }
        s_resamp_n_int16[i] = 0;

        s_ready[i] = xSemaphoreCreateBinary();
        s_free[i]  = xSemaphoreCreateBinary();
        if (!s_ready[i] || !s_free[i]) return ESP_ERR_NO_MEM;
        xSemaphoreGive(s_free[i]);
    }

    // Persistent resampler state: zero delay lines, phase 0. Also
    // calls resample_256_to_250_init to populate the s_coeffs_pp
    // table (singleton, idempotent across multiple init calls).
    memset(s_persist_delay_i, 0, sizeof(s_persist_delay_i));
    memset(s_persist_delay_q, 0, sizeof(s_persist_delay_q));
    s_persist_start_pos = 0;
    {
        resample_256_to_250_t tmp;
        resample_256_to_250_init(&tmp);   // populates s_coeffs_pp
    }

    s_dispatch = xQueueCreate(4, sizeof(dispatch_msg_t));
    if (!s_dispatch) return ESP_ERR_NO_MEM;

    s_next_acquire_slot = 0;

    // Spawn the coordinator on Core 1 (matches the previous topology
    // so USB/Core-0 latency profile is unchanged). Workers are
    // UNPINNED — FreeRTOS schedules them onto whichever core has
    // cycles, so the parallelism actually materialises.
    BaseType_t ok = xTaskCreatePinnedToCore(ingest_task, "ingest_coord",
                                            8192, NULL, 8, NULL, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "ingest_coord task spawn failed");
        return ESP_FAIL;
    }
    // Worker priority one notch below the coordinator (7 vs 8) so
    // the coordinator can always preempt them for the next dispatch.
    // tskNO_AFFINITY = unpinned.
    ok = xTaskCreatePinnedToCore(resample_worker_task, "rs_worker_a",
                                  4096, &s_worker_a, 7, &s_worker_a.task,
                                  tskNO_AFFINITY);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "rs_worker_a spawn failed");
        return ESP_FAIL;
    }
    ok = xTaskCreatePinnedToCore(resample_worker_task, "rs_worker_b",
                                  4096, &s_worker_b, 7, &s_worker_b.task,
                                  tskNO_AFFINITY);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "rs_worker_b spawn failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Ingest pipeline initialised (coord + 2 unpinned workers, 2 slots × 32 KB)");
    return ESP_OK;
}

uint8_t *ingest_core1_acquire_raw(int *out_slot)
{
    int slot = s_next_acquire_slot;
    int64_t t_wait = esp_timer_get_time();
    xSemaphoreTake(s_free[slot], portMAX_DELAY);
    uint32_t waited = (uint32_t)(esp_timer_get_time() - t_wait);
    if (waited > 100) {
        s_acc_slot_wait_us += waited;
        s_acc_consumer_waits++;
    }
    *out_slot = slot;
    s_next_acquire_slot = (slot + 1) % INGEST_NUM_SLOTS;
    return s_raw[slot];
}

void ingest_core1_dispatch(int slot, size_t bytes_filled)
{
    dispatch_msg_t msg = { .slot = slot, .bytes = bytes_filled };
    xQueueSend(s_dispatch, &msg, 0);
}

int16_t *ingest_core1_take_converted(int slot, size_t *out_n_int16)
{
    xSemaphoreTake(s_ready[slot], portMAX_DELAY);
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
    s_acc_convert_us = 0;
    s_acc_push_us = 0;
    s_acc_resample_us = 0;
    s_acc_sbpush_us = 0;
    s_acc_dispatches = 0;
    s_acc_slot_wait_us = 0;
    s_acc_consumer_waits = 0;
}
