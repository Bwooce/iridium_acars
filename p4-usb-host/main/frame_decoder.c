// frame_decoder — consumer task for the worker -> higher-layer decode
// pipeline. See frame_decoder.h for the architecture rationale.
//
// Phase D (current): pop frames, classify with iridium_frame_classify,
// log type + LW subtype, count occurrences. SBD reassembly + libacars
// dispatch land in subsequent commits.

#include <stdio.h>
#include <stdatomic.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "frame_decoder.h"
#include "frame_queue.h"
#include "c6_forwarder.h"
#include "iridium_frame.h"
#include "ida_decode.h"
#include "ibc_decode.h"
#include "ira_decode.h"
#include "ims_decode.h"
#include "sbd_reassembler.h"
#include <libacars/libacars.h>
#include <libacars/acars.h>
#include <libacars/reassembly.h>
#include <sys/time.h>

static const char *TAG = "FRMDEC";

#define FRAME_QUEUE_SLOTS  64        // 64 × 432 B ≈ 27 KB in PSRAM
#define DECODER_STACK      6144
#define DECODER_PRIO       4         // < worker (5), < ingest (8), > logger (1)
#define DECODER_CORE       1

static frame_queue_t  *s_queue       = NULL;
static TaskHandle_t    s_task        = NULL;
static volatile bool   s_initialised = false;

static _Atomic uint64_t s_class_unknown  = 0;
static _Atomic uint64_t s_class_ms       = 0;
static _Atomic uint64_t s_class_tl       = 0;
static _Atomic uint64_t s_class_bc       = 0;
static _Atomic uint64_t s_class_lw_da    = 0;
static _Atomic uint64_t s_class_lw_other = 0;

// SBD reassembler instance — single global, not thread-safe (only the
// decoder task touches it). 8 sessions × ~330 B ≈ 2.6 KB in BSS.
static sbd_reassembler_t s_sbd;
static _Atomic uint64_t  s_sbd_complete = 0;     // SBD messages reassembled
static _Atomic uint64_t  s_acars_decoded = 0;    // ACARS messages successfully parsed
static _Atomic uint64_t  s_acars_fragments = 0;  // ACARS fragments awaiting reassembly

// D14: libacars reassembly context. Maintains per-flight-id session
// state so multi-block ACARS messages (block_id > 0, more_blocks_follow)
// arriving across multiple SBD messages get joined. Created once at
// decoder init.
static la_reasm_ctx *s_reasm_ctx = NULL;

// Walk a la_proto_node tree to find the la_acars_msg payload.
extern la_type_descriptor const la_DEF_acars_message;
static la_acars_msg *find_acars_msg(la_proto_node *node)
{
    while (node) {
        if (node->td == &la_DEF_acars_message && node->data) {
            return (la_acars_msg *)node->data;
        }
        node = node->next;
    }
    return NULL;
}

// Try to parse the reassembled SBD payload as ACARS. Logs the
// decoded fields if a recognisable ACARS frame is found.
static void try_acars(const sbd_message_t *msg)
{
    if (!msg || msg->payload_len < 8) return;
    la_msg_dir dir = msg->uplink ? LA_MSG_DIR_GND2AIR : LA_MSG_DIR_AIR2GND;
    // D14: use la_acars_parse_and_reassemble with our persistent
    // la_reasm_ctx so multi-block ACARS messages (block_id 1-5 with
    // more_blocks_follow set) accumulate across SBD packets. The
    // returned la_acars_msg has reasm_status set:
    //   LA_REASM_COMPLETE      → fully reassembled, log + emit
    //   LA_REASM_IN_PROGRESS   → fragment buffered, return silently
    //   LA_REASM_SKIPPED       → single-block (immediate complete)
    //   LA_REASM_DUPLICATE     → already-seen fragment, drop
    //   LA_REASM_FRAG_OUT_OF_SEQUENCE → unrecoverable, drop
    struct timeval rx_time = {
        .tv_sec  = (time_t)(msg->timestamp_us / 1000000ULL),
        .tv_usec = (suseconds_t)(msg->timestamp_us % 1000000ULL),
    };
    la_proto_node *node = la_acars_parse_and_reassemble(
        msg->payload, msg->payload_len, dir, s_reasm_ctx, rx_time);
    if (!node) return;
    la_acars_msg *a = find_acars_msg(node);
    if (a) {
        if (a->reasm_status == LA_REASM_COMPLETE ||
            a->reasm_status == LA_REASM_SKIPPED) {
            atomic_fetch_add_explicit(&s_acars_decoded, 1, memory_order_relaxed);
            ESP_LOGI(TAG, "ACARS: %s mode=%c label='%.2s' block=%c msgnum='%.4s' "
                     "flight='%.6s' crc=%s txt=\"%s\"",
                     msg->uplink ? "UL" : "DL",
                     a->mode ? a->mode : '?',
                     a->label, a->block_id ? a->block_id : '?',
                     a->msg_num, a->flight_id,
                     a->crc_ok ? "OK" : "BAD",
                     a->txt ? a->txt : "");

            // D17: forward to C6 companion. Non-blocking. Sequence
            // number tracks decoded message order.
            static uint32_t s_seq = 0;
            irp_acars_msg_t out = {0};
            out.seq = ++s_seq;
            out.ts_us = msg->timestamp_us;
            out.direction = msg->uplink ? 1 : 0;
            out.mode = (uint8_t)(a->mode ? a->mode : '?');
            memcpy(out.label, a->label, IRP_ACARS_LABEL_LEN);
            {
                size_t n = strnlen(a->flight_id, IRP_ACARS_FLIGHT_LEN);
                memcpy(out.flight, a->flight_id, n);
            }
            {
                size_t n = strnlen(a->msg_num, IRP_ACARS_MSGNUM_LEN);
                memcpy(out.msg_num, a->msg_num, n);
            }
            if (a->txt) {
                size_t n = strnlen(a->txt, IRP_ACARS_TEXT_LEN);
                memcpy(out.text, a->txt, n);
            }
            // SBD message doesn't track per-message SNR/freq;
            // these come from the burst metadata earlier in the
            // pipeline. Defer until burst→ACARS provenance is
            // threaded through (deferred to follow-up).
            out.snr_db = 0.0f;
            out.freq_hz = 0;
            (void)c6_forwarder_post_acars(&out);
        } else if (a->reasm_status == LA_REASM_IN_PROGRESS) {
            atomic_fetch_add_explicit(&s_acars_fragments, 1, memory_order_relaxed);
            ESP_LOGD(TAG, "ACARS fragment buffered: label='%.2s' block=%c "
                     "msgnum='%.4s' flight='%.6s'",
                     a->label, a->block_id ? a->block_id : '?',
                     a->msg_num, a->flight_id);
        }
        // DUPLICATE / OUT_OF_SEQUENCE / ARGS_INVALID: silent drop.
    }
    la_proto_tree_destroy(node);
}

static void process_one(const frame_queue_item_t *it)
{
    iridium_frame_t classified = { 0 };
    ir_frame_direction_t dir = (it->direction == DIR_DOWNLINK)
                               ? IR_FRM_DIR_DOWNLINK
                               : IR_FRM_DIR_UPLINK;
    int rc = iridium_frame_classify(it->bits, it->n_bits, dir, &classified);
    if (rc != 0) {
        ESP_LOGW(TAG, "classify rc=%d (n_bits=%u dir=%u)",
                 rc, it->n_bits, it->direction);
        atomic_fetch_add_explicit(&s_class_unknown, 1, memory_order_relaxed);
        return;
    }

    switch (classified.type) {
    case IR_FRAME_MS: {
        atomic_fetch_add_explicit(&s_class_ms, 1, memory_order_relaxed);
        // D15: extract MS header (block, frame, group, length). Body
        // parsing (paging/alphanumeric) is out of scope -- complex
        // tables and not on ACARS critical path. Header tells us the
        // super-frame coordinates and message length.
        ims_decoded_t ims = { 0 };
        ims_decode(&classified, &ims);
        if (ims.bch_ok) {
            const char *grp = (ims.ms_type == 1) ? "Acq" :
                              (ims.group == 0) ? "B"  :
                              (ims.group == 1) ? "C"  :
                              (ims.group == 2) ? "D"  : "E";
            ESP_LOGI(TAG, "FRAME: IMS block=%d frame=%d grp=%s len=%d "
                          "bin=%ld snr=%.1f",
                     ims.block, ims.frame, grp, ims.bch_blocks,
                     (long)it->peak_bin, (double)it->snr_db);
        } else {
            ESP_LOGI(TAG, "FRAME: IMS (bch_fail) bin=%ld snr=%.1f",
                     (long)it->peak_bin, (double)it->snr_db);
        }
        break;
    }
    case IR_FRAME_TL:
        atomic_fetch_add_explicit(&s_class_tl, 1, memory_order_relaxed);
        ESP_LOGI(TAG, "FRAME: TL bin=%ld snr=%.1f freq=%lu",
                 (long)it->peak_bin, (double)it->snr_db,
                 (unsigned long)it->freq_hz);
        break;
    case IR_FRAME_BC: {
        atomic_fetch_add_explicit(&s_class_bc, 1, memory_order_relaxed);
        // D15: extract IBC body fields (sv_id, beam, time). The
        // post-BCH 168-bit payload tells us which satellite/cell sent
        // the broadcast and (for bc_type=0 sub=1) the L-band frame
        // counter timestamp.
        ibc_decoded_t ibc = { 0 };
        ibc_decode(&classified, &ibc);
        if (ibc.header_ok && ibc.bc_type == 0 && ibc.block0_ok) {
            if (ibc.block1_subtype == 1 && ibc.iri_time > 0) {
                uint64_t ux = ibc_iri_time_to_unix(ibc.iri_time);
                ESP_LOGI(TAG, "FRAME: IBC sv=%d beam=%d slot=%d acq=%d "
                              "time=%llu (iri=%lu) bin=%ld snr=%.1f",
                         ibc.sv_id, ibc.beam_id, ibc.slot, ibc.sv_blocking,
                         (unsigned long long)ux, (unsigned long)ibc.iri_time,
                         (long)it->peak_bin, (double)it->snr_db);
            } else {
                ESP_LOGI(TAG, "FRAME: IBC sv=%d beam=%d slot=%d acq=%d "
                              "sub=%d bin=%ld snr=%.1f",
                         ibc.sv_id, ibc.beam_id, ibc.slot, ibc.sv_blocking,
                         ibc.block1_subtype,
                         (long)it->peak_bin, (double)it->snr_db);
            }
        } else {
            ESP_LOGI(TAG, "FRAME: IBC (hdr_ok=%d bc_type=%d blk0_ok=%d "
                          "blk1_ok=%d) bin=%ld snr=%.1f",
                     ibc.header_ok, ibc.bc_type, ibc.block0_ok, ibc.block1_ok,
                     (long)it->peak_bin, (double)it->snr_db);
        }
        break;
    }
    case IR_FRAME_LW:
        if (classified.lw_subtype == IR_LW_DA) {
            atomic_fetch_add_explicit(&s_class_lw_da, 1, memory_order_relaxed);
            // Run the IDA -> SBD -> ACARS chain. Log the IDA header
            // fields up front so we can see what kind of DA content is
            // in the stream (CRC pass/fail, payload length, flags).
            ida_decoded_t ida = { 0 };
            int rc_ida = ida_decode(&classified, &ida);
            ESP_LOGI(TAG, "FRAME: LW.DA bin=%ld snr=%.1f "
                          "bch_ok=%d blocks=%d/%d errs=%d "
                          "hdr_ok=%d ctr=%d len=%u crc=%s",
                     (long)it->peak_bin, (double)it->snr_db,
                     ida.ok, ida.blocks_ok, ida.n_blocks, ida.total_errors,
                     ida.header_ok, ida.da_ctr, (unsigned)ida.payload_len,
                     ida.crc_ok ? "OK" : "BAD");
            if (rc_ida == 0 && ida.ok && ida.header_ok) {
                sbd_message_t sbd;
                int rc_sbd = sbd_reassembler_feed(&s_sbd, &ida,
                                                  it->direction == 1,
                                                  (uint64_t)it->timestamp_us,
                                                  &sbd);
                if (rc_sbd == 1) {
                    atomic_fetch_add_explicit(&s_sbd_complete, 1,
                                              memory_order_relaxed);
                    ESP_LOGI(TAG, "SBD: type=%s %s len=%u (msg %u/%u)",
                             sbd_type_wire_name(sbd.type),
                             sbd.uplink ? "UL" : "DL",
                             sbd.payload_len, sbd.msg_no, sbd.msg_count);
                    try_acars(&sbd);
                }
            }
        } else {
            atomic_fetch_add_explicit(&s_class_lw_other, 1, memory_order_relaxed);
            ESP_LOGI(TAG, "FRAME: LW.%s bin=%ld snr=%.1f freq=%lu",
                     iridium_lw_subtype_name(classified.lw_subtype),
                     (long)it->peak_bin, (double)it->snr_db,
                     (unsigned long)it->freq_hz);
        }
        break;
    case IR_FRAME_RA: {
        // D15: Iridium Ring Alert -- carries the satellite's broadcast
        // position. Log sv_id/beam + lat/lon/alt where the BCH decoded.
        // Counted in the "BC" bucket for now (the smoke test's
        // expected counts predate RA classification).
        atomic_fetch_add_explicit(&s_class_bc, 1, memory_order_relaxed);
        ira_decoded_t ira = { 0 };
        ira_decode(&classified, &ira);
        if (ira.bch_ok) {
            ESP_LOGI(TAG, "FRAME: IRA sv=%d beam=%d pos=(%+d, %+d, %+d) "
                          "lat=%+.2f lon=%+.2f alt=%.0fkm "
                          "bin=%ld snr=%.1f",
                     ira.sv_id, ira.beam_id, ira.pos_x, ira.pos_y, ira.pos_z,
                     (double)ira.lat_deg, (double)ira.lon_deg,
                     (double)ira.alt_km,
                     (long)it->peak_bin, (double)it->snr_db);
        } else {
            ESP_LOGI(TAG, "FRAME: IRA (bch_fail) bin=%ld snr=%.1f",
                     (long)it->peak_bin, (double)it->snr_db);
        }
        break;
    }
    case IR_FRAME_UNKNOWN:
    default:
        atomic_fetch_add_explicit(&s_class_unknown, 1, memory_order_relaxed);
        ESP_LOGD(TAG, "FRAME: ?? bin=%ld snr=%.1f", (long)it->peak_bin,
                 (double)it->snr_db);
        break;
    }
}

static void decoder_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Decoder task started on Core %d", xPortGetCoreID());

    // Register with the task watchdog so the decoder participates in
    // panic-on-stuck behaviour like the other long-running tasks.
    esp_err_t wdt_rc = esp_task_wdt_add(NULL);
    if (wdt_rc != ESP_OK && wdt_rc != ESP_ERR_INVALID_ARG) {
        ESP_LOGW(TAG, "esp_task_wdt_add returned %d (%s)",
                 wdt_rc, esp_err_to_name(wdt_rc));
    }

    frame_queue_item_t item;
    uint64_t last_tick = (uint64_t)esp_timer_get_time();
    while (1) {
        bool got = frame_queue_pop(s_queue, &item);
        // Tick the SBD reassembler periodically (~1 Hz) so stale
        // multi-frame sessions get expired even when no frames arrive.
        uint64_t now = (uint64_t)esp_timer_get_time();
        if (now - last_tick > 1000000ULL) {
            sbd_reassembler_tick(&s_sbd, now);
            last_tick = now;
        }
        if (got) {
            process_one(&item);
            // Yield once after each item so IDLE1 / lower-prio tasks
            // (status_logger) get a slice even if bursts arrive
            // back-to-back. taskYIELD here would also work but a
            // 1-tick delay is more predictable.
            vTaskDelay(1);
        } else {
            // Empty — sleep one tick (10 ms at 100 Hz tick rate). Note:
            // pdMS_TO_TICKS(2) rounds to 0 ticks at the default 100 Hz
            // and won't yield to IDLE1, so the WDT trips on IDLE1
            // starvation. Using `1` directly forces at least one tick.
            vTaskDelay(1);
        }
        esp_task_wdt_reset();
    }
}

esp_err_t frame_decoder_init(void)
{
    if (s_initialised) return ESP_OK;

    s_queue = frame_queue_create(FRAME_QUEUE_SLOTS);
    if (!s_queue) {
        ESP_LOGE(TAG, "frame_queue_create(%d) failed (PSRAM exhausted?)",
                 FRAME_QUEUE_SLOTS);
        return ESP_ERR_NO_MEM;
    }
    sbd_reassembler_init(&s_sbd);
    // D14: libacars reassembly context for multi-block ACARS messages.
    s_reasm_ctx = la_reasm_ctx_new();
    if (!s_reasm_ctx) {
        ESP_LOGE(TAG, "la_reasm_ctx_new() failed");
        frame_queue_destroy(s_queue);
        s_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    // PSRAM stack — see feedback_task_stacks_in_psram memory note.
    // frame_decoder runs at per-frame rate (lower than tagger) doing
    // IDA + libacars parsing; ms-scale compute per wake, so PSRAM
    // stack overhead is < 1%. Frees DECODER_STACK bytes of internal
    // SRAM that would otherwise fragment a contiguous region the
    // tagger init needs.
    BaseType_t ok = xTaskCreatePinnedToCoreWithCaps(decoder_task, "frame_decoder",
                                                    DECODER_STACK, NULL,
                                                    DECODER_PRIO, &s_task,
                                                    DECODER_CORE,
                                                    MALLOC_CAP_SPIRAM);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreatePinnedToCore failed");
        frame_queue_destroy(s_queue);
        s_queue = NULL;
        return ESP_FAIL;
    }

    s_initialised = true;
    ESP_LOGI(TAG, "frame_decoder ready: %d-slot queue (~%u KB PSRAM), "
             "task on Core %d prio %d",
             FRAME_QUEUE_SLOTS,
             (unsigned)(FRAME_QUEUE_SLOTS * sizeof(frame_queue_item_t) / 1024),
             DECODER_CORE, DECODER_PRIO);
    return ESP_OK;
}

bool frame_decoder_push(const uint8_t *bits, size_t n_bits,
                        ir_direction_t direction,
                        uint32_t freq_hz, int peak_bin, float snr_db)
{
    if (!s_initialised || !bits) return false;
    if (n_bits == 0 || n_bits > FRAME_QUEUE_MAX_BITS) return false;

    frame_queue_item_t item;
    item.timestamp_us = (uint32_t)esp_timer_get_time();
    item.freq_hz      = freq_hz;
    item.peak_bin     = peak_bin;
    item.snr_db       = snr_db;
    item.n_bits       = (uint16_t)n_bits;
    item.direction    = (uint8_t)((direction == DIR_DOWNLINK) ? 0 : 1);
    item.pad          = 0;
    memcpy(item.bits, bits, n_bits);
    if (n_bits < FRAME_QUEUE_MAX_BITS) {
        memset(item.bits + n_bits, 0, FRAME_QUEUE_MAX_BITS - n_bits);
    }
    return frame_queue_push(s_queue, &item);
}

uint64_t frame_decoder_pushed(void)
{
    return s_queue ? frame_queue_pushed(s_queue) : 0;
}

uint64_t frame_decoder_popped(void)
{
    return s_queue ? frame_queue_popped(s_queue) : 0;
}

uint64_t frame_decoder_dropped(void)
{
    return s_queue ? frame_queue_dropped(s_queue) : 0;
}

size_t frame_decoder_queue_count(void)
{
    return s_queue ? frame_queue_count(s_queue) : 0;
}

void frame_decoder_get_class_counts(frame_decoder_class_counts_t *out)
{
    if (!out) return;
    out->unknown  = atomic_load_explicit(&s_class_unknown,  memory_order_relaxed);
    out->ms       = atomic_load_explicit(&s_class_ms,       memory_order_relaxed);
    out->tl       = atomic_load_explicit(&s_class_tl,       memory_order_relaxed);
    out->bc       = atomic_load_explicit(&s_class_bc,       memory_order_relaxed);
    out->lw_da    = atomic_load_explicit(&s_class_lw_da,    memory_order_relaxed);
    out->lw_other = atomic_load_explicit(&s_class_lw_other, memory_order_relaxed);
}
