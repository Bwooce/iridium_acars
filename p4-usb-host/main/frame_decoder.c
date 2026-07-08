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
#include "frame_decoder.h"
#include "frame_queue.h"
#include "iridium_frame.h"
#include "ida_decode.h"
#include "ida_reassembler.h"
#include "ibc_decode.h"
#include "ira_decode.h"
#include "ims_decode.h"
#include "tl_decode.h"
#include "sbd_reassembler.h"
#include "msg_ring.h"
#include "acars_push.h"
#include "sd_log.h"
#include <libacars/libacars.h>
#include <libacars/acars.h>
#include <libacars/reassembly.h>
#include <sys/time.h>

static const char *TAG = "FRMDEC";

#define FRAME_QUEUE_SLOTS 64 // 64 × ~2064 B ≈ 132 KB in PSRAM
#define DECODER_STACK 6144
#define DECODER_PRIO 6 // Core 0: == dsp_feed (6), < usb_pump (7),
                       // > httpd (5), > sd_log (2), > logger (1).
                       // Was 4, but under Path A's clean fast streaming
                       // usb_pump(7) <-> dsp_feed(6) ping-pong keeps Core 0
                       // continuously busy at prio >=6, so a prio-4 decoder
                       // is NEVER the highest-ready task and starves for the
                       // full 60 s TASK_WDT window -> frame_decoder abort/
                       // reboot + zero decodes (2026-07-08). Equal to
                       // dsp_feed so it gets scheduled when dsp_feed blocks
                       // for the next USB block; it self-throttles (ONE
                       // frame per wake then taskYIELD, below) so it takes
                       // only a single decode's slice and yields straight
                       // back to the tagger — it cannot runaway-starve
                       // dsp_feed at equal priority.
#define DECODER_CORE 0 // Moved from Core 1 → Core 0 (#123,
                       // 2026-05-31). Core 1's worker (prio
                       // 5) was being preempted by anything
                       // else at ≥4; relocating the per-
                       // frame decode work to Core 0 (which
                       // is bursty too — only class_driver
                       // is steady-state heavy) dissolves
                       // the Core-1 priority equilibrium
                       // documented in
                       // project_core1_cpu_budget_scheduling.

static frame_queue_t *s_queue       = NULL;
static TaskHandle_t   s_task        = NULL;
static volatile bool  s_initialised = false;

static _Atomic uint64_t s_class_unknown  = 0;
static _Atomic uint64_t s_class_ms       = 0;
static _Atomic uint64_t s_class_tl       = 0;
static _Atomic uint64_t s_class_bc       = 0;
static _Atomic uint64_t s_class_lw_da    = 0;
static _Atomic uint64_t s_class_lw_other = 0;

// SBD reassembler instance — single global, not thread-safe (only the
// decoder task touches it). 8 sessions × ~330 B ≈ 2.6 KB in BSS.
static sbd_reassembler_t s_sbd;
// Cross-burst IDA fragment reassembler (see ida_reassembler.h) — feeds
// s_sbd a complete SBD envelope even when it spanned multiple physical
// LW.DA bursts. 4 sessions × ~330 B ≈ 1.3 KB in BSS.
static ida_reassembler_t s_ida_reasm;
static _Atomic uint64_t  s_sbd_complete    = 0; // SBD messages reassembled
static _Atomic uint64_t  s_acars_decoded   = 0; // ACARS messages successfully parsed
static _Atomic uint64_t  s_acars_fragments = 0; // ACARS fragments awaiting reassembly

// D14: libacars reassembly context. Maintains per-flight-id session
// state so multi-block ACARS messages (block_id > 0, more_blocks_follow)
// arriving across multiple SBD messages get joined. Created once at
// decoder init.
static la_reasm_ctx *s_reasm_ctx = NULL;

// Rolling decode-rate counters (#117). Closes the "silent DSP wedge"
// gap: today health_wdt watches USB liveness + gateway, neither of
// which catches "USB flowing, /status happy, but no classified frames
// for hours." 60-minute and 24-hour rolling counts of classified-as-
// known-type frames (anything other than UNKNOWN), rolled by a 1-min
// esp_timer. Warn-on-decline fires when 24h > 10 && 1h == 0 = "we
// used to work, we no longer do" (an OTA regression / antenna change).
#define DRATE_MIN_BUCKETS 60 // per-minute, 1 h coverage
#define DRATE_HR_BUCKETS 24  // per-hour, 24 h coverage
static volatile uint32_t  s_drate_min[DRATE_MIN_BUCKETS];
static volatile uint32_t  s_drate_hr[DRATE_HR_BUCKETS];
static volatile uint64_t  s_drate_classified_at_last_roll = 0; // s_class_*-sum snapshot
static volatile uint8_t   s_drate_min_head                = 0;
static volatile uint8_t   s_drate_hr_head                 = 0;
static volatile uint8_t   s_drate_min_in_hr               = 0; // 0..59
static esp_timer_handle_t s_drate_timer                   = NULL;

static uint64_t drate_classified_sum(void)
{
    return atomic_load_explicit(&s_class_ms, memory_order_relaxed) + atomic_load_explicit(&s_class_tl, memory_order_relaxed) + atomic_load_explicit(&s_class_bc, memory_order_relaxed) + atomic_load_explicit(&s_class_lw_da, memory_order_relaxed) + atomic_load_explicit(&s_class_lw_other, memory_order_relaxed);
}

static void drate_tick(void *arg)
{
    (void)arg;
    uint64_t now_total              = drate_classified_sum();
    uint32_t delta                  = (uint32_t)(now_total - s_drate_classified_at_last_roll);
    s_drate_classified_at_last_roll = now_total;

    // Advance the minute bucket; write the delta into the new head.
    s_drate_min_head              = (s_drate_min_head + 1) % DRATE_MIN_BUCKETS;
    s_drate_min[s_drate_min_head] = delta;

    // Every 60 ticks, roll a fresh hour: write the hour's sum, advance.
    s_drate_min_in_hr++;
    if (s_drate_min_in_hr >= 60) {
        s_drate_min_in_hr = 0;
        uint32_t hr_sum   = 0;
        for (int i = 0; i < DRATE_MIN_BUCKETS; i++)
            hr_sum += s_drate_min[i];
        s_drate_hr_head             = (s_drate_hr_head + 1) % DRATE_HR_BUCKETS;
        s_drate_hr[s_drate_hr_head] = hr_sum;

        // Warn-on-decline: 1h is the buckets above; 24h is the hours.
        uint32_t sum_1h  = hr_sum;
        uint32_t sum_24h = 0;
        for (int i = 0; i < DRATE_HR_BUCKETS; i++)
            sum_24h += s_drate_hr[i];
        if (sum_24h > 10 && sum_1h == 0) {
            ESP_LOGW(TAG, "decode-rate decline: 24h=%u classified frames "
                          "but last 1h=0 — possible DSP regression / "
                          "antenna change / OTA broke decode",
                     sum_24h);
        }
    }
}

void frame_decoder_get_rolling_rates(uint32_t *out_1h, uint32_t *out_24h)
{
    uint32_t sum_1h = 0, sum_24h = 0;
    for (int i = 0; i < DRATE_MIN_BUCKETS; i++)
        sum_1h += s_drate_min[i];
    // T35: s_drate_hr[s_drate_hr_head] is the most-recently-archived
    // hour, captured from the SAME minute ring that sum_1h above is
    // still rolling over (the ring always holds the last 60 minutes,
    // straddling the hour boundary). Summing all DRATE_HR_BUCKETS
    // buckets and then adding sum_1h double-counts that most-recent
    // hour (up to 2x when little history exists yet) and stretches the
    // reported span to ~25h. Skip the head bucket here so the 23 older,
    // non-overlapping hourly buckets plus the live rolling 1h add up to
    // a true last-24h window.
    uint8_t hr_head = s_drate_hr_head;
    for (int i = 0; i < DRATE_HR_BUCKETS; i++) {
        if (i == hr_head) continue;
        sum_24h += s_drate_hr[i];
    }
    // 24h totals (above) exclude the in-progress hour — add the 1h
    // buckets to give the caller the actual last-24h coverage.
    sum_24h += sum_1h;
    if (out_1h) *out_1h = sum_1h;
    if (out_24h) *out_24h = sum_24h;
}

// Walk a la_proto_node tree to find the la_acars_msg payload.
extern la_type_descriptor const la_DEF_acars_message;
static la_acars_msg            *find_acars_msg(la_proto_node *node)
{
    while (node) {
        if (node->td == &la_DEF_acars_message && node->data) {
            return (la_acars_msg *)node->data;
        }
        node = node->next;
    }
    return NULL;
}

// Strip Iridium SBD's own leading ACARS content-type marker (SOH
// 0x01) and, when present, an additional 8-byte 0x03-tagged header
// block of unknown meaning -- mirrors iridium-toolkit's
// iridiumtk/reassembler/sbd.py:ReassembleIDASBDACARS.consume_l2() and
// tests/host/acars_tail.c's strip_acars_prefix() (kept in sync; see
// that file's comment for the discovery: real SBD-derived ACARS
// payloads carry this marker, but nothing in this decode chain
// stripped it before la_acars_parse_and_reassemble(), which expects
// it pre-stripped -- see test_libacars_link.c's frame-layout comment).
static void strip_acars_prefix(const uint8_t **p, int *n)
{
    if (*n < 1 || (*p)[0] != 0x01) return; // no SOH marker -- leave as-is
    (*p)++;
    (*n)--;
    if (*n >= 8 && (*p)[0] == 0x03) {
        (*p) += 8;
        (*n) -= 8;
    }
}

// Try to parse the reassembled SBD payload as ACARS. Logs the
// decoded fields if a recognisable ACARS frame is found.
static void try_acars(const sbd_message_t *msg,
                      int32_t peak_bin, float snr_db)
{
    if (!msg || msg->payload_len < 8) return;
    const uint8_t *acars_buf = msg->payload;
    int            acars_len = msg->payload_len;
    strip_acars_prefix(&acars_buf, &acars_len);
    if (acars_len < 8) return;
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
        acars_buf, acars_len, dir, s_reasm_ctx, rx_time);
    if (!node) return;
    la_acars_msg *a = find_acars_msg(node);
    if (a) {
        if (a->reasm_status == LA_REASM_COMPLETE ||
            a->reasm_status == LA_REASM_SKIPPED) {
            atomic_fetch_add_explicit(&s_acars_decoded, 1, memory_order_relaxed);
            // a->reg is libacars's raw fixed-width 7-char field, NUL-
            // terminated but left-padded with '.' for short registrations
            // (e.g. "A62001" -> ".A62001") -- strip the padding the same
            // way iridium-toolkit's own pretty-printer and
            // tests/host/test_acars_tail_real.c do, so the logged REG
            // reads as the human-readable tail number a device operator
            // (or the smoke-test verdict parser) can match against.
            const char *reg_nodot = a->reg;
            while (*reg_nodot == '.')
                reg_nodot++;
            ESP_LOGI(TAG, "ACARS: %s mode=%c reg=%s label='%.2s' block=%c "
                          "msgnum='%.4s' flight='%.6s' crc=%s txt=\"%s\"",
                     msg->uplink ? "UL" : "DL",
                     a->mode ? a->mode : '?',
                     reg_nodot,
                     a->label, a->block_id ? a->block_id : '?',
                     a->msg_num, a->flight_id,
                     a->crc_ok ? "OK" : "BAD",
                     a->txt ? a->txt : "");

            // Push to /messages-visible ring.
            acars_msg_t out  = {0};
            out.timestamp_us = msg->timestamp_us;
            out.uplink       = msg->uplink;
            out.mode         = a->mode ? a->mode : '?';
            out.label[0]     = a->label[0];
            out.label[1]     = a->label[1];
            out.block_id     = a->block_id ? a->block_id : '?';
            memcpy(out.msg_num, a->msg_num, 4);
            memcpy(out.flight_id, a->flight_id, 6);
            out.crc_ok   = a->crc_ok;
            out.peak_bin = peak_bin;
            out.snr_db   = snr_db;
            if (a->txt) {
                strlcpy(out.txt, a->txt, sizeof(out.txt));
            }
            msg_ring_push(&out);
            acars_push_emit(&out);
            sd_log_emit(&out);
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
    iridium_frame_t      classified = {0};
    ir_frame_direction_t dir        = (it->direction == DIR_DOWNLINK)
                                          ? IR_FRM_DIR_DOWNLINK
                                          : IR_FRM_DIR_UPLINK;
    int                  rc         = iridium_frame_classify(it->bits, it->n_bits, dir, &classified);
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
        ims_decoded_t ims = {0};
        ims_decode(&classified, &ims);
        if (ims.bch_ok) {
            const char *grp = (ims.ms_type == 1) ? "Acq" : (ims.group == 0) ? "B"
                                                       : (ims.group == 1)   ? "C"
                                                       : (ims.group == 2)   ? "D"
                                                                            : "E";
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
    case IR_FRAME_TL: {
        atomic_fetch_add_explicit(&s_class_tl, 1, memory_order_relaxed);
        tl_decoded_t tl = {0};
        tl_decode(&classified, &tl);
        if (tl.version >= 1 && tl.plane >= 0) {
            ESP_LOGI(TAG, "FRAME: TL V%d plane=%d bin=%ld snr=%.1f freq=%lu",
                     tl.version, tl.plane,
                     (long)it->peak_bin, (double)it->snr_db,
                     (unsigned long)it->freq_hz);
        } else {
            ESP_LOGI(TAG, "FRAME: TL (v%d plane=%d unknown) bin=%ld snr=%.1f freq=%lu",
                     tl.version, tl.plane,
                     (long)it->peak_bin, (double)it->snr_db,
                     (unsigned long)it->freq_hz);
        }
        break;
    }
    case IR_FRAME_BC: {
        atomic_fetch_add_explicit(&s_class_bc, 1, memory_order_relaxed);
        // D15: extract IBC body fields (sv_id, beam, time). The
        // post-BCH 168-bit payload tells us which satellite/cell sent
        // the broadcast and (for bc_type=0 sub=1) the L-band frame
        // counter timestamp.
        ibc_decoded_t ibc = {0};
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
            ida_decoded_t ida    = {0};
            int           rc_ida = ida_decode(&classified, &ida);
            ESP_LOGI(TAG, "FRAME: LW.DA bin=%ld snr=%.1f "
                          "bch_ok=%d blocks=%d/%d errs=%d "
                          "hdr_ok=%d ctr=%d len=%u crc=%s",
                     (long)it->peak_bin, (double)it->snr_db,
                     ida.ok, ida.blocks_ok, ida.n_blocks, ida.total_errors,
                     ida.header_ok, ida.da_ctr, (unsigned)ida.payload_len,
                     ida.crc_ok ? "OK" : "BAD");
            if (rc_ida == 0 && ida.ok && ida.header_ok && ida.crc_ok) {
                // Stage 1: chain cross-burst IDA fragments (da_cont/
                // da_ctr) into one complete SBD envelope -- a single
                // physical LW.DA burst caps at 24 payload bytes, but
                // real SBD/ACARS envelopes routinely need more than
                // that (see ida_reassembler.h).
                uint8_t merged[IDA_REASM_MAX_BYTES];
                int     merged_len = 0;
                int     rc_reasm   = ida_reassembler_feed(
                    &s_ida_reasm, &ida, it->direction == 1,
                    (uint32_t)it->freq_hz, it->timestamp_us, merged,
                    (int)sizeof(merged), &merged_len);
                if (rc_reasm == 1) {
                    // Stage 2: SBD envelope-level (msgno/msgcnt) reassembly.
                    sbd_message_t sbd;
                    int           rc_sbd = sbd_reassembler_feed(&s_sbd, merged,
                                                                merged_len,
                                                                it->direction == 1,
                                                                it->timestamp_us,
                                                                &sbd);
                    if (rc_sbd == 1) {
                        atomic_fetch_add_explicit(&s_sbd_complete, 1,
                                                  memory_order_relaxed);
                        ESP_LOGI(TAG, "SBD: type=%s %s len=%u (msg %u/%u)",
                                 sbd_type_wire_name(sbd.type),
                                 sbd.uplink ? "UL" : "DL",
                                 sbd.payload_len, sbd.msg_no, sbd.msg_count);
                        try_acars(&sbd, it->peak_bin, it->snr_db);
                    }
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
        ira_decoded_t ira = {0};
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

    // NOT subscribed to the task watchdog. The decoder is a best-effort
    // downstream consumer: if Core 0 is momentarily saturated by the
    // usb_pump/dsp_feed (tagger) real-time path and this task is starved,
    // the correct behaviour is to let frames queue/drop while ingest and
    // detection keep running — NOT to panic-reboot the whole device (which
    // tears down the USB stream, the tagger, and any in-flight pass). This
    // task previously WDT-rebooted every ~60 s under Path A's fast streaming
    // when it ran at prio 4; it now runs at prio 6 (see DECODER_PRIO) so it
    // is scheduled, but a WDT subscription here is the wrong safety net for
    // a throughput deficit. Same rationale as class_driver's unsubscribe
    // (memory: task-wdt-subscription-pitfalls). Stream-stall / health
    // watchdogs elsewhere still cover a genuinely wedged pipeline.

    // Decoder task is a single serial consumer (one xTaskCreatePinnedToCoreWithCaps
    // instance, non-reentrant). Move the ~2.1 KB frame_queue_item_t from stack to
    // static .bss to relieve stack pressure (6144 B stack was marginal). Safe
    // because the item is not captured/reused across task iterations.
    static frame_queue_item_t item      = {0};
    uint64_t                  last_tick = (uint64_t)esp_timer_get_time();
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
            // ONE frame per wake, then taskYIELD — do NOT drain a batch.
            // Now that this task runs at prio 6 (== dsp_feed, raised from 4
            // to escape the usb_pump<->dsp_feed ping-pong that TASK_WDT-
            // rebooted it), it must not hold Core 0 across several frames or
            // it would delay the tagger (dsp_feed) it shares the priority
            // with. Draining one frame then yielding bounds its hold to a
            // single decode. taskYIELD (not vTaskDelay) keeps draining a
            // deep queue at full rate — the loop re-pops as soon as the
            // tagger has taken its slice — so it does NOT reintroduce the
            // ~100 frame/s cap a per-item vTaskDelay(1) would impose.
            taskYIELD();
        } else {
            // Empty — sleep one tick (10 ms at 100 Hz tick rate). Note:
            // pdMS_TO_TICKS(2) rounds to 0 ticks at the default 100 Hz
            // and won't yield to IDLE1, so the WDT trips on IDLE1
            // starvation. Using `1` directly forces at least one tick.
            vTaskDelay(1);
        }
        // No esp_task_wdt_reset() — this task is not WDT-subscribed
        // (see the rationale where the subscription used to be).
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
    ida_reassembler_init(&s_ida_reasm);
    // D17 message ring — recent ACARS decodes, served via HTTP /messages.
    msg_ring_init();
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

    // Rolling decode-rate timer (#117): 1-minute tick.
    const esp_timer_create_args_t drate_args = {
        .callback        = drate_tick,
        .arg             = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name            = "decode_rate",
    };
    if (esp_timer_create(&drate_args, &s_drate_timer) == ESP_OK) {
        esp_timer_start_periodic(s_drate_timer, 60ULL * 1000000ULL);
    } else {
        ESP_LOGW(TAG, "decode-rate timer create failed; rolling counters unavailable");
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
                        uint32_t freq_hz, int peak_bin, float snr_db,
                        uint64_t timestamp_us)
{
    if (!s_initialised || !bits) return false;
    if (n_bits == 0 || n_bits > FRAME_QUEUE_MAX_BITS) return false;

    frame_queue_item_t item;
    // Capture time from the caller (burst sample position); 0 = stamp now.
    item.timestamp_us = timestamp_us ? timestamp_us : (uint64_t)esp_timer_get_time();
    item.freq_hz      = freq_hz;
    item.peak_bin     = peak_bin;
    item.snr_db       = snr_db;
    item.n_bits       = (uint16_t)n_bits;
    item.direction    = (uint8_t)((direction == DIR_DOWNLINK) ? 0 : 1);
    item.pad          = 0;
    // No tail zero-fill: every consumer (iridium_frame_classify and the
    // ida/ibc/ims/tl/ira decoders it dispatches to) bounds-checks against
    // item.n_bits before indexing into bits[], so bits[n_bits..2047] is
    // never read. frame_queue_push() also only copies the valid prefix.
    memcpy(item.bits, bits, n_bits);
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
    out->unknown  = atomic_load_explicit(&s_class_unknown, memory_order_relaxed);
    out->ms       = atomic_load_explicit(&s_class_ms, memory_order_relaxed);
    out->tl       = atomic_load_explicit(&s_class_tl, memory_order_relaxed);
    out->bc       = atomic_load_explicit(&s_class_bc, memory_order_relaxed);
    out->lw_da    = atomic_load_explicit(&s_class_lw_da, memory_order_relaxed);
    out->lw_other = atomic_load_explicit(&s_class_lw_other, memory_order_relaxed);
}

uint64_t frame_decoder_acars_decoded_total(void)
{
    return atomic_load_explicit(&s_acars_decoded, memory_order_relaxed);
}
uint64_t frame_decoder_sbd_complete_total(void)
{
    return atomic_load_explicit(&s_sbd_complete, memory_order_relaxed);
}
