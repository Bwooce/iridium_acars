// acars_tail — see acars_tail.h.

#include "acars_tail.h"
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <libacars/libacars.h>
#include <libacars/acars.h>

// Walk a la_proto_node tree to find the la_acars_msg payload. Same
// helper as frame_decoder.c's find_acars_msg() / test_libacars_link.c's
// copy.
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
// iridiumtk/reassembler/sbd.py:ReassembleIDASBDACARS.consume_l2()
// (`if q.data[0]!=1: return; q.data=q.data[1:]; if q.data[0]==0x3:
// hdr=q.data[0:8]; data=data[8:]`). la_acars_parse_and_reassemble()
// expects both already stripped -- see test_libacars_link.c's
// "Frame layout (NOT including the leading SOH 0x01 ...)" comment.
// Real SBD-derived payloads carry this marker; hand-built test
// fixtures that start directly with the mode byte don't, so this is a
// no-op for them (backward compatible).
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

int acars_tail_feed(sbd_reassembler_t *sbd_ctx, la_reasm_ctx *reasm_ctx,
                    ida_reassembler_t *ida_reasm, const ida_decoded_t *ida,
                    bool uplink, uint32_t freq_hz, uint64_t timestamp_us,
                    acars_tail_result_t *out)
{
    if (!out) return -1;
    memset(out, 0, sizeof(*out));
    if (!sbd_ctx || !ida_reasm || !ida) return -1;

    // Stage 1: chain cross-burst IDA fragments (da_cont/da_ctr) into
    // one complete SBD envelope. rc==0 (chain still open) or rc==-1
    // (orphan/overflow, dropped) both mean "nothing to hand upstream
    // yet" -- pass the code straight through, matching
    // sbd_reassembler_feed()'s own return convention.
    uint8_t merged[IDA_REASM_MAX_BYTES];
    int     merged_len = 0;
    int     rc_ida     = ida_reassembler_feed(ida_reasm, ida, uplink, freq_hz,
                                              timestamp_us, merged,
                                              (int)sizeof(merged), &merged_len);
    if (rc_ida != 1) return rc_ida;

    // Stage 2: SBD envelope-level (msgno/msgcnt) reassembly.
    sbd_message_t sbd;
    int           rc_sbd = sbd_reassembler_feed(sbd_ctx, merged, merged_len,
                                                uplink, timestamp_us, &sbd);
    if (rc_sbd != 1) return rc_sbd;

    out->sbd_ready = true;
    out->sbd       = sbd;

    // Mirrors try_acars()'s early-out.
    if (sbd.payload_len < 8) return 1;

    const uint8_t *acars_buf = sbd.payload;
    int            acars_len = sbd.payload_len;
    strip_acars_prefix(&acars_buf, &acars_len);
    if (acars_len < 8) return 1;

    la_msg_dir     dir     = sbd.uplink ? LA_MSG_DIR_GND2AIR : LA_MSG_DIR_AIR2GND;
    struct timeval rx_time = {
        .tv_sec  = (time_t)(sbd.timestamp_us / 1000000ULL),
        .tv_usec = (suseconds_t)(sbd.timestamp_us % 1000000ULL),
    };
    la_proto_node *node = la_acars_parse_and_reassemble(
        acars_buf, acars_len, dir, reasm_ctx, rx_time);
    if (!node) return 1;

    la_acars_msg *a = find_acars_msg(node);
    if (a && (a->reasm_status == LA_REASM_COMPLETE ||
              a->reasm_status == LA_REASM_SKIPPED)) {
        out->acars_ready = true;
        out->mode        = a->mode;
        snprintf(out->reg, sizeof(out->reg), "%s", a->reg);
        out->ack      = a->ack;
        out->label[0] = a->label[0];
        out->label[1] = a->label[1];
        out->label[2] = '\0';
        out->block_id = a->block_id;
        // a->msg_num is already NUL-terminated at index 3 by the
        // parser (msg_num[3] = '\0'); copy only the 3 meaningful
        // chars and append the separate seq char ourselves so the
        // full 4-char msgnum ("M04A") is visible in out->msg_num.
        memcpy(out->msg_num, a->msg_num, 3);
        out->msg_num[3] = a->msg_num_seq;
        out->msg_num[4] = '\0';
        memcpy(out->flight_id, a->flight_id, 6);
        out->flight_id[6] = '\0';
        out->crc_ok       = a->crc_ok;
        if (a->txt) {
            snprintf(out->txt, sizeof(out->txt), "%s", a->txt);
        }
    }
    la_proto_tree_destroy(node);
    return 1;
}
