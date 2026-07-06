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

int acars_tail_feed(sbd_reassembler_t *sbd_ctx, la_reasm_ctx *reasm_ctx,
                    const ida_decoded_t *ida, bool uplink,
                    uint64_t timestamp_us, acars_tail_result_t *out)
{
    if (!out) return -1;
    memset(out, 0, sizeof(*out));
    if (!sbd_ctx || !ida) return -1;

    sbd_message_t sbd;
    int           rc_sbd = sbd_reassembler_feed(sbd_ctx, ida, uplink, timestamp_us, &sbd);
    if (rc_sbd != 1) return rc_sbd;

    out->sbd_ready = true;
    out->sbd       = sbd;

    // Mirrors try_acars()'s early-out.
    if (sbd.payload_len < 8) return 1;

    la_msg_dir     dir     = sbd.uplink ? LA_MSG_DIR_GND2AIR : LA_MSG_DIR_AIR2GND;
    struct timeval rx_time = {
        .tv_sec  = (time_t)(sbd.timestamp_us / 1000000ULL),
        .tv_usec = (suseconds_t)(sbd.timestamp_us % 1000000ULL),
    };
    la_proto_node *node = la_acars_parse_and_reassemble(
        sbd.payload, sbd.payload_len, dir, reasm_ctx, rx_time);
    if (!node) return 1;

    la_acars_msg *a = find_acars_msg(node);
    if (a && (a->reasm_status == LA_REASM_COMPLETE ||
              a->reasm_status == LA_REASM_SKIPPED)) {
        out->acars_ready = true;
        out->mode        = a->mode;
        out->label[0]    = a->label[0];
        out->label[1]    = a->label[1];
        out->label[2]    = '\0';
        out->block_id    = a->block_id;
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
