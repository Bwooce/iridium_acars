// acars_tail — shared IDA -> SBD -> ACARS tail glue for host tooling.
//
// Mirrors the LW.DA dispatch path in p4-usb-host/main/frame_decoder.c
// (process_one()'s IR_FRAME_LW/IR_LW_DA case + try_acars()), minus the
// on-device-only side effects (msg_ring_push / acars_push_emit /
// sd_log_emit). Both decode_burst_capture.c (the CLI replay tool) and
// test_acars_tail_synthetic.c (the positive-control unit test) call
// this exact function so they exercise identical semantics.
//
// Caller contract (matches frame_decoder.c's process_one()):
//   - `ida` must come from a successful ida_decode() call
//     (rc == 0 && ida->ok && ida->header_ok && ida->crc_ok).
//   - `sbd_ctx` must be sbd_reassembler_init()'d once and reused across
//     calls (it holds multi-frame reassembly state).
//   - `reasm_ctx` must be a live la_reasm_ctx (la_reasm_ctx_new()),
//     reused across calls so multi-block ACARS messages join.

#ifndef ACARS_TAIL_H
#define ACARS_TAIL_H

#include <stdint.h>
#include <stdbool.h>
#include "ida_decode.h"
#include "sbd_reassembler.h"
#include <libacars/reassembly.h> // la_reasm_ctx

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // SBD stage.
    bool          sbd_ready; // sbd_reassembler_feed() returned 1 (out->sbd valid)
    sbd_message_t sbd;

    // ACARS stage (only meaningful if sbd_ready).
    bool acars_ready; // a complete/skipped la_acars_msg was extracted
    char mode;
    char label[3]; // 2 chars + NUL
    char block_id;
    char msg_num[5];   // 3 chars + seq char + NUL
    char flight_id[7]; // 6 chars + NUL
    bool crc_ok;
    char txt[256];
} acars_tail_result_t;

// Feed one IDA-decoded frame through sbd_reassembler_feed() and, on a
// completed SBD message whose payload is long enough to be worth
// parsing (mirrors try_acars()'s `payload_len < 8` guard), through
// la_acars_parse_and_reassemble().
//
// Returns the same convention as sbd_reassembler_feed():
//    1  SBD message assembled (out->sbd_ready set; out->acars_ready
//       set iff a complete/skipped ACARS message was also extracted)
//    0  frame consumed, multi-frame session still in progress
//   -1  frame filtered / not SBD
int acars_tail_feed(sbd_reassembler_t *sbd_ctx, la_reasm_ctx *reasm_ctx,
                    const ida_decoded_t *ida, bool uplink,
                    uint64_t timestamp_us, acars_tail_result_t *out);

#ifdef __cplusplus
}
#endif

#endif // ACARS_TAIL_H
