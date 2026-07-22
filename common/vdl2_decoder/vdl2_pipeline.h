// vdl2_pipeline — VDL Mode 2 (136.975 MHz D8PSK 10.5 kBd) implementation
// of the band_pipeline_t interface. FOUNDATION STUB: the vtable exists
// so band=vdl2 boots, runs the shared front end (tagger → extract →
// rotate → decim) and counts bursts, but process_burst demodulates
// NOTHING yet. The real chain lands per
// docs/2026-07-22-vdl2-implementation-plan.md:
//
//   process_burst (this file, Phase V3 integration):
//     vdl2_demod_burst()   — D8PSK carrier/timing recovery + training-
//                            sequence sync + descrambler   [vdl2_demod.c]
//     -> per 249-byte block: rs_255_249_decode()           [rs_255_249.c,
//                            built separately — see plan §RS plug-in]
//     -> avlc_deframe()    — HDLC flags, bit-unstuffing, FCS (CRC-16),
//                            address/control parse          [avlc.c]
//     -> ACARS-over-AVLC payloads -> frame_decoder / libacars
//        (la_acars_parse_and_reassemble — the same call Iridium SBD
//        payloads already make in p4-usb-host/main/frame_decoder.c)
//
// Each stage is host-tested in isolation and cross-validated against
// dumpvdl2 on shared IQ captures before it is wired here.

#pragma once

#include <stdint.h>
#include "band_pipeline.h"

// Returns the (static, immutable) VDL2 pipeline vtable. Stub: prefilter
// accepts everything; process_burst counts the burst and returns 0.
const band_pipeline_t *vdl2_pipeline(void);

// Diagnostic: bursts handed to the stub since boot (never reset). Lets
// a live band=vdl2 soak prove the front-end plumbing works (tagger →
// worker → pipeline dispatch) before any demod exists.
uint32_t vdl2_pipeline_bursts_seen(void);
