// vdl2_pipeline — VDL Mode 2 (136.975 MHz D8PSK 10.5 kBd) implementation
// of the band_pipeline_t interface. process_burst runs the real D8PSK
// demod chain (vdl2_demod.h: resample 250 k -> 105 k, phase-domain
// training-sequence sync, differential 8-PSK slicing, descrambler,
// burst-header decode) and fires the callback once per locked burst
// with the descrambled PHY bit vector — header included. Multiple
// back-to-back CSMA transmissions inside one tagger window are all
// demodulated (bounded loop).
//
// Downstream (docs/2026-07-22-vdl2-implementation-plan.md, Phase V3):
//   -> per RS block: rs_vdl2_decode (common/vdl2/rs_vdl2.h)
//   -> avlc_deframe() — HDLC flags, bit-unstuffing, FCS
//   -> ACARS-over-AVLC payloads -> frame_decoder / libacars
//      (la_acars_parse_and_reassemble — the same call Iridium SBD
//      payloads already make in p4-usb-host/main/frame_decoder.c)
//
// Prefilter is still accept-all: VDL2 junk gates await V1 capture
// calibration; the training-sequence lock is the effective gate.

#pragma once

#include <stdint.h>
#include "band_pipeline.h"

// Returns the (static, immutable) VDL2 pipeline vtable.
const band_pipeline_t *vdl2_pipeline(void);

// Diagnostics (never reset; the /status counter pattern):
// bursts handed to the pipeline since boot,
uint32_t vdl2_pipeline_bursts_seen(void);
// preamble+header locks (incl. window-truncated frames),
uint32_t vdl2_pipeline_sync_count(void);
// complete frames emitted with demod_ok=true.
uint32_t vdl2_pipeline_frames_ok(void);
