# vdl2_decoder — VDL Mode 2 band pipeline: D8PSK demod (vdl2_demod.c,
# dumpvdl2-derived, host-tested via tests/host/test_vdl2_demod.c) behind
# the band_pipeline_t vtable (vdl2_pipeline.c), plus the V3 L2 feed
# (vdl2_l2.c: byte pack -> RS-block de-interleave -> RS(255,249) ->
# AVLC deframe; the RS/AVLC codecs themselves live in common/vdl2).
# docs/2026-07-22-vdl2-implementation-plan.md. Pure C, no ESP-IDF
# runtime deps, no PIE. vdl2_mod.c (synthetic test modulator) is
# host-test-only and deliberately NOT compiled into the firmware.
# firmr_s16.c comes from iridium_decoder (shared polyphase core).
# vdl2 is a public REQUIRES: vdl2_l2.h exposes avlc_frame_cb_t.
idf_component_register(
    SRCS "vdl2_pipeline.c" "vdl2_demod.c" "vdl2_l2.c"
    INCLUDE_DIRS "."
    REQUIRES band_pipeline iridium_decoder vdl2
)
