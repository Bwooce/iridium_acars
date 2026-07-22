# vdl2_decoder — VDL Mode 2 band pipeline: D8PSK demod (vdl2_demod.c,
# dumpvdl2-derived, host-tested via tests/host/test_vdl2_demod.c) behind
# the band_pipeline_t vtable (vdl2_pipeline.c). RS(255,249) lives in
# common/vdl2; AVLC lands here per
# docs/2026-07-22-vdl2-implementation-plan.md. Pure C, no ESP-IDF
# runtime deps, no PIE. vdl2_mod.c (synthetic test modulator) is
# host-test-only and deliberately NOT compiled into the firmware.
# firmr_s16.c comes from iridium_decoder (shared polyphase core).
idf_component_register(
    SRCS "vdl2_pipeline.c" "vdl2_demod.c"
    INCLUDE_DIRS "."
    REQUIRES band_pipeline iridium_decoder
)
