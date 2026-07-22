# vdl2_decoder — VDL Mode 2 band pipeline (foundation stub; the D8PSK
# demod, RS(255,249) and AVLC modules land here per
# docs/2026-07-22-vdl2-implementation-plan.md, each host-tested and
# cross-validated against dumpvdl2 before being wired into
# vdl2_pipeline.c). Pure C, no ESP-IDF runtime deps.
idf_component_register(
    SRCS "vdl2_pipeline.c"
    INCLUDE_DIRS "."
    REQUIRES band_pipeline
)
