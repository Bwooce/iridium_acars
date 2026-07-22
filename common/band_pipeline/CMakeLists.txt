# band_pipeline — per-band profile table + the band_pipeline_t demod
# interface (VHF/VDL2 foundation). Pure C, dependency-free: the generic
# layer must not depend on any concrete band decoder (iridium_decoder /
# vdl2_decoder depend on THIS, never the reverse). Host tests compile
# band_profile.c directly (tests/host/test_band_profile.c).
idf_component_register(
    SRCS "band_profile.c"
    INCLUDE_DIRS "."
)
