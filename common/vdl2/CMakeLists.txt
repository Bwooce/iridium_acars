# vdl2 — VDL Mode 2 L2 codecs: RS(255,249) (rs_vdl2.c) + AVLC/HDLC
# deframer (avlc.c). Pure C, no ESP-IDF runtime deps; the same sources
# compile on host (tests/host: test_rs_vdl2, test_avlc, the V3 chain
# tests). Firmware phase V3 wires them into the band=vdl2 decode path
# (vdl2_decoder's vdl2_l2.c + frame_decoder.c). iridium_decoder is a
# public REQUIRES for crc16.h (avlc.c uses the shared CRC-16/X-25).
idf_component_register(
    SRCS "rs_vdl2.c" "avlc.c"
    INCLUDE_DIRS "."
    REQUIRES iridium_decoder
)
