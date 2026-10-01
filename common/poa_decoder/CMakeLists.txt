# poa_decoder — plain VHF ACARS (POA) channelizer + MSK demod + ACARS L2,
# ported from acarsdec (Thierry Leconte, GPLv2). Pure C11 + libm, no ESP-IDF
# deps — host tests (tests/host/test_poa_decoder, test_poa_frontend) compile
# these sources directly and cross-validate byte-for-byte against acarsdec.
# Wired into the device DSP path (dsp_processor CHANNELIZED front end) in P2.
#
# poa_mod.c (synthetic POA test modulator — the acarsdec-inverse used by
# tests/host/test_poa_mod_roundtrip) is normally host-test-only and NOT in the
# firmware. The on-device POA smoke (CONFIG_SMOKE_TEST_POA) generates golden
# ACARS bursts in RAM with it, so pull it in for that build only — mirrors
# common/vdl2_decoder/CMakeLists.txt's vdl2_mod.c gating; production images
# are unchanged.
set(_poa_srcs
    "poa_decoder.c" "poa_frontend.c"
    # ESP32-P4 PIE Q15 channelizer kernel. Self-gates on
    # __riscv && SOC_CPU_HAS_PIE so it compiles to a no-op on host
    # (host runs the ANSI-C reference in poa_frontend.c).
    "poa_mix_arp4.S"
)
if(CONFIG_SMOKE_TEST_POA)
    list(APPEND _poa_srcs "poa_mod.c")
endif()
idf_component_register(
    SRCS ${_poa_srcs}
    INCLUDE_DIRS "."
)
