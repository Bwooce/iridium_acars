# poa_decoder — plain VHF ACARS (POA) channelizer + MSK demod + ACARS L2,
# ported from acarsdec (Thierry Leconte, GPLv2). Pure C11 + libm, no ESP-IDF
# deps — host tests (tests/host/test_poa_decoder, test_poa_frontend) compile
# these sources directly and cross-validate byte-for-byte against acarsdec.
# Wired into the device DSP path (dsp_processor CHANNELIZED front end) in P2.
idf_component_register(
    SRCS "poa_decoder.c" "poa_frontend.c"
        # ESP32-P4 PIE Q15 channelizer kernel. Self-gates on
        # __riscv && SOC_CPU_HAS_PIE so it compiles to a no-op on host
        # (host runs the ANSI-C reference in poa_frontend.c).
        "poa_mix_arp4.S"
    INCLUDE_DIRS "."
)
