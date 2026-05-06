# Iridium ACARS shared decoder component.
#
# Cross-board code: BCH(31,21) decoder and DQPSK demod with PLL phase
# tracking. Used by the host board (USB ingest + FFT detection)
# in-process today; will be used by future child-processor P4 boards
# (worker side) when the multi-board pipeline lands.
#
# Pure C, no FreeRTOS / USB / ESP-IDF runtime dependencies beyond
# esp_log (used only for status messages, easily stubbed for host
# unit tests under tests/host/).
idf_component_register(
    SRCS
        "bch_decoder.c"
        "qpsk_demod.c"
    INCLUDE_DIRS "."
)
