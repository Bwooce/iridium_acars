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
        # Phase A/B frame classifier and BCH polynomial helpers.
        "iridium_bch.c"
        "iridium_frame.c"
        # Phase D queue between worker and frame_decoder task.
        "frame_queue.c"
        # Phase D part 4: IDA frame post-LCW decoder (de-interleave + BCH).
        "ida_decode.c"
    INCLUDE_DIRS "."
    # libacars_idf is the vendored ACARS frame parser. It's pulled in
    # here (rather than directly by p4-usb-host/main) so that any board
    # depending on iridium_decoder transitively gets the upstream ACARS
    # decoder for free.
    REQUIRES libacars_idf
    # frame_queue uses heap_caps_calloc with MALLOC_CAP_SPIRAM.
    PRIV_REQUIRES esp_mm
)
