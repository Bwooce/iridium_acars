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
        # Phase D part 5: SBD multi-frame reassembler.
        "sbd_reassembler.c"
        # Phase D7: polyphase FFT channelizer (front-end multi-burst separation).
        "polyphase_channelizer.c"
        "channelizer_detector.c"
        # D20 step 3: PIE/SIMD polyphase MAC kernel.
        "polyphase_mac_arp4.S"
        # D8: fine carrier frequency estimator for the worker
        # (companion to D7's coarse channelizer).
        "freq_estimator.c"
        # D10: Gardner symbol-timing recovery for qpsk_demod.
        "sym_timing.c"
        # D10 (gr-iridium-style): UW cross-correlator for one-shot
        # timing + direction estimation.
        "uw_correlator.c"
        # Per-burst pipeline orchestration shared between worker_core1
        # and the host pipeline test. Single source of truth for the
        # gr-iridium-aligned D13 → CFO → RRC → UW → demod chain.
        "burst_pipeline.c"
        # Portable Q15 64-pt FFT used by the host channelizer (target
        # uses esp-dsp's hardware-accelerated dsps_fft2r_sc16_arp4
        # instead — this file's fft_sc16_64() symbol is dead code on
        # target but still compiled to keep the build target list
        # uniform).
        "fft_sc16_64.c"
        # Phase 3.6.M wideband-tagger path. All portable C — same
        # source compiles for host unit tests and P4 firmware.
        # The fft_sc16_2048 hot loop runs in ~18% of one P4 core at
        # 2.5 MSPS per the Phase 3.6.M feasibility analysis; could
        # be swapped for esp-dsp's PIE-accelerated dsps_fft2r_sc16
        # later if we need more headroom.
        "fft_sc16_2048.c"
        "fft_burst_tagger.c"
        "direct_if_decim.c"
        "resample_256_to_250.c"
        # firmr_s16 was previously host-only (P4 called esp-dsp's
        # dsps_firmr_s16 directly). resample_256_to_250 depends on
        # it, so compile firmr_s16.c into the IDF target too. Same
        # ANSI math; identical to dsps_firmr_s16_ansi on identical
        # input.
        "firmr_s16.c"
    INCLUDE_DIRS "."
    # libacars_idf is the vendored ACARS frame parser. It's pulled in
    # here (rather than directly by p4-usb-host/main) so that any board
    # depending on iridium_decoder transitively gets the upstream ACARS
    # decoder for free.
    REQUIRES libacars_idf
    # frame_queue uses heap_caps_calloc with MALLOC_CAP_SPIRAM.
    # polyphase_channelizer.c (D20 step 2) calls dsps_fft2r_fc32_arp4
    # on target; gated by #ifdef ESP_PLATFORM so it does not affect
    # host unit-test builds.
    PRIV_REQUIRES esp_mm espressif__esp-dsp
)
