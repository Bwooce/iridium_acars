# Iridium ACARS shared decoder component.
#
# Cross-board code: BCH(31,21) decoder, DQPSK demod with PLL phase
# tracking, wideband fft_burst_tagger front end, direct-IF decimator,
# and per-burst pipeline orchestration.
#
# Pure C, no FreeRTOS / USB / ESP-IDF runtime dependencies beyond
# esp_log (used only for status messages, easily stubbed for host
# unit tests under tests/host/).
#
# Historical: the polyphase channelizer was the active front end
# through Phase 3.5 / D7. Phase 3.6.M replaced it with the
# wideband fft_burst_tagger (matches gr-iridium's architecture).
# The channelizer code, its detector/MAC kernels, and the per-frame
# windowing/magnitude PIE asm that fed it were all removed in commit
# (TBD when this commit lands). Last revision with the channelizer
# present and active: bf0bb9f^ (the commit just before the wideband
# cutover); last revision with channelizer dead-but-linked:
# 7eab12a (this commit's parent).
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
        # Fine carrier frequency estimator for the worker (legacy companion
        # to the channelizer; retained for diagnostic use).
        "freq_estimator.c"
        # Gardner symbol-timing recovery for qpsk_demod.
        "sym_timing.c"
        # gr-iridium-style UW cross-correlator for one-shot timing +
        # direction estimation. Drives D13 / CFO / pre-rotate / demod
        # inside burst_pipeline.
        "uw_correlator.c"
        # Per-burst pipeline orchestration shared between worker_core1
        # and the host pipeline test. Single source of truth for the
        # gr-iridium-aligned D13 → CFO → RRC → UW → demod chain.
        "burst_pipeline.c"
        # Phase 3.6.M wideband-tagger path. All portable C — same
        # source compiles for host unit tests and P4 firmware.
        # fft_sc16_2048 internals are scalar Q15; esp-dsp's
        # PIE-accelerated dsps_fft2r_sc16_arp4 could replace them
        # later if more headroom is needed.
        "fft_sc16_2048.c"
        "fft_burst_tagger.c"
        "direct_if_decim.c"
        "resample_256_to_250.c"
        # Absolute-phase IQ rotation shared with the host wideband
        # test. Lives here (not in worker_core1 / the test) because
        # both call sites need the SAME Q15-magnitude-decay-avoiding
        # implementation; the header explains the trap. Companion
        # P4 PIE asm (rotate_to_dc_arp4.S) is the 8-lane SIMD kernel
        # called by rotate_to_dc_q15_simd_arp4_at — the .S file
        # self-gates on `__riscv && SOC_CPU_HAS_PIE` so it compiles
        # to a no-op on host (and on Xtensa) and lets the same
        # CMakeLists serve both target builds.
        "rotate_to_dc.c"
        "rotate_to_dc_arp4.S"
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
    # direct_if_decim on target uses dsps_fird_s16_arp4.
    PRIV_REQUIRES esp_mm espressif__esp-dsp
)
