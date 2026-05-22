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
        # Phase D15: IBC (Iridium Broadcast) body parser -- extracts
        # sv_id / beam_id / slot + bc_type=1 iri_time. Lets the frame
        # decoder log "which satellite / which beam / what time" for
        # the broadcast frames in the stream.
        "ibc_decode.c"
        # Phase D15: IRA (Iridium Ring Alert) body parser -- extracts
        # sv_id / beam_id + 3D satellite position (x, y, z in 4-km
        # units, converted to geocentric lat/lon/altitude).
        "ira_decode.c"
        # Phase D15: IMS (Iridium Messaging) header parser -- extracts
        # ms_type / block / frame / bch_blocks / group from the first
        # post-header BCH block. Body parsing (paging/alphanumeric) is
        # out of scope -- complex tables, not on ACARS critical path.
        "ims_decode.c"
        # Fine carrier frequency estimator for the worker (legacy companion
        # to the channelizer; retained for diagnostic use).
        "freq_estimator.c"
        # Gardner symbol-timing recovery for qpsk_demod.
        "sym_timing.c"
        # gr-iridium-style UW cross-correlator for one-shot timing +
        # direction estimation. Drives D13 / CFO / pre-rotate / demod
        # inside burst_pipeline.
        "uw_correlator.c"
        # Codegenned const FFT precompute tables (bit-reversal,
        # twiddles, Blackman windows). Lives in .rodata (flash),
        # frees ~37 KiB from .ext_ram.bss. Regenerate via
        # tests/scripts/gen_uw_correlator_tables.py.
        "uw_correlator_tables.c"
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
        # PIE asm 9-tap MAC for the resampler hot path. Self-gates
        # on __riscv && SOC_CPU_HAS_PIE so it compiles to a no-op
        # on host (and on non-P4 targets).
        "resample_arp4.S"
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

# Hot DSP inner loops -- push past the project-wide -O2 to -O3 with
# loop unrolling. Same rationale as the per-file overrides in
# p4-usb-host/main/CMakeLists.txt: these files contain MAC inner
# loops where the compiler can do significantly more (unroll fully,
# vectorise scalar accumulators, hoist invariant loads). Functional
# regression coverage by the host ctest suite gates the more
# aggressive scheduling.
set_source_files_properties(resample_256_to_250.c
    PROPERTIES COMPILE_FLAGS
    "-O3 -funroll-loops"
)

# fft_burst_tagger's compute_magnitude_shifted (per-bin re²+im²) and
# update_baseline_ema (per-bin sub-old/add-new int32 RMW) are
# per-tagger-step SIMD candidates. -O3 + unroll lets the compiler
# SLP-vectorise the contiguous int32 inner loops, eliminating the
# scalar lower bound observed on -O2 (89 µs mag / 158 µs EMA per
# step on RAW_IRIDIUM smoke).
set_source_files_properties(fft_burst_tagger.c
    PROPERTIES COMPILE_FLAGS
    "-O3 -funroll-loops"
)
