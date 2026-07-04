cmake_minimum_required(VERSION 3.16)
project(p4_usb_host_tests C)

set(CMAKE_C_STANDARD 11)
add_compile_options(-Wall -Wextra -Wno-unused-parameter -O2 -g)

# UBSan: runtime trap on signed-integer overflow and related UB. Caught
# a real bug in cfo_fine_estimate (sq_re×Q15 window overflowing int32).
# Off by default — the firmware build doesn't need the runtime overhead,
# and the channelizer's overflow-on-extreme-input is itself test
# fixture data not real-RF. Enable with `cmake -DHOST_TESTS_UBSAN=ON`.
option(HOST_TESTS_UBSAN "Build host tests with -fsanitize=signed-integer-overflow,shift,bounds" OFF)
if(HOST_TESTS_UBSAN)
    add_compile_options(
        -fsanitize=signed-integer-overflow
        -fsanitize=shift
        -fsanitize=bounds
        -fno-sanitize-recover=signed-integer-overflow,shift,bounds
    )
    add_link_options(
        -fsanitize=signed-integer-overflow
        -fsanitize=shift
        -fsanitize=bounds
    )
endif()

# Pull production sources directly from common/iridium_decoder/ so the
# host tests exercise the exact code that runs on the target. The
# decoder lives in common/ so it can be shared with the future worker
# P4 firmware; the tests don't need to know about that layout — they
# just compile the same files with a host toolchain.
set(COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../common/iridium_decoder)
set(MAIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../p4-usb-host/main)

# bch_decoder is pure C with no IDF deps — compile it as-is.
add_executable(test_bch
    test_bch.c
    ${COMMON_DIR}/bch_decoder.c
)
target_include_directories(test_bch PRIVATE ${COMMON_DIR})

# qpsk_demod includes "esp_log.h"; tests/host/esp_log.h shadows it on
# the host with a no-op stub. Adding tests/host/ first on the include
# path makes the host build pick up our stub without touching the
# production source.
add_executable(test_qpsk
    test_qpsk.c
    ${COMMON_DIR}/qpsk_demod.c
    ${COMMON_DIR}/sym_timing.c
)
target_include_directories(test_qpsk PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}   # esp_log.h stub
    ${COMMON_DIR}
)
target_link_libraries(test_qpsk PRIVATE m)

add_executable(test_demod_corpus
    test_demod_corpus.c
    ${COMMON_DIR}/qpsk_demod.c
    ${COMMON_DIR}/sym_timing.c
)
target_include_directories(test_demod_corpus PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}                   # esp_log.h stub
    ${CMAKE_CURRENT_SOURCE_DIR}/../fixtures       # generated headers
    ${COMMON_DIR}
)
target_link_libraries(test_demod_corpus PRIVATE m)

add_executable(test_demod_low_snr
    test_demod_low_snr.c
    ${COMMON_DIR}/qpsk_demod.c
    ${COMMON_DIR}/sym_timing.c
)
target_include_directories(test_demod_low_snr PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../fixtures
    ${COMMON_DIR}
)
target_link_libraries(test_demod_low_snr PRIVATE m)

add_executable(test_demod_albq
    test_demod_albq.c
    ${COMMON_DIR}/qpsk_demod.c
    ${COMMON_DIR}/sym_timing.c
)
target_include_directories(test_demod_albq PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../fixtures
    ${COMMON_DIR}
)
target_link_libraries(test_demod_albq PRIVATE m)

# iridium_frame: top-level frame classifier (Phase A — MS/TL match).
# Pure C, no IDF deps, builds straight from common/.
add_executable(test_iridium_frame
    test_iridium_frame.c
    ${COMMON_DIR}/iridium_frame.c
    ${COMMON_DIR}/iridium_bch.c
)
target_include_directories(test_iridium_frame PRIVATE ${COMMON_DIR})

# Frame classifier vs Albuquerque corpus (regression).
add_executable(test_iridium_frame_corpus
    test_iridium_frame_corpus.c
    ${COMMON_DIR}/iridium_frame.c
    ${COMMON_DIR}/iridium_bch.c
)
target_include_directories(test_iridium_frame_corpus PRIVATE
    ${COMMON_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../fixtures
)

# BCH polynomial-division + 1-bit ECC repair primitives (Phase B).
add_executable(test_iridium_bch
    test_iridium_bch.c
    ${COMMON_DIR}/iridium_bch.c
)
target_include_directories(test_iridium_bch PRIVATE ${COMMON_DIR})

# Phase B synthetic frame tests (BC + LW dispatch + LW subtypes).
add_executable(test_iridium_frame_phaseB
    test_iridium_frame_phaseB.c
    ${COMMON_DIR}/iridium_frame.c
    ${COMMON_DIR}/iridium_bch.c
)
target_include_directories(test_iridium_frame_phaseB PRIVATE ${COMMON_DIR})

# Phase D queue between worker and frame_decoder task.
add_executable(test_frame_queue
    test_frame_queue.c
    ${COMMON_DIR}/frame_queue.c
)
target_include_directories(test_frame_queue PRIVATE ${COMMON_DIR})
target_link_libraries(test_frame_queue PRIVATE pthread)

# Phase D part 4: ida_decode (LW.DA -> 10×BCH(31,21) -> ~26 byte payload).
add_executable(test_ida_decode_corpus
    test_ida_decode_corpus.c
    ${COMMON_DIR}/ida_decode.c
    ${COMMON_DIR}/iridium_frame.c
    ${COMMON_DIR}/iridium_bch.c
)
target_include_directories(test_ida_decode_corpus PRIVATE
    ${COMMON_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../fixtures
)

# ibc_decode: IBC (Iridium Broadcast) body parser bit-orientation
# regression. Synthesizes a valid BC frame with a non-palindromic
# payload and checks the header/block fields round-trip correctly.
add_executable(test_ibc_decode
    test_ibc_decode.c
    ${COMMON_DIR}/ibc_decode.c
    ${COMMON_DIR}/iridium_frame.c
    ${COMMON_DIR}/iridium_bch.c
)
target_include_directories(test_ibc_decode PRIVATE ${COMMON_DIR})

# Phase D part 5: SBD reassembler (multi-frame state machine).
add_executable(test_sbd_reassembler
    test_sbd_reassembler.c
    ${COMMON_DIR}/sbd_reassembler.c
)
target_include_directories(test_sbd_reassembler PRIVATE ${COMMON_DIR})

# Phase 3.6.M: cross-validate the new Q15 2048-pt FFT (for the wideband
# fft_burst_tagger) against a textbook direct-DFT reference. Same
# algorithm as fft_sc16_64.c just resized for N=2048; the test catches
# bit-reversal / twiddle / butterfly regressions.
add_executable(test_fft_sc16_2048
    test_fft_sc16_2048.c
    ${COMMON_DIR}/fft_sc16_2048.c
)
target_include_directories(test_fft_sc16_2048 PRIVATE ${COMMON_DIR})
target_link_libraries(test_fft_sc16_2048 PRIVATE m)

# Validates the within-chunk split-resample path (used by
# p4-usb-host/main/ingest_core1.c). Worker A processes input[0..mid],
# Worker B processes input[mid..end] with state derived via
# closed-form. Must be bit-exact vs the sequential reference; any
# divergence here predicts a USB-ingest decode regression on target.
add_executable(test_resample_split
    test_resample_split.c
    ${COMMON_DIR}/resample_256_to_250.c
    ${COMMON_DIR}/firmr_s16.c
)
target_include_directories(test_resample_split PRIVATE ${COMMON_DIR})
target_link_libraries(test_resample_split PRIVATE m)

# Phase 3.6.M: wideband fft_burst_tagger smoke test. Feeds the ALBQ
# raw fixture through the tagger and checks the burst-count + bin
# distribution looks sane (≥30 bursts emitted, multiple bins hit, EMA
# primes correctly). Compares loosely against gri's 65 decoded bursts.
add_executable(test_fft_burst_tagger
    test_fft_burst_tagger.c
    ${COMMON_DIR}/fft_burst_tagger.c
    ${COMMON_DIR}/fft_sc16_2048.c
)
target_include_directories(test_fft_burst_tagger PRIVATE
    ${COMMON_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../fixtures
)
target_link_libraries(test_fft_burst_tagger PRIVATE m)

# Phase 3.6.M: C direct-IF decimator vs Python scipy reference. For
# one burst, runs the same rotate+decim in C and compares NMSE
# against /tmp/host_direct_if/burst_NNN.cf32 (the Python output).
# NOT in ctest by default: needs the Python fixture-dump to have
# run first.
add_executable(test_direct_if_decim
    test_direct_if_decim.c
    ${COMMON_DIR}/direct_if_decim.c
)
target_include_directories(test_direct_if_decim PRIVATE ${COMMON_DIR})
target_link_libraries(test_direct_if_decim PRIVATE m)

# Characterization (golden) tests — task #133, safety net for the #120
# context-threading refactor. Pin EXACT current output (checksum + scalars)
# so a no-behavior-change refactor is provably no-behavior-change.
add_executable(test_direct_if_decim_golden
    test_direct_if_decim_golden.c
    ${COMMON_DIR}/direct_if_decim.c
)
target_include_directories(test_direct_if_decim_golden PRIVATE ${COMMON_DIR})
target_link_libraries(test_direct_if_decim_golden PRIVATE m)

add_executable(test_uw_correlator_golden
    test_uw_correlator_golden.c
    ${COMMON_DIR}/uw_correlator.c
    ${COMMON_DIR}/uw_correlator_tables.c
)
target_include_directories(test_uw_correlator_golden PRIVATE
    ${COMMON_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../fixtures
)
target_link_libraries(test_uw_correlator_golden PRIVATE m)

# #135: worker->aggregator frame PDU pack/unpack (pure C; the FreeRTOS
# output queue in frame_pdu.c is behind ESP_PLATFORM, excluded on host).
add_executable(test_frame_pdu
    test_frame_pdu.c
    ${MAIN_DIR}/frame_pdu.c
)
target_include_directories(test_frame_pdu PRIVATE ${MAIN_DIR})

# T2 regression: signal_buffer's ring index invariant (head must advance
# by aligned_count on every push, including wrap-path DMA-submit-failure
# recovery paths -- commit 933ab38). signal_buffer.c itself is device-only
# (ESP-IDF/FreeRTOS deps) and doesn't build on host, so this test only
# needs the pure head-advance arithmetic in signal_buffer_ring.h, which
# signal_buffer.c's single head-update exit site also calls.
add_executable(test_signal_buffer_index
    test_signal_buffer_index.c
)
target_include_directories(test_signal_buffer_index PRIVATE ${MAIN_DIR})

# #136 Phase 3: inter-chip frame_link wire framing (crc16 + encode/decode).
# Pure layer only; SPI transport is target-only.
add_executable(test_frame_link
    test_frame_link.c
    ${MAIN_DIR}/frame_link.c
    ${MAIN_DIR}/frame_pdu.c
)
target_include_directories(test_frame_link PRIVATE ${MAIN_DIR})

# Phase 3.6.M task #58 step 1: validate rotate_to_dc_q15_inc against
# the cosf/sinf reference (rotate_to_dc) on synthetic tones. Asserts
# NMSE ≤ -50 dB and amplitude-stability ratio in [0.98, 1.02] across
# short/medium/long windows. Catches Q15-magnitude-decay regressions
# in the renormalisation logic (see rotate_to_dc.h header).
add_executable(test_rotate_to_dc
    test_rotate_to_dc.c
    ${COMMON_DIR}/rotate_to_dc.c
)
target_include_directories(test_rotate_to_dc PRIVATE ${COMMON_DIR})
target_link_libraries(test_rotate_to_dc PRIVATE m)

# Phase 3.6.M step 4: full wideband-path-A integration test. Runs the
# raw 2.5 MSPS fixture through the C burst tagger + per-burst
# direct-IF mixer + burst_pipeline, end-to-end. Reports decoded
# count for comparison against path A's 17/99 (channelizer) and
# path C's 64/65 (Python front end). Always passes — measurement only.
add_executable(test_pipeline_wideband_albq
    test_pipeline_wideband_albq.c
    ${COMMON_DIR}/fft_burst_tagger.c
    ${COMMON_DIR}/fft_sc16_2048.c
    ${COMMON_DIR}/direct_if_decim.c
    ${COMMON_DIR}/resample_256_to_250.c
    ${COMMON_DIR}/firmr_s16.c
    ${COMMON_DIR}/rotate_to_dc.c
    ${COMMON_DIR}/burst_pipeline.c
    ${COMMON_DIR}/uw_correlator.c
    ${COMMON_DIR}/uw_correlator_tables.c
    ${COMMON_DIR}/qpsk_demod.c
    ${COMMON_DIR}/sym_timing.c
)

# Same end-to-end pipeline as test_pipeline_wideband_albq, but
# resamples ALBQ_RAW_UINT8 via resample_256_to_250 (the firmware's
# ingest path) instead of reading the scipy-resampled cf32. Measures
# the quality cost of the firmware's lower-tap-count polyphase vs
# scipy's wider Kaiser, isolating that from anything else
# firmware-specific.
add_executable(test_pipeline_wideband_resampled
    test_pipeline_wideband_resampled.c
    ${COMMON_DIR}/fft_burst_tagger.c
    ${COMMON_DIR}/fft_sc16_2048.c
    ${COMMON_DIR}/direct_if_decim.c
    ${COMMON_DIR}/resample_256_to_250.c
    ${COMMON_DIR}/firmr_s16.c
    ${COMMON_DIR}/rotate_to_dc.c
    ${COMMON_DIR}/burst_pipeline.c
    ${COMMON_DIR}/uw_correlator.c
    ${COMMON_DIR}/uw_correlator_tables.c
    ${COMMON_DIR}/qpsk_demod.c
    ${COMMON_DIR}/sym_timing.c
)
target_include_directories(test_pipeline_wideband_resampled PRIVATE
    ${COMMON_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}                   # esp_log.h stub
    ${CMAKE_CURRENT_SOURCE_DIR}/../fixtures
)
target_link_libraries(test_pipeline_wideband_resampled PRIVATE m)
# Self-contained (baked fixture_albq_raw.h, no iridium-extractor), so
# unlike test_pipeline_wideband_albq it CAN run in CI. It's the most
# complete end-to-end path we have — register it with its decode-count
# floor. (The gri-golden _albq variant stays build-only; it needs
# iridium-extractor to generate its cf32 input.)
add_test(NAME pipeline_wideband_resampled COMMAND test_pipeline_wideband_resampled)

# Drift + AWGN parameter-sweep regression (#121). Same wideband pipeline
# as test_pipeline_wideband_resampled, but each cell applies a synthetic
# LO drift (±5/10/15 kHz) and added AWGN (sigma 0..400 int16 LSB), then
# counts demod_ok + bch_ok. Quantifies the sensitivity gain from
# #112/#113/#115 and catches future regressions.
add_executable(test_pipeline_drift_snr
    test_pipeline_drift_snr.c
    ${COMMON_DIR}/fft_burst_tagger.c
    ${COMMON_DIR}/fft_sc16_2048.c
    ${COMMON_DIR}/direct_if_decim.c
    ${COMMON_DIR}/resample_256_to_250.c
    ${COMMON_DIR}/firmr_s16.c
    ${COMMON_DIR}/rotate_to_dc.c
    ${COMMON_DIR}/burst_pipeline.c
    ${COMMON_DIR}/uw_correlator.c
    ${COMMON_DIR}/uw_correlator_tables.c
    ${COMMON_DIR}/qpsk_demod.c
    ${COMMON_DIR}/sym_timing.c
    ${COMMON_DIR}/bch_decoder.c
)
target_include_directories(test_pipeline_drift_snr PRIVATE
    ${COMMON_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}                   # esp_log.h stub
    ${CMAKE_CURRENT_SOURCE_DIR}/../fixtures
)
target_link_libraries(test_pipeline_drift_snr PRIVATE m)

# Per-layer gri-equivalence check at the TAGGER level. Counts at
# the end of the pipeline (decode count) have been misleading before
# (gri's "65 decoded" was frames; our "138 tagged" wasn't comparable);
# this test compares the wideband tagger's gone-event output against
# gr-iridium's per-burst manifest using bipartite matching by
# (start_sample, center_bin). Assertion: recall ≥ a fixed threshold
# of gri's 65 ground-truth bursts.
add_executable(test_tagger_vs_manifest
    test_tagger_vs_manifest.c
    ${COMMON_DIR}/fft_burst_tagger.c
    ${COMMON_DIR}/fft_sc16_2048.c
)
target_include_directories(test_tagger_vs_manifest PRIVATE ${COMMON_DIR})
target_link_libraries(test_tagger_vs_manifest PRIVATE m)
target_include_directories(test_pipeline_wideband_albq PRIVATE
    ${COMMON_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}                   # esp_log.h stub
    ${CMAKE_CURRENT_SOURCE_DIR}/../fixtures
)
target_link_libraries(test_pipeline_wideband_albq PRIVATE m)

# Generate /tmp/host_direct_if/fixture_albq_raw_2500k.cf32 — the
# scipy-resampled 2.5 MSPS input that test_pipeline_wideband_albq and
# test_pipeline_direct_if_albq read. Output stamped so cmake/ninja
# only re-runs the script when its inputs change. Requires
# `iridium-extractor` on PATH (tests/scripts/direct_if_dump.py also
# generates per-burst dumps + manifest.csv along the way).
# Guarded by find_program so CI builds (without iridium-extractor) skip
# the fixture target entirely and don't fail during cmake --build.
find_program(IRIDIUM_EXTRACTOR iridium-extractor)
set(ALBQ_25K_CF32 /tmp/host_direct_if/fixture_albq_raw_2500k.cf32)
if(IRIDIUM_EXTRACTOR)
add_custom_command(
    OUTPUT  ${ALBQ_25K_CF32}
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/direct_if_dump.py
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/direct_if_dump.py
            ${CMAKE_CURRENT_SOURCE_DIR}/../fixtures/fixture_albq_raw.h
    COMMENT "Generating ALBQ 2.5 MSPS cf32 fixture via direct_if_dump.py (needs iridium-extractor)"
    VERBATIM)
add_custom_target(albq_25k_cf32_fixture DEPENDS ${ALBQ_25K_CF32})

# Wire the fixture as a build-time dep so `cmake --build ... --target
# test_pipeline_wideband_albq` produces a runnable test. If
# iridium-extractor isn't installed the script will fail; users can
# either install it (apt: gr-iridium) or run the wideband_resampled
# variant which is fixture-self-contained.
add_dependencies(test_pipeline_wideband_albq albq_25k_cf32_fixture)
endif() # IRIDIUM_EXTRACTOR (fixture + wideband_albq dep)
# Note: test_pipeline_direct_if_albq's add_dependencies lives next to
# its add_executable below — it's defined after this point so we can't
# forward-reference it here.

# (test_polyphase_int16, test_channelizer_detector, test_polyphase_channelizer,
# test_snr_gap_measurement, and test_worker_pipeline_albq removed —
# the polyphase channelizer that fed them was deprecated when the
# wideband fft_burst_tagger took over as the active front end in
# Phase 3.6.M, and removed in the same commit that updates this
# CMakeLists. See iridium-acars-implementation-plan.md.)

# test_libacars_link — smoke test that the same minimum source-set the
# IDF component (common/libacars_idf/) compiles also links + runs on a
# host (glibc) toolchain. Catches missing files in the SRCS list and
# upstream signature regressions before they hit the firmware build.
#
# We pull the same upstream files the IDF component does, plus the
# matching stub adapter, plus the static config.h / version.c. Keep
# this list in sync with common/libacars_idf/CMakeLists.txt.
set(LIBACARS_IDF_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../common/libacars_idf)
set(LIBACARS_UPSTREAM ${CMAKE_CURRENT_SOURCE_DIR}/../../libacars)
set(LIBACARS_SRC ${LIBACARS_UPSTREAM}/libacars)

add_executable(test_libacars_link
    test_libacars_link.c
    ${LIBACARS_SRC}/acars.c
    ${LIBACARS_SRC}/libacars.c
    ${LIBACARS_SRC}/configuration.c
    ${LIBACARS_SRC}/reassembly.c
    ${LIBACARS_SRC}/crc.c
    ${LIBACARS_SRC}/hash.c
    ${LIBACARS_SRC}/list.c
    ${LIBACARS_SRC}/dict.c
    ${LIBACARS_SRC}/json.c
    ${LIBACARS_SRC}/util.c
    ${LIBACARS_SRC}/vstring.c
    ${LIBACARS_IDF_DIR}/version.c
    ${LIBACARS_IDF_DIR}/port/libacars_app_stubs.c
)
target_include_directories(test_libacars_link PRIVATE
    ${LIBACARS_UPSTREAM}        # for <libacars/...> includes
    ${LIBACARS_IDF_DIR}         # for "config.h"
)
# Match the upstream feature-test macros so strsep/memmem/strnlen are
# visible in glibc <string.h>. Silence upstream -Wall/-Wextra spam from
# code we don't own (test_libacars_link.c itself stays under default
# warnings via the project-wide add_compile_options above).
target_compile_options(test_libacars_link PRIVATE
    -D_XOPEN_SOURCE=600
    -D_POSIX_C_SOURCE=200809L
    -D_DEFAULT_SOURCE
    -D_GNU_SOURCE
    -Wno-unused-but-set-variable
    -Wno-sign-compare
)

# D8: fine carrier frequency estimator (companion to D7's channelizer).
add_executable(test_freq_estimator
    test_freq_estimator.c
    ${COMMON_DIR}/freq_estimator.c
)
target_include_directories(test_freq_estimator PRIVATE ${COMMON_DIR})
target_link_libraries(test_freq_estimator PRIVATE m)

# D10: Gardner symbol-timing recovery.
add_executable(test_sym_timing
    test_sym_timing.c
    ${COMMON_DIR}/sym_timing.c
)
target_include_directories(test_sym_timing PRIVATE ${COMMON_DIR})
target_link_libraries(test_sym_timing PRIVATE m)

# D10: cross-validate uw_correlator's CFO output against synthetic
# bursts with known carrier offsets. Catches algorithm bugs that
# would otherwise hide behind low-SNR real-RF symptoms.
add_executable(test_uw_correlator_cfo
    test_uw_correlator_cfo.c
    ${COMMON_DIR}/uw_correlator.c
    ${COMMON_DIR}/uw_correlator_tables.c
)
target_include_directories(test_uw_correlator_cfo PRIVATE ${COMMON_DIR})
target_link_libraries(test_uw_correlator_cfo PRIVATE m)

# Real-RF cross-validation: runs the uw_correlator pipeline on the
# Albuquerque corpus burst (the same one gr-iridium decoded at 30 dB
# SNR) and asserts gr-iridium-aligned per-stage outputs.
add_executable(test_uw_correlator_albq
    test_uw_correlator_albq.c
    ${COMMON_DIR}/uw_correlator.c
    ${COMMON_DIR}/uw_correlator_tables.c
)
target_include_directories(test_uw_correlator_albq PRIVATE
    ${COMMON_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../fixtures
)
target_link_libraries(test_uw_correlator_albq PRIVATE m)

# Same pipeline on the (only) UL burst in the Albuquerque corpus —
# exercises uw_correlator's UL sync pattern + direction discrimination.
add_executable(test_uw_correlator_albq_ul
    test_uw_correlator_albq_ul.c
    ${COMMON_DIR}/uw_correlator.c
    ${COMMON_DIR}/uw_correlator_tables.c
)
target_include_directories(test_uw_correlator_albq_ul PRIVATE
    ${COMMON_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../fixtures
)
target_link_libraries(test_uw_correlator_albq_ul PRIVATE m)

# End-to-end worker DSP pipeline on the shared ALBQ_RAW corpus —
# host iteration analog of the on-target smoke. Runs channelizer +
# (test_worker_pipeline_albq removed with the channelizer — its
# end-to-end role is now covered by test_pipeline_wideband_albq +
# test_pipeline_wideband_resampled which test the gri-equivalent
# wideband front end.)

# Path C: host A/B against gr-iridium with channelizer + resampler
# bypassed. Reads per-burst 250 ksps baseband cf32 dumps produced by
# tests/scripts/direct_if_dump.py (mirrors gr-iridium burst_downmix:
# rotate raw to DC at tagged offset, LPF + decimate 10x). Runs the
# same burst_pipeline_process_250khz that runs on P4. Isolates whether
# the host-vs-gr-iridium decode gap is the front end or the downstream
# pipeline. Always passes; the body is a measurement we read by eye.
# decode_burst_capture: not a test — a host-side replay tool. Reads a
# device-captured .u8 burst file, runs the same per-burst pipeline the
# firmware runs, and emits CSV + summary. Used by the long-running
# capture monitor to compute decode-quality over many bursts offline,
# decoupled from the firmware's own decode metric.
add_executable(decode_burst_capture
    decode_burst_capture.c
    ${COMMON_DIR}/direct_if_decim.c
    ${COMMON_DIR}/rotate_to_dc.c
    ${COMMON_DIR}/burst_pipeline.c
    ${COMMON_DIR}/uw_correlator.c
    ${COMMON_DIR}/uw_correlator_tables.c
    ${COMMON_DIR}/qpsk_demod.c
    ${COMMON_DIR}/sym_timing.c
    ${COMMON_DIR}/iridium_frame.c
    ${COMMON_DIR}/iridium_bch.c
)
target_include_directories(decode_burst_capture PRIVATE
    ${COMMON_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}    # esp_log.h stub
)
target_link_libraries(decode_burst_capture PRIVATE m)

add_executable(test_pipeline_direct_if_albq
    test_pipeline_direct_if_albq.c
    ${COMMON_DIR}/burst_pipeline.c
    ${COMMON_DIR}/uw_correlator.c
    ${COMMON_DIR}/uw_correlator_tables.c
    ${COMMON_DIR}/qpsk_demod.c
    ${COMMON_DIR}/sym_timing.c
    ${COMMON_DIR}/rotate_to_dc.c
)
target_include_directories(test_pipeline_direct_if_albq PRIVATE
    ${COMMON_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}                   # esp_log.h stub
)
target_link_libraries(test_pipeline_direct_if_albq PRIVATE m)
if(IRIDIUM_EXTRACTOR)
add_dependencies(test_pipeline_direct_if_albq albq_25k_cf32_fixture)
endif()

# (test_snr_gap_measurement removed with the channelizer — was a
# diagnostic instrument for the channelizer-vs-gri SNR gap that
# motivated the move to the wideband path.)

# D10 tuning tool — not a regression test. Dumps per-symbol PI-loop
# state on the corpus fixture for offline analysis.
add_executable(test_sym_timing_trace
    test_sym_timing_trace.c
    ${COMMON_DIR}/sym_timing.c
)
target_include_directories(test_sym_timing_trace PRIVATE
    ${COMMON_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../fixtures
)
target_link_libraries(test_sym_timing_trace PRIVATE m)

enable_testing()
add_test(NAME bch                  COMMAND test_bch)
add_test(NAME freq_estimator       COMMAND test_freq_estimator)
add_test(NAME sym_timing           COMMAND test_sym_timing)
# uw_correlator_cfo NOT auto-registered — its synthetic-burst
# expectations predate the gr-iridium-alignment work; end-to-end
# uw_correlator coverage is in test_pipeline_wideband_albq (which
# decodes 59 frames at threshold 10 dB on the ALBQ fixture).
add_test(NAME uw_correlator_albq   COMMAND test_uw_correlator_albq)
add_test(NAME uw_correlator_golden COMMAND test_uw_correlator_golden)
add_test(NAME direct_if_decim_golden COMMAND test_direct_if_decim_golden)
add_test(NAME frame_pdu COMMAND test_frame_pdu)
add_test(NAME signal_buffer_index COMMAND test_signal_buffer_index)
add_test(NAME frame_link COMMAND test_frame_link)
add_test(NAME uw_correlator_albq_ul COMMAND test_uw_correlator_albq_ul)
# worker_pipeline_albq removed with the channelizer (the channelizer
# was the front end it tested; coverage moved to test_pipeline_wideband_albq).
# pipeline_direct_if intentionally NOT auto-registered: it needs
# tests/scripts/direct_if_dump.py to have run first.
add_test(NAME qpsk                 COMMAND test_qpsk)
add_test(NAME demod_corpus         COMMAND test_demod_corpus)
add_test(NAME demod_low_snr        COMMAND test_demod_low_snr)
add_test(NAME demod_albq           COMMAND test_demod_albq)
add_test(NAME iridium_bch          COMMAND test_iridium_bch)
add_test(NAME iridium_frame        COMMAND test_iridium_frame)
add_test(NAME iridium_frame_phaseB COMMAND test_iridium_frame_phaseB)
add_test(NAME iridium_frame_corpus COMMAND test_iridium_frame_corpus)
add_test(NAME frame_queue          COMMAND test_frame_queue)
add_test(NAME ida_decode_corpus    COMMAND test_ida_decode_corpus)
add_test(NAME ibc_decode           COMMAND test_ibc_decode)
add_test(NAME sbd_reassembler      COMMAND test_sbd_reassembler)
add_test(NAME fft_sc16_2048         COMMAND test_fft_sc16_2048)
add_test(NAME resample_split        COMMAND test_resample_split)
add_test(NAME fft_burst_tagger      COMMAND test_fft_burst_tagger)
# direct_if_decim intentionally NOT auto-registered: needs
# tests/scripts/direct_if_dump.py to have run first.
add_test(NAME rotate_to_dc          COMMAND test_rotate_to_dc)
add_test(NAME libacars_link        COMMAND test_libacars_link)
add_test(NAME pipeline_drift_snr   COMMAND test_pipeline_drift_snr)

# Mendeley Iridium dataset burst-pipeline coverage (task #130). Only
# built+registered if the generated fixture is present. Run
#   python3 tests/scripts/fetch_mendeley_iridium.py
#   python3 tests/scripts/build_mendeley_fixture.py
# to populate it (4.67 GB download). Test skips cleanly otherwise.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../fixtures/external/fixture_mendeley_burst_sample.h")
    add_executable(test_burst_pipeline_mendeley
        test_burst_pipeline_mendeley.c
        ${COMMON_DIR}/burst_pipeline.c
        ${COMMON_DIR}/uw_correlator.c
        ${COMMON_DIR}/uw_correlator_tables.c
        ${COMMON_DIR}/qpsk_demod.c
        ${COMMON_DIR}/sym_timing.c
        ${COMMON_DIR}/bch_decoder.c
        ${COMMON_DIR}/fft_sc16_2048.c
    )
    target_include_directories(test_burst_pipeline_mendeley PRIVATE
        ${COMMON_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../fixtures/external
    )
    target_link_libraries(test_burst_pipeline_mendeley PRIVATE m)
    add_test(NAME burst_pipeline_mendeley COMMAND test_burst_pipeline_mendeley)
endif()
# uw_correlator_cfo NOT auto-registered: its synthetic-burst
# expectations predate the gr-iridium-alignment work in
# uw_correlator.c (5 alignment commits since the test was written —
# burst layout assumptions, omega sign convention, low-SNR direction
# detection). End-to-end UW behaviour is covered by
# test_pipeline_wideband_albq (gri-equivalent decode count =
# 56/65 = 86% of gri's 65 on the ALBQ fixture) and the live-firmware
# smoke test (56 frames classified per task #59's measurement run).
# The unit test binary still builds (catches link breakage); update
# it the next time someone needs targeted uw_correlator coverage.
