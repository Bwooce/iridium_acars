cmake_minimum_required(VERSION 3.16)
project(p4_usb_host_tests C)

set(CMAKE_C_STANDARD 11)
add_compile_options(-Wall -Wextra -Wno-unused-parameter -O2 -g)

# Pull production sources directly from common/iridium_decoder/ so the
# host tests exercise the exact code that runs on the target. The
# decoder lives in common/ so it can be shared with the future worker
# P4 firmware; the tests don't need to know about that layout — they
# just compile the same files with a host toolchain.
set(COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../common/iridium_decoder)

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
)
target_include_directories(test_qpsk PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}   # esp_log.h stub
    ${COMMON_DIR}
)
target_link_libraries(test_qpsk PRIVATE m)

add_executable(test_demod_corpus
    test_demod_corpus.c
    ${COMMON_DIR}/qpsk_demod.c
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

# Phase D part 5: SBD reassembler (multi-frame state machine).
add_executable(test_sbd_reassembler
    test_sbd_reassembler.c
    ${COMMON_DIR}/sbd_reassembler.c
)
target_include_directories(test_sbd_reassembler PRIVATE ${COMMON_DIR})

# Phase D7: polyphase channelizer (single-FFT detector replacement).
add_executable(test_polyphase_channelizer
    test_polyphase_channelizer.c
    ${COMMON_DIR}/polyphase_channelizer.c
)
target_include_directories(test_polyphase_channelizer PRIVATE
    ${COMMON_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../fixtures
)
target_link_libraries(test_polyphase_channelizer PRIVATE m)

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

enable_testing()
add_test(NAME bch                  COMMAND test_bch)
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
add_test(NAME sbd_reassembler      COMMAND test_sbd_reassembler)
add_test(NAME polyphase_channelizer COMMAND test_polyphase_channelizer)
add_test(NAME libacars_link        COMMAND test_libacars_link)
