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

enable_testing()
add_test(NAME bch            COMMAND test_bch)
add_test(NAME qpsk           COMMAND test_qpsk)
add_test(NAME demod_corpus   COMMAND test_demod_corpus)
add_test(NAME demod_low_snr  COMMAND test_demod_low_snr)
