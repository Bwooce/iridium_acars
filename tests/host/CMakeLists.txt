cmake_minimum_required(VERSION 3.16)
project(p4_usb_host_tests C)

set(CMAKE_C_STANDARD 11)
add_compile_options(-Wall -Wextra -Wno-unused-parameter -O2 -g)

# Pull production sources directly from main/ so the host tests exercise
# the exact code that runs on the target. No file moves required.
set(MAIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../p4-usb-host/main)

# bch_decoder is pure C with no IDF deps — compile it as-is.
add_executable(test_bch
    test_bch.c
    ${MAIN_DIR}/bch_decoder.c
)
target_include_directories(test_bch PRIVATE ${MAIN_DIR})

# qpsk_demod includes "esp_log.h"; tests/host/esp_log.h shadows it on
# the host with a no-op stub. Adding tests/host/ first on the include
# path makes the host build pick up our stub without touching the
# production source.
add_executable(test_qpsk
    test_qpsk.c
    ${MAIN_DIR}/qpsk_demod.c
)
target_include_directories(test_qpsk PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}   # esp_log.h stub
    ${MAIN_DIR}
)
target_link_libraries(test_qpsk PRIVATE m)

add_executable(test_demod_corpus
    test_demod_corpus.c
    ${MAIN_DIR}/qpsk_demod.c
)
target_include_directories(test_demod_corpus PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}                   # esp_log.h stub
    ${CMAKE_CURRENT_SOURCE_DIR}/../fixtures       # generated headers
    ${MAIN_DIR}
)
target_link_libraries(test_demod_corpus PRIVATE m)

enable_testing()
add_test(NAME bch           COMMAND test_bch)
add_test(NAME qpsk          COMMAND test_qpsk)
add_test(NAME demod_corpus  COMMAND test_demod_corpus)
