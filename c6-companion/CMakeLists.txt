# ESP32-C6 companion firmware (D17).
# Standalone IDF project — does NOT use esp_hosted. Runs the
# Wi-Fi stack natively and serves a web UI of decoded ACARS
# messages received from the P4 over UART.
#
# Build/flash via the same vendored IDF:
#   cd c6-companion
#   idf.py -DIDF_TARGET=esp32c6 build
#   idf.py -DIDF_TARGET=esp32c6 -p /dev/ttyACMX flash monitor

cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(c6-companion)
