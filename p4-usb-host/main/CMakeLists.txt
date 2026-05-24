set(_app_srcs
    "class_driver.c" "esp_libusb.c" "tuner_r82xx.c" "tuner_fc2580.c"
    "tuner_fc0013.c" "tuner_fc0012.c" "tuner_e4k.c" "librtlsdr.c"
    "usb_host_lib_main.c" "dsp_processor.c" "signal_buffer.c"
    "worker_core1.c" "ingest_core1.c" "status_logger.c" "smoke_test.c"
    "frame_decoder.c" "pie_fft_diff_test.c" "app_config.c" "agc.c"
    "wifi_link.c" "http_server.c" "msg_ring.c" "captive_dns.c"
    "acars_push.c" "ota_runner.c"
)
set(_app_reqs
    usb esp_timer esp_ringbuf esp-dsp esp_driver_dma esp_mm nvs_flash
    esp_wifi esp_netif esp_event esp_http_server esp_app_format
    esp_https_ota app_update lwip iridium_decoder
)
# SD log subsystem (#63) is opt-in: linking SDMMC + FATFS pulls
# ~50 KB of internal-SRAM reservations that compete with USB DMA
# and break live-SDR throughput. Off by default; flip the
# CONFIG_ENABLE_SD_LOG menuconfig option to include it.
if(CONFIG_ENABLE_SD_LOG)
    list(APPEND _app_srcs "sd_log.c")
    list(APPEND _app_reqs esp_driver_sdmmc fatfs esp_driver_gpio)
endif()

idf_component_register(SRCS ${_app_srcs}
                    INCLUDE_DIRS "." "../../tests/fixtures"
                    PRIV_REQUIRES ${_app_reqs})
set_source_files_properties(class_driver.c
    PROPERTIES COMPILE_FLAGS
    -Wno-restrict
)
set_source_files_properties(tuner_r82xx.c
    PROPERTIES COMPILE_FLAGS
    "-Wno-unused-function -Wno-unused-variable"
)

# Push the hot DSP files past the global -O2 to -O3 with loop unrolling.
# These files contain the per-frame inner loops (window, magnitude squared,
# baseline EMA, convert, ingest) — squeezing them harder than the rest of
# the codebase is fine because they're well-isolated and we have functional
# regression tests (smoke_test, host bch/qpsk) to gate the more aggressive
# scheduling.
set_source_files_properties(dsp_processor.c ingest_core1.c
    PROPERTIES COMPILE_FLAGS
    "-O3 -funroll-loops"
)