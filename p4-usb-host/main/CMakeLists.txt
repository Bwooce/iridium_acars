set(_app_srcs
    "class_driver.c" "esp_libusb.c" "tuner_r82xx.c" "tuner_fc2580.c"
    "tuner_fc0013.c" "tuner_fc0012.c" "tuner_e4k.c" "librtlsdr.c"
    "usb_host_lib_main.c" "c6_ota.c" "dsp_processor.c" "signal_buffer.c"
    "worker_core1.c" "ingest_core1.c" "usbring.c" "status_logger.c" "smoke_test.c"
    "frame_decoder.c" "pie_fft_diff_test.c" "app_config.c" "agc.c"
    "wifi_link.c" "http_server.c" "msg_ring.c" "captive_dns.c"
    "acars_push.c" "ota_runner.c" "serial_cmd.c" "fault_inject.c"
    "frame_pdu.c" "aggregator_ingest.c" "frame_link.c"
    "scanner.c" "scanner_map.c" "autotune.c" "autotune_sched.c"
    "panic_capture.c"
)
set(_app_reqs
    usb esp_timer esp_ringbuf esp-dsp esp_driver_dma esp_mm nvs_flash
    esp_wifi esp_netif esp_event esp_http_server esp_app_format
    esp_https_ota app_update lwip iridium_decoder
    # SDMMC + FATFS always declared so idf_component_register sees them
    # on first-pass CMake evaluation (CONFIG_* isn't readable until pass
    # two, but PRIV_REQUIRES is consumed on pass one). The runtime cost
    # comes from sd_log.c calling these APIs; that source file is what
    # the CONFIG_ENABLE_SD_LOG gate actually excludes.
    esp_driver_sdmmc fatfs esp_driver_gpio esp_driver_uart esp_iot_log
    # esp_system: esp_private/panic_internal.h (panic_info_t) for the
    # --wrap=esp_panic_handler crash-capture hook. riscv: RvExcFrame.
    esp_system riscv
)
# SD log subsystem (#63): opt-in via CONFIG_ENABLE_SD_LOG. Off by
# default. When off, sd_log.c isn't compiled and sd_log.h exposes
# its API as inline no-ops, so no SDMMC / FATFS runtime allocations
# happen even though the components are linked.
if(CONFIG_ENABLE_SD_LOG)
    list(APPEND _app_srcs "sd_log.c" "sd_capture.c")
endif()

idf_component_register(SRCS ${_app_srcs}
                    INCLUDE_DIRS "." "../../tests/fixtures"
                    PRIV_REQUIRES ${_app_reqs})

# Remote crash forensics (panic_capture.c): intercept esp_panic_handler() so we
# can stash the RISC-V exception frame to RTC memory before the reboot. --wrap
# redirects the call in esp_system's panic_handler.c to __wrap_esp_panic_handler
# and exposes the original as __real_esp_panic_handler.
target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=esp_panic_handler")
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