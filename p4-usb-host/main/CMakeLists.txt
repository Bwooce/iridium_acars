idf_component_register(SRCS "class_driver.c" "esp_libusb.c" "tuner_r82xx.c" "tuner_fc2580.c" "tuner_fc0013.c" "tuner_fc0012.c" "tuner_e4k.c" "librtlsdr.c" "usb_host_lib_main.c" "dsp_processor.c" "dsp_window_arp4.S" "signal_buffer.c" "worker_core1.c" "ingest_core1.c" "status_logger.c" "smoke_test.c"
                    INCLUDE_DIRS "." "../../tests/fixtures"
                    PRIV_REQUIRES usb esp_timer esp_ringbuf esp-dsp esp_driver_dma esp_mm
                                  iridium_decoder)
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