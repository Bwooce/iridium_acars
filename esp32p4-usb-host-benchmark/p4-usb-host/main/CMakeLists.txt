idf_component_register(SRCS "class_driver.c" "esp_libusb.c" "tuner_r82xx.c" "tuner_fc2580.c" "tuner_fc0013.c" "tuner_fc0012.c" "tuner_e4k.c" "librtlsdr.c" "usb_host_lib_main.c" "dsp_processor.c" "signal_buffer.c" "worker_core1.c" "ingest_core1.c" "qpsk_demod.c" "bch_decoder.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES usb esp_timer esp_ringbuf esp-dsp esp_driver_dma esp_mm)
set_source_files_properties(class_driver.c
    PROPERTIES COMPILE_FLAGS
    -Wno-restrict
)
set_source_files_properties(tuner_r82xx.c
    PROPERTIES COMPILE_FLAGS
    "-Wno-unused-function -Wno-unused-variable"
)