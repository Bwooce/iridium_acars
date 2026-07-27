# acars_feed — format one decoded ACARS message into airframes.io's wire
# JSON (dumpvdl2-native for VDL2, iridium-toolkit for Iridium). Pure C11,
# no ESP-IDF runtime deps, so the same source compiles on host (tests/host:
# test_airframes_fmt with byte-exact wire fixtures). The firmware links it
# via the p4-usb-host `main` REQUIRES; acars_push.c builds the af_msg_t and
# transmits the result.
idf_component_register(
    SRCS "airframes_fmt.c"
    INCLUDE_DIRS "."
)
