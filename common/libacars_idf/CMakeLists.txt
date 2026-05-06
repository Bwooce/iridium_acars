# libacars_idf — ESP-IDF component vendoring libacars 2.2.1 for the
# ESP32-P4 firmware.
#
# We compile only the core ACARS frame-parser from the upstream tree at
# ../../libacars/ (relative to this file: <repo>/libacars/). The
# heavyweight sub-decoders (CPDLC ASN.1, ADS-C, MIAM, OHMA, ARINC-622,
# media-adv) are NOT compiled — they would pull in ~242 ASN.1 source
# files plus zlib/jansson/libxml2 and aren't needed for plain-text
# Iridium SBD ACARS messages. Their entry-point symbols are resolved
# via stubs in port/libacars_app_stubs.c.
#
# config.h is provided as a static replacement for libacars's
# autoconf-style config.h.in (see config.h for the rationale).
# version.c is provided as a static replacement for libacars's
# git-describe-driven version.cmake (see version.c).
#
# Adding a sub-decoder later: drop the upstream .c into the SRCS list,
# delete the matching stub from libacars_app_stubs.c, and add any
# transitive deps it needs (e.g. enabling MIAM/OHMA needs zlib).

# Resolve the upstream tree once. Depth: <repo>/common/libacars_idf →
# <repo>/libacars (../../libacars).
set(LA_UPSTREAM "${CMAKE_CURRENT_LIST_DIR}/../../libacars")
set(LA_SRC      "${LA_UPSTREAM}/libacars")

# Source files from the vendored upstream that we compile.
set(LA_VENDORED_SRCS
    "${LA_SRC}/acars.c"          # core ACARS frame parser
    "${LA_SRC}/libacars.c"       # la_proto_node tree, la_proto_tree_*
    "${LA_SRC}/configuration.c"  # la_config_get_int / "acars_bearer"
    "${LA_SRC}/reassembly.c"     # multi-block message reassembly
    "${LA_SRC}/crc.c"            # la_crc16_ccitt
    "${LA_SRC}/hash.c"           # la_hash_string (used by acars key hash)
    "${LA_SRC}/list.c"           # la_list — needed by hash.c / reassembly.c
    "${LA_SRC}/dict.c"           # la_dict — reassembly status names
    "${LA_SRC}/json.c"           # la_json_append_* (la_acars_format_json)
    "${LA_SRC}/util.c"           # LA_XCALLOC, la_base64_decode, ...
    "${LA_SRC}/vstring.c"        # la_vstring
)

idf_component_register(
    SRCS
        ${LA_VENDORED_SRCS}
        # --- Generated-equivalents we provide as static files ---
        "version.c"
        # --- Stubs for sub-decoders we deliberately do NOT compile ---
        "port/libacars_app_stubs.c"
    INCLUDE_DIRS
        # Public: client code does `#include <libacars/acars.h>`.
        # Headers live at ${LA_UPSTREAM}/libacars/*.h, so adding
        # ${LA_UPSTREAM} (the repo root) makes "<libacars/...>" resolve.
        "${LA_UPSTREAM}"
        # Public so this component's own config.h is reachable via
        # "config.h" from libacars sources, AND so any client of this
        # component can include local helper headers if added later.
        "."
)

# libacars upstream targets C11 (CMAKE_C_STANDARD 11 in its top-level
# CMakeLists.txt). ESP-IDF v6.1 defaults to -std=gnu23, which is a hard
# break for libacars: it uses pre-C23 empty-parens function-pointer
# declarations like `void (*node_free)()` to mean "any args" (e.g.
# list.h's la_list_free_full prototype), but C23 reinterprets `()` as
# "no args" — making every call site a -Wincompatible-pointer-types
# error under IDF's -Werror.
#
# We override the language standard on upstream sources only by
# re-appending -std=gnu11 (later -std= wins on the gcc command line).
# Our adapter files (port/, version.c) inherit the default gnu23 and
# therefore stay aligned with the rest of the firmware.
#
# We also replicate the libacars feature-test macros (so picolibc
# exposes strsep/memmem/strnlen) and silence -Wall/-Wextra noise from
# code we don't own; our adapter files keep the default warnings so we
# still police bugs we introduce there.
#
# Apply via COMPILE_FLAGS as a single string to avoid CMake's split
# behaviour between COMPILE_FLAGS / COMPILE_OPTIONS on the same source
# file (only one takes effect).
set(_la_compile_flags
    "-std=gnu11 \
     -D_XOPEN_SOURCE=600 \
     -D_POSIX_C_SOURCE=200809L \
     -D_DEFAULT_SOURCE \
     -D_GNU_SOURCE \
     -Wno-unused-parameter \
     -Wno-unused-but-set-variable \
     -Wno-sign-compare \
     -Wno-unused-variable")

foreach(_src IN LISTS LA_VENDORED_SRCS)
    set_source_files_properties(${_src}
        PROPERTIES COMPILE_FLAGS "${_la_compile_flags}")
endforeach()
