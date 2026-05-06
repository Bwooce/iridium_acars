/*
 * libacars_idf — static config.h replacement.
 *
 * Upstream libacars generates this file at CMake-configure time from
 * libacars/libacars/config.h.in. We're vendoring libacars as an ESP-IDF
 * component and don't run libacars's autoconf-style probe, so this file
 * is hand-written for the ESP-IDF v6.1 / RISC-V (P4) toolchain
 * (picolibc-based newlib).
 *
 * IMPORTANT: keep the macro names identical to those in
 * libacars/libacars/config.h.in — the upstream sources test these via
 * #ifdef. If you upgrade libacars and a new switch appears in
 * config.h.in, replicate it here.
 *
 *   WITH_ZLIB     — zlib not provided by ESP-IDF (esp_rom ships miniz,
 *                   different API). Leave undefined: la_inflate() is
 *                   compiled out and OHMA / MIAM-Core compressed payloads
 *                   are unsupported.
 *   WITH_LIBXML2  — not vendored. Leave undefined.
 *   WITH_JANSSON  — not vendored. Leave undefined.
 *   IS_BIG_ENDIAN — RISC-V is little-endian. Leave undefined.
 *
 *   HAVE_STRSEP   — picolibc provides strsep(3) in <string.h>.
 *   HAVE_MEMMEM   — picolibc provides memmem(3) in <string.h>.
 *   HAVE_SYS_TIME_H / HAVE_UNISTD_H — both present in newlib/picolibc.
 *
 *   LFIND_NMEMB_SIZE_SIZE_T — only used by asn1-util.c which is not
 *   compiled in this minimum build. Define it for safety in case the
 *   ASN.1 sources are added later.
 */
#ifndef _CONFIG_H
#define _CONFIG_H

/* Optional features — all OFF for the embedded build. */
/* #undef WITH_ZLIB    */
/* #undef WITH_LIBXML2 */
/* #undef WITH_JANSSON */

/* Endianness — P4 is little-endian. */
/* #undef IS_BIG_ENDIAN */

/* libc capability flags — picolibc/newlib on the riscv32-esp-elf
 * toolchain supplies all of these. */
#define HAVE_STRSEP
#define HAVE_MEMMEM
#define HAVE_SYS_TIME_H
#define HAVE_UNISTD_H

/* Only used by ASN.1 code path (not compiled here). Set the size_t
 * variant to match the POSIX prototype. */
#define LFIND_NMEMB_SIZE_SIZE_T

#endif /* !_CONFIG_H */
