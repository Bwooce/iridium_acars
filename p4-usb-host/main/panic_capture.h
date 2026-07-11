// Remote crash forensics for a wireless deployment.
//
// The device panics on a fixed ~24.7-min uptime cadence outdoors, where there
// is no serial console to read the UART coredump and no flash coredump
// partition (adding one needs a full USB reflash). This module recovers the
// crash location anyway: a linker `--wrap` hook on esp_panic_handler() stashes
// the RISC-V exception frame (mepc / ra / sp / mcause / mtval / fault addr /
// reason) into an RTC_NOINIT variable, which survives the panic reboot. On the
// next boot, panic_capture_report() formats it for the (connectionless) iot_log
// so the crashing PC reaches us over the network. addr2line the mepc/ra offline
// against the matching .elf to name the function.
//
// The --wrap is wired in main/CMakeLists.txt (-Wl,--wrap=esp_panic_handler).

#pragma once

#include <stdbool.h>
#include <stddef.h>

// If the previous reboot was a panic captured by the wrap hook, format a
// one-line summary into `out` (mepc/ra/sp/mcause/mtval/addr/reason) and return
// true, CONSUMING the record so it is reported only once. Returns false when no
// panic is pending (clean boot, or already consumed). Safe to call early in
// boot; does not allocate.
bool panic_capture_report(char *out, size_t out_len);
