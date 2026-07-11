// See panic_capture.h.

#include "panic_capture.h"

#include "esp_attr.h"                   // RTC_NOINIT_ATTR
#include "esp_private/panic_internal.h" // panic_info_t
#include "riscv/rvruntime-frames.h"     // RvExcFrame (mepc/ra/...)

#include <stdint.h>
#include <stdio.h>

// "PAN1" — distinguishes a real captured panic from the random contents an
// RTC_NOINIT variable holds after a cold power-on (1-in-4-billion false match).
#define PANIC_CAP_MAGIC 0x50414E31u

typedef struct {
    uint32_t magic;
    uint32_t mepc; // PC at the exception (the crashing instruction)
    uint32_t ra;   // return address (its caller — frame 2 of the backtrace)
    uint32_t sp;
    uint32_t mcause; // RISC-V trap cause
    uint32_t mtval;  // faulting address / bad value
    uint32_t addr;   // panic_info_t.addr (IDF's computed fault instr addr)
    int32_t  core;
    char     desc[32]; // panic_info_t.description, e.g. "Load access fault"
} panic_cap_t;

// RTC_NOINIT: preserved across a SW/panic reset, NOT zeroed by the C runtime.
static RTC_NOINIT_ATTR panic_cap_t s_cap;

// The genuine handler, resolved by the linker's -Wl,--wrap=esp_panic_handler.
extern void __real_esp_panic_handler(panic_info_t *info);

// Runs in panic context: interrupts off, minimal stack, no heap/locks. Keep it
// to trivial reads of the already-valid frame struct and fixed RTC writes — no
// function calls (beyond __real) and no dereferencing of arbitrary stack, so it
// cannot itself double-fault and clobber the very record we are trying to save.
void __wrap_esp_panic_handler(panic_info_t *info)
{
    s_cap.magic = 0; // invalidate up front, before we start writing fields

    const RvExcFrame *f = info ? (const RvExcFrame *)info->frame : NULL;
    s_cap.mepc          = f ? (uint32_t)f->mepc : 0;
    s_cap.ra            = f ? (uint32_t)f->ra : 0;
    s_cap.sp            = f ? (uint32_t)f->sp : 0;
    s_cap.mcause        = f ? (uint32_t)f->mcause : 0;
    s_cap.mtval         = f ? (uint32_t)f->mtval : 0;
    s_cap.addr          = info ? (uint32_t)info->addr : 0;
    s_cap.core          = info ? info->core : -1;

    const char *d = (info && info->description) ? info->description
                                                : ((info && info->reason) ? info->reason : "?");
    size_t      i = 0;
    for (; i < sizeof(s_cap.desc) - 1 && d[i]; i++) {
        s_cap.desc[i] = d[i];
    }
    s_cap.desc[i] = '\0';

    s_cap.magic = PANIC_CAP_MAGIC; // commit last: record is now valid to read

    __real_esp_panic_handler(info); // reboots; never returns
}

bool panic_capture_report(char *out, size_t out_len)
{
    if (s_cap.magic != PANIC_CAP_MAGIC) {
        return false;
    }
    s_cap.magic = 0; // consume: emit the record once, not every window

    snprintf(out, out_len,
             "PANIC-BT core=%ld reason=%s mepc=0x%08lx ra=0x%08lx sp=0x%08lx "
             "mcause=%lu mtval=0x%08lx addr=0x%08lx",
             (long)s_cap.core, s_cap.desc,
             (unsigned long)s_cap.mepc, (unsigned long)s_cap.ra,
             (unsigned long)s_cap.sp, (unsigned long)s_cap.mcause,
             (unsigned long)s_cap.mtval, (unsigned long)s_cap.addr);
    return true;
}
