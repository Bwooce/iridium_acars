// See panic_capture.h.

#include "panic_capture.h"

#include "esp_attr.h"                   // RTC_NOINIT_ATTR
#include "esp_private/panic_internal.h" // panic_info_t
#include "riscv/rvruntime-frames.h"     // RvExcFrame (mepc/ra/...)
#include "freertos/FreeRTOS.h"          // pcTaskGetName — which task crashed
#include "freertos/task.h"

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
    uint32_t is_abort; // 1 if this was an abort()/assert (see desc)
    char     task[16]; // name of the task that was running when it crashed
    // For an abort()/assert, the real message (incl. "assert failed: file:line
    // (expr)") lives in g_panic_abort_details, NOT info->description/reason —
    // those are NULL'd inside esp_panic_handler, which runs AFTER our --wrap.
    // Capture the abort details directly; fall back to the exception
    // description otherwise. Wide enough for a typical assert string.
    char desc[112];
} panic_cap_t;

// RTC_NOINIT: preserved across a SW/panic reset, NOT zeroed by the C runtime.
static RTC_NOINIT_ATTR panic_cap_t s_cap;

// The genuine handler, resolved by the linker's -Wl,--wrap=esp_panic_handler.
extern void __real_esp_panic_handler(panic_info_t *info);

// IDF panic.c globals: set by panic_abort() before it traps. g_panic_abort is
// true when the panic is an abort()/assert; g_panic_abort_details is its
// message (e.g. "assert failed: <func> <file>:<line> (<expr>)").
extern bool  g_panic_abort;
extern char *g_panic_abort_details;

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
    s_cap.is_abort      = g_panic_abort ? 1u : 0u;

    // Prefer the abort details (the real assert message) when this is an abort;
    // else the exception description/reason.
    const char *d = (g_panic_abort && g_panic_abort_details) ? g_panic_abort_details
                    : (info && info->description)            ? info->description
                    : (info && info->reason)                 ? info->reason
                                                             : "?";
    size_t      i = 0;
    for (; i < sizeof(s_cap.desc) - 1 && d[i]; i++) {
        s_cap.desc[i] = d[i];
    }
    s_cap.desc[i] = '\0';

    // Which task was running on the faulting core. pcTaskGetName(NULL) reads
    // the current TCB's name (same thing IDF's own panic printout does) — the
    // single most useful identifier for "who did this". Guard against a NULL
    // return (scheduler not started).
    const char *tn = pcTaskGetName(NULL);
    size_t      j  = 0;
    for (; tn && j < sizeof(s_cap.task) - 1 && tn[j]; j++) {
        s_cap.task[j] = tn[j];
    }
    s_cap.task[j] = '\0';

    s_cap.magic = PANIC_CAP_MAGIC; // commit last: record is now valid to read

    __real_esp_panic_handler(info); // reboots; never returns
}

bool panic_capture_report(char *out, size_t out_len)
{
    if (s_cap.magic != PANIC_CAP_MAGIC) {
        return false;
    }
    s_cap.magic = 0; // consume: emit the record once, not every window

    // Registers FIRST, the long (truncatable) abort string LAST: the iot_log
    // emission clips the line at a fixed length, and the coproc trap-storm
    // abort message is long enough to eat mepc/mtval off the end. Ordering the
    // hex registers up front guarantees they survive the clip (they're the
    // datum that discriminates the FPU-vs-PIE EXT_ILL misdispatch — mepc =
    // faulting instruction, mtval = its raw encoding).
    snprintf(out, out_len,
             "PANIC-BT core=%ld task=%s mepc=0x%08lx ra=0x%08lx sp=0x%08lx "
             "mcause=%lu mtval=0x%08lx addr=0x%08lx %s=\"%s\"",
             (long)s_cap.core, s_cap.task,
             (unsigned long)s_cap.mepc, (unsigned long)s_cap.ra,
             (unsigned long)s_cap.sp, (unsigned long)s_cap.mcause,
             (unsigned long)s_cap.mtval, (unsigned long)s_cap.addr,
             s_cap.is_abort ? "abort" : "reason", s_cap.desc);
    return true;
}
