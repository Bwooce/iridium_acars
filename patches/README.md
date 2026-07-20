# Vendored-code patches

Patches against gitignored vendored code: the vendored ESP-IDF
(`esp-idf/`) and component-manager downloads
(`p4-usb-host/managed_components/`).

`libacars/` is a different case: it's a plain **tracked** copy of upstream
libacars 2.2.1 (no submodule, not gitignored), so there's no apply/patch
step -- edits just live in the tree. The no-editing-vendored-.c policy
still applies by default; deliberate exceptions are recorded below.

IDF patches — apply after cloning / updating the IDF:

```sh
cd esp-idf
git apply ../patches/0001-esp_dma_utils-defer-stash-alloc-until-overflow-confirmed.patch
git apply ../patches/0003-freertos-riscv-coproc-save-area-in-internal-ram-for-psram-stacks.patch
git apply ../patches/0004-freertos-riscv-pie-coproc-trap-storm-watchdog.patch   # apply AFTER 0003
git apply ../patches/0007-bootloader_support-invalidate-mmap-cache-before-app-ota-verify.patch
git apply ../patches/0008-fatfs-enable-exfat.patch
git apply ../patches/0009-freertos-riscv-pie-coproc-trap-storm-recovery.patch   # apply AFTER 0004
git apply ../patches/0011-freertos-riscv-eager-coproc-enable-plus-recovery-counters.patch   # apply AFTER 0009
```

**0005 and 0006 were FALSIFIED and REMOVED (2026-07-19).** Both edited the
`pie_save_regs`/`pie_restore_regs` macros as *save/restore-side* attempts to fix
the coproc trap storm; both were bench-tested and STILL wedged (HP-WDT), so
neither is the fix. The production PIE gate is **0002+0003+0004+0009** — 0009
(recovery) is the real fix. The .patch files were deleted; see git history and
project_pie_save_deadlock_smoke for the falsification record.

Managed-component patches — apply from the repo root after the
component manager has populated `managed_components/` (first
`idf.py reconfigure` / `scripts/build.sh`), and re-apply whenever the
directory is regenerated (fresh checkout/worktree, deleted
`managed_components/`, or `idf.py update-dependencies`):

```sh
git apply --directory=p4-usb-host/managed_components/espressif__esp-dsp \
    patches/0002-esp-dsp-replace-hwlp-with-counter-loops-in-pie-kernels.patch
```

A normal `idf.py reconfigure` does NOT clobber or reject the patched
files (verified on IDF v6.1: the component manager leaves modified
managed components in place as long as `dependencies.lock` is
satisfied). Check the patch is live with
`grep -rl "iridium_acars patch" p4-usb-host/managed_components/espressif__esp-dsp/`.

## 0001 — async_memcpy_gdma: place link list descriptors in PSRAM

**File:** `components/esp_driver_dma/src/async_memcpy_gdma.c`
**IDF version:** v6.1 — vendored checkout tracks `release/v6.1` (was master snapshot; upgraded 2026-07-04). Re-verify this patch applies on IDF updates.

`gdma_new_link_list` was called with `items_in_ext_mem = false`, placing
GDMA descriptor arrays in the DMA-INT heap (`MALLOC_CAP_DMA |
MALLOC_CAP_INTERNAL`). That heap is only ~5 KB on ESP32-P4. Combined
with the unconditional stash buffer allocation in `esp_dma_split_rx_buffer_to_cache_aligned`
(128 bytes per call, also from DMA-INT), the heap was exhausted and
fragmented after ~6 hours at 625 DMA transfers/sec, producing
`stash_alloc_fails` and `audio_dropped` events in `signal_buffer.c`.

Fix: set `items_in_ext_mem = true` for both TX and RX link lists.
The ESP32-P4's AXI-GDMA can fetch descriptors from PSRAM over the AXI
bus — that is the design intent of the AXI-GDMA (vs the AHB-GDMA which
is SRAM-only). The IDF code already has the full SPIRAM allocation +
L2-cache-sync + non-cached-address path for this case; the `false`
setting was a conservative TODO left by Espressif.

This moves descriptor arrays to PSRAM (unlimited headroom), leaving
the 5 KB DMA-INT heap exclusively for the stash buffer (128 bytes per
unaligned transfer, rare for our cache-line-aligned paths).

## 0002 — esp-dsp: replace HWLP (`esp.lp.setup`) with counter loops in PIE kernels

**Files:** `modules/fir/fixed/dsps_fird_s16_arp4.S`,
`modules/fft/fixed/dsps_fft2r_sc16_arp4.S`,
`modules/fft/float/dsps_fft2r_fc32_arp4.S`
**Component version:** espressif/esp-dsp 1.8.1 (pinned in
`p4-usb-host/dependencies.lock`). Re-verify on component upgrades.

Production firmware crash-looped with Core-1 Illegal-instruction
panics, always at `dsps_fird_s16_arp4.S:123` (`.skip_main_loop3`),
called from `uw_correlator_find_burst_start` (`uw_correlator.c`).
Mechanism: these kernels use `esp.lp.setup` zero-overhead hardware
loops (HWLP). FreeRTOS on P4 saves the PIE coprocessor lazily — the
first PIE instruction after a task switch traps as an illegal
instruction and `rtos_save_pie_coproc` runs from the trap handler.
Two P4 silicon bugs around HWLP state
(`SOC_CPU_HAS_HWLOOP_STATE_BUG` plus a spurious EXT_ILL reason bit;
see esp-idf `vectors.S:301` and `portasm.S:217/795`) make that
interaction unreliable when armed HWLP state exists at the switch,
and the trap escalates to a panic. See
`docs/superpowers/ANALYSIS-2026-07-06-path-to-first-acars.md` (P2).

Fix: replace all six `esp.lp.setup` loops (2 in fird_s16, 3 in
fft2r_sc16, 1 in fft2r_fc32) with standard RISC-V `addi`/`bnez`
counter loops, preserving the PIE vector instruction sequence inside
each loop body bit-for-bit. With no HWLP state ever armed, the lazy
PIE save can never hit the silicon bug. Cost: +2 scalar instructions
per loop iteration (~5-15 % on these kernels; see commit message).

These are the only three `_arp4` kernels linked into the firmware
(checked with `nm` on the ELF), and none of our own hand-written PIE
asm (`resample_arp4`, `rotate_q15`) uses HWLP, so the patched image
contains zero `esp.lp.setup` instructions.

NOTE: this changes device-DSP behavior but lives outside the
pre-push hook's `DSP_PATHS` list — a device smoke run
(`Smoke-verified:` evidence) is still mandatory before trusting it.

## 0003 — freertos/riscv: coprocessor save areas in internal RAM for PSRAM stacks

**Files:** `components/freertos/FreeRTOS-Kernel/portable/riscv/port.c`,
`components/riscv/include/riscv/rvruntime-frames.h`
**IDF version:** v6.1 (vendored `release/v6.1` checkout). Re-verify on IDF updates.

FreeRTOS on the P4 saves coprocessor (FPU/HWLP/PIE) contexts lazily: the
first coprocessor instruction after losing ownership traps as an illegal
instruction, and `rtos_save_pie_coproc` (portasm.S) saves the previous
owner's PIE registers into a save area that `pxPortGetCoprocArea`
(port.c) carves from the *owner task's stack bottom*. Our PIE-owning
tasks (`worker_core1`, `rs_worker_a/b`) have PSRAM stacks
(`xTaskCreatePinnedToCoreWithCaps(..., MALLOC_CAP_SPIRAM)`), so the PIE
context was saved with `esp.vst.128.ip` 128-bit vector stores INTO
PSRAM — and this project has repeatedly documented PIE 128-bit ld/st
misbehaving depending on target memory region (`fft_sc16_2048.c:51`,
`uw_correlator.c:460`). This is the surviving mechanism for the
device-smoke RAW GOLDEN HP-WDT hang at the PIE lazy-save path after
HWLP removal (patch 0002) and the rev-0 silicon workarounds both failed
to fix it.

Fix: when a task's stack is in external RAM
(`esp_ptr_external_ram`), pre-allocate a small internal-RAM pool
(`heap_caps_aligned_alloc(16, ~400 B, MALLOC_CAP_INTERNAL)`) at task
creation (`uxInitialiseCoprocSaveArea`, called from
`pxPortInitialiseStack`) and let the lazy carve use that pool instead of
the stack. The allocation MUST be eager: the lazy carve runs inside the
illegal-instruction trap with interrupts masked, where the heap is not
usable. The pool is sized for all three coprocessor areas (FPU 132 B +
HWLP 24 B + PIE 216 B + per-area alignment) so it can never overflow.
Freed in `vPortCleanUpCoprocArea` (task-context TCB cleanup). On
allocation failure it falls back to the previous stack-carving behaviour
with a one-shot warning. Tasks with internal-RAM stacks are unaffected.

The new `sa_intpool` field is appended to `RvCoprocSaveArea`; portasm.S
only reads the `RV_COPROC_ENABLE`/`RV_COPROC_SA` offsets, which are
unchanged.

Verification: apply on the bench checkout, rebuild + flash, then
`scripts/smoke_run.sh raw` must complete (no HP-WDT hang) with GOLDEN
matched>=40.

## 0004 — freertos/riscv: PIE/coprocessor lazy-save trap-storm watchdog

**Files:** `components/freertos/FreeRTOS-Kernel/portable/riscv/port.c`,
`components/freertos/FreeRTOS-Kernel/portable/riscv/portasm.S`
**IDF version:** v6.1 (vendored `release/v6.1` checkout). Re-verify on IDF
updates. **Apply AFTER 0003** (both patches touch `port.c`; 0004 is diffed
against the 0003-applied tree).

The FreeRTOS-P4 coprocessor context switch is lazy: the first coprocessor
instruction a task runs after losing ownership traps as an illegal
instruction and `rtos_save_<name>_coproc` (portasm.S) runs from the trap
handler, with interrupts masked, to swap the owner and save/restore the
128-bit PIE (or FPU/DSP) context. That routine contains **no loop**, so a
program counter parked there is never a spin — it is a *re-entry storm*:
the retried instruction `mret`s and immediately re-faults, forever, tick
starved, until the hardware watchdog resets the chip (`rst:0x7
HP_SYS_HP_WDT_RESET`, `Core1 Saved PC = rtos_save_pie_coproc`). This is the
deterministic device-smoke RAW/REAL wedge. It was localised this cycle
(H-A) to the PIE **owner-swap** save/restore body: forcing a single PIE
owner per core (see `CONFIG_DIAG_SINGLE_PIE_OWNER`) makes the wedge vanish,
where every prior run wedged at the 2nd burst callback. Patches 0002
(HWLP removal), REV_MIN=0 and 0003 (save areas in internal RAM) each
addressed a plausible mechanism but the smoke hang survived all of them.

Fix: a bounded re-entry watchdog. `xPortCoprocTrapStormCheck(frame,
coproc)` is called at the top of every `rtos_save_<name>_coproc` (one
insertion in the shared `generate_coprocessor_routine` macro; `sp` still
holds the `RvExcFrame` and `ra` is preserved in `s0`, so calling C is
safe). It records the faulting `mepc` per core/coproc; when the SAME PC
re-faults `COPROC_TRAP_STORM_LIMIT` (16) times inside a short cycle-count
window (so a hot call site legitimately re-trapping across unrelated
context switches — ms apart, with forward progress in between — cannot
trip it), it prints MEPC/MTVAL/coproc/owner with the panic-safe
`esp_rom_printf` and simulates an abort exactly like `vPortCoprocUsedInISR`
(interrupts are masked, so `abort()` cannot be used).

This is a diagnostic net that **ships regardless** of whether the root
save/restore fault is fully cured: it converts a silent HP-WDT wedge into
an attributable panic + clean recoverable reboot (production strictly
safer) and, on the bench, prints the exact re-faulting instruction that
identifies the culprit. A non-buggy coprocessor never re-faults the same
PC without retiring it, so the guard cannot false-positive; overhead is a
handful of instructions on the already-heavy lazy-save path.

Verification: apply on the bench checkout (after 0003), rebuild with
`SMOKE_TEST_RAW`, flash, `scripts/smoke_run.sh raw`. Expected: either a
clean GOLDEN pass (`matched>=40`, no HP-WDT — root fault cured) **or** an
attributable `Coprocessor lazy-save trap storm` panic with the MEPC/MTVAL
of the faulting save/restore instruction (root fault not yet cured, but no
silent wedge).

## 0005 / 0006 — FALSIFIED save/restore-side PIE trap-storm attempts (REMOVED 2026-07-19)

0005 (fence/nop/`.balignw` spacing screen) and 0006 (force strict-aligned PIE CFG
inside the save/restore) were two hypothesis-driven attempts to fix the coproc
lazy-save trap storm *from the save/restore side*. BOTH were bench-tested and
STILL produced the HP-WDT wedge (0006's "CFG-unaligned is the cause" hypothesis
was falsified) — see project_pie_save_deadlock_smoke and git history for the full
analysis. The real fix is **0009** (recover the storm by enabling the correct
coprocessor for the misrouted rev<3 P4 FPU-EXT-ILL trap). The 0005/0006 .patch
files were deleted; their diffs remain in git history if ever needed.

## libacars — fix `uper_decode()` hard-zeroing `consumed` on RC_FAIL

**File:** `libacars/libacars/asn1/per_decoder.c` (tracked copy, not a
`.patch` file -- see the note above).
**Upstream version:** libacars 2.2.1.

`asn_codecs.h`'s own documented contract for `asn_dec_rval_t.consumed`
says the consumed-byte count must stay meaningful even when
`code == RC_FAIL`, "to indicate the number of successfully decoded
bytes... providing a possibility to fail with more diagnostics". The uPER
decoder's `uper_decode()` violated this: on failure it hard-zeroed
`rval.consumed = 0` instead of reporting `pd.moved`, the bit offset the
decoder had already tracked internally at the point of failure (asserted
equal to `rval.consumed` on the success path two lines above, at
`per_decoder.c:86`).

Fix: `rval.consumed = pd.moved;` on the failure branch. This is what
makes best-effort/partial decode (design
`docs/superpowers/plans/2026-07-07-libacars-best-effort-decode.md` §3)
possible at all -- without it, callers of `la_asn1_decode_as()` have no
way to find out how far a uPER decode got before desyncing.
`uper_decode_complete()` (same file, unchanged) already rounds any
nonzero `rval.consumed` up to a byte count before returning it, so this
also matches the byte-granularity the BER/XER codecs already provide on
failure -- `la_asn1_decode_as()`'s `consumed` output is therefore
byte-granular (rounded up from the bit-exact `pd.moved`), not bit-exact;
see cpdlc.c's `consumed_bits` field for how that gets turned back into a
bit count for display.

Independently upstreamable (no interaction with the best-effort feature
itself -- it's a standalone contract-violation bugfix). AI-assisted; see
git history for authorship.

## 0007 — bootloader_support: invalidate flash mmap cache before app-side OTA verify

**File:** `components/bootloader_support/src/esp_image_format.c`
**IDF version:** v6.1 — vendored checkout tracks `release/v6.1`. Re-verify on IDF updates.
**Upstream:** esp-idf#17855 (ESP32-P4 + PSRAM, UNFIXED upstream as of this writing).

`esp_ota_end()` (via `esp_https_ota_finish()`) calls `esp_image_verify()`,
which re-reads the just-written image out of flash and re-hashes it. In app
mode that read goes through `bootloader_mmap()` — a **cached** flash mapping —
in `process_segment_data()`. On ESP32-P4 + PSRAM the CPU data cache still holds
**stale lines** for the reused mmap vaddr window: the OTA payload was written to
flash over SPI (`esp_ota_write` → `spi_flash_write`), not through this cache, and
nothing invalidates it before the verify read. The SHA is therefore computed
over stale bytes and verification fails with `ESP_ERR_OTA_VALIDATE_FAILED` even
though flash is byte-correct — proven by an `esptool read-flash 0x620000` of the
rejected `ota_1` matching the source SHA256. (The header/appended-digest reads
use `bootloader_flash_read`, which is uncached/fresh, so they pass — only the
segment mmap read is stale.)

Fix: right after the `bootloader_mmap()` in `process_segment_data()`, invalidate
the mapped span (page-aligned, mirroring the post-remap invalidate in
`spi_flash/flash_mmap.c`) via `cache_hal_invalidate_addr()` so every read below
fetches fresh flash. The invalidate is **synchronous** (ROM `Cache_Invalidate_Addr`),
so no race with the SHA read — verified deterministic across 3/3 device OTAs
(both fresh-boot fast-download and warmed-up `dma_free=2KB` slow-download
regimes; the slow regime reproduced the pre-fix failure). Guarded
`#if !defined(BOOTLOADER_BUILD)`: the second-stage bootloader verifies from a
cold post-reset cache and must not gain this runtime dependency; its binary is
byte-unchanged by this patch. A quiet `ESP_LOGD` records the invalidate
(`OTA17855: segN ... ok=1`) for field diagnosis.

Adds `#include "hal/cache_hal.h"` (the file already pulls `hal/cache_ll.h`);
uses `SPI_FLASH_MMU_PAGE_SIZE` and `ALIGN_UP`, both already available in the file.

## 0008 — fatfs: enable exFAT (`FF_FS_EXFAT`)

**File:** `components/fatfs/src/ffconf.h`
**IDF version:** v6.1 — vendored checkout tracks `release/v6.1`. Re-verify on IDF updates.

FatFs ships exFAT hard-disabled (`FF_FS_EXFAT 0`) with no ESP-IDF Kconfig
toggle, because exFAT is patent-encumbered (Microsoft) — you must opt in
deliberately. We want it for a large (256 GB SDXC) capture/log card:
FAT32 on 256 GB forces 32-128 KB clusters (wasteful for many small NDJSON
logs), while exFAT uses right-sized clusters and mounts the card in its
native shipped format (no reformat).

Fix: `FF_FS_EXFAT 0 -> 1`. Prereqs already met: `FF_USE_LFN == 3`
(`CONFIG_FATFS_LFN_HEAP=y`). Memory cost is only the LFN working-buffer
bump (~600 B extra with exFAT) which is on the heap and lands in **PSRAM**
(`CONFIG_FATFS_ALLOC_PREFER_EXTRAM=y`) — **zero internal DMA-INT cost**.
exFAT support is additive: FAT12/16/32 cards still mount. Note the ffconf
caveat that exFAT "discards ANSI C (C89) compatibility" (needs 64-bit
`QWORD`) — a non-issue for this C11 build.

## 0009 — freertos/riscv: RECOVER the coproc trap storm (rev<3 P4 FPU EXT_ILL bug)

**File:** `components/freertos/FreeRTOS-Kernel/portable/riscv/port.c`
**IDF version:** v6.1. **Apply AFTER 0004** (extends its `xPortCoprocTrapStormCheck`).

The rev<3 P4 `SOC_CPU_HAS_FPU_EXT_ILL_BUG`: the EXT_ILL "reason" CSR can set the
PIE bit for an FPU `flw/fsw`, so `vectors.S` misroutes the FPU trap to
`rtos_save_pie_coproc` (enables PIE, leaves FPU off) -> same-PC re-fault forever =
the "Coprocessor lazy-save trap storm" on worker_core1 (the #19 live reboots;
0004 only DETECTS it -> clean reboot). This is the residual wedge Path A (single
PIE owner) did NOT cure because worker_core1 uses BOTH PIE and FPU.

Fix: in the already-storming branch of `xPortCoprocTrapStormCheck` (count>=4, well
below the abort LIMIT=16), if the coproc is PIE, `frame->mtval` decodes as an FPU
load/store (masks copied verbatim from the vectors.S FPU_EXT_ILL fallback), AND
the current task already OWNS the FPU, enable `mstatus.FS` (a live write — mstatus
is not restored on exception return) and return -> the flw/fsw retires = RECOVERY
instead of reboot. Not the FPU owner -> do nothing, fall through to the 0004
abort/reboot (unchanged). Only the storming path is touched; normal
save/restore/dispatch are untouched. Owner-gate prevents stale-FP recovery.

Verify: smoke PASS (normal path intact) + streaming soak shows "coproc storm
RECOVERED" and no reboot under load. Gated behind
SOC_CPU_HAS_FPU && SOC_CPU_HAS_PIE && SOC_CPU_HAS_FPU_EXT_ILL_BUG &&
CONFIG_ESP32P4_SELECTS_REV_LESS_V3 (compiles out on unaffected silicon).

## 0011 — freertos/riscv: EAGER coproc enable (the #20 fix) + recovery counters

**Files:** `components/freertos/FreeRTOS-Kernel/portable/riscv/port.c`,
`components/freertos/FreeRTOS-Kernel/portable/riscv/portasm.S`
**IDF version:** v6.1. **Apply AFTER 0009** (diffed against the 0009-applied tree).

Two parts, both on top of 0009's FPU-EXT-ILL recovery:

1. **Eager coproc enable — the real #20 fix.** `context_switch_requested` leaves the
   incoming task's coprocessors DISABLED (lazy), so its first PIE/FPU op traps; on
   rev<3 P4 that lazy PIE re-enable STORMS (coproc==PIE, retried instruction never
   retires — 0004 only detects it, 0009's FPU recovery can't cure it, and a symmetric
   PIE-recovery attempt was falsified, see NOTE). Fix: `vPortEagerEnableOwnedCoprocs()`
   (port.c), called from portasm.S `rtos_int_exit` AFTER `hwlp_restore_if_used` (which
   consumes a0=TCB, so this uses pxCurrentTCBs not a0): if the incoming task ALREADY
   owns PIE, enable it now (`csrw 0x7f2` — a live CSR surviving mret) so no trap ever
   fires; if it owns FPU, return 1 so portasm ORs mstatus.FS into the restored mstatus.
   Owner-gated (non-owner → unchanged lazy path). PROVEN: worker_core1 soaked 6h then
   11h+ across reboots with ZERO coproc recoveries and no reboot, where the storm
   previously reboot-looped at the 2nd burst callback.
2. **Recovery counters + logging.** 0009's FPU recovery is counted
   (`g_coproc_fpu_recoveries`) + one-shot-logged, exposed via /status `fpu_recover`
   as live soak evidence.

NOTE — a symmetric PIE-recovery attempt (informally "0010", NEVER a committed .patch)
was FALSIFIED and removed: enabling PIE state post-fault (the same `csrw 0x7f2`) does
NOT cure a PIE re-fault, and 0011's eager-enable PREVENTS the trap upstream, superseding
it. Its counter (`g_coproc_pie_recoveries`) and the /status `pie_recover` field were
removed with it.

DEVICE: golden RAW smoke NOT run for this patch; validated instead by ~11h+ live OTA
soak (no reboot, decoding real IDA, P4≈gr-iridium) per explicit user authorization —
see the main-repo commit that ships the matching app-side observability.
