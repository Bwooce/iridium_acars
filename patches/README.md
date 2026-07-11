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
git apply ../patches/0006-freertos-riscv-pie-coproc-force-aligned-cfg-in-save-restore.patch   # apply AFTER 0004
git apply ../patches/0007-bootloader_support-invalidate-mmap-cache-before-app-ota-verify.patch
```

**0005 and 0006 are mutually exclusive** — both edit the
`pie_save_regs`/`pie_restore_regs` macros. The current production gate uses
**0006** (the CFG-alignment fix); 0005 (the fence/nop/`.balignw` screen) is the
superseded experiment. Apply one or the other, never both.

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

## 0005 — freertos/riscv: PIE save/restore spacing/fence/alignment screen (EXPERIMENT)

**File:** `components/freertos/FreeRTOS-Kernel/portable/riscv/portasm.S`
**IDF version:** v6.1 (vendored `release/v6.1` checkout). Re-verify on IDF
updates. **Apply AFTER 0004** (context lines are diffed against the
0004-applied tree; 0005 touches only the `pie_save_regs`/`pie_restore_regs`
macros, which 0004 does not).

This is a **cheap, falsifiable experiment**, not a proven fix. The device-smoke
RAW/REAL wedge hard-hangs Core 1 (HP-WDT, no coredump) inside
`rtos_save_pie_coproc` on the back-to-back 128-bit PIE Q-register run
(`esp.vst.128.ip`/`esp.vld.128.ip`, `.insn 0x82012{2,6,a,e}3b`). Prior work
ruled out memory region (0003), re-entry storm (0004 watchdog saw none), and our
own kernels (single-PIE-owner-per-core avoids it). The surviving lead is
esp-dsp issue **#102 "Problematic HW Loops on ESP32-P4"** (DSP-158): on P4 rev-1.0
silicon a custom-coproc op (`esp.lp.setup`) that is **not 4-byte aligned** makes
the core "go haywire"; the fix is `.balignw 4,0x0001`. Hypothesis: the PIE
Q-register hang is a related pipeline / memory-ordering / alignment hazard on
back-to-back custom-coproc ops.

The patch ADDS only register-safe padding — every functional PIE op is kept
intact and in its original order — testing three hypotheses at once (a "does ANY
spacing help" screen; if it passes, a follow-up bisects which element mattered):

- **H1 alignment:** `.balignw 4,0x0001` (mirror of esp-dsp #102) before each
  run of coproc ops (the Q-register block, the QACC/UA block, and the XACC op),
  in both macros.
- **H2 memory-ordering:** `fence` before the first vector access and between the
  Q-register block and the exotic-register (QACC/UA/XACC/SAR) block.
- **H3 pipeline hazard:** two `nop`s between each pair of consecutive
  `esp.vst.128.ip`/`esp.vld.128.ip` and QACC/UA ops.

`fence`/`nop`/`.balignw` are all register-safe (no clobber of the `a1`/`a2`
scratch the macros already use), which matters because these macros run in the
illegal-instruction trap handler with a tight register budget.

**Build-verified** (SMOKE_TEST_RAW, 0002+0003+0004+0005 applied): compiles
clean; disassembly of `rtos_save_pie_coproc` confirms all 16 Q-register ops
survive with correct encodings, separated by the nops/fences.

**Known caveat (important for interpreting a bench result):** in the linked
image the coproc ops land at **2-mod-4** addresses, i.e. the `.balignw` did NOT
achieve absolute 4-byte alignment — `rtos_save_pie_coproc` is placed at a
2-mod-4 base and IRAM does not honor `R_RISCV_ALIGN` to 4 here, so the padding
tracked the assembler's local frame parity only. The H2 (fence) and H3 (2-nop)
perturbations ARE genuinely applied; H1 as-emitted is "padding present, ops
still 2-mod-4". So a still-hangs bench result cleanly falsifies H2+H3 but only
partially H1 — a targeted 4-byte-alignment retry (force routine/section
alignment) would be the H1 follow-up before discarding alignment entirely.

**Bench procedure (main session):** apply 0005 after 0004, set
`CONFIG_SMOKE_TEST_MODE=y` + `CONFIG_SMOKE_TEST_RAW_IRIDIUM=y`, build, flash,
`scripts/smoke_run.sh raw`. GOLDEN pass (`matched>=40`, no HP-WDT) ⇒ spacing/
fence/alignment FIXED the wedge (bisect which element next). Still HP-WDT hang
(or a 0004 attributable trap-storm panic) ⇒ falsified; proceed to PIE-ownership
re-tiering (research plan path (b)). Restore the production sdkconfig afterward
(smoke_run leaves `CONFIG_SMOKE_TEST_MODE=y` in the gitignored sdkconfig).

## 0006 — freertos/riscv: force strict-aligned PIE CFG inside the save/restore

**File:** `components/freertos/FreeRTOS-Kernel/portable/riscv/portasm.S`
**IDF version:** v6.1 (vendored `release/v6.1` checkout). Re-verify on IDF
updates. **Apply AFTER 0004.** 0006 touches only the
`pie_save_regs`/`pie_restore_regs` macros, which 0004 does not.

**SUPERSEDES 0005 — do NOT apply both.** 0005 (the fence/nop/`.balignw` screen)
and 0006 both edit the same two macros and will conflict. 0006 is the
targeted, hypothesis-driven follow-up; the production gate uses 0002+0003+0004+**0006**
(not 0005).

**Root-cause hypothesis (from the git+code archaeology, see
`docs/superpowers/plans/2026-07-07-pie-regression-archaeology.md`):** the IDF
FreeRTOS-P4 port does **not** context-switch the PIE **CFG** register — its
lazy save/restore saves Q0–7, QACC, UA_STATE, XACC, SAR, SAR_BYTES, FFT_BIT_WIDTH,
but **never CFG** (verified: no `esp.movx.*cfg` anywhere in `portasm.S`; no
`RV_PIE_CFG` field in `rvruntime-frames.h`). Every PIE-Q kernel this firmware runs
— ours (`resample_arp4.S:54-58`, `rotate_to_dc_arp4.S:37-39`) and esp-dsp's
(`dsps_fird_s16_arp4.S:50-52`) — sets **CFG bit 1 (unaligned 128-bit vld)** and
never clears it. So once any PIE kernel has run on a core, that core is left in
unaligned-vld mode permanently, **including while `rtos_save_pie_coproc` runs its
own `esp.vst.128`/`esp.vld.128`/`esp.st.ua.state` against the (16-byte-aligned)
save frame**. The regression origin (`baaff6d`, 2026-05-22, PIE-ised resample →
`ingest_core1` became a *second* PIE-Q owner on Core 1 alongside `worker_core1`)
made the owner-swap save/restore body actually execute; the deterministic Core-1
HP-WDT wedge lands on the 4th Q-register op (`esp.vld.128.ip q3`) in
`pie_restore_regs`. Hypothesis: the save/restore was validated with CFG at its
reset (strict-aligned) value, and running it in unaligned mode against a stale
UA_STATE stalls the pipeline. This state (CFG left set during the save/restore)
was never tested — 0005 added spacing/fences but left CFG set the whole time.

**Fix:** at the top of `pie_save_regs` **and** `pie_restore_regs`, read the caller
PIE CFG into `t3`, clear bit 1 (`andi t4, t3, ~0x2` → the routine's own vector
loads/stores run in strict 16-byte-aligned mode, which is correct because the
coproc save frame is 16-byte aligned via patch 0003), and restore the caller's
CFG (`esp.movx.w.cfg t3`) at the end of each macro so application PIE code still
gets unaligned mode. `t3`/`t4` are safe scratch here: `rtos_save_pie_coproc` has
already clobbered all caller-saved regs via its C calls (`xPortCoprocTrapStormCheck`,
`rtos_current_tcb`, `pxPortUpdateCoprocOwner`, `pxPortGetCoprocArea`), and the trap
entry saves/restores the full GP register file around the handler. The
`esp.movx.r/w.cfg` ops and the `t3..t6` encoding match the three arp4 kernels
above (proven-valid under this toolchain).

Only the CFG save/clear/restore bracket is added — every functional PIE op
(8× Q + 4× QACC + UA_STATE + XACC + SAR/SAR_BYTES/FFT_BIT_WIDTH) is kept intact
and in its original order.

**Build-verified** (SMOKE_TEST_RAW, 0002+0003+0004+0006 applied, `scripts/build.sh`
exit 0): disassembly of `rtos_save_pie_coproc` confirms the CFG bracket on both
sides (`esp.movx.r.cfg t3` / `andi t4,t3,-3` / `esp.movx.w.cfg t4` … `esp.movx.w.cfg t3`,
`-3` == `~0x2`) and all 16 Q-register ops + QACC/UA_STATE/XACC/SAR ops surviving
with correct encodings. **NOT bench-verified** — this is a hypothesis test.

**Bench procedure (main session):** apply 0006 after 0004 (do NOT apply 0005),
`CONFIG_SMOKE_TEST_MODE=y` + `CONFIG_SMOKE_TEST_RAW_IRIDIUM=y`, build, flash,
`scripts/smoke_run.sh raw`. GOLDEN pass (`matched>=40`, no HP-WDT) ⇒ the unaligned
CFG mode during the save/restore was the trigger (small, upstreamable port patch +
root-cause explanation). Still HP-WDT hang (or a 0004 attributable trap-storm
panic) ⇒ hypothesis falsified; the last distinct software lever is exhausted and
the remaining paths are PIE-ownership re-tiering or shipping behind a `Smoke-skip`
(see `docs/superpowers/plans/2026-07-07-pie-wedge-research.md`). Restore the
production sdkconfig afterward.

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
