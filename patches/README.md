# Vendored-code patches

Patches against gitignored vendored code: the vendored ESP-IDF
(`esp-idf/`) and component-manager downloads
(`p4-usb-host/managed_components/`).

IDF patches — apply after cloning / updating the IDF:

```sh
cd esp-idf
git apply ../patches/0001-esp_dma_utils-defer-stash-alloc-until-overflow-confirmed.patch
git apply ../patches/0003-freertos-riscv-coproc-save-area-in-internal-ram-for-psram-stacks.patch
```

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
