# ESP32-P4 CPU-revision build variants (pre_v3 / v3.0 / v3.1) — 2026-07-28

Compile-time build framework so the firmware can target either ESP32-P4
silicon family. **Only `pre_v3` is hardware-verified today** — it is the
device we own (v0.x/v1.x engineering samples). The v3 variants exist so that
when real v3 silicon arrives, bring-up is "tweak the PIE code + validate",
not "build the plumbing."

## The two families (mutually exclusive)

IDF (`components/esp_hw_support/port/esp32p4/Kconfig.hw_support`) splits the P4
into two families that are **mutually exclusive** ("huge hardware difference"):

| Variant | `SELECTS_REV_LESS_V3` | `REV_MIN_FULL` | Silicon | Status |
|---|---|---|---|---|
| `pre_v3` | `y` | 0 | rev v0.x / v1.x eng samples | **VERIFIED (our device)** |
| `v3_0` | `n` | 300 | rev v3.0 | configures+builds, **HW-unverified** |
| `v3_1` | `n` | 301 | rev v3.1 | configures+builds, **HW-unverified** |

Revision number = major·100 + minor. IDF's own choice default is `REV_MIN_301`
(v3.1); our `sdkconfig.defaults` **explicitly** forces `pre_v3` because that's
our silicon and it re-enables the HWLP workaround (below).

## How to build

```
scripts/build.sh                 # pre_v3 (default) — build/ + sdkconfig, unchanged
scripts/build.sh --rev v3_0      # -> build-v3_0/  (HW-unverified)
scripts/build.sh --rev v3_1      # -> build-v3_1/  (HW-unverified)
```

`pre_v3` is byte-for-byte the pre-framework behaviour: base defaults only, the
existing `build/` and `sdkconfig`, the ninja fast-path. The v3 variants layer
`sdkconfig.rev_v3_*.defaults` over `sdkconfig.defaults` into an **isolated**
build dir *and* sdkconfig (`-B build-v3_0 -D SDKCONFIG=build-v3_0/sdkconfig`),
because `SDKCONFIG` otherwise defaults to `<proj>/sdkconfig` regardless of `-B`
and would silently inherit the pre_v3 config.

**Verification gate (do NOT trust exit code):** the `REV_MIN` choice is
mutually exclusive and can silently fall back to its Kconfig default. After a
v3 configure, assert the *generated* config resolved correctly:

```
grep CONFIG_ESP32P4_REV_MIN_FULL build-v3_0/sdkconfig   # must be =300
grep CONFIG_ESP32P4_REV_MIN_FULL build-v3_1/sdkconfig   # must be =301
```

## What a real v3 bring-up still has to do (the deferred work)

A green v3 build is **not** a working v3 image. Flipping the family changes
~15 downstream defaults (esp_pm CPU/peripheral power-down, VBAT ranges,
bootloader min-rev 90→100, CPU-freq choice). The load-bearing items:

1. **PIE hot path (the "tweak the PIE code" item).** `pre_v3` pins `REV_MIN_0`
   specifically to re-enable the `SOC_CPU_HAS_HWLOOP_STATE_BUG` workarounds that
   stopped ~1-in-600 PIE-trap Illegal-instruction panics in `dsps_fird_s16_arp4`
   (2026-07-06). On v3 the workaround compiles out (use-site gated by
   `REV_MIN_FULL`/`SELECTS_REV_LESS_V3`, NOT the soc cap — see Accuracy notes),
   so a v3 image runs the vanilla lazy save — `common/iridium_decoder/*_arp4.S` must be
   re-validated **bit-exact** on real v3 silicon, and `patches/0002` (HWLP
   removal, justified for <v3) revisited. The single seam for a v3-specific PIE
   variant is `common/iridium_decoder/pie_target.h` → `PIE_TARGET_P4_V3`
   (1 on v3, 0 on pre_v3). Add v3 PIE code behind `#if PIE_TARGET_P4_V3`; do
   NOT add empty branches before there's real v3 code.
2. **MSPI-750** (USB/SDMMC unaligned-DMA stale PSRAM read) is a v3.0-only
   erratum; IDF auto-enables its workaround at `REV_MIN_300`. We keep
   `USB_HOST_DWC_DMA_CAP_MEMORY_IN_PSRAM=n` so USB DMA never lands in PSRAM —
   MSPI-750 stays closed. Fixed in v3.1.
3. **Flash/deploy:** the v3 image lands in `build-v3_0/` etc.; `flash.sh` /
   `smoke_run.sh` assume `build/`. Point them at the v3 build dir when a v3
   board exists (out of scope until then).

## v3 bring-up TODO checklist (Fable consult, 2026-07-28)

For when real v3 silicon arrives. **BLOCKING** = must pass before a v3 image is
trusted; **opportunistic** = perf/cleanup after a clean soak; **keep** = do NOT
remove on v3.

### Blocking (both v3.0 and v3.1)
1. **[PIE lazy-save] Soak the STOCK lazy coproc save/restore.** The entire <v3
   trap-storm mitigation stack compiles out on v3: `patches/0009` (recovery body)
   and `patches/0011` (`vPortEagerEnableOwnedCoprocs`) are gated
   `CONFIG_ESP32P4_SELECTS_REV_LESS_V3`; IDF's FPU EXT_ILL fallback
   (`riscv/vectors.S:322`) and HWLP-state workaround (`portasm.S:226,804`) also
   drop out. A v3 image runs the vanilla lazy path — the exact machinery that
   storm-rebooted <v3 at the 2nd burst callback. Silicon is *expected* fixed,
   never proven. Verify: `smoke_run.sh raw` (GOLDEN matched≥40) + ≥6 h live soak
   with the real two-PIE-owners-per-core layout; `/status fpu_recover`==0, zero
   reboots, no "trap storm". Discriminator if it wedges: `CONFIG_DIAG_SINGLE_PIE_OWNER`.
   (`patches/0004` trap-storm watchdog is un-gated → any residual storm is an
   attributable panic, not a silent wedge.)
2. **[PIE bit-exact] Re-validate every PIE kernel on v3 silicon.** No documented
   PIE ISA change and none of our `.S` uses HWLP, but IDF calls the families
   "huge hardware difference". Kernels: `common/iridium_decoder/{resample_arp4.S,
   rotate_to_dc_arp4.S,fft_burst_tagger_arp4.S}` + the 0002-patched esp-dsp
   (`dsps_fird_s16_arp4`, `dsps_fft2r_{sc16,fc32}_arp4`). Host golden gates cover
   the C refs only — silicon truth = RAW golden smoke + device-vs-gr-iridium on a
   reference capture. Scalar A/B on the same silicon if diverging: `FBT_USE_PIE_KERNELS=0`,
   `RS25_DISABLE_PIE_ASM`. Divergence seam: `pie_target.h` → `PIE_TARGET_P4_V3`.
3. **[config] Audit pre_v3 sdkconfig drift.** `build-v3_x/sdkconfig` is generated
   purely from defaults+fragment; the live `p4-usb-host/sdkconfig` has values not
   in defaults. Observed: live may carry `ESP_CONSOLE_UART_BAUDRATE=921600` vs
   defaults' 115200 — the v3 image silently takes the defaults side of every such
   drift. `diff p4-usb-host/sdkconfig p4-usb-host/build-v3_x/sdkconfig` and explain
   every non-revision line before first flash. (See side-note below — worth a look
   independent of v3.)
4. **[config] Prove the family-default flips boot + hold throughput.** Regenerated
   v3 diff shows CPU 360→**400 MHz** (our REV_MIN_0-only 360 pin becomes
   unsatisfiable, `Kconfig.cpu:14`), **flash freq 40→80 MHz**, bootloader clk
   90→100 MHz, `PM_POWER_DOWN_CPU_IN_LIGHT_SLEEP`/`PM_CPU_RETENTION_DYNAMIC` newly
   default-on. 80 MHz flash on an unknown v3 board is a real boot risk. Verify:
   clean boot @400/80, SPIRAM memtest, full-pipeline rate (~4.77 MB/s class).
5. **[tooling] Point flash/smoke at `build-v3_x/` + wire the REV gate.** `flash.sh`/
   `smoke_run.sh` assume `build/`; automate `grep CONFIG_ESP32P4_REV_MIN_FULL
   build-v3_x/sdkconfig` (=300/301) so a silent choice-fallback can't ship.
6. **[safety] Confirm cross-family flash/OTA rejection.** v3 image `REV_MIN_FULL`
   300/301 vs pre_v3 `REV_MAX_FULL=199` — bootloader must refuse a cross-family
   image; never OTA a v3 image to the pre_v3 fleet. `esptool chip_id` a board's rev
   before first flash; boot-test the rejection once. (`BOOTLOADER_APP_ROLLBACK` is the net.)
7. **[patches] Re-audit the shared patched trees** — the v3 build inherits ALL of
   `patches/`. Disposition: **0002** (esp-dsp HWLP→counter loops) KEEP initially
   (bit-preserving, 5-15% slower; can't be per-build-reverted — `managed_components`
   is one shared copy, so a v3 HWLP restore needs an in-.S `#if`). **0003**
   (coproc save areas in internal RAM) KEEP (architectural). **0004** KEEP
   (diagnostic). **0009/0011** self-remove via their config gate — but 0011 leaves
   an *unconditional* `call vPortEagerEnableOwnedCoprocs` in `context_switch_requested`,
   a dead call on v3: gate or drop it.

### Blocking (v3.0 only)
12. **[errata] MSPI-749/750/751** (introduced v3.0, FIXED v3.1): unaligned USB/SDMMC
    DMA can read stale PSRAM. IDF auto-enables `P4_REV3_MSPI_CRASH_AFTER_POWER_UP_WORKAROUND`
    at REV_MIN_300 (confirmed in `build-v3_0/sdkconfig`). Our
    `USB_HOST_DWC_DMA_CAP_MEMORY_IN_PSRAM=n` keeps USB DMA out of PSRAM →
    MSPI-750 closed; **never flip it on v3.0**, no `USB_TRANSFER_FLAG_ALLOC_DMA`.
    Run the SD smoke stage on v3.0 (SDMMC is also an MSPI-750 surface; our 512 B
    sector alignment in `sd_log.c` already avoids unaligned DMA).
13. **[errata] Fresh ESP-Chip-Errata pass for v3.0** (USB-HS/SDMMC/GDMA) beyond the
    2026-06 v1 audit, before trusting the image.

### v3.1
14. **[config] Confirm MSPI workaround absent** (`build-v3_1/sdkconfig` shows
    `P4_REV3_MSPI_WORKAROUND_SIZE=0` — correct). v3.1 is IDF's default choice and
    the preferred first bring-up target; do v3.1 before v3.0.
15. **[perf] PSRAM-resident USB DMA is legal on v3.1** (MSPI-750 fixed) but keep it
    off — the internal-SRAM-landing + AXI-GDMA rationale is revision-independent;
    only revisit with a measured A/B.

### Opportunistic
8. **[PIE/HWLP perf] Restore `esp.lp.setup` kernels (revert 0002) for v3** after
   item 1's soak proves lazy-save survives armed HWLP state — recovers the 5-15%.
   Blocked on the shared-tree problem (item 7: needs in-.S `#if`).
10. **[perf] Re-baseline the PSRAM ceiling:** `SPIRAM_SPEED_250M` exists only on the
    v3 family (base pins 200M); 400 MHz CPU adds ~11% on the UW-correlator-bound
    worker. A/B 200M vs 250M over a full pass (no partial-data verdicts).

### Keep — do NOT remove on v3
9. **Heap-position PIE defenses** (early-alloc dance + `esp_ptr_in_dram` guards,
   `uw_correlator.c` / `worker_core1.c:1392`) — cheap and loud; keep regardless.
11. **`panic_capture.c` + coredump-to-UART** — rev-agnostic bring-up diagnostics.
   Plus the single-PIE-owner-per-core rule, `patches/0003`, `patches/0004`.

### Accuracy notes (memory vs current code)
- `SOC_CPU_HAS_HWLOOP_STATE_BUG` is **not** soc-cap-gated by revision — the cap is
  unconditionally `=y` (`soc_caps.h:199,201`). The gating is at *use sites*
  (`portasm.S:226,804` `&& REV_MIN_FULL<=1`; `vectors.S:322` `&& SELECTS_REV_LESS_V3`).
  So the workaround compiles out on v3, but grepping the soc cap in a v3 sdkconfig
  misleads — **check `REV_MIN_FULL`, not the cap.**
- The HWLP-state gate is `REV_MIN_FULL <= 1` (IDF believes the bug fixed at v1.0),
  yet our (claimed v1.x) silicon empirically needed `REV_MIN_0` to stop the fird
  panics — keep that anomaly in mind interpreting v3 lazy-save behaviour.

## Files

- `p4-usb-host/sdkconfig.defaults` — pre_v3 baseline (unchanged config; added a
  pointer comment only).
- `p4-usb-host/sdkconfig.rev_v3_0.defaults`, `…rev_v3_1.defaults` — override
  fragments (explicit un-set of the base choice member + family flip + v3 member).
- `scripts/build.sh` — `--rev` selection + isolated build dir/sdkconfig.
- `common/iridium_decoder/pie_target.h` — the `PIE_TARGET_P4_V3` seam.
