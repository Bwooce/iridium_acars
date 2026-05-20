# ESP32-P4 `.bss` audit — candidates for PSRAM migration

Date: 2026-05-20
Artefact: `p4-usb-host/build/p4-usb-host.elf` / `.map`

## Summary

L2MEM (ESP32-P4 internal SRAM) is split by the linker into:

| Region | Origin | Length |
|---|---|---|
| `sram_low`  | `0x4FF00000` | `0x2CBD0` = 183,248 B (≈179 KiB) |
| `sram_high` | `0x4FF40000` | `0x40000`  = 262,144 B (256 KiB) |

`.bss` lives entirely in L2MEM and is split:

| Section | Address | Size | Notes |
|---|---|---|---|
| `.dram0.bss` | `0x4FF13A50` | `0x16E20` = 93,728 B | sram_low tail (shares region with `.iram0.text` + `.dram0.data`) |
| `.dram1.bss` | `0x4FF40000` | `0x17498` = 95,384 B | sram_high head |
| **Total**    | —            | **189,112 B (≈185 KiB)** | matches the brief's ~200 KiB estimate |

Of that, roughly **140-150 KiB is firmware-owned** (uw_correlator, dsp_processor, bch_decoder, frame_decoder, ingest_core1, worker_core1, burst_pipeline). The rest belongs to FreeRTOS, esp_libc, mbedtls, USB, etc. — library code, out of scope here.

`EXT_RAM_BSS_ATTR` (`esp_attr.h`) is already in use in this tree (`p4-usb-host/main/smoke_test.c:475` for the synthetic-USB buffer), so the migration mechanism is established.

### Reclaimable budget

Aggregating every CAN_MOVE entry ≥ 512 B in firmware code (see table below):

- **Strict CAN_MOVE (no DMA/PIE touch, cold or scalar-sequential): ~145 KiB**
- After migration the L2MEM heap (currently ~283 KiB free) gains effectively all 145 KiB back as DMA-capable internal SRAM, bringing usable internal heap to ~425 KiB.

## Top static allocations (firmware-owned, sorted by size)

| # | Symbol | Size (B) | Source | Classification | Justification |
|---|---|---:|---|---|---|
| 1 | `.bss.re.7` (function-static `re[CFO_FFT_N]` in `cfo_fft_f32`/`cfo_fine_estimate_q15`) | 16384 | `common/iridium_decoder/uw_correlator.c:844` | **CAN_MOVE** | Per-burst float FFT scratch. Called once per detected burst; scalar FPU loop reads sequentially. No DMA, no PIE. |
| 2 | `.bss.im.6` (function-static `im[CFO_FFT_N]`) | 16384 | uw_correlator.c:844 | **CAN_MOVE** | Pair with #1. |
| 3 | `.bss.syn_ra` (BCH syndrome LUT, 1024 × 8 B) | 8192 | `common/iridium_decoder/bch_decoder.c:6` | **CAN_MOVE** (better: convert to `static const` and put in `.rodata`) | Lookup-once-per-codeword; ~few-hundred lookups per frame. Single 8-byte random read each. PSRAM latency adds ~30 ns/lookup — negligible at frame rates. |
| 4 | `.bss.s_sync_ul_fft_re_f` | 8192 | uw_correlator.c:314 | **CAN_MOVE** | Precomputed reversed-conj sync FFT (float). Sequential scalar read inside the per-burst matched-filter multiply. |
| 5 | `.bss.s_sync_ul_fft_im_f` | 8192 | uw_correlator.c:315 | **CAN_MOVE** | Pair with #4. |
| 6 | `.bss.s_sync_dl_fft_re_f` | 8192 | uw_correlator.c:312 | **CAN_MOVE** | DL counterpart. |
| 7 | `.bss.s_sync_dl_fft_im_f` | 8192 | uw_correlator.c:313 | **CAN_MOVE** | DL counterpart. |
| 8 | `.bss.s_cfo_tw_re_f` (CFO FFT twiddles, float) | 8192 | uw_correlator.c:663 | **CAN_MOVE** | Read sequentially in `radix2_fft_f32` (no PIE — it's a hand-rolled scalar FFT). |
| 9 | `.bss.s_cfo_tw_im_f` | 8192 | uw_correlator.c:664 | **CAN_MOVE** | Pair with #8. |
| 10 | `.bss.s_cfo_brev` (CFO bit-reversal table, uint16) | 8192 | uw_correlator.c:650 | **CAN_MOVE** | Read once per FFT-stage permutation; sequential strides. |
| 11 | `.bss.s_accum` (FBT FFT input accumulator, 2 × 2048 int16) | 8192 | `p4-usb-host/main/dsp_processor.c:71` | **CAN_MOVE** | Filled by `memcpy` from incoming PSRAM ringbuffer, then read element-wise in `window_multiply` (`fft_burst_tagger.c:184`). No DMA pulls from this address; it's a scalar CPU producer-consumer between Core 0 ingest and the tagger. |
| 12 | `.bss.ftmp_re.9` (function-static in sync-FFT init block) | 8192 | uw_correlator.c:573 | **CAN_MOVE** | Init-only scratch (sync FFT pre-computation runs once at boot). |
| 13 | `.bss.ftmp_im.8` | 8192 | uw_correlator.c:573 | **CAN_MOVE** | Pair with #12. |
| 14 | `.bss.fifft_re.11` (function-static in matched-filter fn) | 8192 | uw_correlator.c:1130 | **CAN_MOVE** | Per-burst IFFT scratch, scalar float FFT/IFFT. |
| 15 | `.bss.fifft_im.10` | 8192 | uw_correlator.c:1130 | **CAN_MOVE** | Pair with #14. |
| 16 | `.bss.fburst_re.13` | 8192 | uw_correlator.c:1129 | **CAN_MOVE** | Per-burst forward FFT input, scalar. |
| 17 | `.bss.fburst_im.12` | 8192 | uw_correlator.c:1129 | **CAN_MOVE** | Pair with #16. |
| 18 | `.bss.s_corr_tw_re_f` (matched-filter FFT twiddles) | 4096 | uw_correlator.c:310 | **CAN_MOVE** | Hand-rolled float FFT, no PIE. |
| 19 | `.bss.s_corr_tw_im_f` | 4096 | uw_correlator.c:311 | **CAN_MOVE** | Pair with #18. |
| 20 | `.bss.s_corr_brev` (matched-filter FFT bit-reversal) | 4096 | uw_correlator.c:290 | **CAN_MOVE** | Read-only after init. |
| 21 | `.bss.s_sbd` (SBD reassembler state, 8 sessions) | 2968 | `p4-usb-host/main/frame_decoder.c:46` | **CAN_MOVE** | Touched only on `frame_decoder` task — cold relative to DSP loops (one frame ≈ 1 ms vs ~30 ms wall per burst). |
| 22 | `.bss.s_rs` (256→250 resampler: 1125 coeffs + delay + bookkeeping) | 2336 | `p4-usb-host/main/ingest_core1.c:32` | CAN_MOVE (**watch perf**) | Hot path — every USB ingress sample goes through this. Inner loop reads `coeffs[phase][k]` and `delay[k]`. Sequential reads through one phase's row per output sample (9 taps × 2 bytes = 18 B/output × 2 channels). At 250 ksps × 18 B × 2 ≈ 9 MB/s — well within PSRAM bandwidth. Worth measuring before committing. |
| 23 | `.bss.s_sync_ul_re` (float, SYNC_RRC_LEN=271) | 1084 | uw_correlator.c:148 | **CAN_MOVE** | Init-only; consumed by `build_shaped_sync` then written into the FFT input. |
| 24 | `.bss.s_sync_ul_im` | 1084 | uw_correlator.c:149 | **CAN_MOVE** | Same. |
| 25 | `.bss.s_sync_dl_re` | 1084 | uw_correlator.c:146 | **CAN_MOVE** | Same. |
| 26 | `.bss.s_sync_dl_im` | 1084 | uw_correlator.c:147 | **CAN_MOVE** | Same. |
| 27 | `.bss.s_decim` (`direct_if_decim_t`, taps[144] + scratch + PIE state) | 1024 | `p4-usb-host/main/worker_core1.c:374` | **MUST_STAY_INTERNAL** | `d->taps[144]` is the coeffs pointer handed to `dsps_fird_s16_arp4`, which uses `esp.vld.128.ip` — PIE vector loads cannot reach PSRAM. Moving would silently downgrade to the ANSI fallback (or hang). |
| 28 | `.bss.s_cfo_window_full_f` (Blackman window) | 1024 | uw_correlator.c:665 | **CAN_MOVE** | Read once per CFO call, sequential scalar. |
| 29 | `.bss.s_cfo_window_uw_f` | 480 | uw_correlator.c:666 | **CAN_MOVE** | Same. |
| 30 | `.bss.s_start_lp_taps_padded` | 368 | uw_correlator.c:110 | **MUST_STAY_INTERNAL** | Coeffs for `dsps_fird_s16_arp4` (D13 start-finder LP filter). PIE vector load. |
| 31 | `.bss.s_dump_dir` (debug dump path string) | 256 | `common/iridium_decoder/burst_pipeline.c:13` | **CAN_MOVE** | Cold debug-only string. |
| 32 | `.bss.s_rc_taps` (float, RRC ⊛ RRC) | 204 | uw_correlator.c:151 | **CAN_MOVE** | Init-only. |
| 33 | `.bss.s_rrc_taps` (float) | 204 | uw_correlator.c:150 | **CAN_MOVE** | Init-only. |
| 34 | `.bss.s_rrc_taps_padded` (Q15, RRC_NTAPS_PADDED=56) | 112 | uw_correlator.c:133 | **MUST_STAY_INTERNAL** | Coeffs for `dsps_fird_s16_arp4`. |
| 35 | `.bss.s_rrc_fir_i` / `.bss.s_rrc_fir_q` (`fir_s16_t` PIE state) | 40 each | uw_correlator.c:134 | **MUST_STAY_INTERNAL** | Touched by PIE asm. |

Skipped (library code, not ours to annotate): `xIsrStack` (3072 B, FreeRTOS), `s_intr_handlers` (512 B, RISC-V interrupt table), various 5xx-byte `s_*_fft_re_f` from esp-dsp's internal tables, etc.

### Aggregate reclaimable

Summing every `CAN_MOVE` row above:

```
2 × 16384  (re/im CFO)               =  32768
12 × 8192  (sync_*_fft_f, cfo_tw/brev, accum,
            ftmp/fifft/fburst pairs) =  98304
3 ×  4096  (corr_tw/brev)            =  12288
1 ×  2968  (s_sbd)                   =   2968
1 ×  2336  (s_rs, with perf caveat)  =   2336
4 ×  1084  (sync_{ul,dl}_{re,im})    =   4336
1 ×  1024  (s_cfo_window_full_f)     =   1024
1 ×   480  (s_cfo_window_uw_f)       =    480
1 ×   256  (s_dump_dir)              =    256
2 ×   204  (s_rc_taps, s_rrc_taps)   =    408
                                      -------
                                      154 168 bytes  (~151 KiB)
```

If we exclude `s_rs` (the only hot-path candidate that warrants measurement), the safe-without-perf-risk subset is **~149 KiB**.

## First three wins (recommended order)

These three changes, applied to the source as suggested, reclaim ~88 KiB of internal SRAM with negligible perf risk and the smallest blast radius:

### 1. `syn_ra` in `common/iridium_decoder/bch_decoder.c` — reclaim 8192 B

```c
#include "esp_attr.h"  // (already pulled in indirectly via esp_log etc;
                       // add explicitly if not present)

static EXT_RAM_BSS_ATTR struct { int errs; uint32_t locator; } syn_ra[1024];
```

Single allocator, no DMA, host build uses plain `static` (use a `#ifdef ESP_PLATFORM` guard or define a host stub). Best long-term: make this `static const` by computing the table at build-time (it's deterministic from `BCH_POLY_RA`); that would move it to `.flash.rodata` and free 8 KB *and* reduce boot init work.

### 2. Six per-burst FFT scratch buffers in `common/iridium_decoder/uw_correlator.c` — reclaim 49152 B (six × 8192)

Lines 573 (`ftmp_re`/`ftmp_im` — init-only), 1129-1130 (`fburst_re`/`fburst_im`/`fifft_re`/`fifft_im` — per-burst):

```c
static EXT_RAM_BSS_ATTR float ftmp_re[CORR_FFT_N], ftmp_im[CORR_FFT_N];
...
static EXT_RAM_BSS_ATTR float fburst_re[CORR_FFT_N], fburst_im[CORR_FFT_N];
static EXT_RAM_BSS_ATTR float fifft_re[CORR_FFT_N], fifft_im[CORR_FFT_N];
```

All are scalar-FPU FFT scratch; `radix2_fft_f32` is the hand-rolled `float` butterfly (no PIE). The matched filter runs once per detected burst (~10-100 / second worst case in ALBQ fixture), so a few extra microseconds of PSRAM latency per burst is invisible.

Same treatment for the function-static `re[CFO_FFT_N]` / `im[CFO_FFT_N]` at line 844 (`cfo_fft_f32` / `cfo_fine_estimate`): another **2 × 16384 = 32 768 B**. Adding these brings this win to **~80 KiB**.

### 3. CFO and matched-filter precomputed FFT tables in `uw_correlator.c` — reclaim ~52 KiB

All read-only after init, all accessed by scalar float FFT loops:

```c
static EXT_RAM_BSS_ATTR uint16_t s_cfo_brev[CFO_FFT_N];            //  8192
static EXT_RAM_BSS_ATTR float    s_cfo_tw_re_f[CFO_FFT_N / 2];     //  8192
static EXT_RAM_BSS_ATTR float    s_cfo_tw_im_f[CFO_FFT_N / 2];     //  8192
static EXT_RAM_BSS_ATTR float    s_cfo_window_full_f[CFO_INPUT_N]; //  1024
static EXT_RAM_BSS_ATTR float    s_cfo_window_uw_f[CFO_UW_ONLY_N]; //   480

static EXT_RAM_BSS_ATTR uint16_t s_corr_brev[CORR_FFT_N];          //  4096
static EXT_RAM_BSS_ATTR float    s_corr_tw_re_f[CORR_FFT_N / 2];   //  4096
static EXT_RAM_BSS_ATTR float    s_corr_tw_im_f[CORR_FFT_N / 2];   //  4096

static EXT_RAM_BSS_ATTR float    s_sync_dl_fft_re_f[CORR_FFT_N];   //  8192
static EXT_RAM_BSS_ATTR float    s_sync_dl_fft_im_f[CORR_FFT_N];   //  8192
static EXT_RAM_BSS_ATTR float    s_sync_ul_fft_re_f[CORR_FFT_N];   //  8192
static EXT_RAM_BSS_ATTR float    s_sync_ul_fft_im_f[CORR_FFT_N];   //  8192
```

These would ideally become `const` initialised at compile time (twiddles, bit-reversal, and Blackman windows are all closed-form), which would push them all to `.flash.rodata` (no RAM cost at all). That's an evening's refactor and not strictly needed if the goal is just to reclaim internal SRAM. The `EXT_RAM_BSS_ATTR` route is the smaller change.

### Cumulative after first three wins

| Step | Reclaimed | Cumulative |
|---|---:|---:|
| 1. `syn_ra` | 8192 | 8 KiB |
| 2. CFO/matched-filter scratch (incl. `re`/`im` 16 K pair) | 81920 | 88 KiB |
| 3. CFO + corr precomputed tables | 53248 | **~140 KiB** |

That gets the firmware to within a few KiB of the achievable maximum (~149 KiB without touching the resampler, ~151 KiB if `s_rs` proves benign under PSRAM).

## Secondary wins (recommended but lower priority)

- `s_sbd` (2968 B) — `frame_decoder.c:46`. Decoder-task-only, runs at single-frame cadence. Easy `EXT_RAM_BSS_ATTR`.
- The four 1084-byte `s_sync_{ul,dl}_{re,im}` float buffers — init-only after the matched-filter pre-compute.
- `s_dump_dir[256]` — only set when debug dumps enabled.
- `s_rc_taps` / `s_rrc_taps` (float, 204 B each) — init-only.

Aggregate: ~7 KiB extra.

## Do NOT move

These remain `MUST_STAY_INTERNAL` because PIE vector loads (`esp.vld.128.ip` in `dsps_fird_s16_arp4`) refuse to service PSRAM addresses, silently falling back to ANSI scalar code or hanging in the cache:

- `s_decim` (full `direct_if_decim_t` — `.taps[144]` is the PIE-loaded coeffs pointer)
- `s_rrc_taps_padded` (RRC matched-filter taps)
- `s_start_lp_taps_padded` (D13 start-finder LP filter taps)
- `s_rrc_fir_i`, `s_rrc_fir_q` (`fir_s16_t` PIE state owned by esp-dsp)

If we later want to recover these too, the path is either (a) split each struct into a small internal "DMA-touched" part and a large PSRAM part, or (b) keep them internal but make them `static const` so they live in `.flash.rodata` — but `dsps_fird_init_s16` writes back into the `fir_s16_t` it's given, so option (b) needs `const`-able variants of the esp-dsp helpers.

## Verification plan after migrating

1. `idf.py size-components` before/after — confirm `.bss` shrinks by the expected amount and `.ext_ram.bss` grows correspondingly.
2. `heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA)` at startup — should jump by ~140 KiB after the three wins.
3. Re-run the ALBQ smoke test (`scripts/build.sh && scripts/flash.sh`); compare decoded-frame count vs the current baseline. No regression expected; if any appears, isolate by toggling individual annotations.
4. Profile `s_rs` if it gets migrated: log `s_acc_convert_us` (already wired in `ingest_core1.c`) before/after.
