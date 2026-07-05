# Iridium Frequency Scanner (Phase 1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add runtime LO retune (no reboot) plus a live narrowband-burst-density map, driven over the serial command interface, so an operator can sweep the ~10 MHz Iridium duplex band and find where channel-shaped traffic concentrates.

**Architecture:** A new control-plane `scanner` module retunes the R820T2 live via a `class_driver_retune` accessor, re-primes the tagger noise floor, and reads per-hop narrowband-burst counters tapped from `dsp_processor`'s burst dispatch (using T60's `width_bins`). Pure logic (center enumeration, ranking) is split into a host-testable `scanner_map` unit; hardware orchestration and the `hop`/`scan`/`map` serial commands live in `scanner.c`. No worker/PIE code is touched.

**Tech Stack:** ESP-IDF v6.1 (C), FreeRTOS, esp-dsp; host unit tests are plain C + `assert` built by `tests/host/CMakeLists.txt`.

## Global Constraints

- Branch: `freq-scanner` (already created off `t60-narrowband-priority`; depends on T60 `width_bins` / `BURST_WIDTH_BINS`).
- Approach A: retune live, never stop the USB bulk stream. Do NOT add stream stop/start.
- Control plane is serial (`serial_cmd.c`); do NOT add HTTP endpoints (httpd is unreliable per the SDIO-throttle note).
- Scanner is opt-in; default boot behavior (tune from NVS `lo_hz`) is unchanged.
- No AI attribution / `Co-Authored-By` trailers in commits (global rule).
- Commit messages: `subsystem: short description` (e.g. `scanner:`, `dsp_processor:`, `fft_burst_tagger:`).
- `NARROWBAND_MAX_BINS = 48` (matches T60's `BURST_NARROW_MAX_BINS`).
- Scan defaults: start 1616000000 Hz, stop 1626000000 Hz, step 2500000 Hz, dwell 2000 ms, settle 500 ms.
- Device verification only (no real-ACARS validation possible); the mandatory decode-affecting smoke gate does not apply here because no worker/decode-path code changes — but note in each device-verify step that `rate` must hold ~4.8 MB/s and no reboot occurs.

---

### Task 1: Tagger baseline reset entry point

**Files:**
- Modify: `common/iridium_decoder/fft_burst_tagger.c` (add function near the init logic at lines ~275–282)
- Modify: `common/iridium_decoder/fft_burst_tagger.h` (declare)
- Test: `tests/host/test_scanner_tagger_reset.c` (new) + register in `tests/host/CMakeLists.txt`

**Interfaces:**
- Produces: `void fft_burst_tagger_reset_baseline(fft_burst_tagger_t *t);` — resets the noise-floor EMA and active bursts so the tagger re-primes over `FBT_HISTORY_SIZE` steps; preserves `d_index` and `burst_id` (sample-position and ID continuity).

- [ ] **Step 1: Write the failing host test**

Create `tests/host/test_scanner_tagger_reset.c`:

```c
// Host test: after priming, fft_burst_tagger_reset_baseline() returns the
// tagger to the un-primed state so it re-learns the noise floor. Behavior is
// observed through the public step() return value (false while re-priming).
#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "fft_burst_tagger.h"

int main(void)
{
    int32_t *hist = calloc((size_t)FBT_FFT_SIZE * FBT_HISTORY_SIZE, sizeof(int32_t));
    assert(hist);
    fft_burst_tagger_t *t = fft_burst_tagger_init(2 * FBT_FFT_SIZE, 40000, 32, 10.0f, hist);
    assert(t);

    int16_t in[2 * FBT_FFT_SIZE];
    int16_t look[2 * (2 * FBT_FFT_SIZE)];
    memset(in, 0, sizeof(in));
    memset(look, 0, sizeof(look));

    // Prime: step until step() reports primed (returns true).
    fbt_burst_t nb[16], gb[16];
    int nn, ng;
    bool primed = false;
    for (int i = 0; i < FBT_HISTORY_SIZE + 4 && !primed; i++) {
        nn = 16; ng = 16;
        primed = fft_burst_tagger_step(t, in, look, nb, &nn, gb, &ng);
    }
    assert(primed);  // sanity: tagger reached primed state

    // Reset — next step must report un-primed again.
    fft_burst_tagger_reset_baseline(t);
    nn = 16; ng = 16;
    bool after = fft_burst_tagger_step(t, in, look, nb, &nn, gb, &ng);
    assert(after == false);

    fft_burst_tagger_destroy(t);
    free(hist);
    return 0;
}
```

- [ ] **Step 2: Register the test and run it to verify it fails**

`test_tagger_vs_manifest` already builds the tagger for the host, so copy its
recipe exactly. First read its block:

```bash
grep -n -A6 "add_executable(test_tagger_vs_manifest" tests/host/CMakeLists.txt
```

Then add an identical block for the new test — same source-file list (the tagger
`.c` files it compiles, e.g. `fft_burst_tagger.c` + FFT deps), same
`target_include_directories`, same `target_link_libraries` — but with
`test_scanner_tagger_reset.c` as the test source and `test_scanner_tagger_reset`
as the target name. Do NOT invent a library target; mirror whatever
`test_tagger_vs_manifest` uses verbatim. Finish with:

```cmake
add_test(NAME test_scanner_tagger_reset COMMAND test_scanner_tagger_reset)
```

Run:
```bash
cd tests/host && cmake -S . -B build >/dev/null && cmake --build build --target test_scanner_tagger_reset 2>&1 | tail -5
```
Expected: FAIL — link error, `undefined reference to 'fft_burst_tagger_reset_baseline'`.

- [ ] **Step 3: Declare the function in the header**

In `common/iridium_decoder/fft_burst_tagger.h`, after the `fft_burst_tagger_flush` declaration, add:

```c
// Reset the per-bin noise-floor EMA and clear active bursts so the tagger
// re-learns the floor over FBT_HISTORY_SIZE steps (used after a live LO
// retune — the old band's floor is meaningless at the new center). step()
// returns false until re-primed. d_index and burst_id are preserved so
// sample-position and burst-id continuity are unbroken.
void fft_burst_tagger_reset_baseline(fft_burst_tagger_t *t);
```

- [ ] **Step 4: Implement the function**

In `common/iridium_decoder/fft_burst_tagger.c`, after `fft_burst_tagger_flush`, add (mirrors the init block at lines ~275–282 but leaves `d_index`/`burst_id` alone):

```c
void fft_burst_tagger_reset_baseline(fft_burst_tagger_t *t)
{
    if (!t) return;
    const int N = FBT_FFT_SIZE;
    memset(t->baseline_history, 0, sizeof(int32_t) * N * FBT_HISTORY_SIZE);
    memset(t->baseline_sum, 0, sizeof(t->baseline_sum));
    for (int i = 0; i < N; i++)
        t->burst_mask[i] = 1;
    t->history_index  = 0;
    t->history_primed = false;
    t->n_bursts       = 0;
}
```

- [ ] **Step 5: Run the test to verify it passes**

```bash
cd tests/host && cmake --build build --target test_scanner_tagger_reset 2>&1 | tail -3 && ./build/test_scanner_tagger_reset && echo PASS
```
Expected: `PASS` (exit 0).

- [ ] **Step 6: Commit**

```bash
git add common/iridium_decoder/fft_burst_tagger.c common/iridium_decoder/fft_burst_tagger.h tests/host/test_scanner_tagger_reset.c tests/host/CMakeLists.txt
git commit -m "fft_burst_tagger: add reset_baseline entry point for live retune"
```

---

### Task 2: scanner_map pure logic (host-tested)

**Files:**
- Create: `p4-usb-host/main/scanner_map.h`
- Create: `p4-usb-host/main/scanner_map.c`
- Test: `tests/host/test_scanner_map.c` (new) + register in `tests/host/CMakeLists.txt`

**Interfaces:**
- Produces:
  - `typedef struct { uint32_t center_hz; uint32_t narrowband_bursts; uint32_t all_bursts; float mean_snr_db; uint32_t dwell_ms; } scanner_pos_t;`
  - `#define SCANNER_MAX_POSITIONS 16`
  - `int scanner_enumerate_centers(uint32_t start_hz, uint32_t stop_hz, uint32_t step_hz, uint32_t *out, int max);` — fills `out` with centers `start, start+step, …` while `<= stop`; returns count (clamped to `max`). Returns 0 if `step==0` or `start>stop`.
  - `float scanner_pos_narrowband_rate(const scanner_pos_t *p);` — `narrowband_bursts * 1000.0f / dwell_ms` (0 if `dwell_ms==0`).
  - `int scanner_rank_hottest(const scanner_pos_t *pos, int n);` — index of max narrowband rate; `-1` if `n<=0`; lowest index on ties.

- [ ] **Step 1: Write the failing host test**

Create `tests/host/test_scanner_map.c`:

```c
#include <assert.h>
#include <stdint.h>
#include "scanner_map.h"

int main(void)
{
    uint32_t c[SCANNER_MAX_POSITIONS];

    // 1616..1626 MHz step 2.5 MHz -> 5 centers (1616,1618.5,1621,1623.5,1626).
    int n = scanner_enumerate_centers(1616000000u, 1626000000u, 2500000u, c, SCANNER_MAX_POSITIONS);
    assert(n == 5);
    assert(c[0] == 1616000000u);
    assert(c[4] == 1626000000u);

    // Degenerate inputs.
    assert(scanner_enumerate_centers(1626000000u, 1616000000u, 2500000u, c, 16) == 0);
    assert(scanner_enumerate_centers(1616000000u, 1626000000u, 0u, c, 16) == 0);

    // Rate + ranking.
    scanner_pos_t p[3] = {
        { 1616000000u, 4, 10, 12.0f, 2000 },  // 2.0/s
        { 1618500000u, 9, 20, 14.0f, 2000 },  // 4.5/s  <-- hottest
        { 1621000000u, 0, 30, 0.0f,  2000 },  // 0/s
    };
    assert(scanner_pos_narrowband_rate(&p[0]) == 2.0f);
    assert(scanner_pos_narrowband_rate(&p[1]) == 4.5f);
    assert(scanner_rank_hottest(p, 3) == 1);
    assert(scanner_rank_hottest(p, 0) == -1);
    return 0;
}
```

- [ ] **Step 2: Register and run to verify it fails**

Add to `tests/host/CMakeLists.txt`:

```cmake
add_executable(test_scanner_map test_scanner_map.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../p4-usb-host/main/scanner_map.c)
target_include_directories(test_scanner_map PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../p4-usb-host/main)
add_test(NAME test_scanner_map COMMAND test_scanner_map)
```

Run:
```bash
cd tests/host && cmake -S . -B build >/dev/null && cmake --build build --target test_scanner_map 2>&1 | tail -5
```
Expected: FAIL — `scanner_map.h` / `scanner_map.c` do not exist.

- [ ] **Step 3: Create the header**

`p4-usb-host/main/scanner_map.h`:

```c
#pragma once
#include <stdint.h>

#define SCANNER_MAX_POSITIONS 16

typedef struct {
    uint32_t center_hz;
    uint32_t narrowband_bursts;
    uint32_t all_bursts;
    float    mean_snr_db;
    uint32_t dwell_ms;
} scanner_pos_t;

int   scanner_enumerate_centers(uint32_t start_hz, uint32_t stop_hz,
                                uint32_t step_hz, uint32_t *out, int max);
float scanner_pos_narrowband_rate(const scanner_pos_t *p);
int   scanner_rank_hottest(const scanner_pos_t *pos, int n);
```

- [ ] **Step 4: Implement the pure logic**

`p4-usb-host/main/scanner_map.c`:

```c
#include "scanner_map.h"

int scanner_enumerate_centers(uint32_t start_hz, uint32_t stop_hz,
                              uint32_t step_hz, uint32_t *out, int max)
{
    if (step_hz == 0 || start_hz > stop_hz || max <= 0) return 0;
    int n = 0;
    for (uint32_t f = start_hz; f <= stop_hz && n < max; f += step_hz)
        out[n++] = f;
    return n;
}

float scanner_pos_narrowband_rate(const scanner_pos_t *p)
{
    if (!p || p->dwell_ms == 0) return 0.0f;
    return (float)p->narrowband_bursts * 1000.0f / (float)p->dwell_ms;
}

int scanner_rank_hottest(const scanner_pos_t *pos, int n)
{
    if (!pos || n <= 0) return -1;
    int   best = 0;
    float best_rate = scanner_pos_narrowband_rate(&pos[0]);
    for (int i = 1; i < n; i++) {
        float r = scanner_pos_narrowband_rate(&pos[i]);
        if (r > best_rate) { best_rate = r; best = i; }
    }
    return best;
}
```

- [ ] **Step 5: Run the test to verify it passes**

```bash
cd tests/host && cmake --build build --target test_scanner_map 2>&1 | tail -3 && ./build/test_scanner_map && echo PASS
```
Expected: `PASS`.

- [ ] **Step 6: Commit**

```bash
git add p4-usb-host/main/scanner_map.h p4-usb-host/main/scanner_map.c tests/host/test_scanner_map.c tests/host/CMakeLists.txt
git commit -m "scanner: add host-tested center enumeration and density ranking"
```

---

### Task 3: Narrowband-density counters in dsp_processor

**Files:**
- Modify: `p4-usb-host/main/dsp_processor.h` (struct-free additions: define + typedef + getters)
- Modify: `p4-usb-host/main/dsp_processor.c` (fields, dispatch tap, getters)

**Interfaces:**
- Produces:
  - `#define DSP_NARROWBAND_MAX_BINS 48`
  - `typedef struct { uint32_t narrowband_bursts; uint32_t all_bursts; float mean_snr_db; } dsp_density_t;`
  - `void dsp_processor_read_reset_density(dsp_processor_t *p, dsp_density_t *out);` — snapshots and zeroes the counters accumulated since the last call.
  - `void dsp_processor_reset_tagger_baseline(dsp_processor_t *p);` — wraps `fft_burst_tagger_reset_baseline(p->tagger)` so the scanner never touches the tagger directly.
- Consumes: `fft_burst_tagger_reset_baseline` (Task 1); `BURST_WIDTH_BINS`-equivalent — the raw `b->width_bins` is already available in `dispatch_gone_burst`.
- Note: the density map keys on the scanner-known `center_hz`, so per-burst absolute-frequency reporting (spec §4 `set_lo_hz`) is **deferred** — it would be dead code in Phase 1 (nothing reads it). Revisit if a future readout needs absolute burst frequencies.

- [ ] **Step 1: Add fields to the dsp_processor struct**

In `p4-usb-host/main/dsp_processor.c`, in `struct dsp_processor` (near the existing `_Atomic` accumulators around line 105), add:

```c
    // Scanner density counters (Phase 1). Tapped in dispatch_gone_burst,
    // read-and-reset by dsp_processor_read_reset_density from the serial_cmd
    // task. Same relaxed-atomic idiom as the diagnostics above.
    _Atomic(uint32_t) acc_nb_bursts;    // width <= DSP_NARROWBAND_MAX_BINS
    _Atomic(uint32_t) acc_dens_bursts;  // all gone bursts in the window
    _Atomic(uint64_t) acc_snr_milli;    // sum of SNR(dB) * 1000
```

- [ ] **Step 2: Tap dispatch_gone_burst**

In `dispatch_gone_burst` (dsp_processor.c:132), before `p->user_cb(&out);`, add:

```c
    atomic_fetch_add_explicit(&p->acc_dens_bursts, 1u, memory_order_relaxed);
    if (b->width_bins > 0 && b->width_bins <= DSP_NARROWBAND_MAX_BINS)
        atomic_fetch_add_explicit(&p->acc_nb_bursts, 1u, memory_order_relaxed);
    atomic_fetch_add_explicit(&p->acc_snr_milli,
                              (uint64_t)(int64_t)(b->magnitude_db * 1000.0f),
                              memory_order_relaxed);
```

- [ ] **Step 3: Add the define, typedef, and getter declarations to the header**

In `p4-usb-host/main/dsp_processor.h`, before `#endif`, add:

```c
// Scanner (Phase 1) narrowband-density readout. Counts bursts whose spectral
// width (T60) is <= DSP_NARROWBAND_MAX_BINS, i.e. channel-shaped Iridium
// rather than wide broadband RFI.
#define DSP_NARROWBAND_MAX_BINS 48

typedef struct {
    uint32_t narrowband_bursts;
    uint32_t all_bursts;
    float    mean_snr_db;
} dsp_density_t;

void dsp_processor_read_reset_density(dsp_processor_t *p, dsp_density_t *out);
void dsp_processor_reset_tagger_baseline(dsp_processor_t *p);
```

- [ ] **Step 4: Implement the getters**

In `p4-usb-host/main/dsp_processor.c`, add `#include "fft_burst_tagger.h"` if not already present (it is), then after `dsp_processor_get_stage_stats`:

```c
void dsp_processor_read_reset_density(dsp_processor_t *p, dsp_density_t *out)
{
    if (!p || !out) return;
    uint32_t nb   = atomic_exchange_explicit(&p->acc_nb_bursts, 0u, memory_order_relaxed);
    uint32_t all  = atomic_exchange_explicit(&p->acc_dens_bursts, 0u, memory_order_relaxed);
    uint64_t smil = atomic_exchange_explicit(&p->acc_snr_milli, 0u, memory_order_relaxed);
    out->narrowband_bursts = nb;
    out->all_bursts        = all;
    out->mean_snr_db       = all ? (float)((double)smil / 1000.0 / (double)all) : 0.0f;
}

void dsp_processor_reset_tagger_baseline(dsp_processor_t *p)
{
    if (!p || !p->tagger) return;
    fft_burst_tagger_reset_baseline(p->tagger);
}
```

- [ ] **Step 5: Build the firmware to verify it compiles**

```bash
./scripts/build.sh > /tmp/scanner_build.log 2>&1; grep -E "binary size|error:" /tmp/scanner_build.log | tail -3
```
Expected: a `binary size` line, no `error:`.

- [ ] **Step 6: Commit**

```bash
git add p4-usb-host/main/dsp_processor.c p4-usb-host/main/dsp_processor.h
git commit -m "dsp_processor: narrowband burst-density counters for the scanner"
```

---

### Task 4: class_driver_retune accessor

**Files:**
- Modify: `p4-usb-host/main/class_driver.c` (add function; uses static `rtldev` at line 53)
- Modify: `p4-usb-host/main/class_driver.h` (declare)

**Interfaces:**
- Produces: `esp_err_t class_driver_retune(uint32_t hz);` — live-retunes the R820T2; returns `ESP_ERR_INVALID_STATE` if the SDR is not open/streaming, `ESP_OK` on success, `ESP_FAIL` if the tuner call fails.
- Consumes: `rtlsdr_set_center_freq(rtlsdr_dev_t*, uint32_t)` (librtlsdr.c:882); static `rtldev` (class_driver.c:53).

- [ ] **Step 1: Declare in the header**

In `p4-usb-host/main/class_driver.h`, add (with `#include "esp_err.h"` if not present):

```c
// Live-retune the SDR LO without stopping the stream (Approach A). Safe to
// call from any task once streaming has started; returns ESP_ERR_INVALID_STATE
// before the device is open.
esp_err_t class_driver_retune(uint32_t hz);
```

- [ ] **Step 2: Implement in class_driver.c**

Near the gain-setter that already guards on `rtldev` (around line 217), add:

```c
esp_err_t class_driver_retune(uint32_t hz)
{
    if (!rtldev) return ESP_ERR_INVALID_STATE;
    int r = rtlsdr_set_center_freq(rtldev, hz);
    return (r == 0) ? ESP_OK : ESP_FAIL;
}
```

- [ ] **Step 3: Build to verify it compiles**

```bash
./scripts/build.sh > /tmp/scanner_build.log 2>&1; grep -E "binary size|error:" /tmp/scanner_build.log | tail -3
```
Expected: `binary size` line, no `error:`.

- [ ] **Step 4: Commit**

```bash
git add p4-usb-host/main/class_driver.c p4-usb-host/main/class_driver.h
git commit -m "class_driver: add live LO retune accessor (Approach A)"
```

---

### Task 5: scanner module (hop + scan orchestration)

**Files:**
- Create: `p4-usb-host/main/scanner.h`
- Create: `p4-usb-host/main/scanner.c`
- Modify: `p4-usb-host/main/CMakeLists.txt` (add `scanner.c`, `scanner_map.c` to `SRCS`)
- Modify wiring: wherever `dsp_processor_create` is called at boot, call `scanner_init(dsp)` (find via `grep -rn dsp_processor_create p4-usb-host/main`).

**Interfaces:**
- Produces:
  - `void scanner_init(dsp_processor_t *dsp);`
  - `esp_err_t scanner_hop(uint32_t hz, bool persist);`
  - `void scanner_scan(uint32_t start_hz, uint32_t stop_hz, uint32_t step_hz, uint32_t dwell_ms);`
  - `void scanner_print_last_map(void);`
  - `#define SCAN_START_HZ 1616000000u`, `SCAN_STOP_HZ 1626000000u`, `SCAN_STEP_HZ 2500000u`, `SCAN_DWELL_MS 2000u`, `SCAN_SETTLE_MS 500u`
- Consumes: `class_driver_retune` (Task 4); `dsp_processor_read_reset_density`, `dsp_processor_reset_tagger_baseline`, `dsp_density_t` (Task 3); `scanner_enumerate_centers`, `scanner_rank_hottest`, `scanner_pos_t`, `SCANNER_MAX_POSITIONS`, `scanner_pos_narrowband_rate` (Task 2); `app_config_set_lo_freq_hz` (app_config.h).

- [ ] **Step 1: Create the header**

`p4-usb-host/main/scanner.h`:

```c
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "dsp_processor.h"

#define SCAN_START_HZ  1616000000u
#define SCAN_STOP_HZ   1626000000u
#define SCAN_STEP_HZ   2500000u
#define SCAN_DWELL_MS  2000u
#define SCAN_SETTLE_MS 500u

// Wire the scanner to the running detector (for density reads + baseline
// reset). Call once at boot after dsp_processor_create.
void scanner_init(dsp_processor_t *dsp);

// Live-retune to hz and re-prime the tagger noise floor. persist=true also
// writes NVS lo_hz. Returns class_driver_retune's status.
esp_err_t scanner_hop(uint32_t hz, bool persist);

// Sweep [start,stop] by step; at each center hop, settle, then measure
// narrowband density over dwell_ms; print a ranked map and park on the
// hottest center. Runs in the caller's (serial_cmd) task.
void scanner_scan(uint32_t start_hz, uint32_t stop_hz, uint32_t step_hz, uint32_t dwell_ms);

// Reprint the last density map (or "no scan yet").
void scanner_print_last_map(void);
```

- [ ] **Step 2: Implement scanner.c**

`p4-usb-host/main/scanner.c`:

```c
#include "scanner.h"
#include "scanner_map.h"
#include "class_driver.h"
#include "app_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SCANNER";
static dsp_processor_t *s_dsp = NULL;
static scanner_pos_t    s_map[SCANNER_MAX_POSITIONS];
static int              s_map_n = 0;

void scanner_init(dsp_processor_t *dsp) { s_dsp = dsp; }

esp_err_t scanner_hop(uint32_t hz, bool persist)
{
    esp_err_t err = class_driver_retune(hz);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "hop %lu Hz failed: retune err=%d", (unsigned long)hz, (int)err);
        return err;
    }
    if (s_dsp) {
        dsp_processor_reset_tagger_baseline(s_dsp);
    }
    if (persist) app_config_set_lo_freq_hz(hz);
    ESP_LOGI(TAG, "hopped to %lu Hz (persist=%d) — re-priming", (unsigned long)hz, persist);
    return ESP_OK;
}

void scanner_scan(uint32_t start_hz, uint32_t stop_hz, uint32_t step_hz, uint32_t dwell_ms)
{
    if (!s_dsp) { ESP_LOGW(TAG, "scan: no detector wired"); return; }
    uint32_t centers[SCANNER_MAX_POSITIONS];
    int n = scanner_enumerate_centers(start_hz, stop_hz, step_hz, centers, SCANNER_MAX_POSITIONS);
    if (n == 0) { ESP_LOGW(TAG, "scan: bad range/step"); return; }

    for (int i = 0; i < n; i++) {
        if (scanner_hop(centers[i], false) != ESP_OK) continue;
        vTaskDelay(pdMS_TO_TICKS(SCAN_SETTLE_MS));
        dsp_density_t discard;
        dsp_processor_read_reset_density(s_dsp, &discard);   // drop settle window
        vTaskDelay(pdMS_TO_TICKS(dwell_ms));
        dsp_density_t d;
        dsp_processor_read_reset_density(s_dsp, &d);
        s_map[i] = (scanner_pos_t){ centers[i], d.narrowband_bursts, d.all_bursts,
                                    d.mean_snr_db, dwell_ms };
        ESP_LOGI(TAG, "  %lu Hz: nb=%lu all=%lu snr=%.1f (%.2f nb/s)",
                 (unsigned long)centers[i], (unsigned long)d.narrowband_bursts,
                 (unsigned long)d.all_bursts, d.mean_snr_db,
                 scanner_pos_narrowband_rate(&s_map[i]));
    }
    s_map_n = n;

    int hot = scanner_rank_hottest(s_map, n);
    scanner_print_last_map();
    if (hot >= 0) {
        ESP_LOGI(TAG, "parking on hottest: %lu Hz (%.2f nb/s)",
                 (unsigned long)s_map[hot].center_hz,
                 scanner_pos_narrowband_rate(&s_map[hot]));
        scanner_hop(s_map[hot].center_hz, false);
    }
}

void scanner_print_last_map(void)
{
    if (s_map_n == 0) { ESP_LOGI(TAG, "no scan yet"); return; }
    ESP_LOGI(TAG, "=== density map (nb/s = narrowband bursts/sec) ===");
    for (int i = 0; i < s_map_n; i++)
        ESP_LOGI(TAG, "  %lu Hz  nb/s=%.2f  all=%lu  snr=%.1f dB",
                 (unsigned long)s_map[i].center_hz,
                 scanner_pos_narrowband_rate(&s_map[i]),
                 (unsigned long)s_map[i].all_bursts, s_map[i].mean_snr_db);
}
```

- [ ] **Step 3: Add sources to the app CMakeLists and wire scanner_init**

In `p4-usb-host/main/CMakeLists.txt`, add `"scanner.c"` and `"scanner_map.c"` to the `SRCS` list. Then find the boot wiring:

```bash
grep -rn "dsp_processor_create" p4-usb-host/main
```
Immediately after the `dsp_processor_create(...)` call that stores the handle (e.g. `s_dsp = dsp_processor_create(cb);`), add `#include "scanner.h"` at the top of that file and `scanner_init(<that handle>);` after the create.

- [ ] **Step 4: Build to verify it compiles and links**

```bash
./scripts/build.sh > /tmp/scanner_build.log 2>&1; grep -E "binary size|error:|undefined" /tmp/scanner_build.log | tail -5
```
Expected: `binary size` line, no `error:`/`undefined`.

- [ ] **Step 5: Commit**

```bash
# add scanner sources, the CMakeLists, and ONLY the one boot-wiring file the
# grep in Step 3 identified (replace <wiring_file.c> with that exact path):
git add p4-usb-host/main/scanner.c p4-usb-host/main/scanner.h p4-usb-host/main/CMakeLists.txt p4-usb-host/main/<wiring_file.c>
git commit -m "scanner: live-hop + density-sweep orchestration"
```

---

### Task 6: serial commands (hop / scan / map) + device verification

**Files:**
- Modify: `p4-usb-host/main/serial_cmd.c` (dispatch + handlers; add `#include "scanner.h"`)

**Interfaces:**
- Consumes: `scanner_hop`, `scanner_scan`, `scanner_print_last_map`, scan defaults (Task 5); `uart_puts` (serial_cmd.c:22-area).

- [ ] **Step 1: Add command handlers and dispatch**

In `p4-usb-host/main/serial_cmd.c`, add `#include "scanner.h"` at the top. Add handlers above the dispatch function:

```c
static void cmd_hop(char *args)
{
    char *hz_s  = strtok(args, " \t");
    char *save  = strtok(NULL, " \t");
    if (!hz_s) { uart_puts("ERR usage: hop <hz> [save]\r\n"); return; }
    uint32_t hz = (uint32_t)strtoul(hz_s, NULL, 10);
    bool persist = (save && strcmp(save, "save") == 0);
    esp_err_t e = scanner_hop(hz, persist);
    if (e == ESP_OK) uart_puts("OK hopped (settling ~0.5s)\r\n");
    else if (e == ESP_ERR_INVALID_STATE) uart_puts("ERR SDR not streaming yet\r\n");
    else uart_puts("ERR retune failed\r\n");
}

static void cmd_scan(char *args)
{
    uint32_t start = SCAN_START_HZ, stop = SCAN_STOP_HZ, step = SCAN_STEP_HZ, dwell = SCAN_DWELL_MS;
    char *a;
    if ((a = strtok(args, " \t"))) start = (uint32_t)strtoul(a, NULL, 10);
    if ((a = strtok(NULL, " \t"))) stop  = (uint32_t)strtoul(a, NULL, 10);
    if ((a = strtok(NULL, " \t"))) step  = (uint32_t)strtoul(a, NULL, 10);
    if ((a = strtok(NULL, " \t"))) dwell = (uint32_t)strtoul(a, NULL, 10);
    uart_puts("OK scanning (see log for map)\r\n");
    scanner_scan(start, stop, step, dwell);
}
```

In the command dispatch (`serial_cmd.c`, before the final `ERR unknown command` `uart_puts`), add:

```c
    if (strcmp(cmd, "hop") == 0)  { cmd_hop(strtok(NULL, "")); return; }
    if (strcmp(cmd, "scan") == 0) { cmd_scan(strtok(NULL, "")); return; }
    if (strcmp(cmd, "map") == 0)  { scanner_print_last_map(); return; }
```

Update the final fallback string to `"ERR unknown command (set/get/config/reboot/hop/scan/map/nettest)\r\n"`.

- [ ] **Step 2: Build and flash**

```bash
./scripts/build.sh > /tmp/scanner_build.log 2>&1; grep -E "binary size|error:" /tmp/scanner_build.log | tail -2
```
Then (separate invocation): `./scripts/flash.sh` (stop the serial logger first to free the port).

- [ ] **Step 3: Device verify — hop retunes without reboot or stall**

Start the serial logger, then send `hop 1620000000` over `/dev/ttyACM0` (pyserial, DTR/RTS low). Expected in the log within ~1 s:
- `SCANNER: hopped to 1620000000 Hz` and `[R82XX] set_pll freq=1623570000` (= 1620 + 3.57 MHz IF).
- No `SPI_FAST_FLASH_BOOT` (no reboot); `STATUS: rate=` stays ~4.8 MB/s across the hop.

- [ ] **Step 4: Device verify — positive control (the map means something)**

The smoke tone injector detects a known tone (see `SMOKE: Injecting tone at FFT bin 200`). With a known CW/tone present at a known frequency, run `scan 1616000000 1626000000 2500000 2000` and confirm the `SCANNER: density map` ranks the center nearest the tone highest in `nb/s`. If no controllable tone source is available on live RF, instead assert the weaker property: the map prints all positions with finite `nb/s` and `parking on hottest` selects the max — and record that the ranking's *correctness* is validated by the host test in Task 2 plus this end-to-end plumbing check.

- [ ] **Step 5: Device verify — USB stability across many hops**

Send 10 hops in a row (e.g. sweep the band twice). Expected: no `SPI_FAST_FLASH_BOOT`, no `STATUS-ERR` stream stall, `rate` holds ~4.8 MB/s, `worker[dropped]` behaves as before. If a hop glitches the bulk pipe (rate drops / stall watchdog fires), STOP and switch to Approach C (one-buffer flush barrier in ingest) — note this in the plan's follow-up and open a decision with the user.

- [ ] **Step 6: Commit**

```bash
git add p4-usb-host/main/serial_cmd.c
git commit -m "serial_cmd: add hop/scan/map scanner commands"
```

---

## Notes for the implementer

- **Do not touch worker_core1.c / uw_correlator.c / the PIE path.** The deferred smoke-gate PIE-save hang (`project_pie_save_deadlock_smoke`) is unrelated to this change; keeping the scanner control-plane-only is what avoids re-triggering it.
- **Run build and flash as separate tool invocations** (project convention); stop the serial logger before flashing to free the port.
- **The antenna is the real limit** — device verification here is *mechanism* correctness (hop works, map plumbs through, USB stable), not real ACARS. Real-traffic validation waits for a better antenna.
- If `class_driver.h` lacks `#include "esp_err.h"`, add it (Task 4 returns `esp_err_t`).
