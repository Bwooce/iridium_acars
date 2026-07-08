# Near-DC Tagger Mask + Fine Diagnostic Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Kill the self-inflicted RTL DC/LO artifact burst-flood by adding a configurable, live-tunable near-DC exclusion window to the FFT burst tagger, plus an always-on fine near-DC histogram to pin the artifact bins and monitor thermal/temporal drift.

**Architecture:** Three isolated units. (A) A per-FFT-bin diagnostic histogram in `worker_core1.c` covering DC ±192 bins, exposed on a new `/diag/dcfine` endpoint with kHz metadata. (B) A DC-exclusion window in `fft_burst_tagger.c` backed by **file-static** state (not struct fields — the struct `sizeof` must stay byte-identical), applied at the single new-burst-scan choke point. (C) NVS/serial plumbing mirroring the existing `tag_thr` knob so the window is tunable without a reflash. Default is **disabled** (behaviour bit-identical to today) until the diagnostic pins the bins and we bench-tune.

**Tech Stack:** C (ESP-IDF for device, plain C host tests via `build-host/` CMake+CTest), ESP32-P4, RTL-SDR @ 2.5 MSPS, 2048-pt FFT.

## Global Constraints

- **Struct `sizeof(fft_burst_tagger_t)` MUST stay byte-identical (66672 on riscv32).** Growing it trips the P4 PIE position-sensitivity bug. No new struct fields — use file-static state. (Verbatim from the tagger struct comments.)
- **Default behaviour must be bit-identical to today.** The mask ships DISABLED (`dc_mask_lo=+1, dc_mask_hi=-1` ⇒ empty range).
- **`N = FBT_FFT_SIZE = 2048`; DC bin = `N/2 = 1024`.** Bin width = `FS_DETECT_HZ/N = 2500000/2048 ≈ 1220.703 Hz`.
- **Acceptance is BCH-ok/hour, NOT burst counts** (per `feedback_115_cfo_peak_filter_reverted`: a host-green filter killed device decode).
- **Device-smoke is mandatory** after any DSP-path commit (`feedback_device_smoke_mandatory_after_dsp`): `.githooks/pre-push` enforces a `Smoke-verified:` trailer. Smoke: `OUTDIR=/tmp/smoke_pathA2 scripts/smoke_run.sh raw` (GOLDEN `matched≥40`, ~61 healthy), then `OUTDIR=/tmp/smoke_pathA2 scripts/smoke_run.sh restore` to rebuild production.
- **Build/flash are SEPARATE invocations** (`scripts/build.sh` then `scripts/flash.sh`); flash port = CH343 `1a86:55d3` (detect by vendor, ACM index shuffles).
- **Host tests build/run under `build-host/`** via CTest.

---

### Task 1: Unit A — fine near-DC diagnostic histogram + `/diag/dcfine`

**Files:**
- Create: `p4-usb-host/main/worker_dcfine.h` (pure, host-includable: constants + `worker_dcfine_index()` inline)
- Modify: `p4-usb-host/main/worker_core1.h` (add getter decl only)
- Modify: `p4-usb-host/main/worker_core1.c` (include `worker_dcfine.h`, add `s_hist_dcfine[]`, record in `worker_core1_push_burst`, add getter)
- Modify: `p4-usb-host/main/http_server.c` (include `worker_dcfine.h`, add `diag_dcfine_get` handler + route)
- Create: `tests/host/test_dcfine_index.c` (unit test for the pure bucket-index math)
- Modify: `tests/host/CMakeLists.txt` (register the new test)

> **Why a separate header:** `worker_core1.h` includes `esp_err.h`/`dsp_processor.h` (device-only), so a host test can't include it. The pure index math + constants live in a standalone `worker_dcfine.h` with only `<math.h>`, included by `worker_core1.c`, `http_server.c`, and the host test.

**Interfaces:**
- Produces:
  - `#define WORKER_DCFINE_HALF 192` and `#define WORKER_DCFINE_BINS 384` (in `worker_dcfine.h`)
  - `static inline int worker_dcfine_index(float rel_freq_hz)` → `0..383`, or `-1` if outside the DC ±192-bin window (in `worker_dcfine.h`)
  - `void worker_core1_get_dcfine(uint32_t *out, int max, uint32_t *total_out)` (in `worker_core1.h`/`.c`)
  - HTTP `GET /diag/dcfine` → `{"dcfine_bin0_Hz":-234432,"dcfine_bin_Hz_width":1221,"dcfine_total":N,"dcfine":[...384...]}`

- [ ] **Step 1: Write the failing host test for the bucket-index math**

Create `tests/host/test_dcfine_index.c`:

```c
// Unit test for worker_dcfine_index() — the pure bin-index math for the
// fine near-DC diagnostic. Kept in worker_core1.h as a static inline so it
// is testable on host without pulling in the device worker.
#include <stdio.h>
#include "worker_dcfine.h"

static int fails = 0;
static void expect_eq(const char *what, int got, int want)
{
    if (got != want) {
        printf("  FAIL %s: got %d want %d\n", what, got, want);
        fails++;
    }
}

int main(void)
{
    // DC (0 Hz offset) → centre bucket = WORKER_DCFINE_HALF.
    expect_eq("rel=0", worker_dcfine_index(0.0f), WORKER_DCFINE_HALF);
    // +1 bin (≈+1220.7 Hz) → centre+1.
    expect_eq("rel=+1220.7", worker_dcfine_index(1220.703f), WORKER_DCFINE_HALF + 1);
    // -1 bin → centre-1.
    expect_eq("rel=-1220.7", worker_dcfine_index(-1220.703f), WORKER_DCFINE_HALF - 1);
    // The observed artifact region ~ -100 kHz → -82 bins → centre-82.
    expect_eq("rel=-100k", worker_dcfine_index(-100000.0f), WORKER_DCFINE_HALF - 82);
    // Just inside the low edge: -191 bins is valid (index 1).
    expect_eq("rel=-191bins", worker_dcfine_index(-191.0f * 1220.703f), 1);
    // Outside the window (beyond -192 bins) → -1 (ignored).
    expect_eq("rel=-300k", worker_dcfine_index(-300000.0f), -1);
    // Outside the window (beyond +192 bins) → -1 (ignored).
    expect_eq("rel=+300k", worker_dcfine_index(300000.0f), -1);

    if (fails) { printf("test_dcfine_index: %d FAILURES\n", fails); return 1; }
    printf("test_dcfine_index: PASS\n");
    return 0;
}
```

- [ ] **Step 2: Register the test and run it to verify it fails to compile**

Add to `tests/host/CMakeLists.txt` (near the other tagger tests, ~line 258):

```cmake
add_executable(test_dcfine_index test_dcfine_index.c)
target_include_directories(test_dcfine_index PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../p4-usb-host/main)
target_link_libraries(test_dcfine_index PRIVATE m)
add_test(NAME test_dcfine_index COMMAND test_dcfine_index)
```

Run:
```bash
cmake --build build-host --target test_dcfine_index 2>&1 | tail -20
```
Expected: FAIL — `worker_dcfine.h` does not exist yet (`fatal error: worker_dcfine.h: No such file`).

- [ ] **Step 3: Create the pure `worker_dcfine.h` and declare the getter**

Create `p4-usb-host/main/worker_dcfine.h`:

```c
#pragma once
// Pure (no device deps) fine near-DC diagnostic constants + bucket math.
// Split out of worker_core1.h (which pulls in esp_err.h/dsp_processor.h) so
// it is host-testable. Included by worker_core1.c, http_server.c, and the
// host test. (near-DC tagger-mask work, 2026-07-08.)
#include <math.h>

// Per-FFT-bin occupancy over DC ±WORKER_DCFINE_HALF bins, to pin the
// tuner-internal DC/LO artifact and watch it drift over time.
// Bin width = FS_DETECT_HZ / FBT_FFT_SIZE = 2500000/2048 ≈ 1220.703 Hz.
#define WORKER_DCFINE_HALF 192
#define WORKER_DCFINE_BINS (2 * WORKER_DCFINE_HALF)   // 384

// Map a detection's offset-from-LO (Hz) to a fine-histogram bucket.
// Returns 0..WORKER_DCFINE_BINS-1, or -1 if outside the ±HALF window.
static inline int worker_dcfine_index(float rel_freq_hz)
{
    // rel / (FS/N) = rel * N / FS  → signed bin offset from DC.
    int off = (int)lrintf(rel_freq_hz * 2048.0f / 2500000.0f);
    int idx = off + WORKER_DCFINE_HALF;
    if (idx < 0 || idx >= WORKER_DCFINE_BINS) return -1;
    return idx;
}
```

Add the getter declaration to `worker_core1.h` next to `worker_core1_get_histograms`:

```c
// Copy the fine near-DC histogram (cumulative since boot). Copies
// min(max, WORKER_DCFINE_BINS) entries; *total_out gets the sum (may be NULL).
void worker_core1_get_dcfine(uint32_t *out, int max, uint32_t *total_out);
```

- [ ] **Step 4: Run the test to verify it passes**

```bash
cmake --build build-host --target test_dcfine_index 2>&1 | tail -5 && ctest --test-dir build-host -R test_dcfine_index --output-on-failure
```
Expected: `test_dcfine_index: PASS` / `100% tests passed`.

- [ ] **Step 5: Add the recorder + getter to `worker_core1.c`**

Add `#include "worker_dcfine.h"` with the other includes at the top of `worker_core1.c`. Then alongside `s_hist_freq` (~line 279) add:

```c
static _Atomic uint32_t s_hist_dcfine[WORKER_DCFINE_BINS];

static inline void hist_dcfine_record(float rel_freq_hz)
{
    int idx = worker_dcfine_index(rel_freq_hz);
    if (idx >= 0) s_hist_dcfine[idx]++;
}
```

In `worker_core1_push_burst` (~line 1251), right after the existing `hist_freq_record(burst->rel_freq_hz);`:

```c
    hist_dcfine_record((float)burst->rel_freq_hz);              // fine near-DC diagnostic
```

Add the getter next to `worker_core1_get_histograms` (~line 1301):

```c
void worker_core1_get_dcfine(uint32_t *out, int max, uint32_t *total_out)
{
    if (!out) return;
    uint32_t total = 0;
    int n = max < WORKER_DCFINE_BINS ? max : WORKER_DCFINE_BINS;
    for (int i = 0; i < n; i++) {
        out[i] = s_hist_dcfine[i];
        total += s_hist_dcfine[i];
    }
    if (total_out) *total_out = total;
}
```

- [ ] **Step 6: Add the `/diag/dcfine` HTTP handler + route in `http_server.c`**

Add a handler near `diag_histograms_get` (~line 400):

```c
// /diag/dcfine — fine near-DC occupancy histogram (WORKER_DCFINE_BINS
// buckets, ~1221 Hz each, DC ±234 kHz). Separate endpoint because the
// 384-value array does not fit /diag/histograms' shared 3072-byte body.
static esp_err_t diag_dcfine_get(httpd_req_t *req)
{
    static uint32_t dc[WORKER_DCFINE_BINS];
    uint32_t total = 0;
    worker_core1_get_dcfine(dc, WORKER_DCFINE_BINS, &total);

    char body[4096];
    int  n = 0, m;
    // bin0_Hz = -HALF * width; width = round(FS/N) = 1221. Consumer:
    // offset_Hz(i) = bin0_Hz + i*width; i=HALF is DC.
    m = snprintf(body, sizeof(body),
                 "{\"dcfine_bin0_Hz\":%d,\"dcfine_bin_Hz_width\":1221,"
                 "\"dcfine_total\":%u,\"dcfine\":[",
                 -(WORKER_DCFINE_HALF * 1221), (unsigned)total);
    if (m > 0) n = m;
    for (int i = 0; i < WORKER_DCFINE_BINS; i++) {
        if (n >= (int)sizeof(body) - 2) break;
        m = snprintf(body + n, sizeof(body) - n, "%s%u", i ? "," : "", (unsigned)dc[i]);
        if (m > 0) n += m;
    }
    if (n < (int)sizeof(body) - 2) n += snprintf(body + n, sizeof(body) - n, "]}");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, n);
}
```

Add the route to the `routes[]` array (~line 1668, next to the other `/diag/*`):

```c
        {.uri = "/diag/dcfine", .method = HTTP_GET, .handler = diag_dcfine_get, .user_ctx = NULL},
```

(Route count goes 24→25; the `_Static_assert(... <= HTTPD_URI_LIMIT)` at line 1691 with `HTTPD_URI_LIMIT 32` covers it — no bump needed. Add `#include "worker_dcfine.h"` to `http_server.c` for the constants; `worker_core1.h` is already included for `worker_core1_get_histograms`.)

- [ ] **Step 7: Build the firmware to verify it compiles**

```bash
scripts/build.sh 2>&1 | tail -15
```
Expected: build succeeds (`Project build complete`). Fix any missing-include errors.

- [ ] **Step 8: Commit**

```bash
git add p4-usb-host/main/worker_dcfine.h p4-usb-host/main/worker_core1.h \
        p4-usb-host/main/worker_core1.c p4-usb-host/main/http_server.c \
        tests/host/test_dcfine_index.c tests/host/CMakeLists.txt
git commit -m "worker: add fine near-DC diagnostic histogram + /diag/dcfine

Per-FFT-bin occupancy over DC +/-192 bins (~1221 Hz/bin, +/-234 kHz),
recorded for every pushed detection. Pins the tuner-internal DC/LO
artifact and monitors thermal/temporal drift. Always-on (~1.5 KB .bss);
new /diag/dcfine endpoint keeps it off the shared /diag/histograms body."
```

---

### Task 2: Unit B — configurable DC-exclusion window in the tagger

**Files:**
- Modify: `common/iridium_decoder/fft_burst_tagger.h` (declare `fft_burst_tagger_set_dc_mask`)
- Modify: `common/iridium_decoder/fft_burst_tagger.c` (file-static window + `bin_in_dc_mask` + scan hook + setter)
- Create: `tests/host/test_tagger_dc_mask.c` (TDD: inside-window suppressed, outside emitted, disabled=default)
- Modify: `tests/host/CMakeLists.txt` (register the new test)

**Interfaces:**
- Consumes: `fft_burst_tagger_init`, `fft_burst_tagger_step`, `fft_burst_tagger_set_start`, `fft_burst_tagger_destroy`, `fbt_burst_t`, `FBT_FFT_SIZE`, `FBT_MAX_BURSTS`, `FBT_HISTORY_SIZE` (existing).
- Produces: `void fft_burst_tagger_set_dc_mask(fft_burst_tagger_t *t, int lo, int hi);` — `lo`/`hi` are signed FFT-bin offsets from DC (`N/2`). `lo > hi` ⇒ disabled (no bins excluded). Bins in `[N/2+lo, N/2+hi]` (clamped to `[0,N-1]`) can never spawn a new burst. State is process-global (matches the tagger's existing file-static singleton design); default DISABLED.

- [ ] **Step 1: Write the failing host test**

Create `tests/host/test_tagger_dc_mask.c`:

```c
// TDD for the configurable near-DC exclusion window. Feeds seeded noise to
// prime the per-bin EMA, then a strong tone at a chosen bin, and checks that
// the DC-mask window gates NEW bursts at that bin — and ONLY there.
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "fft_burst_tagger.h"

#define N        FBT_FFT_SIZE      // 2048
#define DC       (N / 2)           // 1024
#define FS       2500000.0

static int32_t s_baseline_history[FBT_HISTORY_SIZE * FBT_FFT_SIZE];

// Fill n_complex interleaved int16 IQ with seeded uniform noise in [-amp,amp].
static void gen_noise(int16_t *buf, int n_complex, unsigned *seed, int amp)
{
    for (int i = 0; i < 2 * n_complex; i++)
        buf[i] = (int16_t)((int)(rand_r(seed) % (2 * amp + 1)) - amp);
}

// Add a complex tone at FFT bin `bin` (absolute, 0..N-1) with amplitude amp.
static void add_tone(int16_t *buf, int n_complex, int bin, int amp, double *phase)
{
    double w = 2.0 * M_PI * (double)(bin - DC) / (double)N; // cycles/sample
    for (int i = 0; i < n_complex; i++) {
        buf[2 * i + 0] = (int16_t)(buf[2 * i + 0] + (int)(amp * cos(*phase)));
        buf[2 * i + 1] = (int16_t)(buf[2 * i + 1] + (int)(amp * sin(*phase)));
        *phase += w;
    }
}

// Run the tagger over `prime` noise-only steps then `active` noise+tone steps
// at `tone_bin`; return how many NEW bursts landed within ±16 bins of tone_bin.
static int run_case(int tone_bin, int mask_lo, int mask_hi)
{
    fft_burst_tagger_t *t = fft_burst_tagger_init(
        2 * FBT_FFT_SIZE, (int)(FS * 16e-3), 32, 10.0f, s_baseline_history);
    fft_burst_tagger_set_start(t, 0);
    fft_burst_tagger_set_dc_mask(t, mask_lo, mask_hi);

    int16_t     buf[2 * N];
    fbt_burst_t nb[FBT_MAX_BURSTS], gb[FBT_MAX_BURSTS];
    unsigned    seed = 1234;
    double      phase = 0.0;
    int         hits = 0;

    for (int s = 0; s < FBT_HISTORY_SIZE + 60; s++) {
        gen_noise(buf, N, &seed, 200);
        if (s >= FBT_HISTORY_SIZE + 8) add_tone(buf, N, tone_bin, 9000, &phase);
        int nn = FBT_MAX_BURSTS, ng = FBT_MAX_BURSTS;
        bool primed = fft_burst_tagger_step(t, buf, NULL, nb, &nn, gb, &ng);
        if (primed)
            for (int i = 0; i < nn; i++)
                if (abs(nb[i].center_bin - tone_bin) <= 16) hits++;
    }
    fft_burst_tagger_destroy(t);
    return hits;
}

int main(void)
{
    int fails = 0;
    int tone = DC - 40; // ~ -49 kHz, in the near-DC artifact region

    // A) Mask DISABLED (lo>hi) → the tone triggers bursts.
    int a = run_case(tone, 1, -1);
    printf("A disabled: hits=%d (want >0)\n", a);
    if (a <= 0) fails++;

    // B) Mask window COVERS the tone bin → zero bursts there.
    int b = run_case(tone, -44, -36);
    printf("B covered:  hits=%d (want 0)\n", b);
    if (b != 0) fails++;

    // C) Mask window ELSEWHERE (does not cover the tone) → tone still triggers.
    int c = run_case(tone, 30, 38);
    printf("C elsewhere:hits=%d (want >0)\n", c);
    if (c <= 0) fails++;

    if (fails) { printf("test_tagger_dc_mask: %d FAILURES\n", fails); return 1; }
    printf("test_tagger_dc_mask: PASS\n");
    return 0;
}
```

- [ ] **Step 2: Register the test and run it to verify it fails**

Add to `tests/host/CMakeLists.txt` (near `test_fft_burst_tagger`, ~line 258). Mirror that target's source list and includes (it compiles `fft_burst_tagger.c`):

```cmake
add_executable(test_tagger_dc_mask
    test_tagger_dc_mask.c
    ${COMMON_DIR}/iridium_decoder/fft_burst_tagger.c)
target_include_directories(test_tagger_dc_mask PRIVATE ${COMMON_DIR}/iridium_decoder)
target_link_libraries(test_tagger_dc_mask PRIVATE m)
add_test(NAME test_tagger_dc_mask COMMAND test_tagger_dc_mask)
```

(Match whatever `test_fft_burst_tagger` uses for `COMMON_DIR` / include dirs at line 249-258; copy its pattern exactly.)

Run:
```bash
cmake --build build-host --target test_tagger_dc_mask 2>&1 | tail -20
```
Expected: FAIL — `fft_burst_tagger_set_dc_mask` undefined (not yet implemented).

- [ ] **Step 3: Implement the mask in `fft_burst_tagger.c`**

Add file-static window state near the other file-statics (e.g. by `s_clamp_run`, or just under `#define N FBT_FFT_SIZE` at line 59). DISABLED default:

```c
// Configurable near-DC exclusion window (near-DC tagger-mask work,
// 2026-07-08). Signed FFT-bin offsets from DC (N/2). lo>hi = DISABLED.
// File-static (not a struct field) so sizeof(fft_burst_tagger_t) stays
// byte-identical — growing the struct trips the P4 PIE position bug.
// Matches the tagger's existing singleton file-static idiom (s_clamp_run).
static int s_dc_mask_lo = 1;   // default: empty range => disabled
static int s_dc_mask_hi = -1;

static inline bool bin_in_dc_mask(int bin)
{
    int lo = (N / 2) + s_dc_mask_lo;
    int hi = (N / 2) + s_dc_mask_hi;
    return bin >= lo && bin <= hi;   // false for all bins when lo>hi
}
```

In `create_new_bursts_internal`, at the scan loop (~line 756), extend the existing guard:

```c
        if (!t->burst_mask[bin] || bin_in_dc_mask(bin)) continue;
```

Add the public setter (near `fft_burst_tagger_set_start`):

```c
void fft_burst_tagger_set_dc_mask(fft_burst_tagger_t *t, int lo, int hi)
{
    (void)t; // window is process-global (single detector); t kept for API symmetry
    // Clamp so DC+lo / DC+hi stay in [0, N-1]; an inverted range = disabled.
    if (lo < -(N / 2)) lo = -(N / 2);
    if (hi >  (N / 2) - 1) hi = (N / 2) - 1;
    s_dc_mask_lo = lo;
    s_dc_mask_hi = hi;
}
```

- [ ] **Step 4: Declare the setter in `fft_burst_tagger.h`**

After `fft_burst_tagger_set_start` (~line 130):

```c
// Set the near-DC new-burst exclusion window. `lo`/`hi` are signed FFT-bin
// offsets from DC (N/2); bins in [DC+lo, DC+hi] can never spawn a new burst.
// `lo > hi` disables the window (default). Process-global (single detector);
// safe to call live to re-tune. Does not affect the noise-floor EMA — only
// new-burst declaration.
void fft_burst_tagger_set_dc_mask(fft_burst_tagger_t *t, int lo, int hi);
```

- [ ] **Step 5: Run the test to verify it passes**

```bash
cmake --build build-host --target test_tagger_dc_mask 2>&1 | tail -5 && ctest --test-dir build-host -R test_tagger_dc_mask --output-on-failure
```
Expected: `test_tagger_dc_mask: PASS`. If case A emits 0 (tone too weak to prime-then-trigger), raise the tone amplitude (9000→15000) or lengthen active steps — do NOT weaken the assertion.

- [ ] **Step 6: Run the full host tagger regression to confirm default is bit-identical**

```bash
ctest --test-dir build-host -R "tagger|fft_burst" --output-on-failure 2>&1 | tail -20
```
Expected: all pass — with the window at its DISABLED default, `test_fft_burst_tagger` / `test_tagger_*` counts are unchanged (the new guard short-circuits to the old behaviour).

- [ ] **Step 7: Commit**

```bash
git add common/iridium_decoder/fft_burst_tagger.c common/iridium_decoder/fft_burst_tagger.h \
        tests/host/test_tagger_dc_mask.c tests/host/CMakeLists.txt
git commit -m "tagger: configurable near-DC new-burst exclusion window

File-static signed bin-offset window (lo>hi = disabled default) applied at
the single new-burst scan choke point; excludes the tuner-internal DC/LO
artifact bins from spawning bursts without touching the noise EMA. No struct
field, so sizeof stays byte-identical (P4 PIE position bug). Default disabled
= bit-identical to prior behaviour."
```

---

### Task 3: Unit C — NVS + serial plumbing, wire into detector create

**Files:**
- Modify: `p4-usb-host/main/app_config.h` (struct fields + setter decls)
- Modify: `p4-usb-host/main/app_config.c` (defaults, NVS load, `SET_FIELD_NUM` setters)
- Modify: `p4-usb-host/main/serial_cmd.c` (get/set for `dcmask_lo`/`dcmask_hi`)
- Modify: `p4-usb-host/main/dsp_processor.c` (call `fft_burst_tagger_set_dc_mask` at create)

**Interfaces:**
- Consumes: `fft_burst_tagger_set_dc_mask` (Task 2); `app_config_t`, `nvs_get_i16_or`, `commit_one_i16`, `SET_FIELD_NUM` (existing).
- Produces: NVS keys `dcmask_lo` / `dcmask_hi` (int16, default `+1` / `-1`); `app_config_set_dcmask_lo(int16_t)` / `app_config_set_dcmask_hi(int16_t)`; serial `get dcmask_lo|dcmask_hi` and `set dcmask_lo|dcmask_hi <n>`.

- [ ] **Step 1: Add struct fields + setter decls in `app_config.h`**

In the `app_config_t` struct (after `coalesce_min_bursts`, ~line 60):

```c
    // Near-DC tagger exclusion window (near-DC tagger-mask work). Signed
    // FFT-bin offsets from DC; lo>hi = disabled. Applied at detector create
    // and live-settable. NVS keys "dcmask_lo"/"dcmask_hi".
    int16_t dcmask_lo;
    int16_t dcmask_hi;
```

With the other setter declarations (~line 94):

```c
esp_err_t app_config_set_dcmask_lo(int16_t v);
esp_err_t app_config_set_dcmask_hi(int16_t v);
```

- [ ] **Step 2: Add defaults, NVS load, and setters in `app_config.c`**

Near `#define DEFAULT_TAGGER_THRESHOLD_DB` (~line 27):

```c
#define DEFAULT_DCMASK_LO 1    // lo>hi => disabled by default
#define DEFAULT_DCMASK_HI (-1)
```

In the defaults block (~line 116, after `coalesce_min_bursts`):

```c
    s_cfg.dcmask_lo = DEFAULT_DCMASK_LO;
    s_cfg.dcmask_hi = DEFAULT_DCMASK_HI;
```

In the NVS load block (~line 157, after the `coal_n` line):

```c
    nvs_get_i16_or(h, "dcmask_lo", &s_cfg.dcmask_lo, DEFAULT_DCMASK_LO);
    nvs_get_i16_or(h, "dcmask_hi", &s_cfg.dcmask_hi, DEFAULT_DCMASK_HI);
```

With the other `SET_FIELD_NUM` lines (~line 274):

```c
SET_FIELD_NUM(app_config_set_dcmask_lo, dcmask_lo, int16_t, "dcmask_lo", commit_one_i16)
SET_FIELD_NUM(app_config_set_dcmask_hi, dcmask_hi, int16_t, "dcmask_hi", commit_one_i16)
```

- [ ] **Step 3: Add serial get/set in `serial_cmd.c`**

In the `get` handler chain (after the `coal_n` branch, ~line 88):

```c
    else if (strcmp(key, "dcmask_lo") == 0)
        snprintf(buf, sizeof(buf), "%d\r\n", (int)c.dcmask_lo);
    else if (strcmp(key, "dcmask_hi") == 0)
        snprintf(buf, sizeof(buf), "%d\r\n", (int)c.dcmask_hi);
```

In the `set` handler chain (after the `coal_n`/relevant numeric branch):

```c
    else if (strcmp(key, "dcmask_lo") == 0)
        rc = app_config_set_dcmask_lo((int16_t)atoi(val));
    else if (strcmp(key, "dcmask_hi") == 0)
        rc = app_config_set_dcmask_hi((int16_t)atoi(val));
```

Also add both to the always-printed config dump if there is one (mirror the `tag_thr=`/`coal_n=` lines at ~52-54):

```c
    snprintf(buf, sizeof(buf), "dcmask_lo=%d dcmask_hi=%d\r\n", (int)c.dcmask_lo, (int)c.dcmask_hi);
```
(append to the same output path as the other fields).

- [ ] **Step 4: Wire into detector create in `dsp_processor.c`**

Right after `fft_burst_tagger_set_start(p->tagger, 0);` (~line 327):

```c
    fft_burst_tagger_set_dc_mask(p->tagger, cfg.dcmask_lo, cfg.dcmask_hi);
```

(Confirm `cfg` is the local `app_config_t` already used for `cfg.tagger_threshold_db`/`cfg.coalesce_min_bursts` in this function; if the include for `fft_burst_tagger.h` isn't already present it is — the file calls `fft_burst_tagger_init`.)

- [ ] **Step 5: Build the firmware**

```bash
scripts/build.sh 2>&1 | tail -15
```
Expected: build succeeds. Fix any type/decl mismatches.

- [ ] **Step 6: Commit**

```bash
git add p4-usb-host/main/app_config.h p4-usb-host/main/app_config.c \
        p4-usb-host/main/serial_cmd.c p4-usb-host/main/dsp_processor.c
git commit -m "config: NVS + serial knobs for near-DC tagger mask window

dcmask_lo/dcmask_hi (int16, signed bin offsets from DC, default disabled)
loaded from NVS, settable over serial, and applied at detector create via
fft_burst_tagger_set_dc_mask. Mirrors the tag_thr plumbing; live-tunable
without a reflash."
```

---

### Task 4: Device-smoke gate + push

**Files:** none (verification + push only).

- [ ] **Step 1: Run the device-smoke RAW GOLDEN**

```bash
OUTDIR=/tmp/smoke_pathA2 scripts/smoke_run.sh raw 2>&1 | tee /tmp/smoke_pathA2/dcmask_smoke.log | tail -30
```
Expected: `GOLDEN` line with `matched >= 40` (~61 healthy). Because the mask defaults DISABLED and the diagnostic is additive, decode must be unchanged. If `matched < 40`, STOP — investigate (most likely a heap-position shift from the diag `.bss` or an accidental behaviour change); do not push.

- [ ] **Step 2: Restore the production build**

```bash
OUTDIR=/tmp/smoke_pathA2 scripts/smoke_run.sh restore 2>&1 | tail -10
```
Expected: production sdkconfig rebuilt (smoke taint removed).

- [ ] **Step 3: Add the Smoke-verified trailer to the DSP-path commits and push**

The pre-push hook (`.githooks/pre-push`) requires a `Smoke-verified:` trailer on DSP-path commits. Amend the three firmware commits' messages (or add the trailer as the hook expects — check `.githooks/pre-push` for the exact required form and which commits it inspects) with e.g.:

```
Smoke-verified: RAW GOLDEN matched=<N> (>=40) @ <shortsha> 2026-07-08
```

Then:
```bash
git fetch && git status -sb
git push
```
Expected: hook passes, push succeeds. If the hook rejects, read its message and fix the trailer form — do not `--no-verify`.

- [ ] **Step 4: Flash + live-verify the diagnostic and knob on the bench**

```bash
scripts/flash.sh 2>&1 | tail -15
```
Then (device at `192.168.1.235`):
```bash
curl -s http://192.168.1.235/diag/dcfine | head -c 400
```
Expected: JSON with `dcfine_bin0_Hz`, `dcfine_bin_Hz_width:1221`, `dcfine_total`, and a 384-element `dcfine` array populating over a few seconds. Identify the peak bucket → convert to kHz (`bin0_Hz + i*1221`) → that is the artifact offset.

Set the window over serial to cover it (example, tune to the observed peak) and confirm the flood drops without hurting decode over a dwell:
```bash
# via serial_cmd over /dev/ttyACM0 (CH343): set dcmask_lo <lo> ; set dcmask_hi <hi> ; reboot
# then watch: curl /diag/histograms (snr_pushed flood), /status (decode.frames), FRMDEC bch_ok/h
```
Acceptance: `snr_pushed` flood + `worker_dropped` drop AND `bch_ok/h` does not regress.

---

## Out of scope (follow-up, separate commit)

Once the bench dwell confirms a tuned, non-disabled window that reduces the flood with **no** BCH-ok/hour regression, bake it as the default (`DEFAULT_DCMASK_LO/HI`) in a follow-up commit — again device-smoke-verified. Record the tuned window + any observed drift in the handoff / project memory. An adaptive/auto-locating mask is a possible later enhancement if drift data warrants it; not in this plan.
