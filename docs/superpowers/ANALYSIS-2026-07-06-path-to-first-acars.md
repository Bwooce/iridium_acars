# Deep analysis — path to the first ACARS decode (2026-07-06)

Written after a four-way parallel investigation (tagger flood, DMA-internal
budget, ACARS evidence map, PIE deadlock) over the live bench logs
(`/tmp/p4_serial.log{,.1}`, ~14 h at LO 1622 MHz on the `freq-scanner` build),
the code, the vendored IDF, and the reference toolchain. Companion to
`HANDOFF-2026-07-06-freq-scanner.md`, several of whose claims this document
corrects.

## 1. Corrections to the handoff narrative (all evidence-verified)

1. **"Still ZERO real decodes" is FALSE.** The 14 h logs contain **233 real
   frame decodes** (`FRMDEC: FRAME: LW.DA … blocks=10/10 crc=OK`, `IBC sv=6
   beam=28`) at 22–31 dB SNR, ~17/h, pass-clustered (DEMOD SUCCESS per 10 min
   swings 1→112). The bench monitor reported `uw=+0` all night because its
   grep patterns (`UW match`, `LW:`) never matched the actual log format
   (`FRMDEC: FRAME: LW.DA`, `bch_ok=1`) — a watcher with no positive control.
2. **"The antenna is the binding constraint" is REFUTED for this regime.**
   Real decodable bursts exist and are being *discarded*: the worker sheds
   ≥99.9 % of tagged bursts (median 730 dropped/s vs ~0.66 processed/s). The
   antenna bounds the ceiling; firmware sets the floor.
3. **`STATUS: frames=~883/s` is a log artifact** — it counts FFT steps
   (`status_logger.c:202`), ~72–77 % of the true 1221 steps/s, proportional to
   USB byte rate and constant by construction. The real flood metric is
   `worker[dropped=]` in STATUS-ERR.
4. **The overnight reboots are NOT a worker-stall watchdog** — all four are
   Core-1 panics `ERROR: Coprocessors must not be used in ISRs!` → clean
   SW_CPU_RESET: the PIE lazy-save machinery failing **in production**. The
   PIE problem is a production correctness bug, not a smoke-fixture nuisance.
5. **The scanner's DMA problem is churn, not size.** One hop needs ~21–25 EP0
   control transfers but holds <1 KB of DMA-INT at a time. Scans fail because
   ~22 alloc/free cycles per hop grind against a heap whose largest free block
   is 0–2 KB at runtime.
6. **Pushing is a policy call, not blocked**: `.githooks/pre-push` accepts an
   honest `Smoke-skip: <reason>` trailer (line 46; format at line 87).

## 2. The goal, re-anchored

The goal is **one ACARS message decoded from live SDR data**. Evidence map:

| Stage | Proof class | Status |
|---|---|---|
| USB ingest → tagger → demod/UW → classify → BCH → **IDA decode** | LIVE over-the-air | **PROVEN** (233 frames, crc=OK payloads) |
| IDA → SBD reassembler | synthetic only | unproven on real data — **a DATA gap, not a known bug**: no real SBD user payload (≥5 B) has ever entered the system |
| SBD → libacars → ACARS text | synthetic only (link test) | unproven; never invoked with real data |
| Ground truth | — | **No corpus we possess contains a single ACARS message** (verified: iridium-toolkit reassembler on the 1.25 s ALBQ parse → 0 packets; Mendeley is ring-alerts only; PRBS synthetic). The reference toolchain has the same gap on our data. |

So the first unproven link (IDA→SBD) cannot be proven from any data on disk.
Getting real ACARS bytes requires the live pipeline to stop throwing away
99.9 % of bursts, run stably for hours, and ideally sit on the right 2.4 MHz
window (in the ALBQ 12 MHz snapshot the busiest IDA cluster was
1618.0–1618.7 MHz; 1622 saw only 6/82 frames — but assignments move per
beam/minute, which is what the scanner is for).

## 3. Problem-by-problem analysis and fix paths

### P1 — Tagger frozen-baseline latch (the decode-rate killer; do first)

**Root cause.** `update_baseline_ema` returns while `n_bursts > 0`
(`fft_burst_tagger.c:689`). gr-iridium has the same freeze but adds three
protections our port dropped: `d_max_burst_len` (225 ms force-close), a forced
noise-floor refresh after force-close (`fft_burst_tagger_impl.cc:265-285`),
and a burst squelch (`:327-340`). Without them, any persistent carrier keeps
`n_bursts>0` forever → baseline frozen at boot-time quiet → threshold never
adapts → 700–1200 junk bursts/s, flat across day and night, slots ~60/64
occupied so the latch never releases. Detection math is unchanged on this
branch (scanner's `reset_baseline` never executed on the bench — zero hop
lines); the T60 commit message already described the symptom.

**Knock-on effects.** Unbounded burst length (stale lag to 51 s; any burst
active ≥ ~1.6 s = one 16 MB ring span is dead on arrival). The worker burns
0.65–3.8 s per 250 ms-clamped garbage window (~25–30 UW retries each). T60's
priority queue is *defeated*: the flood is 1–3 bins wide (classified
narrowband = preferred class) and strongest-first selects the most expensive
long-carrier windows. No staleness check at push, no age term.

**Fix path (gri alignment, not tuning — per project policy):**
- Port `max_burst_len` force-close + post-close forced baseline refresh +
  burst squelch from `fft_burst_tagger_impl.cc:265-340`.
- Add stale-reject at PQ push (dead if `length ≥ ring span` or
  `head − start` near span) and consider a cheap age check at pop.
- Numbers come from gri (225 ms etc.), not from sweeping our bench.

**Expected effect:** worker inflow drops from ~730/s junk to O(100/s) noise +
real bursts; processed rate rises orders of magnitude; the 17/h decode
trickle becomes the actual RF-limited rate. This is the single biggest step
toward catching an SBD/ACARS payload.

**Validation gates:** existing host tagger tests + `test_tagger_vs_manifest`
(gri manifest equivalence — register it in ctest), device smoke variants that
run (RAW/REAL/CORPUS), then live A/B on `worker[dropped=]` and
`FRMDEC: FRAME:` rate. Discriminating pre-checks that need no code: `hop
1622000000` (re-primes baseline; dropped collapsing then creeping back =
latch confirmed), `GET /diag/histograms`, `GET /diag/dsp_health`,
`set tag_thr 14`.

### P2 — PIE coprocessor save (production panics + smoke gate)

**Mechanism (vendored IDF v6.1).** PIE is lazily context-switched via an
illegal-instruction trap into `rtos_save_pie_coproc` (portasm.S:54-127).
HWLP (`esp.lp.setup`) is a *separate* coprocessor saved eagerly at interrupt
entry; the PIE trap path never parks HWLP state. The save routine contains no
loop — the smoke "hang at rtos_save_pie_coproc" is a trap re-entry storm or a
wedged 128-bit save, not a wait. PIE save areas are carved from the owner
task's **stack bottom**, and worker/rs_worker stacks are **PSRAM**
(`esp.vst.128` into PSRAM is suspect given this project's documented PIE
region sensitivity). Two P4 silicon bugs are adjacent
(`SOC_CPU_HAS_HWLOOP_STATE_BUG`, spurious EXT_ILL HWLP reason bit) and their
IDF workarounds are **compiled out** by `CONFIG_ESP32P4_REV_MIN_FULL=100`.
`vTaskSuspendAll` failed because it defers only the scheduler; every
interrupt entry still juggles coprocessor state mid-window. Our own asm never
uses `lp.setup` — only vendored esp-dsp kernels (`dsps_fird_s16_arp4`,
`dsps_fft2r_fc32_arp4`, `dsps_fft2r_sc16_arp4`) do. Production exposure is
constant: both cores have ≥2 PIE-owning tasks.

**Fix path, in order:**
1. **1-hour discriminating diagnostics** (each against the deterministic
   smoke GOLDEN repro): (a) rebuild with PIE-task stacks in internal RAM —
   tests the PSRAM-save-area hypothesis; (b) single-PIE-owner-per-core build
   (scalar resample) — with one owner the save body never runs; (c)
   `CONFIG_ESP32P4_REV_MIN=0` — re-enables the silicon workarounds.
2. Based on which hypothesis survives: **patch the three vendored esp-dsp
   kernels to drop `esp.lp.setup`** (addi/bnez, ~1–2 cycles/iter; our golden
   bit-exact harness validates) **or** ~15-line IDF portasm patch to park
   HWLP on the PIE trap path (upstreamable), **or** move coproc save areas to
   internal RAM (`pxPortGetCoprocArea`).
3. Re-run smoke GOLDEN to a pass → restores the honest `Smoke-verified:` gate
   and should eliminate the production panics.

### P3 — DMA-internal budget + scan reliability

Internal SRAM heap is ~203 KB and structurally oversubscribed ~30 KB (hence
4/8 transfers, 35 KB pre-stream vs 62 KB target, runtime largest 0–2 KB,
8.3 k stash fallbacks). Ranked reclaim (audit has the full consumer table):
1. **Reusable pre-allocated control URB** in `esp_libusb_control_transfer`
   (`esp_libusb.c:114-141`) — eliminates ~22 DMA-INT alloc/free per hop;
   directly targets the mid-scan EP0-stall/NO_MEM; LOW risk.
2. **Right-size `ASYNC_TRANSFER_COUNT` 8→5-6** (`esp_libusb.h:49`) — the
   system already runs on 4; kills the boot alloc error and creates real
   headroom; reconcile the three-way-inconsistent
   `SPIRAM_MALLOC_RESERVE_INTERNAL=144 KB` comment while there.
3. **Drop mdns** (pinned internal 8–15 KB; delivers nothing — C6 multicast TX
   is broken) and move serial_cmd/ota stacks to PSRAM (8–16 KB).
4. Later/measured: `SPIRAM_MALLOC_ALWAYSINTERNAL` 16384→512-2048 (needs SD
   soak); `USB_HOST_DWC_DMA_CAP_MEMORY_IN_PSRAM=y` only as an A/B experiment.
   Do NOT touch tagger struct sizes or L2 cache without the full
   heap-position/decode A-B protocol.

**Separate scan hazard found:** `xfer_mutex` serializes per-control-transfer,
not per-sequence — an AGC gain CT can interleave *inside* a retune's
i2c-repeater-on…off window (tuner-state corruption). Scan needs a
sequence-level lock or AGC quiesce around retune.

### P4 — Prove the ACARS tail (host glue + device injection)

- **Host glue (~100 lines):** extend `tests/host/decode_burst_capture.c` to
  call `ida_decode → sbd_reassembler_feed → la_acars_parse…` on LW.DA frames
  and print ACARS text (all three link individually today; only the CMake
  wiring is missing). This gives an IQ-file→ACARS-text tool for any capture.
- **Device tail proof now (synthetic):** wrap a valid ACARS frame in SBD/IDA
  framing and push through `frame_decoder_push` via the smoke-corpus
  mechanism (`smoke_test.c:270`) or `POST /debug/inject` — proves the wired
  on-device tail (`frame_decoder.c` → `msg_ring` + `acars_push_emit`
  UDP → sd_log) end-to-end. This is a code-path proof, not the SDR proof.
- **Ground truth acquisition:** device `sd_capture` continuous mode records
  the raw uint8 USB stream — exactly what `iridium-extractor -f rtl` eats
  (~13 min per 4 GB card) — enabling gr-iridium cross-validation of any live
  interval, including what our tagger *misses*. A host-PC RTL-SDR +
  `iridium-extractor` running for hours is the fastest independent path to
  "we possess one real ACARS capture".

### P5 — Bench monitor honesty

Fix the patterns to what the firmware actually logs (`FRMDEC: FRAME:`,
`bch_ok=1`, `crc=OK`) and **verify each pattern against a historical
known-positive log line before arming** (the positive-control rule). Watch
`worker[dropped=]` not `frames=`. Also: the last ~10 min of today's log show
rate 5.17–5.41 MiB/s (> the physical 4.88 MiB/s of 2.56 Msps) with
processed=0 — either the rate estimator or ingest is misbehaving post-reboot;
check before trusting new numbers.

## 4. Recommended order of work

The goal-critical path is P1 → P2 → (P3 ∥ P4) → dwell/scan for the first
ACARS. P5 and the P4 synthetic tail proof are same-day quick wins. Scanner
Phase 2 stays deferred until P1–P3 land. The pending whole-branch review
(`.superpowers/sdd/review-7f77955..d392249.diff`) should run before merging
`freq-scanner`.

**Push decision (human call, recommended):** push `t60-narrowband-priority`
and `freq-scanner` now with an honest trailer, e.g.
`Smoke-skip: GOLDEN hangs on tracked PIE-coproc-save deadlock
(project_pie_save_deadlock_smoke); RAW/REAL/CORPUS results: <counts>` —
running the variants that don't hang and recording their counts. Two branches
× 15 commits of unpushed work is multi-machine risk for no benefit.

## 5. Agent dispatch plan (model tier + instructions)

Principle (project tiering policy): cheapest model whose mistakes our gates
catch mechanically; top model for trust-bearing judgment and anything the
gates can't check.

| Task | Tier | Why / gates | Key instructions to give |
|---|---|---|---|
| P1 tagger gri-alignment | **Top (Opus/Fable)** | DSP-critical; gri parity is judgment, host tests alone don't gate device behavior | Port `fft_burst_tagger_impl.cc:265-340` semantics exactly; NO parameter tuning (feedback_no_local_test_optimization); keep struct size unchanged or follow heap-position protocol; register `test_tagger_vs_manifest` in ctest; device smoke variants + live `worker[dropped=]` A/B mandatory; `Smoke-verified:`/`Smoke-skip:` trailer |
| P2 diagnostics (3 rebuild experiments) | **Mid (Sonnet)** | Mechanical config/build changes; smoke GOLDEN is a deterministic binary gate | One experiment per run; separate build/flash invocations; record exact sdkconfig delta + GOLDEN outcome; do not "fix" anything, report only |
| P2 real fix (esp-dsp asm or IDF portasm) | **Top** | Hand asm on the context-switch path; a wrong fix corrupts silently | Bit-exact golden-fixture validation of every touched kernel (harness exists); GOLDEN must pass ×3 consecutive; production soak ≥2 h with zero Coprocessor panics |
| P3 items 1–3 (control URB, pool size, mdns, stacks) | **Mid** | Small, well-specified; gates: boot `Pre-stream DMA-internal free ≥62 KB`, 30-min stream at full rate, scan 5-hop soak | One change per commit with pathspec; check `LIBUSB: Pre-stream` line after each; don't touch PIE-adjacent allocs; respect early-alloc dance (class_driver.c:333-350) |
| P3 retune/AGC mutex sequencing | **Top** | Concurrency design on the USB control path | Design sequence-level exclusion; prove with scan soak + AGC active |
| P4 host glue tool | **Mid** | Host-only, CMake + calls into already-tested code; gate: synthetic ACARS fixture decodes to known text | Extend decode_burst_capture, don't fork a new tool; add a ctest with a synthetic SBD/IDA-framed ACARS fixture as positive control |
| P4 device tail injection proof | **Mid** | Uses existing smoke-corpus/inject mechanisms | Verify `FRMDEC: ACARS:` + `/messages` + UDP receipt; no firmware changes beyond the corpus entry |
| P5 monitor fix | **Any/cheap** | Trivial; gate: patterns must match historical positive lines | Grep log.1 for known decodes first; ask before killing the running monitor (user rule) |
| Whole-branch review | **Top** | Trust-bearing adjudication | Review `.superpowers/sdd/review-7f77955..d392249.diff` + the minor-findings roll-up in the handoff |

Cross-cutting instructions for every agent: `date -Iseconds` prefixes on long
bash; tee expensive outputs to files; separate build and flash tool calls;
never kill processes you didn't start; device smoke mandatory after any
DSP-path commit; pathspec commits; SI units in reports.

## 6. Open questions for Bruce

1. Approve pushing both branches with `Smoke-skip:` trailers now?
2. P1 (tagger latch) before P2 (PIE) — agreed? (P1 multiplies decode rate;
   P2 removes ~hourly reboots and unblocks the honest gate. They're
   independent; can run as parallel worktrees.)
3. Is a host-PC RTL-SDR available to run `iridium-extractor` for hours as the
   independent ground-truth path to a first real ACARS capture?
