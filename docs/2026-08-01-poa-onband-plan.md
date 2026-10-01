# POA (plain VHF ACARS) as a first-class third band — implementation plan

> Status: PLAN (task #36). 2026-08-01. No code written yet.
> Decision basis: live 6 h coverage run at this site — POA is the BUSIEST
> band here (~153 ACARS msgs / 20 flights vs VDL2's 17 / 8, roughly 9:1).
> The device currently feeds the quiet band and ignores the loud one.
> Reference decoder (the port source + oracle): acarsdec at
> `~/dev/vdl2-tools/acarsdec` — it decodes real POA on the Mac today.
> This supersedes the "skip POA" call in
> `docs/2026-07-22-vhf-acars-feasibility.md`, which was made on the
> assumption VDL2 would carry the traffic. It doesn't, at this site.

## TL;DR

Add `BAND_POA = 2` alongside `BAND_IRIDIUM`/`BAND_VDL2`
(`common/band_pipeline/band_profile.h:21-25`). POA is 2400 bps MSK audio on
an **AM** carrier in 25 kHz channels around 129–132 MHz — a *continuous
narrowband* signal on *fixed known channels*, not a spectrum-detected PSK
burst. It therefore **does not go through the fft_burst_tagger → worker →
process_burst path at all**. Instead, `dsp_processor` grows a second
front-end mode: a channelized always-on demod ported from acarsdec's RTL
path (`rtl.c` mix–integrate–dump channelizer + `msk.c` MSK demod + `acars.c`
framing/CRC-fix), running per-channel at 12.5 kHz out of the same 2.5 MSPS
int16 stream `class_driver.c:1018` already feeds. Completed ACARS blocks ride
the existing `frame_queue` to the `frame_decoder` task, whose new
`process_one_poa()` does the byte-work (parity/CRC syndrome fix) and hands
the bare block to the **existing shared** `acars_deliver()`
(`p4-usb-host/main/frame_decoder.c:421`) — so libacars reassembly,
`msg_ring`, SD log, and the airframes feed are reused unchanged. Compute is
tiny (~25 % of one core in plain float for 4 channels — no PIE, no Q15
needed). One band per boot, as today.

This is a **VDL2-scale feature** (new `common/poa_decoder/` component, a new
front-end mode, a third NVS namespace, a third airframes wire schema, host +
device test infrastructure), not a patch. The demod itself is the *small*
part (acarsdec's msk.c is 139 lines); the framework ripple and the test
discipline are the bulk.

---

## 1. The load-bearing decision: POA does not fit the burst-tagger front end

This is the crux, so it gets argued explicitly.

**What the existing front end is.** USB ingest converts to int16 and calls
`dsp_processor_feed()` (`p4-usb-host/main/class_driver.c:1018`,`:1051` →
`dsp_processor.c:387`), which accumulates 2048-sample chunks and runs the
`fft_burst_tagger` (`common/iridium_decoder/fft_burst_tagger.h`). Detected
bursts flow to `worker_core1_push_burst` (`class_driver.c:451`), the worker
extracts/rotates/decimates the window to 250 ksps and dispatches through the
`band_pipeline_t` vtable — `prefilter` + `process_burst`
(`common/band_pipeline/band_pipeline.h:74-89`, call sites
`worker_core1.c:1297-1357`). Both existing bands are *burst PSK*: the tagger
is an energy-rise detector and the pipeline gets a bounded window.

**Why POA cannot be force-fit into that:**

1. **Duration.** A plain ACARS transmission (16+ SYN preamble + up to 220
   text chars at 2400 bps) runs ~0.3–1.2 s. The tagger force-closes any
   burst at `FBT_MAX_BURST_LEN` = 225 000 samples = **90 ms** at 2.5 MSPS
   (`fft_burst_tagger.h:65`) — every POA transmission would be chopped into
   ~10 fragments, destroying the bit-sync state the decoder needs.
2. **Detection statistic mismatch.** The tagger thresholds a per-bin power
   rise over an EMA noise floor, tuned for PSK burst edges
   (`band_profile.h:100-109`). An AM carrier keys up slowly and sits there;
   worse, the *useful* information is in the ±2.4 kHz audio sidebands of a
   dominant carrier. `tag_thr`, `fbt_width_bins`, pre/post pads — none of
   the profile's tagger semantics apply.
3. **The demod is stateful across arbitrary time.** acarsdec's decoder is a
   free-running per-channel machine: MSK PLL (`msk.c:67-137`), bit clock,
   and the `WSYN→SYN2→SOH1→TXT→CRC` framing state machine
   (`acars.c:246-374`) that resynchronizes on inverted-SYN
   (`acars.c:258-263`) mid-stream. Windowing it through `process_burst`
   would mean re-acquiring from cold on every fragment.
4. **It's cheaper NOT to tag.** The channelizer + MSK demod for 4 channels
   costs *less* than the tagger it would replace (a 2048-pt FFT every 1024
   samples plus baseline maintenance over a 4 MB PSRAM history,
   `dsp_processor.c:319-323`). Always-on demod is the *lighter* option, and
   it is exactly what the proven oracle does (acarsdec has no energy gate at
   all — same posture as dumpvdl2, already noted in
   `band_profile.h:104-107`).

**Decision: POA gets a parallel channelized-continuous front end; the band
abstraction seam moves up one level.**

Concretely:

- `band_profile_t` gains a front-end discriminator:
  `frontend = BAND_FE_BURST_TAGGER | BAND_FE_CHANNELIZED` (new enum field in
  `band_profile.h:114-125`; IRIDIUM and VDL2 set `BURST_TAGGER`, POA sets
  `CHANNELIZED`). This is data-driven, not `if (band == BAND_POA)` — a
  hypothetical future continuous band (e.g. the parked Inmarsat aero idea,
  `docs/2026-07-24-inmarsat-aero-band-proposal.md`) reuses the mode.
- `dsp_processor_create()` (`dsp_processor.c:300-370`) branches on
  `rt->profile->frontend`: for CHANNELIZED it skips
  `fft_burst_tagger_init` + the 4 MB baseline history entirely and
  initializes `poa_frontend` instead. `dsp_processor_feed()`
  (`dsp_processor.c:387`) routes chunks to `poa_frontend_feed()`.
  `class_driver.c` is untouched — the seam is inside dsp_processor, which
  already resolves the band once at create (`dsp_processor.c:311`).
- The `band_pipeline_t` vtable **stays what it is: the per-burst interface
  for tagger bands**. POA registers an honest stub (accept-all prefilter,
  `process_burst` returns 0 and bumps a "should never run" counter) so
  `band_select_pipeline()` (`band_select.c:13-22`) stays total and the
  worker can never dereference NULL. We deliberately do NOT contort the
  vtable into a universal front-end abstraction — that would be
  over-engineering for one consumer. The plan documents the asymmetry
  instead of hiding it: for POA, the worker task starts and idles forever
  (its burst queue is simply never fed), costing nothing.

**What this means for the vtable abstraction (honest statement):**
`band_pipeline_t` was derived from the worker's call site
(`band_pipeline.h:5-17`) and remains correct for burst bands. The *real*
band abstraction after this work is two-layered: (a) the profile's
`frontend` selects tagger-path vs channelized-path at
`dsp_processor_create`; (b) within the tagger path, `band_pipeline_t`
selects the demod. Both layers resolve from the same
`band_runtime_resolve()` (`band_select.c:24-34`) at boot.

---

## 2. Building blocks

### 2.1 What acarsdec provides (the port source)

All paths relative to `~/dev/vdl2-tools/acarsdec`.

- **Channelizer** (`rtl.c:314-361`, tables built at `rtl.c:272-287`): the
  RTL runs at `rtlInRate = INTRATE × rtlMult` (`rtl.c:214`), INTRATE =
  12 500 (`acarsdec.h:31`). Per channel, a precomputed complex tone table
  `wf[rtlMult]` (`rtl.c:283-286`) is dot-multiplied against each block of
  `rtlMult` input samples and summed — a mix + integrate-and-dump decimator
  — and the **magnitude** `cabsf(D)` (`rtl.c:353`) is taken. That
  `cabsf` *is* the AM envelope demod; its output `dm_buffer[]` is real
  audio at 12.5 kHz. Because only the magnitude survives, inter-block
  phase discontinuity is irrelevant — which is why the naive per-block
  table reuse works and why this stage is so cheap. Our capture rate is
  already exactly `12 500 × 200 = 2 500 000` (Path A native rate,
  `band_profile.h:33`), so **rtlMult = 200 verbatim**.
- **Channel/LO placement rules** (`chooseFc`, `rtl.c:131-168`): channel
  freqs are rounded to the 12.5 kHz INTRATE grid (`rtl.c:245-247`); the
  center must keep every channel ≥ 2×INTRATE = 25 kHz away from DC
  (`rtl.c:158`) and must avoid symmetric image pairs (`rtl.c:160`). This
  independently re-derives the repo's own "never park a signal at DC" rule
  (memory: feedback_dont_center_downconvert_on_burst; same reasoning as the
  VDL2 LO at `band_profile.h:50-57`).
- **MSK demod** (`msk.c:67-137`): per 12.5 kHz audio sample — VCO at
  1800 Hz center (`msk.c:81`), complex mix into an 11-tap delay line
  (`FLEN = INTRATE/1200 + 1`, `msk.c:25`), bit clock at 3π/2 phase
  (`msk.c:96`), a cosine matched filter oversampled ×12 for fractional
  timing (`msk.c:46-47`, `msk.c:102-107`), alternating I/Q slicing
  (`msk.c:115-121`), and a 1-pole PLL (`PLLG/PLLC`, `msk.c:65-66`,
  `:130`). Per-bit soft value goes to `putbit` → the framing machine. Also
  maintains `MskLvlSum` for the per-message level report (`msk.c:112`).
- **ACARS framing + FEC** (`acars.c`): the `WSYN/SYN2/SOH1/TXT/CRC1/CRC2`
  state machine (`acars.c:246-374`) keyed on SYN=0x16/SOH=0x01
  (`acars.c:22-27`), inverted-SYN polarity flip (`acars.c:258-263`),
  DLE-based missed-end recovery (`acars.c:324-333`); then block-level
  repair: parity-error location (`acars.c:136-156`), CRC-CCITT check, and
  syndrome-table correction of up to `MAXPERR = 3` known-position parity
  errors (`fixprerr`, `acars.c:39-64`) or a blind double-bit fix
  (`fixdberr`, `acars.c:66-90`) via `syndrom.h`. STX/ETX forcing at
  `acars.c:131-133`.
- **Direction inference**: block-id character class — digits = downlink
  (`IS_DOWNLINK_BLK`, used at `output.c:527-529`) → maps directly onto the
  `la_msg_dir` our `acars_deliver()` needs.
- **libacars usage** (`output.c:555-650`): acarsdec does its own envelope
  parse then calls `la_reasm_fragment_add` + `la_acars_apps_parse_and_
  reassemble` (`output.c:598`, `:641`). **We will NOT port this layer** —
  our `acars_deliver()` (`frame_decoder.c:421-503`) already runs
  `la_acars_parse_and_reassemble()` on a bare block (mode char onward) with
  a persistent reassembly context, for both existing bands. POA hands it
  the repaired block and gets reassembly + `msg_ring`/SD/feed for free.
- **Wire JSON** (`output.c:230-325`): acarsdec's native JSON
  (`timestamp/station_id/channel/freq/level/error/mode/label/block_id/ack/
  tail/flight/msgno/text/end + app{name,ver}`) — the schema airframes.io's
  ACARS ingest (UDP **5550**) expects. This becomes the third schema in
  `common/acars_feed/airframes_fmt.c`.

### 2.2 What the repo provides (reused unchanged)

| Piece | Where | POA usage |
|---|---|---|
| 2.5 MSPS int16 stream | `class_driver.c:1018` → `dsp_processor_feed` (`dsp_processor.c:387`) | channelizer input, unchanged rate |
| Frame hand-off queue | `common/iridium_decoder/frame_queue.h:67-89` ("band-agnostic — bits + metadata", `:31-33`) | one completed ACARS block per item (≤ 250 bytes, `acarsdec.h:55` — nowhere near the 16 832-bit cap at `frame_queue.h:51`) |
| Byte-work task + libacars | `frame_decoder.c` decoder task; `acars_deliver()` `frame_decoder.c:421` | new `process_one_poa()` sibling of `process_one_vdl2()` (`frame_decoder.c:854`) |
| ACARS emit fan-out | `msg_ring_push` / `acars_push_emit` / `sd_log_emit` (`frame_decoder.c:490-492`), `acars_msg_t` (`msg_ring.h:18-49`) | verbatim; `has_avlc=false` like Iridium |
| Per-band NVS config | `app_config.c:130-233` (namespaced keys + migration) | new `po_` namespace |
| Band plumbing | `band_profile.c:8-29`, `band_select.c`, `serial_cmd.c:179-183` (`set band` via `band_profile_from_str`) | table entries |
| Band-agnostic funnel | `band_decode_stats_get` (`band_select.c:36-57`) | POA branch |
| Airframes feeder | `acars_push.c:159-192` + `common/acars_feed/airframes_fmt.{h,c}` | `AF_BAND_POA` schema, port 5550 |
| Fill-based gain autotune | `autotune.c:298-306` (VDL2 precedent) | same gate for POA |

### 2.3 New component: `common/poa_decoder/`

Modelled on `common/vdl2_decoder/` (pure C, no ESP-IDF deps, host-linkable —
see its CMakeLists' discipline including the test-modulator-only-for-smoke
pattern):

- `poa_channelizer.{h,c}` — N-channel mix/integrate/dump + envelope (port of
  `rtl.c:272-287` + `:332-355`), int16-in/float-out, channels configured
  once at init from the channel list. Includes the freq-grid rounding and
  the DC/image placement validation (`rtl.c:131-168` logic) as an init-time
  check that logs and rejects bad channel/LO combos.
- `poa_msk.{h,c}` — per-channel MSK demod, verbatim float port of `msk.c`
  (state struct per channel replacing acarsdec's `channel_t` globals).
- `poa_acars_frame.{h,c}` — the bit→block framing state machine
  (`acars.c:246-374` port), emitting a raw block (`txt[250]` + crc[2] +
  level + channel index) via callback.
- `poa_l2.{h,c}` — block repair: parity scan + `fixprerr`/`fixdberr` +
  `syndrom.h` tables (`acars.c:39-215` port), returning a cleaned block
  (parity bits stripped, `acars.c:200`) + err count + direction
  (block-id class) for `acars_deliver()`.
- `poa_pipeline.{h,c}` — the honest `band_pipeline_t` stub + the POA stats
  accessors (`poa_pipeline_blocks_seen()` etc., mirroring
  `vdl2_pipeline.h:28-35`).
- `poa_mod.c` — host-test/smoke-only synthetic generator (MSK tones →
  AM-modulated IQ at 2.5 MSPS), the `vdl2_mod.c` analogue, compiled into
  firmware only under `CONFIG_SMOKE_TEST_POA` (same CMake gating as
  `common/vdl2_decoder/CMakeLists.txt`).

acarsdec is GPL-2 — same licensing posture as the dumpvdl2-derived VDL2
code already in-tree; keep the attribution header style used in
`vdl2_demod.c`.

---

## 3. Arithmetic: float is fine, quantified

Per channel, the channelizer does one complex MAC per input sample:
2.5 M cMAC/s ≈ 20 Mflop/s (4 mul + 4 add with a precomputed table, plus the
shared int16→float conversion). For the 4-channel default: **~80 Mflop/s ≈
20–25 % of one 400 MHz P4 core** with the single-precision FPU at -O2
(profile at -O2, per memory feedback_o2_for_optimization). The MSK demod
runs at 12.5 kHz/channel — 11-tap complex matched filter per *bit*
(2400 Hz) plus one `sincosf` per sample — well under 2 % per channel. The
framing/CRC layer is noise.

Context that makes this a non-issue:

- One band per boot: under `band=poa` the Iridium UW-correlator load
  (~172 % worker + 97 % dsp, memory project_worker_compute_profile) simply
  doesn't exist. POA replaces the tagger's own FFT load and frees the 4 MB
  PSRAM baseline history (`dsp_processor.c:319-323`).
- **Verdict: plain float, no PIE, no Q15.** This is a deliberate,
  argued exception to the prefer-int16 rule (memory
  feedback_prefer_int16_over_float): that rule targets *hot* paths where
  8-lane PIE int16 pays 8×; here the whole band fits in a quarter core and
  a float port stays line-for-line comparable to the acarsdec oracle —
  which is worth more (cross-validation discipline, memory
  project_cross_validate_against_gr_iridium). If a future 8–16 channel
  config measures hot, the channelizer inner product is trivially
  Q15-able later; note int32 accumulators need care (200 × 2^15 × 2^15
  overflows 32 bits — needs Q12 coeffs + int64 or block scaling), one more
  reason not to do it speculatively.

Execution context: run channelizer + MSK inline in the
`dsp_processor_feed()` caller (the same task context that runs the tagger
today) — identical latency/priority posture, no new task, no new ring. The
completed-block hand-off to Core 0's frame_decoder already exists
(`frame_queue`), preserving the "PHY on the streaming core, byte-work on
Core 0" split used by VDL2 (`frame_decoder.c:795-870`). If bring-up
profiling shows the feed context can't absorb ~25 % (it absorbs the tagger
today, which costs more), fallback is a small SPSC ring + dedicated POA task
on Core 1 at the worker's priority (worker is idle under POA) — decided by
measurement, not up front.

Memory: per channel ≈ 200-entry float-complex `wf` table (1.6 KB) + 11-tap
delay line + `dm_buffer` block (~4 KB) + block buffer (256 B) → **< 8 KB per
channel, < 64 KB total** from normal heap; nothing DMA-internal (respects
memory feedback_dma_int_budget_audit), and net PSRAM goes *down* 4 MB vs
tagger bands.

---

## 4. Band framework extension — the full BAND_COUNT ripple

Every enum/array/switch site that must change, and how the other two bands
stay safe:

1. **`band_profile.h:21-25`** — add `BAND_POA = 2` before `BAND_COUNT`.
   Add POA profile constants: `BAND_POA_LO_HZ 130800000u` (see §5),
   `BAND_POA_FS_HZ 2500000u`; tagger fields are N/A → set to Iridium's
   values with an explicit "unused (frontend=CHANNELIZED)" comment rather
   than 0, so any accidental tagger-path use behaves, not crashes. Add the
   new `frontend` field (§1) — **this field addition touches all three
   entries** and is pinned by the host profile test.
2. **`band_profile.c:8-29`** — third designated initializer. `:37-44`
   `band_profile_from_str` picks up `"poa"` automatically from the table.
   Unknown-token→iridium semantics unchanged (`band_profile.h:131-133`).
3. **`app_config.c:142-145` `band_key()` — THE dangerous edit.** Today:
   `band == BAND_VDL2 ? "v2_" : "ir_"`. Left alone, `band=poa` would
   silently read/write **Iridium's** namespaced keys — the exact gain
   footgun phase 2 of the band-mode overhaul existed to kill (memory
   project_band_mode_overhaul). Replace with a `static const char
   *k_band_prefix[BAND_COUNT] = {"ir_", "v2_", "po_"}` indexed with the
   same clamp rule as `band_profile_get`. Prefix stays 3 chars → the
   15-char NVS key math at `app_config.c:140-141` holds.
4. **`app_config.c:204-233` migration** — no new migration needed: legacy
   globals migrate to the *active* band on first post-upgrade boot
   (`app_config.c:300`), and a device being switched to POA has already
   booted this firmware at least once (keys migrated to its previous
   band). POA's namespace starts fresh from profile defaults —
   the intended behaviour. Verify by test that `migrate_legacy_band_keys`
   with band=POA is a no-op when legacy keys are gone.
5. **`app_config.{h,c}` new POA keys** — see §5. Loads at
   `app_config.c:334-344` gain the POA-only channel-list read (gated on
   the active band, keeping the flat in-memory struct discipline of
   `app_config.c:135-141`).
6. **`band_select.c:13-22`** — `case BAND_POA: return poa_pipeline();`
   (stub). `:24-34` `band_runtime_resolve` needs no edit (arrays sized by
   `BAND_COUNT`). `:36-57` `band_decode_stats_get` gains the POA branch:
   `decoded = crc_ok_msgs`, `failed = parity_drop + crc_fail`,
   `recovered = crc_fixed` (blocks salvaged by `fixprerr`/`fixdberr`),
   `unknown = 0`. This automatically makes autotune's success metric
   band-correct (`autotune.c:360-363`).
7. **`dsp_processor.c`** — the §1 frontend branch in `create` (`:300-370`)
   and `feed` (`:387`); `fft_burst_tagger_set_trace_enabled(rt->band ==
   BAND_VDL2)` (`:367`) is untouched (no tagger exists under POA). The
   `_Static_assert` block (`:75-85`) is untouched — Iridium pins hold.
8. **`worker_core1.c:1410-1427`** — emit-sink resolution: POA must NOT
   accidentally select `worker_emit_frame_vdl2`; the existing
   `(rt->band == BAND_VDL2) ? ... : worker_emit_frame` default is safe
   as-is (worker never runs under POA), but make it a switch with an
   explicit `BAND_POA` arm + comment so the next band doesn't inherit an
   accident.
9. **`frame_decoder.c`** — `s_band_vdl2` (`:123`, set at `:993`) becomes a
   small per-band dispatch enum; `:950-951` routes POA items to the new
   `process_one_poa()` → `poa_l2` repair → `acars_deliver()`
   (`:421`) with `avlc=NULL`, `dir` from block-id class, `freq_hz` = the
   channel's absolute frequency (finally a band where it's exact). New
   `frame_decoder_get_poa_stats()` mirroring `frame_decoder_get_vdl2_stats`
   (`:873-889`).
10. **`status_logger.c` `rx_update` (`:138-175`)** — POA branch for the
    reception classifier. Mapping (POA has no tagger, so "tagged" needs a
    new source): tagged→blocks started (SOH seen), reached→blocks reaching
    CRC stage, decoded→CRC-ok messages. QUIET/thresholds reused as-is; the
    numbers are per-window deltas exactly like the VDL2 branch
    (`:98-108` snapshot pattern).
11. **`http_server.c`** — the `is_vdl2` presentational branches: page title
    (`:810-811`), config table (`:985-1006` — band name renders from the
    profile, free), reception wording (`:1089-1092`, `:1185-1225`),
    BCH-block gating (`:1267-1272` — must ALSO gate off under POA, same
    reason as VDL2), the per-band decode funnel section (`:1319-1338`,
    `:1650-1675`), and a `"poa"` block in `/status` JSON next to the
    `"vdl2"` block (`:289-294`, `:329`). No new URI route → no
    `max_uri_handlers` bump needed (memory
    feedback_httpd_max_uri_handlers); band is set via serial + POST
    /config's existing paths, matching V0's deliberate no-new-route call
    (`docs/2026-07-22-vdl2-implementation-plan.md` §V0 tail).
12. **`serial_cmd.c:179-183`** — zero code change for `set band poa`
    (token comes from the profile table); help text + `get`/`config`
    output updated; new `set poa_chans …` key (§5).
13. **`autotune.c:298-306`** — widen the VDL2 gate: any
    `frontend == CHANNELIZED` band (or explicitly `BAND_POA`) takes the
    fill-based ADC-knee gain cal and returns; the Iridium IRA-hop sweep
    stays Iridium-only. Rationale identical to VDL2's: no always-on
    reference to decode-maximize against; POA traffic is bursty
    human-schedule traffic. The scheduler's LO re-scan and band-health
    resurvey (`autotune_sched.c`, `band_health.c`, `decode_survey.c`) must
    be verified band-gated under POA in the same pass — POA parks a fixed
    LO; hourly LO steering is meaningless-to-harmful here (memory
    reference_freq_coverage_analysis).
14. **`acars_push.c:159-192`** — 3-way band map: `AF_BAND_POA`, and the
    per-band default ingest port note extends to **5550** for ACARS
    (`app_config.h:177` comment update). Populate `freq_hz` from the
    message's channel (closing the feeder plan's "freq_hz TODO" for POA).
15. **`common/acars_feed/airframes_fmt.{h,c}`** — `AF_BAND_POA` in
    `af_band_t` (`airframes_fmt.h:27`) + the acarsdec-native JSON schema
    (§2.1 last bullet), byte-exact-pinned in
    `tests/host/test_airframes_fmt.c` fixtures like the other two.
16. **`smoke_test.c:41-42`** — new `CONFIG_SMOKE_TEST_POA` variant using
    `app_config_set_band_ram((uint8_t)BAND_POA)` (`app_config.h:209`,
    avoiding the NVS-band-inheritance trap from memory
    project_vdl2_batch_device_traps), modelled on the VDL2 smoke
    (`smoke_test.c:484-620`).
17. **Tests** — `tests/host/test_band_profile.c` pins the POA entry + the
    new `frontend` field for all three bands; `test_band_pipeline_iridium.c`
    (the bit-identity guard) must stay green **untouched**;
    `test_airframes_fmt.c` grows POA fixtures; new suites in §8.

**Not breaking Iridium/VDL2 — the two genuinely risky shared edits** are
(3) `band_key()` (a mistake silently corrupts another band's NVS values —
covered by a host test that exercises all three prefixes + the clamp) and
(1) the `band_profile_t` struct change (every consumer recompiles; the
existing `_Static_assert`s in `dsp_processor.c:75-85` plus
`test_band_profile.c` pin Iridium bit-identity, and
`test_band_pipeline_iridium` pins the decode path). Everything else is
additive: new enum value at the end, designated initializers, new switch
arms with existing defaults preserved. Per repo discipline, any commit
touching the shared DSP path needs the device smoke trailer
(memory feedback_device_smoke_mandatory_after_dsp).

---

## 5. Config: POA NVS tunables

Per-band namespaced (prefix `po_`, mechanism of `app_config.c:130-176`):

| Key | Type | Default | Notes |
|---|---|---|---|
| `po_lo_hz` | u32 | **130 800 000** | Center for the site's channel set {131.550, 130.450, 130.425, 130.025 MHz} → offsets +750/−350/−375/−775 kHz: all ≥ 25 kHz from DC, no symmetric image pairs, all within ±1.25 MHz — satisfies `chooseFc`'s constraints (`rtl.c:154-162`). Profile default; explicit NVS value wins, same rule as today (`app_config.c:319-335`). |
| `po_gain_dbx10` / `po_gain_mode` / `po_bias_tee` | i16/u8/u8 | 280 / MANUAL / on | Start at the VDL2 VHF operating point (28 dB, memory reference_vdl2_gain_and_adc_fill — same antenna, same band-ish); calibrate at bring-up via fill-based autotune. |
| `po_chans` | str | `"131.550,130.025,130.425,130.450"` | The set that replaces `tag_thr` as POA's defining tunable. Parsed at boot: MHz values, rounded to the 12.5 kHz grid (`rtl.c:245-247` behaviour), capped at 8 channels (acarsdec allows 16, `acarsdec.h:30`; 8 is plenty and bounds CPU). Init-time validation against `po_lo_hz` (span, DC clearance, image pairs) with a loud log + per-channel disable on violation. New string-typed band key — add `nvs_get_str_band` beside the typed getters at `app_config.c:149-176`. |
| `po_tag_thr` | — | *not created* | POA has no energy gate, by design (oracle parity: acarsdec demods continuously). The generic `tag_thr` load (`app_config.c:343-344`) is skipped/ignored for CHANNELIZED bands. A squelch is a non-goal (§10). |

Global keys (`rate_hz`, `best_eff`, `uart_log`, `chase2`, af_*, station)
stay global, per the phase-2 split (`app_config.c:333`). Serial `set band
poa` + `set poa_chans …`; POST /config gains the `po_chans` field only on
the band=poa rendering of the form (and the API-POST-wipes-psk gotcha from
memory project_airframes_feeder_landed applies — web form only).

Autotune: gated to fill-based cal (§4 item 13). Success metric for any
future decode-based tuning is already band-correct via
`band_decode_stats_get` = CRC-ok ACARS messages — the right currency
(real decodes, not burst counts — memory reference_vdl2_gain_and_adc_fill).

---

## 6. Reuse vs new (explicit)

**Reused unchanged:** USB/ingest/int16 conversion; `frame_queue`
(`frame_queue.h` — POA items are small; push/pop cost is prefix-sized,
`frame_queue.h:48-50`); `acars_deliver()` + persistent libacars reassembly
ctx + dedupe (`frame_decoder.c:410-418`); `acars_msg_t` + `msg_ring` +
`/messages`; `sd_log_emit` NDJSON; `acars_push_emit` UDP debug feed;
airframes delivery machinery (`acars_push.c:335-362`); per-band NVS
mechanism; serial/web config plumbing; fill-based gain autotune; reception
classifier thresholds.

**New:** `common/poa_decoder/` (≈ 6 files, §2.3); the `frontend` profile
field + dsp_processor branch; `process_one_poa()` + POA stats in
frame_decoder; `AF_BAND_POA` JSON schema; `po_` NVS keys incl. string
channel list; host fixtures/suites + device smoke variant; /status + /diag
POA blocks.

**Modified:** the 17 ripple sites in §4.

---

## 7. Phases

Phasing follows the VDL2 model (`docs/2026-07-22-vdl2-implementation-plan.md`,
memory project_band_mode_overhaul): every phase leaves the tree building,
host suite green, and **both existing bands behaviourally identical**
(bit-identity guard + VDL2 smoke).

### Phase P0 — framework ripple, POA as a countable no-op
Enum + profile entry (+`frontend` field on all three), `band_key` prefix
table, `band_select` arms, pipeline stub, stats stub wired to zeros,
serial/web accept `poa`, /status shows the band name. A `band=poa` device
boots, parks at 130.8 MHz with the right gain, runs *no* tagger, decodes
nothing, and says so honestly.
**Risk:** the two §4 shared edits. **Verify:** full host suite;
`test_band_profile` (extended), `test_band_pipeline_iridium` untouched-green;
new `test_band_key_prefixes` host test; device: Iridium RAW smoke +
VDL2 smoke both pass on this commit; NVS of an existing bench device
upgrades cleanly (no key churn in `ir_`/`v2_` namespaces).

### Phase P1 — host-side demod port + oracle cross-validation (the meat)
`common/poa_decoder/` channelizer/MSK/framing/L2, host-built. Fixture: a
real 2.5 MSPS capture around 130.8 MHz taken on the Mac (rtl_sdr / SoapySDR,
same rig that already decodes POA), decoded by **acarsdec as oracle**; store
the capture in `~/iridium_capture/poa_ref/` with a prep script and embed the
expected message set (reg/flight/label/text/CRC) as a fixture header —
exactly the dumpvdl2 golden pattern (memory
reference_vdl2_oracle_and_golden). Suites: `test_poa_channelizer`
(per-channel envelope vs a reference implementation on synthetic + real IQ),
`test_poa_msk_frame` (12.5 kHz envelope slices → blocks; slices are tiny and
can live in-repo), `test_poa_l2` (parity/CRC repair vectors incl.
`fixprerr`/`fixdberr` cases), `test_poa_e2e_acars` (wideband IQ → expected
ACARS messages, count and content ≥ oracle-parity threshold). Plus
`poa_mod.c` + a synthetic round-trip test (the two-implementations
discipline of `test_vdl2_l2.c` `build_tx()`).
**Risk:** subtle float divergence from acarsdec (timing loop, matched-filter
fractional index `msk.c:102-104`). Mitigate: port verbatim first, refactor
only after e2e parity; don't "improve" constants (memory
feedback_no_test_fitting, feedback_dont_revert_grIridium).
**Verify:** e2e decode set ⊇ oracle set minus an agreed tolerance (target:
100 % of acarsdec's CRC-ok messages on the fixture; document any diff).

### Phase P2 — device front end
`frontend` branch in `dsp_processor_create/feed`; channelizer+MSK inline in
the feed context; counters (blocks started / bits / per-channel activity)
into /status; frame_queue push of completed blocks.
**Risk:** CPU in the feed path (the USB consumer must never stall — memory
feedback_rate_zero_check_dma_int_first lineage). Mitigate: measured at -O2
on-device before/after; the tagger it replaces costs more; fallback task
split is pre-designed (§3). Watch `rb_full_drops`.
**Verify:** Iridium + VDL2 device smokes green (their paths untouched);
`band=poa` boot shows stream up, ~0 % rb_full, per-channel activity counters
moving on live air.

### Phase P3 — L2 on-device + emit
`process_one_poa()` in frame_decoder → `poa_l2` repair → `acars_deliver()`
with direction + exact channel `freq_hz`; POA funnel counters + `/status`
`"poa"` block + dashboard section; `band_decode_stats_get` POA arm
(autotune metric + reception classifier follow for free).
**Risk:** low — byte work on Core 0, VDL2 precedent throughout.
**Verify:** host `test_poa_e2e_acars` re-run through the *device-shaped*
call chain (frame_queue item → process_one_poa → acars_deliver, host-linked
like the VDL2 suites); live bench: first real POA ACARS in /messages + SD
NDJSON. This is the phase with the user-visible win — get here fast.

### Phase P4 — config + feed polish
`po_chans` parsing/validation + serial/web editing; autotune gate;
`AF_BAND_POA` airframes schema + `test_airframes_fmt` byte-exact fixtures +
port-5550 defaults; scanner/band-health/decode-survey verified gated.
**Verify:** feed JSON validated against a real acarsdec sample line;
airframes staging acceptance mirrors the feeder-plan checklist
(`docs/2026-07-27-airframes-feeder-plan.md`).

### Phase P5 — device smoke + live calibration
`CONFIG_SMOKE_TEST_POA` (synthetic `poa_mod` burst through the real
channelizer→MSK→L2→libacars chain, golden reg/text asserted); pre-push
trailer support; then a multi-hour live A/B at this site: POA msgs/h vs the
Mac acarsdec reference on the same antenna (the 6 h run's ~25 msgs/h is the
bar), gain sweep via fill-cal, per-channel yield table to prune/extend
`po_chans`.
**Verify:** smoke green in CI matrix alongside RAW + VDL2 smokes; live
parity ratio recorded in a bring-up doc (full-cycle data before verdicts —
memory feedback_no_premature_conclusions_partial_data).

---

## 8. Test & golden strategy (summary)

- **Oracle:** acarsdec itself, run on the shared wideband capture (it is
  the site's proven POA decoder). Same cross-validation discipline as
  gr-iridium/dumpvdl2 (memory project_cross_validate_against_gr_iridium).
- **Layered fixtures:** tiny in-repo 12.5 kHz envelope slices + parity/CRC
  vectors for unit suites; the big 2.5 MSPS capture stays in
  `~/iridium_capture/poa_ref/` with a fixture-prep script (the
  `direct_if_dump.py` pattern, memory reference_host_test_fixtures).
- **Synthetic independence:** `poa_mod` round-trip guards against porting a
  bug into both encoder and decoder (VDL2 `build_tx()` precedent).
- **Regression walls:** `test_band_pipeline_iridium` (bit-identity),
  `test_band_profile` (all-field pins ×3 bands), VDL2 host suites + both
  existing device smokes on every phase commit.
- **Device smoke:** synthetic (no giant fixture on-flash), per the VDL2
  smoke model (`smoke_test.c:484+`).

## 9. Risks, honestly ranked

1. **Front-end paradigm split** (§1): dsp_processor becomes two-moded;
   the vtable abstraction no longer covers the whole band story. Contained
   by the `frontend` profile field, the honest stub, and the fact that the
   POA path shares zero state with the tagger path. Biggest residual risk
   is *conceptual* — future contributors must learn two front ends; the
   plan's answer is documentation + the profile field making it explicit.
2. **Shared-edit regressions** — `band_key()` and the profile struct (§4);
   guarded by tests named above.
3. **Real-RF divergence from the oracle** — RTL-SDR v4 vs the Mac rig,
   gain, image behaviour at VHF with the HC610 (an L-band antenna — POA
   yield with this antenna is unproven; the 6 h reference run used the Mac
   setup). Bring-up may reveal the *antenna*, not the code, as the limit —
   plan measures against the Mac on the same antenna first (P5).
4. **Feed-context CPU** — measured, with a pre-designed fallback (§3, P2).
5. **Effort honesty:** VDL2 took the same shape and was a multi-week,
   multi-phase effort. The demod port is days; the ripple + tests +
   bring-up is the real cost. Budget accordingly; P0–P3 is the minimum
   shippable slice that decodes POA to /messages.

## 10. Non-goals / explicitly rejected over-engineering

- **No simultaneous multi-band** (Iridium+VDL2+POA concurrently) — one band
  per boot stands; multi-receiver is the existing separate track.
- **No PIE/Q15 demod** (§3) and **no polyphase/FFT channelizer** — direct
  per-channel mix is cheaper to write, verify, and cross-validate at ≤ 8
  channels.
- **No energy gate/squelch for POA**, no `po_tag_thr` — oracle parity.
- **No new HTTP routes** (max_uri_handlers trap) — existing endpoints only.
- **No generalizing `band_pipeline_t` into a universal front-end vtable**
  — one CHANNELIZED consumer doesn't justify it; revisit only if a second
  continuous band actually lands (Inmarsat aero is *parked*).
- **No acarsdec output/label-decode (OOOI) port** beyond what
  `airframes_fmt` needs — libacars app decode already runs in
  `acars_deliver`; `DecodeLabel`/OOOI extraction (`output.c:280-295`) is
  airframes-schema sugar, added only if the 5550 ingest requires it
  (check during P4, not before).
- **No LO steering / scanning for POA** — fixed park.

## 11. Critical files

Framework: `common/band_pipeline/band_profile.{h,c}`,
`common/band_pipeline/band_pipeline.h`, `common/band_pipeline/band_decode_stats.h`,
`p4-usb-host/main/band_select.{h,c}`, `p4-usb-host/main/app_config.{h,c}`
(esp. `band_key`, `app_config.c:142-145`).
Front end: `p4-usb-host/main/dsp_processor.c` (`:300-370`, `:387`),
`p4-usb-host/main/class_driver.c` (call sites only, no edits),
`p4-usb-host/main/worker_core1.c` (`:1410-1427` guard only).
New component: `common/poa_decoder/*` (port of
`~/dev/vdl2-tools/acarsdec/{rtl.c,msk.c,acars.c,syndrom.h}`).
Decode/emit: `p4-usb-host/main/frame_decoder.c` (`:123`, `:421`, `:854`,
`:950`, `:993`), `common/iridium_decoder/frame_queue.h`,
`p4-usb-host/main/msg_ring.h`, `sd_log.c`, `acars_push.c` (`:159-192`),
`common/acars_feed/airframes_fmt.{h,c}`.
Presentation/config: `http_server.c`, `status_logger.c` (`:138-175`),
`serial_cmd.c` (`:179-183`), `autotune.c` (`:298-306`), `smoke_test.c`.
Tests: `tests/host/test_band_profile.c`, `test_band_pipeline_iridium.c`
(guard), `test_airframes_fmt.c`, new `test_poa_*` suites.
