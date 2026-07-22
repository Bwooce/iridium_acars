# VDL2 (band=vdl2) V4 device bring-up — procedure + risk plan

Status: **plan, not yet executed.** The `feat/vhf-vdl2` branch reproduces
8/9 golden ACARS on the sigidwiki baseband capture in host tests, but has
**never run on silicon** and has **never seen a wideband VHF stream** — the
golden fixture is a single 25 kHz baseband channel, not the 2.5 MSPS
wideband path the tagger drives. This document is the checklist to get
band=vdl2 running on the real P4, and the list of things expected to go
wrong first.

Scope of the touched surface (why this needs a gate): worker_core1,
frame_decoder, http_server, dsp_processor (band profile wiring), the
band_pipeline component, and FRAME_QUEUE_MAX_BITS all changed on this
branch. Iridium is the default band and the code paths are shared, so the
Iridium regression must be re-proven on silicon before merge.

---

## (a) Pre-merge gate: RAW_IRIDIUM device smoke — MANDATORY

The one hard gate before this branch merges: **Iridium decode must be
unregressed on real hardware.** Host tests prove the byte-identical claim
for the pure code; they do NOT prove the device DSP/PIE path is
unaffected (see memory: "Device smoke MANDATORY after DSP-path commit" —
host suite ≠ device decode). worker/tagger/frame_decoder were all touched.

### Smoke-build compiles with VDL2 present — VERIFIED

Confirmed on this branch (2026-07-22): the RAW_IRIDIUM smoke variant
builds cleanly with all the VDL2 code compiled in.

```
# from scripts/smoke_run.sh set_config mechanics — enable SMOKE_TEST_MODE
# + CONFIG_SMOKE_TEST_RAW_IRIDIUM, all other SMOKE_TEST_* symbols off,
# then scripts/build.sh
```

Result: `p4-usb-host.bin` linked (6.02 MB smoke binary vs 1.47 MB
production — the fixture corpus is compiled in), **zero `error:` in the
build log**, `CONFIG_SMOKE_TEST_RAW_IRIDIUM=1` present in the generated
`build/config/sdkconfig.h`. The VDL2 component (rs_vdl2 + avlc + demod +
l2) does not break the smoke build. The production sdkconfig was restored
and a clean production build re-verified afterward.

> Use `scripts/smoke_run.sh raw` to run the full build→flash→capture cycle
> on the bench. It snapshots the production sdkconfig, refuses to run on a
> smoke-tainted config (see memory: "smoke-tainted sdkconfig crash-loops
> prod"), and `scripts/smoke_run.sh restore` returns to production.

### Gate procedure

1. `scripts/smoke_run.sh raw` on the bench P4 (Iridium antenna attached,
   HC610 L-band — this is an Iridium test, not VDL2).
2. Require `SMOKE_PASS` in the serial capture. RAW_IRIDIUM drives the full
   wideband tagger→worker→frame_decoder→BCH path on the recorded raw
   Iridium fixture; a pass proves the shared path still decodes Iridium
   after the band refactor.
3. Also run `scripts/smoke_run.sh frame` (FRAME_DECODER variant) —
   proves classify→BCH→reassembler→libacars still yields the golden
   ACARS message (REG A62001).
4. `scripts/smoke_run.sh restore` and confirm a clean production build.
5. Record the pass in the commit trailer (`Smoke-verified: RAW_IRIDIUM,
   FRAME`), per the pre-push convention.

**Do not merge on host-green alone.** The branch commit already carries a
`Smoke-skip:` trailer flagged as WIP — that debt is settled here.

---

## (b) band=vdl2 flash + bring-up steps

### Antenna — the blocking prerequisite

**The HC610 is L-band only (memory: Calian HC610 active, 1616–1626 MHz).
It physically cannot hear 136.975 MHz.** band=vdl2 with the HC610
attached will show **zero bursts and zero decodes, and that is expected,
not a bug.** VDL2 bring-up requires a **VHF airband antenna** (118–137 MHz;
a 1/4-wave ground-plane cut for ~137 MHz, or any airband/ADS-B-ish VHF
whip). Do not attempt VDL2 bring-up until the correct antenna is on the
dongle.

Note also the HC610's 28 dB LNA + bias-tee are irrelevant/absent on the
VHF antenna — expect to raise RTL tuner gain for VDL2 vs the near-minimum
gain the HC610 wanted (memory: RF tuning gain). The provisional tagger
threshold does not depend on absolute gain (it is relative to the per-bin
EMA noise floor), but the demod BER does — fill the ADC without
compressing the R820T.

### Set the band + LO (serial or NVS)

The band is an NVS soft-switch (`app_config` key `band`, default iridium).
Reboot to apply — the band profile + pipeline resolve once at init.

```
# over UART0 serial_cmd (memory: serial_cmd NVS interface):
set band vdl2          # band_profile_from_str; unknown -> iridium
set lo_hz 0            # OPTIONAL — see note
reboot
```

**LO note.** app_config resolves the LO default from the band profile
ONLY when NVS `lo_hz` was never explicitly set (app_config.c: band read
first, then `nvs_get_u32_or(lo_hz, band->default_lo_hz)`). A previously
stored Iridium `lo_hz` (e.g. 1620.6 MHz from prior tuning) will
**override** the VDL2 default and park the tuner in L-band — VDL2 will
hear nothing. Either explicitly `set lo_hz 136812500` (the VDL2 profile
default: parks the LO at 136.8125 MHz so the CSC 136.975 sits at
+162.5 kHz and no channel lands on DC), or clear the stored key so the
band default applies. **Verify the parked LO on `/status` after reboot**
(`lo_now_hz` / the "LO frequency" row) — this is the single most common
"0 decodes" cause and it is not a code bug.

### First-boot checks on /status

The dashboard gains a **"VDL2 decode (band=vdl2)"** table (only rendered
when band=vdl2; the Iridium dashboard is byte-identical otherwise). The
JSON `/status` carries the same counters under `decode.vdl2`. Watch, in
order:

1. **Free PSRAM on first boot.** The frame queue grew from 2048→16832
   max bits per slot (VDL2 max frame), ~+1 MB PSRAM for the 64-slot pool.
   The demod also allocates ~110 KB PSRAM per burst (iq105 + bits + soft),
   MALLOC_CAP_SPIRAM. Confirm the "PSRAM free / largest" row is healthy
   (budget note: ~4 MB headroom before this; +1 MB pool leaves ~3 MB).
   A failed pool alloc = no decode; a failed per-burst alloc = graceful
   bail (NULL-checked), visible as bursts-seen climbing with sync/phy
   stuck at 0.
2. **Does the tagger tag VDL2 bursts at all?** "Bursts → demod sync" — the
   left number is `vdl2_pipeline_bursts_seen` (tagger handed a burst to the
   VDL2 pipeline). **If this stays 0 while VHF traffic is present, the
   PROVISIONAL tagger profile is not detecting VDL2 energy — tune the
   profile, do NOT assume a demod bug** (see risk list).
3. **Demod sync / PHY frames.** The right number of "Bursts → demod sync"
   is preamble+header locks; "PHY frames / L2 fail" is the L2 funnel. Sync
   climbing but PHY/AVLC stuck = demod-BER or L2 issue. AVLC FCS-valid > 0
   with ACARS > 0 = end-to-end success.
4. **WDT / stream stability.** Confirm no health-wdt stream stalls, no
   panic loop, USB `rate` > 0 (memory: SDR online ≠ daemon message — judge
   by rate, not log text). The VDL2 demod runs on the worker (Core1,
   scalar, no PIE — the PIE placement/ownership landmines do not apply),
   but it is heavier per burst than Iridium's; watch worker load %.

---

## (c0) VERIFY + LIKELY CODE CHANGE — worker burst-window cap vs long VDL2 frames

This is not a passive risk; it is an **action item that will probably require a code
change at V4** — do not let it get lost in the risk table below.

**What:** the worker's per-burst sample window (`WB_MAX_BURST_SAMPLES` in the
worker/ingest path — the ~250 ms cap sized for Iridium's short bursts) is *upstream*
of the demod and is **NOT** the same thing as `FRAME_QUEUE_MAX_BITS` (which was already
grown to hold a decoded VDL2 frame's bits). VDL2's **maximum I-frame is ~0.54 s** — more
than 2× that window. A long VDL2 transmission's IQ can therefore be **truncated before
the demod ever sees the whole frame**, so the demod decodes a partial frame and it fails
downstream (PHY/RS/AVLC), silently, for long frames only.

**Why it wasn't fixed now:** the golden capture's frames may not have exercised the long
tail, and sizing the window correctly needs the **real VDL2 frame-length distribution** —
which needs a live wideband capture. Guessing the window size blind risks over-allocating
the (shared, per-burst) sample buffer for both bands.

**Verify at V4:** measure the live frame-length distribution (dumpvdl2 prints frame
sizes; on our side, watch whether `phy_ok`/`avlc_ok` systematically drop for the longest
transmissions while short frames decode fine). If a meaningful fraction of real frames
exceed the ~250 ms window → confirmed.

**Likely fix:** make the burst-sample window **band-dependent** — grow it for `band=vdl2`
to hold ~0.54 s at the VDL2 detect sample rate (and check the RAM cost of the larger
per-burst extract buffer, which is the reason it isn't just globally enlarged). Possibly
multi-window frame assembly if a single grow is too costly. This is real code, not a
`#define` tune — scope it once the distribution is measured.

## (c) Risk list

| Risk | Symptom | Mitigation |
|---|---|---|
| **Provisional-uncalibrated tagger** (pre/post pad, width, threshold in band_profile.h are derived from the VDL2 PHY + dumpvdl2, NOT measured — no wideband VHF capture exists to calibrate against) | Bursts-seen = 0 despite live VDL2 traffic, OR bursts merge/split (sync ≫ phy_ok with truncated frames) | **0 decodes ≠ demod bug.** Tune the tagger FIRST: raise `fbt_post_len` if frames split mid-transmission; lower `tagger_threshold_db` if weak bursts miss detection; widen/narrow `fbt_width_bins`. Capture a wideband VHF IQ slice and calibrate offline, the way Iridium was calibrated vs gr-iridium. |
| **~110 KB PSRAM per-burst demod allocs** (iq105 + bits + soft, MALLOC_CAP_SPIRAM) | Under memory pressure the alloc returns NULL and the burst is dropped (graceful bail); bursts-seen climbs, sync stuck at 0 | Watch PSRAM free on /status; the allocs are NULL-checked and bail cleanly, so this degrades rather than crashes. If chronic, pool the demod scratch instead of per-burst malloc (future work). |
| **Frame-queue PSRAM growth (+~1 MB)** | Lower PSRAM headroom on first boot; in the worst case pool alloc fails and there is no VDL2 decode at all | Confirm PSRAM free row after reboot. ~3 MB headroom remains after the growth; the growth is a compile-time constant (FRAME_QUEUE_MAX_BITS), the same for both bands, so it also costs Iridium ~1 MB — verify Iridium PSRAM is still healthy in the smoke gate. |
| **Demod BER on weak bursts** | Sync locks but PHY/RS/AVLC funnel loses frames; bad_fcs or too_short climb; RS blocks fail | Expected on marginal RF — the 1/9 golden miss is exactly this (short 2-parity RS blocks alias, FCS arbitrates out, matching dumpvdl2's own safety model). Raise gain to fill the ADC without compressing; this is air-truth SNR, not a code fault. A parallel agent is upgrading the demod. |
| **NO remote recovery when the antenna moves** | Swapping to the VHF antenna / re-siting means being physically present; a wedge with the wrong antenna leaves the device unreachable if WiFi also drops | **Be present for VDL2 bring-up.** Have a revert-to-iridium plan ready: `set band iridium; set lo_hz <known-good>; reboot` over serial_cmd (works even when WiFi/web are down — memory: serial_cmd NVS interface). Do not do first VDL2 bring-up on a headless/remote node. |
| **Shared-path Iridium regression** (worker/tagger/frame_decoder touched) | Iridium decode drops on silicon even though host tests are green | The RAW_IRIDIUM + FRAME device smoke gate in (a) is exactly this guard. Mandatory before merge. |

---

## (d) Acceptance: live A/B vs dumpvdl2

The definitive acceptance test, mirroring the Iridium project's
"cross-validate against gr-iridium" discipline (dumpvdl2 is to VDL2 what
gr-iridium is to Iridium):

1. On the SAME VHF antenna and site, run the P4 (band=vdl2) and a
   reference SDR feeding **dumpvdl2** concurrently — or capture one
   wideband VHF IQ file and replay it through both.
2. Compare the decoded ACARS set: registration, mode, label, and the FCS
   verdict per frame. The P4 should reproduce dumpvdl2's FCS-valid ACARS
   frames (allowing for demod-BER-bound marginal misses, as on the
   golden — track the ratio, not an exact match).
3. Log the counters from `/status` `decode.vdl2` alongside dumpvdl2's own
   stats: bursts/sync/phy_ok/rs_ok/avlc_ok/acars, and the FCS-valid floor.
4. Acceptance = the P4's FCS-valid ACARS output tracks dumpvdl2's on the
   same input, within the demod-BER margin. Capture a wideband slice
   during this run and fold it into the host fixtures + use it to
   **calibrate the provisional tagger profile** (closes the loop on the
   uncalibrated caveat above).

Until this A/B is done on a real VHF antenna, band=vdl2 is
demonstrated-in-simulation only. Keep band=iridium as the default.
