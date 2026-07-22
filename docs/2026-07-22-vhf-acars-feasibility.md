# VHF ACARS on the ESP32-P4 + RTL-SDR — feasibility (POA + VDL Mode 2)

> Status: feasibility + design intent. Not yet on the build. 2026-07-22.
> Part of a whole-band ACARS feasibility pass (VHF here; satellite in
> `../inmarsat-acars-feasibility.md`; HFDL assessed and shelved — see §5).

## TL;DR

Adding **VHF ACARS** to this device is the highest value-for-effort extension, and
it's a **software-only** change on the *same hardware* (only the antenna changes —
any cheap VHF airband whip). Two protocols share that antenna:

- **VDL Mode 2** (136.65–136.975 MHz, D8PSK 31.5 kbps) — **highest value of any
  ACARS band** (the majority of modern ACARS traffic), all channels fit one 2.5 MHz
  window. Medium effort.
- **VHF POA** ("Plain Old ACARS", ~129–137 MHz, AM-MSK 2400 bps) — **lowest effort
  of any band** (no FEC), moderate/declining value, but 131.550 is worldwide.

**Recommended order (revised 2026-07-22 for a SYDNEY deployment): VDL2 first; skip
POA.** The original "POA-first as a cheap architecture-forcing step" was reversed on
Sydney-specific grounds (see the Sydney section below): POA carries only a *trickle*
here (one live channel, 131.550, minority link behind VDL2), a working MSK demod
de-risks *none* of VDL2's hard parts, and the shared scaffolding (antenna, band
profile, tagger parameterisation) is paid either way — so POA-first would build a
throwaway demod to defer the band that actually carries Sydney's traffic. Build
**VDL2 (136.975 MHz, D8PSK)** directly; keep POA as a cheap optional bolt-on
(a 131.55 MSK profile) once the scaffolding exists, off the critical path.

## Why this is easy on this hardware

The decisive facts (all verified against the current tree):

1. **RF is covered and easy.** The R820T tunes VHF trivially and is near its
   sensitivity sweet spot there. LO + sample-rate are already NVS runtime config
   (`app_config.h` `lo_freq_hz`, `app_config_set_sample_rate_hz()`, applied in
   `class_driver.c`), so retuning to 131/136 MHz is a config write. A cheap airband
   whip/dipole/ground-plane is the only new part.
2. **Compute is a non-issue.** Iridium (2.5 MHz, 25 ksym/s QPSK, UW-correlator-
   bound) is by far the hardest DSP load here. VHF ACARS is 2.4–31.5 kbps in a
   ≤25 kHz channel — acarsdec runs on a Pi Zero, dumpvdl2 on a Pi 3. The P4 that
   keeps up with Iridium will loaf.
3. **Both are burst signals**, so the existing `dsp_processor.c` /
   `fft_burst_tagger` front end (tag → extract → dispatch) fits naturally — an
   ACARS/VDL2 burst lights up an FFT bin exactly like an Iridium burst. What's
   Iridium-specific is *constants*, not mechanism (`FBT_BURST_WIDTH`, pre/post pads,
   `FS_DETECT_HZ` baked into burst coordinates) — parameterise them per band.
4. **Every band ends in the same call.** `frame_decoder.c` already invokes
   `la_acars_parse_and_reassemble()` (libacars) on a raw ACARS frame. acarsdec and
   dumpvdl2 both terminate in that identical libacars API — so the entire
   application/reassembly/output/telemetry stack (`acars_push.c`, `msg_ring.c`,
   `http_server.c`, `sd_log.c`, airframes.io feeder) carries over **unchanged**.

## Per-protocol

### VHF POA — ~129.125–136.900 MHz, AM-MSK 2400 bps

- **RF:** trivial. One constraint — regional POA channels span up to ~8 MHz (US:
  130.025/131.125/131.550/136.750), wider than the 2.5 MHz window; cover the 3–5
  channels that fit, or time-share via the existing `scanner.c` retune. Europe
  clusters tightly (131.525/131.725/131.825 in <400 kHz). Can drop sample rate to
  ~1 MSPS (already NVS) to shrink tagger load to near-zero.
- **DSP (new, small):** POA transmissions are 0.1–1 s bursts with a 128-bit
  pre-key → tag→extract→demod fits. Reuse `rotate_to_dc` + `direct_if_decim` +
  `sym_timing`; the MSK demod itself is new but small (acarsdec's is a few hundred
  lines). **No FEC decoder needed** (character parity + CRC-16 + optional 1-bit
  repair). The whole `ida_*`/`sbd_reassembler` L2/L3 layer is simply **bypassed** —
  POA frames go straight to libacars, whose own multi-block reassembly (already
  compiled) handles ACARS continuation.
- **Reference:** acarsdec (GPL-2), also usable as cross-validation ground truth on
  shared IQ captures (the repo's established methodology vs gr-iridium).
- **Effort: LOW** (~2–4 weeks incl. host fixtures). **Value:** moderate, declining
  (migrating to VDL2), but nonzero worldwide.

### VDL Mode 2 — 136.650–136.975 MHz, D8PSK 10.5 kBd (31.5 kbps)

- **RF:** all VDL2 channels sit inside ~350 kHz — a *single* window covers every
  channel at once (better than Iridium's 10 MHz problem). Same VHF antenna as POA.
  (But POA and VDL2 channels are up to ~5.4 MHz apart, > 2.5 MHz instantaneous BW —
  **one dongle can't do POA and VDL2 simultaneously**; time-share or pick one.)
- **DSP (new, moderate):** D8PSK CSMA bursts with a known training sequence — the
  closest cousin to what exists. The 2nd-order PLL in `qpsk_demod.c` + `sym_timing.c`
  generalise from 4-ary to 8-ary differential PSK with modest changes. New pieces:
  training-sequence sync (replaces `uw_correlator`), bit descrambler, **Reed–Solomon
  RS(255,249)** (new, small, cheap at these rates), and **AVLC** (HDLC-style)
  framing with bit-stuffing. Above AVLC, some traffic is ATN/X.25 rather than ACARS;
  the ACARS-over-AVLC subset feeds libacars.
- **Reference:** dumpvdl2 (GPL-3) — written by the libacars author, so the cleanest
  possible fit to the existing integration.
- **Effort: MEDIUM** (~1.5–2× POA; D8PSK + RS + AVLC; ~4–8 weeks). **Value:
  highest of any band** — if you add only one, add this.

## Architecture — the NVS soft-switch

Selection is a **soft-switch in config**, as intended: one firmware, an NVS band
profile (`band=iridium|poa|vdl2`). Flash (12 MB app partition) and PSRAM (~4 MB
free; new demods are kilobytes) budgets allow it, and it fits OTA + bench workflow.

**Caveat on "soft":** for VHF the switch really is fully soft — POA↔VDL2 share the
VHF antenna, and iridium↔VHF is a software LO/rate/profile change (the *antenna* is
the only physical swap, and that's manual regardless). This is cleaner than the
satellite case, where the L-band SAW filter is a physical part (see the Inmarsat
doc's soft-switch caveat).

Shared-core / per-band split (concrete):

- **Shared, unchanged:** `class_driver`/`esp_libusb`/`librtlsdr`/`usbring`/
  `ingest_core1` → `signal_buffer` → `frame_queue` → `frame_decoder` task shell →
  `libacars_idf` → `msg_ring`/`acars_push`/`http_server`/`sd_log`/`band_health`/
  `scanner`. The reception-environment heuristic is band-agnostic too.
- **Shared, needs parameterisation (the tedious part):** `dsp_processor` /
  `fft_burst_tagger` — `FS_DETECT_HZ`, `FBT_BURST_WIDTH`, pre/post pads, threshold
  become band-profile fields instead of `#define`s. `detected_burst_t` already
  carries rel-freq in Hz, so downstream survives a rate change.
- **Per-band modules (new work):** a `band_pipeline` interface where
  `burst_pipeline.c` (Iridium) is joined by `poa_pipeline` (MSK + ACARS framing) and
  `vdl2_pipeline` (D8PSK + RS + AVLC), each in `common/` with host tests
  cross-validated against acarsdec/dumpvdl2 fixtures. Per-band L2 replaces
  `iridium_frame`/`ida_*`/`sbd_reassembler`; POA needs essentially none.

**Two standing hazards for any new PIE-touching demod:** the heap-position
early-alloc discipline (PIE buffers must be placed early or decode silently
corrupts), and one-PIE-owner-per-core.

## Hard blockers vs merely tedious

- **Hard (physics, not effort):** 2.5 MHz instantaneous BW ⇒ can't run POA + VDL2
  simultaneously on one dongle (time-share or choose). Everything else on VHF is
  tedious, not blocked.
- **Tedious:** runtime-ifying the tagger/rate constants; the MSK and D8PSK demods;
  RS + AVLC for VDL2 (trivial CPU); band-profile plumbing in `app_config`; building
  per-band ground-truth fixtures.

## §5 — where VHF sits in the full band sweep

| Rank | Band | Effort | New RF HW | Value | Firmware |
|---|---|---|---|---|---|
| 1 | **VDL Mode 2** | Medium | VHF antenna | **Highest** | soft-switch |
| 2 | **VHF POA** | **Low** | VHF antenna | Moderate, declining | soft-switch |
| 3 | Inmarsat Aero | Med-high | L-band patch + 1542 SAW | Good (oceanic ADS-C/CPDLC) | soft-switch + continuous-carrier mode |
| 4 | HFDL | High | upconverter / direct-sampling / V4 + HF antenna | High in principle | shared core only |

**HFDL shelved:** the R820T can't tune below ~24 MHz (needs an upconverter,
direct-sampling dongle, or V4), *and* the 8-bit RTL ADC's dynamic range on crowded
HF is a possibly-fatal sensitivity problem (serious HFDL users run 12–16-bit
frontends). You could do all the work and decode little — the honest answer for
HFDL is a different SDR, which breaks the fixed-hardware premise.

**Everything else swept:** MTSAT retired; SwiftBroadband-Safety / Iris / Iridium
Certus cryptographically closed; VDL 3/4 and LDACS dead/not-deployed; Inmarsat
C-band feeder is out of tuner range (>1766 MHz). VHF POA + VDL2 + Iridium (have) +
Inmarsat Aero (optional) are the live, receivable set on this hardware.

## Sydney deployment — why VDL2 first, skip POA (revised 2026-07-22)

This receiver is in suburban Sydney (near YSSY). The Australian VHF datalink reality
changes the ordering:

- **POA is a trickle here.** The one live POA channel over Australia is **131.550 MHz**
  (SITA Asia/Pacific primary — the US 130.x and Japan's 131.45 aren't used regionally),
  so Sydney offers ~one POA channel, not the 4–5-channel spread of North America. And
  Sydney's fleets (Qantas/Jetstar/Virgin 737/A320/A330/787) are VDL2-default, using POA
  only as opportunistic fallback; the POA-leaning airframes (older/regional/GA) are a
  small slice over a major hub. Expect real but minority POA on 131.55 — **order-of-
  magnitude fewer frames than VDL2** on 136.975. (Directional estimate from allocations
  + fleet equipage, not measured live counts, but VDL2 ≫ POA here is not close.)
- **VDL2 is where Sydney's traffic is** — 136.975 (worldwide VDL2 Common Signalling
  Channel) is active, SITA/ARINC run mature VDL2 ground infrastructure at the AU capitals,
  and Airservices' continental CPDLC runs over ATN/VDL2.
- **POA-first buys almost nothing here.** The shared scaffolding (antenna, band-profile
  config, tagger parameterisation) is paid either way; the *uniquely* POA code (MSK demod
  + character framing) de-risks *none* of VDL2's hard parts (D8PSK carrier/timing recovery,
  training-sequence sync, RS(255,249), AVLC/HDLC bit-stuffing). And the end-to-end
  plumbing smoke (tagger→demod→libacars→push) comes for free inside a stage-wise VDL2
  build cross-validated against dumpvdl2 — no throwaway protocol needed.

## Recommendation

**Build VDL2 (136.975 MHz, D8PSK) directly on the same firmware via an NVS band profile;
skip POA on the critical path.** Keep POA as a cheap optional bolt-on later (a 131.55 MHz
MSK profile) once the antenna + band-profile scaffolding exist — it's a small self-
contained add if the last slice of older-airframe traffic is ever wanted. Cross-validate
every VDL2 DSP stage against dumpvdl2 (same author as the already-vendored libacars),
matching the repo's gr-iridium-style discipline. Reassess Inmarsat Aero (see its doc) once
VHF ships and an L-band patch + 1542 MHz SAW are in the BOM. Leave HFDL until the dongle
changes.

(Historical note: the TL;DR/§ earlier in the *first* draft of this doc recommended
"POA first as a cheap architecture-forcing step" — that generic ordering was reversed
for Sydney per the section above; POA-first only makes sense at a POA-heavy /
VDL2-under-served site, which Sydney is not.)

## References

- acarsdec — https://github.com/TLeconte/acarsdec (VHF POA, GPL-2)
- dumpvdl2 — https://github.com/szpajder/dumpvdl2 (VDL2, GPL-3)
- libacars — https://github.com/szpajder/libacars (shared ACARS app layer, GPL-3;
  already vendored at `common/libacars_idf`)
- Companion: `../inmarsat-acars-feasibility.md` (satellite), and the reception /
  band_health work in `2026-07-22-reception-environment-heuristic.md` +
  `2026-07-22-new-site-commissioning.md`.
