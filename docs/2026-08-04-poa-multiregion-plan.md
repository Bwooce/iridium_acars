# POA multi-region channel configuration + per-region dropdown

Status: IN PROGRESS (branch `feat/poa-onband`). Owner task: see task list
("POA multi-region: discovery soak + airframes data + region dropdown").

## Goal

Let a board be configured for POA (plain VHF ACARS) by picking a **region** at
setup, instead of hand-entering a channel CSV. A region fixes **both** the LO
centre and the channel list, because one RTL dongle only hears a 2.5 MHz window
(LO ± 1.25 MHz) and different regions' ACARS frequencies span more than that.

## Hard constraint: the 2.5 MHz window

- LO 130.8 MHz → hears **129.55–132.05 MHz**. Passband is flat to ~±1.0 MHz,
  soft rolloff (a few dB, NOT a cliff) out to ±1.25. ACARS' ~23 dB integrate-
  dump processing gain absorbs the edge loss, so the whole ±1.2 MHz is usable.
- `POA_MAX_CHANNELS = 8`. Compute is ~free (single-precision decoder, dsp 1-2%
  at 4ch), so channel COUNT is not the limit — the WINDOW is.
- At LO 130.8, **almost every world ACARS freq fits** (130.025, 130.425,
  130.450, 131.125, 131.450, 131.475, 131.525, 131.550, 131.725, 131.825).
  The only major one OUT of window is **129.125** (ARINC worldwide primary).
- So the LO only needs to move for the Americas (to reach 129.125), and you
  cannot get 129.125 AND 131.550 well in one 2.5 MHz window → that's the
  multi-receiver case (task #17) OR the 3.2 MHz experiment below.

## Researched frequencies by region (sources: airframes.io docs,
## radioreference/sigidwiki, thebaldgeek — see session)

- Worldwide primary: **131.550** (SITA), **129.125** (ARINC)
- North America (ARINC): 130.025, 130.425, 130.450, 131.125 (+131.550, 129.125)
- Europe (SITA): 131.525, 131.725, 131.825 (+131.550)
- Japan/Asia: 131.450, 131.475
- Pacific/Australia: 131.550 primary (regional secondaries UNCONFIRMED)

**Provisional** region table (LO + channels) — DO NOT ship until validated:

| Region | LO | Channels |
|---|---|---|
| Australia / Pacific | 130.800 | 131.550 + AP secondaries (MEASURE first) |
| Europe | 130.900 | 131.725, 131.825, 131.550, 131.525 |
| North America (SITA 131.5) | 130.800 → 130.875 | 130.025, 130.450, 131.125, 131.550, 131.725 |
| Americas (ARINC 129.1) | 129.800 | 129.125, 130.025, 130.425, 130.450 |
| Custom | manual | free-text po_chans + po_lo |

## NORTH AMERICA COMPROMISE (the one region that does NOT fit one window)

Every region above fits a single 2.5 MHz window EXCEPT North America. This is
documented here as the canonical record:

- **Why:** NA's active POA channels span **129.125 → 131.825 MHz ≈ 2.7 MHz** —
  wider than one 2.5 MHz window. And 2.5 MSPS is this platform's hard ceiling:
  2.8 and 3.2 MSPS were both TESTED and WEDGE the RTL dongle (device-side
  RTL2832U FIFO/latency limit, not our USB pool — see the section above +
  [[reference_p4_max_sample_rate_2500]]).
- **The crux:** NA's TWO BUSIEST channels are the worldwide primaries
  **131.550 (SITA)** and **129.125 (ARINC)**, and they are **2.425 MHz apart** —
  they physically cannot coexist in one 2.5 MHz capture.
- **The compromise — two complementary presets** (in poa_regions.h; operator
  picks the half matching local traffic):
  | Preset | LO (MHz) | Channels | Captures | Sacrifices |
  |---|---|---|---|---|
  | `north_america` "North America (SITA 131.5)" | 130.875 | 130.025, 130.450, 131.125, 131.550, 131.725 | the 131.550 SITA half + mid ARINC | 129.125 |
  | `americas_arinc` "Americas (ARINC 129.1)" | 129.800 | 129.125, 130.025, 130.425, 130.450 | the 129.125 ARINC-low half | 131.550, 131.725, 131.125 |
  (130.025/130.450 are mid-band → present in BOTH, reachable from either LO.)
- **Full NA at once = a SECOND receiver** on the other half (multi-receiver,
  task #17). It is NOT achievable by widening the window on one radio.
- **Operator guidance:** for a US/Canada site, start with `north_america`
  (131.550 is the busiest single NA channel); switch to `americas_arinc` only if
  the local traffic is ARINC-129 heavy. A daytime per-channel `ok=` soak on each
  half settles which the site actually needs.

## MEASURE-FIRST evidence so far (YSSY, one overnight)

Aggregated 113,085 `POA_STATS` lines (old 4-ch set, ~8 h):

| freq | sync | blk | ok(delivered) |
|---|---|---|---|
| **131.5500 (SITA)** | 522774 | 49 | **37** |
| 130.0250 (NA) | 513189 | 11 | 0 |
| 130.4250 (NA) | 503002 | 7 | 0 |
| 130.4500 (NA) | 514709 | 10 | 0 |

- Only **131.550 is proven** here (37 delivered). NA channels started blocks
  (blk 7-11) but delivered 0 — marginal/unproven, NOT confirmed dead.
- 131.450/475/525 (the Australia guess currently flashed) are Japan/Europe
  freqs with only ~15 min of data — UNMEASURED. Commit 801459b flashed them;
  treat as provisional.
- LESSON ([[feedback_no_premature_conclusions_partial_data]]): the first
  "NA is dead air" call was 6 s of pre-dawn data. Aggregate the full window.

## Plan (advisor-endorsed sequencing)

1. **[this flash] Runtime `po_chans` setter** — so channel changes are a
   POST+reboot, not a firmware reflash. Shared CSV parser (app_config setter
   validates with the SAME parser dsp_processor uses at boot). + `/sdrcfg?chans=`
   + `set po_chans` serial + host test. (Being built now.)
2. **8-channel discovery soak** — load 130.025,130.425,130.450,131.125,131.450,
   131.475,131.525,131.550 (all fit at LO 130.8) across a BUSY daytime window;
   per-channel `ok=` picks the live Australia set. CHECK: 4→8 doubles per-channel
   buffers — verify `poa_frontend_create` PIE-buffer placement is still legal
   ([[project_heap_position_decode_bug]]); the kernel self-test won't catch it.
3. **airframes.io per-location data** — github.com/airframesio/data
   (frequency-stats) + app.airframes.io/about give feeder-OBSERVED freqs by
   location; fill the region table from THAT, not guesses.
4. **Region dropdown** — firmware region table (`po_region` NVS key resolving to
   LO+chans) so it works over serial too; web /config `<select>`; Custom
   override retained. LANDMINE: if it adds a web route, bump
   `cfg.max_uri_handlers` ([[feedback_httpd_max_uri_handlers]]) or boot panic-loops.

## Higher sample rate for a single-window NA — TESTED 2026-08-04: BOTH 3.2 AND 2.8 MSPS FAIL

RESULT: NA cannot be a single preset at ANY sustainable rate. **2.8 MSPS** (Fable's
"plausible" target, just above the 2.56 no-drop ceiling) wedges the dongle with the
SAME signature as 3.2 (Dev 0 EP0 error, completed=0, status_errors=0). 2.5M streamed
clean before AND after each wedge → the rate is the cause, dongle recovers (no reseat).
ROOT CAUSE (Fable, sourced): device-side RTL2832U — tiny FIFO + no flow control, so
worst-case USB IN-latency is the limit (NOT bandwidth: 6.4 MB/s is ~12% of HS bulk).
Osmocom: 2.4 MS/s on regular host controllers, stable 3.2 only on exotic Etron
controllers. A bigger USB pool can't fix it (desktop rtl_sdr's 3.75MB pool still
drops at 3.2M; our DMA-INT reserve caps us at ~4-6×8KB regardless). So ~2.5 MSPS is
this platform's ceiling → NA stays TWO presets; full single-radio NA = multi-rx
(task #17). If ever revisited: verify continuity via RTL testmode counter, not byte
rate (byte rate lies about silent FIFO drops). See [[reference_p4_max_sample_rate_2500]].

--- original 3.2-only test note (superseded by the 2.8 result above) ---
RESULT: NA cannot be a single preset via 3.2 MSPS on the current firmware.
Tested by bumping BAND_POA_FS_HZ->3200000 + coupling the RTL rate to the POA
profile fs (POA-scoped, in class_driver action_start_stream), flashed, POA band.
Outcome: the R82XX tuner PLL LOCKS and DMA-INT is fine (99 KB free, needs 32 KB),
but `usb.completed=0` and `rate=0.00` sustained (18 s and 59 s uptime), with an
`E USBH: Dev 0 EP 0 Error` — the USB bulk stream never delivers a byte at 3.2M.
Disambiguated (NOT a wedged dongle): reverting to 2.5 MSPS on the SAME hardware
immediately restored `completed` climbing / rate 4.77 / 0 drops. So 3.2 MSPS is
the cause. Root cause (likely): the USB transfer pool (4x8KB, tuned for 2.5M =
5 MB/s) can't keep the RTL FIFO drained at 3.2M = 6.4 MB/s, so the dongle halts.
Making it work would need a larger/more-URB USB transfer path — DEEP work tied to
task #29 (USB pipeline), NOT a quick win. Changes reverted (uncommitted).
=> North America stays TWO presets (131.5 cluster / 129.1 cluster); full NA in
one radio remains the multi-receiver story (task #17).

## Optional side-experiment: 3.2 MSPS wider window (POA-only) [SUPERSEDED by the test above]

- fs is a per-band profile field; bands are one-at-a-time → POA can run 3.2 MSPS
  without touching Iridium/VDL2. `rtlMult = 3.2M/12500 = 256` (integer ✓);
  2.56 MSPS does NOT divide (204.8) so it's out.
- 3.2 MHz window (±1.6) fits NA's full 129.125→131.550 span in ONE radio.
- RISK (user: "might fail badly"): RTL 3.2 MSPS is drop-prone; P4 USB rate rises
  ~28% (4.77→~6.1 MB/s) → possible `rb_full_drops`. Bounded/reversible: per-band
  fs field, short soak watching rb_full_drops/status_errors/est_dropped_bursts,
  revert if they climb. DEFER to the NA/multi-region work; not for Sydney.

## Also on the list
- Verify `af_push` (enabled, →192.168.1.64:5557) does not forward crc=BAD frames
  upstream (feed is ON, contrary to an earlier "off by default" claim).
- Pre-merge: this feeds task #37 (3-band grand smoke + real-antenna validation).
