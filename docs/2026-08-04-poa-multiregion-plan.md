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
| North America (131.5) | 130.500 | 130.025, 130.425, 130.450, 131.125, 131.550 |
| Americas (129.1) | 130.000 | 129.125, 130.025, 130.425, 130.450, 131.125 |
| Custom | manual | free-text po_chans + po_lo |

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

## 3.2 MSPS wider window — TESTED 2026-08-04: FAILS (stream won't sustain)

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
