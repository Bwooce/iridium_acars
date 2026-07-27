# Inmarsat Aero (SATCOM ACARS) band — scoped proposal + volume estimate

Status: PROPOSAL (2026-07-24). Not built. Purpose: decide whether an Inmarsat
Classic Aero receiver is worth adding as a third+ band, and estimate the ACARS
volume it would yield vs the existing VDL2 (VHF) and Iridium paths.

## Why (context from this session)
ACARS media selection is preference-ordered: VHF first (VDL2/POA) where ground
coverage exists, SATCOM only as the oceanic/remote fallback. Confirmed
empirically: near Sydney the VDL2 (VHF) capture is rich (local approach traffic)
while the Iridium (satellite) capture was nearly empty — the satellite links
carry the *oceanic/enroute* aircraft out of VHF range, not the approach traffic.
So SATCOM is the way to harvest the transoceanic traffic VHF can't see.

Among SATCOM: **Inmarsat Classic Aero** is the dominant legacy aviation ACARS
medium (large wide-body/long-haul installed base; the oceanic CPDLC/ADS-C/ACARS
standard). Iridium (which this project already decodes) has a *smaller* aviation-
ACARS share (polar, smaller aircraft, newer Certus). So Inmarsat should out-yield
Iridium substantially — hence this proposal.

## RF / signal
- **Satellites over AU**: Inmarsat I-3/I-4/I-6 GEO birds — Pacific Ocean Region
  (POR, ~178°E; high in Sydney's eastern sky) and Indian Ocean Region (IOR,
  ~64°E; lower to the west). POR is the natural first target from Sydney (high
  elevation, heavy trans-Pacific traffic).
- **Band**: L-band, satellite→ground/aircraft downlink ~**1545–1555 MHz** (the
  receivable side; the aircraft→satellite uplink at ~1645 MHz is a directional
  beam up, not groundable). This is BELOW the Iridium band (1616–1626) — the
  HC610 active antenna is spec'd for Iridium; may or may not cover 1545 (check),
  likely needs a dedicated L-band patch + LNA.
- **Antenna**: unlike Iridium (LEO, omni) this is **GEO — fixed directional**: a
  small L-band patch aimed once at the chosen satellite + an LNA/bias-tee. Simple
  and cheap, but a distinct antenna from the VHF (VDL2) and Iridium setups.
- **Modulation**: Aero classes — Aero-L (~600 bps), Aero-I, Aero-H/H+ — BPSK/
  OQPSK burst + TDM channels (P/R/T/C). Symbol rates 600 / 1200 / 10500 sps.
  Reference open-source decoder: **JAERO** (SDRAngel also has an Aero demod) —
  our implementation would follow JAERO's demod + AES/ISU deframing, then feed
  the same **libacars** backend we already use.

## Integration (fits the band-mode framework we just built)
Adds cleanly as a new band:
- `band_profile` entry (BAND_INMARSAT: LO ~1545 MHz, its own detect fs / tagger
  params), per-band gain/bias (phase-2 per-band config already supports it).
- New `band_pipeline_t` vtable: an Aero BPSK/OQPSK `process_burst` (the real work)
  + an AES/ISU L2 in `common/`, emitting `band_frame_t` → libacars.
- `band_select.c` maps BAND_INMARSAT → the new vtable; everything downstream
  (unified stats, /messages funnel, reception classifier) already band-generic.
- Mutually exclusive with VHF/Iridium (different antenna + LO) — soft-switch +
  physical antenna swap, exactly the Iridium↔VDL2 model.

## Effort
Comparable to the VDL2 build: a from-scratch coherent burst demod (BPSK/OQPSK,
the hard part), the AES deframer, channel/logical-channel handling, host tests +
device smoke. Reuses libacars, the band-mode framework, the tagger/USB/ingest
front half. Estimate: a multi-day DSP effort (the demod dominates), NOT a tack-on.

## VOLUME ESTIMATE (the point of this doc)
Rough, order-of-magnitude (site/antenna/satellite dependent):
- A well-sited **JAERO receiver on a busy ocean-region GEO bird** typically logs
  **hundreds to low-thousands of Aero messages/hour** — it sees ALL SATCOM-
  equipped aircraft in that satellite's (continental-scale) footprint, i.e. the
  entire trans-Pacific/Indian-Ocean oceanic fleet, not just one airport's locale.
- Compare: our VDL2 (local, antenna-limited) ~**5 ACARS/h**; our Iridium path,
  historically sparse near-airport.
- So Inmarsat Aero is plausibly **1–2+ orders of magnitude more ACARS** than
  either current path — likely the single highest-volume ACARS source available
  to this project — because its footprint is an ocean, not a 300 km VHF radius.
- BIG caveat: it's **oceanic/enroute** traffic (positions over water), COMPLEMENTARY
  to VDL2's local approach traffic, not overlapping. And the number depends
  heavily on the patch antenna + LNA and which satellite/channels are targeted.

To firm the estimate before building: point a cheap L-band patch at POR, capture
the ~1545 MHz C-channel, and either run JAERO on the capture or just measure
burst density — a few hours' capture gives a real messages/hour figure. That
measure-first step is far cheaper than building the demod on an assumption.

## Recommendation
Highest-volume ACARS opportunity, and architecturally ready to slot in — but it's
a new antenna (GEO patch) + a new coherent demod (multi-day). Sequence: (1) VHF
antenna first (biggest bang for the *existing* VDL2/POA), (2) measure-first
Inmarsat capture (patch at POR + JAERO on the capture) to get a real volume
number, (3) build the Inmarsat band if the captured rate justifies the demod
effort. Don't build the demod before the capture confirms the volume.
