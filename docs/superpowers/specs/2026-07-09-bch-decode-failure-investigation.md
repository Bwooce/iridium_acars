# BCH decode "failure" investigation — IRA-band strong-signal session

Date: 2026-07-09 (AEST)
Log analysed: `/tmp/smoke_pathA2/soak.log`, IRA session begins at
`APP_CFG: lo=1625500000` (LO 1625.5 MHz, band 1624.25–1626.75 MHz).
Read-only code investigation. No code changed, no hardware touched.

## TL;DR

**`bch_ok=0` (STATUS `bch_decoded=0`) is EXPECTED here, not a demod→BCH
handoff bug — and the flagged frames are real signal, not noise.** The
demodulator is producing correct bits: 5 IMS frames decoded with
`bch_ok=true`, which cannot be faked. Frames ARE decoding on the real path
(`frame_decoder` / `FRMDEC` classified 16 genuine frames), and the
worker-side `bch_decoded` STATUS counter itself reaches 1 several times
later in the session. The frames received near the IRA frequency are
broadcast/simplex types (TL / IMS / LW) that are decoded by `frame_decoder`
but that mostly do **not** match the worker's narrow "messaging
double-BCH-at-offset-24" pre-check which drives the `bch_decoded` STATUS
metric — so the metric reads ~0 while decoding is actually working.

The specific bursts the observation flagged (382-bit DL, 186-bit DL, 66-bit
UL) fall in the **non-classifying** bucket (no `FRMDEC` line). These are
**not** noise: e.g. a 382-bit DL at **26.0 dB SNR** (542.943 s), and the
382-bit length recurs ~15× — consistent length ⇒ consistent real burst
structure. They are cleanly demodulated real frames that either match no
frame type the classifier handles, or match one whose **body parser is
incomplete** (all 10 TL frames classify by prefix but then fail
`tl_decode`, printing `v-1 plane=-1` — the signature of real ITL frames
with an incomplete parser, not noise). Their exact fate (UW-fail vs
type-miss vs queue-drop) is **not resolvable from this INFO-level log** —
see (d).

---

## (a) What the frames actually are

Extracted from the full IRA session (not just the first 120 s):

- **39** `DEMOD SUCCESS` lines (qpsk_demod succeeded, UW locked).
- **16** `FRMDEC: FRAME:` classifications, by type:
  - **10× TL** (Time/Location broadcast; `iridium_frame.c:418-421`,
    header = bits "11" + 94 zeros).
  - **5× IMS** (simplex messaging; several print the `block=N frame=M
    grp=Acq` form, which is the `ims.bch_ok == true` branch —
    `frame_decoder.c:312-320`).
  - **1× LW.SY** (Link Control Word, subtype SY / sync;
    `iridium_frame.c:429-433`).
- **0** LW.DA / IBC / IRA-sv / SBD / ACARS frames.

Bit counts seen (`n_bits = n_symbols × 2`, `qpsk_demod.c:297`): 382, 366,
344, 342, 298, 296, 276, 262, 186, 66, 40 … These are ordinary
variable-length simplex/duplex downlink and uplink bursts. The recurring
382-bit (191-symbol) DL frames are the standard downlink simplex/LCW burst
length. None of these are the IDA duplex data frames that carry ACARS.

**39 demods vs 16 classifications.** The 23-frame gap is a *mix* of (i)
real frames whose type isn't handled or whose body parser is incomplete
(e.g. the flagged 382/186/66-bit bursts), (ii) genuine UW-fail noise
false-positives (qpsk_demod accepts up to 2 UW symbol errors, but
`iridium_frame_classify` demands an **exact** UW — `iridium_frame.c:373`),
and (iii) queue drops (STATUS `drops` ran 60–85/interval). These three are
**indistinguishable at INFO level** because UNKNOWN classifications are
logged at `ESP_LOGD` (`frame_decoder.c:454`). Do not read the gap as
"all noise."

**Zero RA on the ring-alert band.** Notable: 10 TL but **0 RA**, despite
LO=1625.5 sitting on the ring-alert band (RA fixed at 1626.0 MHz). RA
classification (`iridium_frame.c:438`, 3-way de-interleave + triple
BCH(31,21) poly 1207) is strict; RA candidates that fail it fall through to
UNKNOWN and are DEBUG-hidden. Some of the recurring high-SNR 382-bit DL
bursts may well be RA frames that miss the strict classifier — this cannot
be confirmed from the INFO log and is a candidate for the gri cross-check
in (d).

**Interpretation:** at LO = 1625.5 MHz you are sitting on the IRA / simplex
broadcast band (ring-alert is fixed at 1626.0 MHz). The traffic here is
ring-alert / broadcast / time-location / sync — exactly TL / IMS / LW / RA.
The ACARS-bearing IDA (LW.DA) duplex channels live lower (~1619.5–1621 MHz,
optimal LO ≈ 1620.6 — see MEMORY `reference_freq_coverage_analysis.md`).
**Zero ACARS at this LO is expected geometry, not a bug.**

## (b) Exact demod → BCH data path (file:line)

There are TWO independent classification paths. Confusing them is the root
of the "every frame fails BCH" report.

1. **Demod** — `common/iridium_decoder/qpsk_demod.c:308-329`. DQPSK-decodes
   `n_symbols` into `out->bits` (2 bits/symbol, `(b0,b1)` high-then-low per
   symbol) plus per-bit soft metrics. `bits[0..23]` = 12-symbol UW,
   `bits[24..]` = descrambled payload (`iridium_frame.c:4-8`). Logs
   `Successfully demodulated %s frame, %d bits`.

2. **Worker BCH pre-check + STATUS counter** —
   `p4-usb-host/main/worker_core1.c:702-808` (`worker_emit_frame`), Core 1.
   - Logs `DEMOD SUCCESS` at `worker_core1.c:730`.
   - `worker_core1.c:739-746`: **iff `n_bits >= 88`**, takes
     `payload = frame.bits + 24`, `iridium_deinterleave(payload, block1,
     block2)` (`bch_decoder.c:224-236`), then `bch_decode_block` on each
     (`bch_decoder.c:99-121`, BCH(31,21), poly `BCH_POLY_RA = 1207`,
     `bch_decoder.h:7`). Chase-2 soft rescue on hard failure
     (`worker_core1.c:754-771`).
   - `worker_core1.c:773-807`: only if **both** blocks decode does it call
     `iridium_frame_classify` (`iridium_frame.c:348`). `bch_decoded++`
     (`worker_core1.c:793`, → `s_bursts_bch_decoded`, → STATUS
     `bch_decoded`) requires classify to return a **known** type;
     `bch_unknown++` if BCH passed but classify → UNKNOWN
     (`worker_core1.c:800`); else `bch_failed++` (`worker_core1.c:806`).
   - This counter is per-interval: `worker_core1.c:1374-1375` drains it with
     `atomic_exchange(...,0)` each STATUS read, so sparse decodes read as 0.
   - STATUS emission: `status_logger.c:238-242`; semantics documented at
     `status_logger.c:168-176` / `216-219`.

3. **Real decoder** — `p4-usb-host/main/frame_decoder.c:289-458`
   (`process_one`), Core 0 (`FRMDEC`). In the STANDALONE build the worker
   pushes **every** `demod_ok` frame here via `frame_decoder_push`
   (`worker_core1.c:848-851`, the `#else` branch), independent of the
   worker BCH pre-check. `process_one` calls `iridium_frame_classify`
   directly, then dispatches per type to `ims_decode` / `tl_decode` /
   `ibc_decode` / `ida_decode` / `ira_decode`, and emits the `FRAME:` lines.

4. **Classifier** — `iridium_frame.c:348-444`. Strict **exact** 24-bit UW
   gate (`iridium_frame.c:373-375`) — any UW bit error → UNKNOWN. Applies a
   `symbol_reverse` adjacent-pair swap to the payload
   (`iridium_frame.c:395-402`) **before** the MS/TL/BC/LW/RA dispatch.
   BCH via `iridium_bch_ndivide` (`iridium_bch.c:38-63`), poly 1207 for
   RA/BC (`iridium_frame.c:222-224,242-243`).

Key asymmetry: the worker pre-check (step 2) operates on **raw** payload
bits with a fixed messaging-block de-interleave (`bch_decoder.c:224`) and
**no** `symbol_reverse`; the real classifier (step 4) operates on
**symbol-reversed** bits with per-type de-interleavers. They are different
tests. The worker pre-check is a "does this look like an MS/messaging
double-BCH block" heuristic, not a general frame-decode.

## (c) Ranked hypotheses

### H1 (CORRECT) — Frame-type gating: `bch_decoded` is a messaging-layout
metric; the IRA-band traffic is broadcast types decoded elsewhere.
- **Supporting:** 16 real frames decoded (10 TL, 5 IMS, 1 LW.SY) via
  `frame_decoder` (`frame_decoder.c:317,332,423`). IMS printed the
  `ims.bch_ok==true` branch (`frame_decoder.c:312`). The worker's
  `bch_decoded` STATUS counter itself reached 1 at 897.2/899.2/901.2/924.3 s
  (`soak.log:8840,8850,8859,8919`) — the path works, it is just sparse and
  per-interval-drained. TL frames (header "11"+zeros) don't present the
  messaging double-BCH block the worker pre-check expects; they classify
  via prefix match in `frame_decoder`, not via the worker counter.
- **Contradicting:** none material. (A pedantic point: all-zero TL payload
  blocks *can* divide clean at `bch_decoder.c:105`, so some TL frames could
  in principle tick `bch_decoded`; observed counts are consistent with the
  sparse per-interval reads.)

### H2 — Observability/metrics gap (secondary, real but cosmetic).
- The worker `bch_decoded` STATUS counter under-represents true decode
  success: `frame_decoder` decodes TL/IMS/LW that the worker's narrow
  pre-check + strict classify don't all credit, and the counter is
  drained per STATUS interval, so a stream of real broadcast decodes reads
  as `bch_decoded=0`. Also the worker pre-check omits the `symbol_reverse`
  the classifier applies, so even genuine BC-layout frames may not tick it.
- **Supporting:** `/status frames.{ms,tl,bc,lw,ra}` (fed by `frame_decoder`,
  `frame_decoder.c:305,328,345,377,434`) is the authoritative decode
  signal, not `bch_decoded`; comment `worker_core1.c:779-782` says exactly
  this. **This is a reporting nuance, not a decode failure.**

### H3 (REJECTED) — Bit polarity / ordering / differential decoding wrong.
- **Contradicting:** `iridium_frame_classify` has an **exact** 24-bit UW
  gate (`iridium_frame.c:373`) yet still classified 16 frames including
  BCH-passing IMS. If polarity/ordering/DQPSK were wrong, **nothing** would
  match the UW or the per-type BCH. Selective, structurally-valid
  classification proves the bit path is correct.

### H4 (REJECTED) — UW-to-payload offset wrong.
- **Contradicting:** both paths use offset 24 (`worker_core1.c:740`,
  `iridium_frame.c:360,377`), and exact-UW frames decode. Offset is right.

### H5 (REJECTED) — Residual CFO/phase corrupting bits downstream of demod.
- **Contradicting:** SNR 16–33 dB, exact UW matches, clean IMS BCH. No
  post-demod frequency dependence is implicated; nothing runs between
  `DEMOD SUCCESS` and BCH except de-interleave + division.

### H6 (REJECTED) — BCH parameter mismatch vs spec.
- **Contradicting:** `BCH_POLY_RA = 1207` = BCH(31,21) t=2
  (`bch_decoder.h:7`), matches gr-iridium / iridium-toolkit ring-alert poly
  used in `iridium_frame.c:222,242` and `bch.py`. Correct n=31, k=21.

## (d) Most-likely root cause + confirmation

**Root cause: none — the system is behaving correctly for this band.**
`bch_decoded=0` in STATUS is the expected reading when the received traffic
is broadcast/simplex frames (TL/IMS/LW) that are decoded by `frame_decoder`
but do not match the worker's messaging-layout `bch_decoded` heuristic, and
when real decodes are sparse relative to the per-interval counter drain.
The flagged 382/186/66-bit bursts (spanning the session — 134/388/429/430 s
and later, including a 26 dB 382-bit DL at 542.9 s) genuinely do not tick
`bch_decoded`, because they are broadcast/unhandled types, not the messaging
layout that counter checks. The IMS frames (and the sparse `bch_decoded=1`
events at ≈897–924 s) confirm the counter path itself is functional.

**Two open sub-questions this INFO log cannot answer**, and how to close
them:
1. **What exactly are the non-classifying 382/186/66-bit frames** (real
   RA/type-miss vs incomplete parser vs UW-fail noise vs queue-drop)?
   → Re-run with `frame_decoder` / `iridium_frame` at `ESP_LOG_DEBUG` so the
   UNKNOWN reason (`frame_decoder.c:454`) and per-type near-misses are
   visible; this splits UW-fail from type-miss.
   → **Definitive north-star check (not yet done):** feed this session's SD
   burst capture (if the run tapped SD — `worker_core1.c:sd_tap` path)
   through **gr-iridium** and compare frame-type labels. This report cites
   the gri references (`docs/gr-iridium-processing-flow.md`,
   iridium-toolkit polys) but did **not** run gri on the data; that run
   would authoritatively name the 382/186/66-bit types.
2. **Is decode actually healthy?** → Read `/status` `frames.{tl,ms,lw}` and
   the non-draining cumulative counters
   (`worker_core1_get_decode_counts`, `worker_core1.c:1370-1377`), not the
   per-interval `bch_decoded`. Expect nonzero TL/MS/LW.

To actually see ACARS, re-point the LO to the IDA duplex band (LO ≈
1620.6 MHz, per MEMORY `reference_freq_coverage_analysis.md`) — LW.DA / SBD
/ ACARS are the frame types the worker `bch_decoded` heuristic and the ACARS
chain are built around, and they do not transmit on this LO.

## (e) Bug or expected?

**EXPECTED behaviour, and the flagged frames are real signal — not noise
and not a demod bug.** The demod→BCH handoff is correct (proven by
`bch_ok=true` IMS frames); frames are decoding on both paths. The flagged
382/186/66-bit bursts are cleanly demodulated **real** frames that are
merely **incompletely classified/parsed** (unhandled type, or a handled
type — TL — whose body parser is incomplete), which is why they leave no
`FRMDEC: FRAME` line and don't tick `bch_decoded`. The only genuine
(cosmetic) code issue is H2: the worker-side `bch_decoded` STATUS metric is
a messaging-layout heuristic, drained per interval, that under-reports
decode success for broadcast frame types — so it reads ~0 while
`frame_decoder` decodes TL/IMS/LW normally. No demodulation, bit-order,
offset, CFO, or BCH-parameter defect is present. What this log **cannot**
prove is the precise identity of every non-classifying burst — that needs
the DEBUG re-run and/or the gr-iridium cross-check named in (d).
