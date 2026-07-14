# RESOLVED: Iridium IDA header bit 4 (`flag1b`) — an L2 fast-path flag on 0x7605 Ring Alerts

Self-contained brief for the Iridium **IDA (LW.DA / data) frame header** bit 4. Original
question below; answered 2026-07-15 by an external agent + **corroborated on the corpus**.

## RESOLUTION (2026-07-15)
- **`0x7605` = an SBD Ring Alert / paging message.** Iridium L3 is GSM-04.08/04.06-derived
  (LAPDm). First byte `0x76` = GSM Protocol Discriminator `0x06` (Radio Resource Management)
  with skip-indicator `0x07`; `0x05` = the Iridium paging/ring-alert message type. When an
  MT-SBD is queued at the gateway, the network blasts this short ring alert on the L-band
  downlink to wake a specific modem, which then does a mailbox check → the actual data comes
  down as the multi-frame `0x7608` messages. (Ref: CCC / Harald Welte OsmoDevCall 2022 —
  "LAPDm messages, type 0x06 and 0x76".) No official spec exists; lineage is GSM.
- **Structure, CORROBORATED on 755 corpus frames** (`76 05 00 4b | TMSI(4B) | 50 | trailer(2B)`):
  bytes 0–1 `76 05` const; bytes 2–3 `00 4b` near-const (routing/beam); bytes 4–7 a 4-byte
  **TMSI** (temporary subscriber id) — **418 distinct values / 755 frames (~55% unique)**,
  exactly identifier-like (mostly different modems, some re-paged); byte 8 `50` const marker;
  bytes 9–10 high-variance trailer (channel assignment / backoff for the mailbox-check reply).
- **`flag1b` (bit 4) = an L2 fast-path / control flag** — "immediate signaling, do NOT route
  to the SBD reassembler." Explains why it's set almost only on single-burst `0x7605` and
  never on multi-fragment `0x7608` data.
- **Ignoring it loses no content** — these frames are single-burst (`cont=0`) and already pass
  our pipeline correctly; bit 4 is only a metadata label. Recommendation (adopted): keep
  `da_flag1b` captured but passive; usable later as a filter for a Ring-Alert/TMSI tracker.

---

## Original brief (for reference)
Goal: identify the meaning of an undecoded flag bit in the Iridium IDA frame header.

## Background / layer
Iridium L-band downlink, 1616–1626.5 MHz. "IDA" (a.k.a. LW.DA) frames carry Short Burst
Data (SBD), which in turn can carry ACARS. The IDA framing is the **proprietary air
interface** — there is **no public spec**; everything is reverse-engineered by gr-iridium
+ iridium-toolkit. (The public *Iridium SBD Developer's Guide v3.0* and gadomski/sbd-rs
document the **DirectIP gateway layer** — IEIs, MO/MT — which is ABOVE this and does not
describe these bits. arXiv:2603.12062 is a security paper that just *uses* iridium-toolkit.)

## The IDA header bit layout (post-BCH bitstream `bitstream_bch`, per iridium-toolkit `bitsparser.py:IridiumDAMessage`)
```
bit   0..2   flags1 (low 3 bits)   — uninterpreted
bit   3      cont                  — continuation flag (1 = more fragments follow). GOVERNS reassembly.
bit   4      flag1b                — *** THE UNKNOWN BIT ***  (iridium-toolkit slices+names it, uses it nowhere)
bits  5..7   da_ctr                — 3-bit fragment sequence counter (mod 8)
bits  8..10  flags2                — uninterpreted
bits 11..15  da_len                — 5-bit payload byte count (0..24; a burst carries ≤24 B)
bit  16      flags3                — uninterpreted
bits 17..19  zero1                 — must be 0 (structural sanity check)
bits 20..    payload (da_len bytes) + CRC16 (CCITT)
```
Multi-fragment messages chain via (cont, da_ctr): opener = (ctr=0, cont=1); continuations
ctr=1,2,…; final = cont=0. All three decoders (iridium-toolkit, iridium-sniffer, our
`common/iridium_decoder/ida_decode.c`) agree on cont=bit3 / ctr=bits5-7 / len=bits11-15
and **discard bit 4**.

## Empirical behaviour of bit 4 (45-min HydraSDR wideband corpus, 7427 IDA frames, good SNR)
- **Set on 947 / 7427 frames (~12.7%)** — NOT reserved/constant (our code wrongly called
  it an "always ~0 spacer").
- **Concentrated on single-burst frames:** of frames with da_ctr=0, 938 have flag1b=1; on
  continuation frames (ctr>0) it's almost always 0. So it rides on **standalone / opener**
  frames, not mid-chain.
- Split by cont: cont=0 → 938 set; cont=1 → only 9 set.
- **Strong correlation with a specific message type.** Restricting to standalone frames
  (cont=0, ctr=0): 934 have flag1b=1, and **764 of those 934 have first two payload bytes
  `0x7605` with da_len=11** (a consistent 11-byte structure). The flag1b=0 standalones are
  a mix of other types (`063a`, `7608`, `8554`, `84d8`, …) with da_len mostly 0 or 2.
  - Sample flag1b=1 payloads (11 B each): `7605004b4a27eccb50ba0d`, `7605004b7927dc8a509ff2`,
    `7605004be62d4c4750d1ad` — pattern `7605 00 4b XX XX XX XX 50 XX XX`.
- For context, `0x7608` is the SBD type that carries ACARS (SBD_TYPE_DATA_DL_7608 in our
  `sbd_reassembler.c`); `0x7605` is a *different* type we don't currently interpret.

## Working hypotheses (unconfirmed)
1. **Type/format sub-flag** — bit 4 co-selects or annotates the `0x7605` message class
   (most likely, given the tight correlation).
2. A priority / ack / "position report present" flag.
3. Unrelated to type; correlation is incidental to whatever populates `0x7605`.

## Questions for the other agent
1. Is there ANY air-interface documentation (Iridium patents, leaked L-band/ISU docs, other
   RE writeups, the GSM-04.64-derived data-link lineage) that names this header bit or the
   `0x7605` message type?
2. What is SBD type `0x7605` (11-byte `7605 00 4b … 50 …`)? Decode its structure.
3. Does bit 4 change how the payload should be parsed (i.e., are we mis/under-parsing the
   ~934 frames that set it)?
4. Confirm/refute: does ignoring bit 4 lose any *message content* (vs just a type label)?

## Our current handling
`ida_decode.c` now **captures** bit 4 as `da_flag1b` (commit 99c4366) but does not use it;
reassembly keys only on `da_cont` (bit 3). No message loss identified — the flag1b=1 frames
are standalone and ARE processed; we just don't interpret the flag or the `0x7605` type.
