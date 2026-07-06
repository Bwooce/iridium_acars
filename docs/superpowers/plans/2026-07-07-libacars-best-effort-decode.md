# Design — libacars best-effort/partial decode (CPDLC uPER + ADS-C) + chain salvage

Design-complete 2026-07-07 (agents, verified citations). Implementation split:
**Phase H (host-side, unblocked)** = §§1–8. **Phase F (firmware)** = §9 + device
enablement — sequenced AFTER P1.5 Design A (same firmware surface).

Motivation: real captures arrive with broken SBD reassembly (37–49% broken
fragments measured); libacars discards everything on any decode failure. The
real ZK-NNC FANS-1/A message (2026-07-06, `~/iridium_bits/acars-milestone2-*.txt`)
renders only "Unparseable FANS-1/A message".

## Core safety contract (applies everywhere)

uPER fails SILENTLY — post-corruption fields decode as plausible garbage. All
salvaged output must be (a) hard-tagged PARTIAL, (b) carry a confidence
boundary, (c) display-only — never fed to automation. Config gate:
**one generic flag `best_effort_decode`, default OFF** (supersedes the earlier
`asn1_best_effort_decode` name; per-protocol trust semantics live in banner
text, not the option namespace).

## 1. Scope survey (verified)

- The ONLY asn1c path: `la_asn1_decode_as` (libacars/libacars/asn1-util.c:20-37),
  sole caller `cpdlc.c:46`, `uper_decode_complete` (asn1/per_decoder.c:10-37).
  PDUs: `asn_DEF_FANSATCUplinkMessage` / `FANSATCDownlinkMessage` (cpdlc.c:33-37);
  the 490-file `asn1/` tree is all one FANS module. No BER/OER anywhere.
- Reachability: `la_arinc_parse` (arinc.c:166) → IMI map (arinc.c:42-48:
  .AT1/.CR1/.CC1/.DR1 → CPDLC; .ADS/.DIS → ADS-C at arinc.c:202). ARINC CRC is
  computed (arinc.c:189-190) but does NOT gate parsing — `crc_ok` is orthogonal
  to `partial`.
- FIRMWARE: `common/libacars_idf/port/libacars_app_stubs.c` stubs
  `la_arinc_parse` — the whole ARINC 622 layer is host-only today. Device
  enablement is Phase F (measured cost: whole libacars = 584 KB text +
  109 KB data on x86-64; tables are const/XIP → ~zero standing RAM).

## 2. Trustworthy prefix (CPDLC)

Both PDUs: `SEQUENCE { header, elementId CHOICE, elementIdSequence OPTIONAL }`.
Header = 2-bit optionals bitmap + fixed-width msgID [+refNum][+timestamp].
Element CHOICE index: **8-bit, non-extensible** (129 downlink / 183 uplink
alternatives) — fixed-width, so the message TYPE name is essentially always
salvageable. Desync starts at the first variable-length primitive inside the
chosen element (PER length-prefixed strings, SEQUENCE OF, nested CHOICEs).

## 3. asn1c failure semantics (memory-safety foundation)

- `uper_decode_complete` does NOT free the struct on failure — caller discards.
- **asn1c bug (fix it)**: `rval.consumed` is hard-zeroed on RC_FAIL
  (per_decoder.c:87-90) violating asn_codecs.h:74-90's own contract; the exact
  bit offset survives in the threaded `asn_per_data_t pd.moved`. Fix ≈ 2 lines:
  `rval.consumed = pd.moved` (bits) on failure. Independently upstreamable.
- Struct state on RC_FAIL: SEQUENCE = calloc'd + fail-fast (constr_SEQUENCE.c:
  1046-1049, 1122-1129) — prior fields valid, later fields zero/NULL.
  **CHOICE race**: `present` committed at constr_CHOICE.c:883 BEFORE the arm
  decodes; failure at :906 doesn't roll back → valid-looking `present` with
  NULL/partial arm (semantic trap only). SET OF appends only complete elements.
- `ASN_STRUCT_FREE` is safe on partial trees AS-IS (null-checked frees:
  constr_SEQUENCE.c:952-976, constr_CHOICE.c:1035-1069). No guards needed.

## 4. API (CPDLC)

- `config_defaults.h`: `LA_CONFIG_SETTING_BOOLEAN("best_effort_decode", false)`.
- `la_asn1_decode_as`: add nullable out-param `asn_dec_rval_t *rval_out`
  (single caller). NOTE the distinct RC_OK-with-trailing-bytes case
  (asn1-util.c:29-36): complete decode + trailing junk — treat as
  partial-success, but with different banner wording than a desync.
- Partial state lives on `la_cpdlc_msg` (cpdlc.h:20-27) replacing two reserved
  slots: `bool partial; size_t consumed_bits;` (size unchanged).
- Flow in `la_cpdlc_parse`: OFF → exactly current behaviour. ON + consumed>0 →
  keep tree, `partial=true`, `err=false`. consumed==0 → err as today.
- Render: text banner before `la_asn1_output_cpdlc_as_text` —
  `-- WARNING: PARTIAL/UNTRUSTED decode (desync after bit %zu of %zu) --
  display only`; JSON adds `"partial"` + `"consumed_bits"` next to `"err"`.
  MVP renders the full partial tree under the banner (guards make it safe;
  post-boundary values are plausible-wrong by contract).

## 5. Formatter audit (CPDLC)

All traversal funnels through six shared walkers in asn1-format-common.c, each
already null-guarding ATF_POINTER members (:147,:171,:193,:208,:87-90,:118-120).
Audit items: (1) bounds-check `present` vs `elements_count` in both CHOICE
walkers; (2) bespoke top-level text formatters
(asn1-format-cpdlc-text.c:521-561) — seqOf guards exist at :535/:556; (3)
verify every leaf scalar formatter in both dispatch tables is reachable only
via a guarded parent (mechanical). The fuzz (§6b) enforces the audit.

## 6. Validation (both protocols)

a. **Positive control**: intact fixtures decode BYTE-IDENTICAL with mode ON vs
   OFF. CPDLC: 3 intact AT1 payloads in libacars/examples/cpdlc_get_position.c:
   33,37-39 (SOUCAYA/MSTEC7X/MELCAYA). ADS-C: 4 payloads in
   examples/adsc_get_position.c:26,30-32 + decode_acars_apps.c:27,34.
b. **Bit-flip fuzz**: flip EVERY bit of every fixture one at a time
   (~2,250 iterations total, sub-second in-process), decode with mode ON under
   **ASan+UBSan**. Assert: zero sanitizer hits; every RC_FAIL yields PARTIAL
   output or clean unparseable; RC_OK flips render identical ON vs OFF.
   NOTE: existing HOST_TESTS_UBSAN (tests/host/CMakeLists.txt:7-23) only covers
   signed-overflow/shift/bounds — ADD `HOST_TESTS_ASAN`
   (-fsanitize=address,undefined -fno-sanitize-recover) (~10 lines).
   ADS-C extra assertion: value-byte flips in fixed-length groups must NOT
   desync framing (subsequent group tags still decode).
c. **Acceptance demo**: the real ZK-NNC message — expected: header msgID +
   element TYPE name under the PARTIAL banner, nonzero consumed_bits.
   (Baseline verified: current code → "Unparseable".)
d. **Build wiring**: no current host target compiles arinc.c/cpdlc.c/asn1/ —
   add a host-only CMake target with arinc.c + cpdlc.c + asn1-util.c +
   asn1-format-cpdlc-{text,json}.c + asn1/ (MIAM/OHMA stay stubbed; no
   zlib/jansson/libxml2). libacars/build/ proves the source set links.

## 7. Vendoring / upstream

`libacars/` is a plain tracked copy of 2.2.1 (no submodule). Patch in place;
record delta-from-upstream in patches/README.md (repo convention; the
per_decoder.c edit is a deliberate documented exception to the
no-editing-vendored-.c policy — it is also the upstream bug fix).
Upstream: PR 1 = consumed-on-fail contract fix (tiny, standalone); PR 2 = the
feature (flag, reserved-slot fields, banners, tests). State AI assistance.

## 8. ADS-C sibling mode (mostly labelling — salvage already exists)

- Parser: adsc.c tag-group loop (la_adsc_parse, adsc.c:1696-1707); groups
  appended BEFORE parsing (:1700-1701); tag tables at :201-415 (downlink,
  mostly fixed lengths: basic report tag 7 = 10 B leads periodic reports) and
  :1128-1199 (uplink; contract requests variable via sub-tags :1201-1338).
- TODAY on failure: unknown tag / short group → err=true + break
  (:1654-1660, :1703-1704) but **prior groups are kept AND rendered**
  ("Unparseable tag %u" :1740-1742 + "Malformed ADS-C message" :1785-1787;
  JSON skips the failed tag :1755-1757, has "err" :1805).
- Work: `partial` flag + `err_offset` in la_adsc_msg reserved slots
  (adsc.h:45-53); group-quantized-trust banner — "first N group(s)
  trustworthy; failed at tag 0xNN, byte offset M" (framing is corruption-proof
  except at tag bytes and the 3 variable-length groups: NACK reason byte
  :452-453, noncompliance counts :485-540, uplink sub-tag lists); config gate;
  JSON "partial"/"failed_tag"/"err_offset".
- **Pre-existing LEAK to fix**: noncomp (:512→:525-527) and contract-request
  (:1621→:1637-1639) assign t->data before failing while t->type is only set
  on success (:1671) — la_adsc_tag_destroy (:1344-1359) then never frees data.
  Fix: set t->type before parsing (also enables retaining failed-group
  partials). la_list_free_full path is otherwise safe.
- No ADS-C in our captures yet — check via
  `grep -aE '\.ADS\.|\.DIS\.' ~/iridium_bits/acars-milestone*.txt` (never grep
  the whole dir — 56 GB raw lives there).
- ~50 LOC production + ~150 tests. Sonnet-tier; top-tier review on the
  destructor keying.

## 9. Our-chain salvage (Phase F — after Design A; firmware surface)

- SBD: `sbd_reassembler_tick()` discards on 5 s timeout
  (sbd_reassembler.c:101-112; also orphan :272-274, table-full :248-250);
  counters exist (sbd_reassembler.h:97-102) but are not exposed. Missing FINAL
  fragment destroys ACARS framing (DEL/CRC/ETX) → salvage CANNOT ride
  la_acars (acars.c:285-314,482 hard-fails) → needs RAW hex/ASCII fallback
  emission. ~70-100 LOC.
- IDA: `expire_stale()` abandons buffered payload (ida_reassembler.c:43-53;
  orphan :102-105; overflow drops whole chain :109-114); SBD 0x10 sub-header
  length check (sbd_reassembler.c:194-196) rejects truncated payloads — needs
  partial-aware relaxation (~5 LOC). ~35-50 LOC.
- RISK: session table is noise-fed (BCH false positives) — salvage must NEVER
  blend into the clean stream. `partial` flag propagates through: msg_ring.h
  acars_msg_t (:18-31), frame_decoder.c populate (:241-258) + FRMDEC log
  (:231-238), acars_push.c JSON (:154-181), sd_log.c NDJSON (:155-192),
  http_server.c /messages (:846-873) + /debug/inject (:1073), /status
  reassembler counters, host sbd.py (:346-349). ~15-20 LOC.
- Total Phase F ≈ 110-150 LOC firmware + trailers + device smoke.

## Device enablement (Phase F tail)

Un-stub la_arinc_parse/cpdlc/adsc in common/libacars_idf (MIAM stays stubbed —
zlib); add sources incl. asn1/ tree. Gates: binary-size report, device smoke,
/messages + UDP showing an interpreted CPDLC/ADS-C decode.

## Tiering

Sonnet-tier: fuzz harness, CMake, mechanical audit, ADS-C labelling, fixtures.
Top-tier review (trust-bearing): per_decoder.c fix + asn1-util/cpdlc semantics
(esp. not conflating trailing-bytes with desync), ADS-C destructor keying,
raw-fallback emission policy (§9).
