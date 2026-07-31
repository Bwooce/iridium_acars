# airframes.io feeder plan (Iridium + VDL2) — 2026-07-27

Design only — no implementation in this commit. Successor to the two-path plan
in memory note `project_airframes_feed` (#11 Mac iridium-toolkit feeder, #12 P4
firmware feed "gated on decode volume"). This doc concretizes #12 for BOTH
bands, now that VDL2 is first-class (band-mode overhaul, `feat/vhf-vdl2`).

## 0. Orientation — what already exists

- **Emit site (both bands):** `frame_decoder.c::acars_deliver()` (~L416-483) is
  the ONE trusted delivery point. Iridium SBD (`try_acars()`, ~L489) and VDL2
  AVLC (`vdl2_avlc_cb()`, ~L767) both funnel into it; on
  `LA_REASM_COMPLETE/SKIPPED` it fills an `acars_msg_t` and calls
  `msg_ring_push() + acars_push_emit() + sd_log_emit()` (~L470-472). PARTIAL /
  best-effort rows (`salvage_emit()`, ~L254) go to `msg_ring_push()` ONLY —
  they can structurally never reach a feeder hooked at `acars_push_emit()`.
- **Transport plumbing:** `acars_push.c` — 16-deep PSRAM queue, prio-3 task,
  per-message config snapshot, cached DNS (30 s TTL, `resolve_target()`
  L62-90), socket-recreate-on-failure (`s_sock_bad`, L58-60), non-blocking
  drop-on-full emit (L297-303). Unicast UDP — which is mandatory here anyway:
  esp_hosted drops outbound multicast (memory `project_esp_hosted_multicast_tx_broken`).
- **Config idiom:** `app_config.c` — NVS namespace `"iridium"`, global keys via
  `nvs_get_str_or()`/`set_str_field()` (L96/L651), per-band tunables in
  band-namespaced keys (phase-2 band overhaul). The feeder is **global**
  (deliberate): the device runs exactly ONE band per boot
  (`band_runtime_resolve()` once in `frame_decoder_init()` L940-949 /
  worker init), so per-band feeder targets would just duplicate config.
- **HTTP:** `/config` POST (`http_server.c` ~L1440) parses form fields into
  `nvs_save_args_t` (L89-98) and hands them to `nvs_save_and_reboot_task()`
  (L100) — NVS writes MUST run on an internal-SRAM stack (PSRAM-stack httpd
  task cannot `nvs_commit`; memory `feedback_psram_stack_no_flash_write`).
  `/status` JSON has the `udp_push:{host,port,enabled}` block (~L297) to clone.

## 1. What airframes.io ingests — RESEARCH CONFIRMED 2026-07-27

Confirmed from docs.airframes.io/docs/feeding/how/, the airframesio/iridium-toolkit
source (`iridiumtk/reassembler/sbd.py::ReassembleIDASBDACARS`), the
airframesio Iridium feeder script (`iridium-acars-to-airframes.py`), and a
locally-generated dumpvdl2 2.6.0 JSON sample (`~/iridium_capture/vdl2_ref`).

- **Transport:** unicast, **newline-delimited JSON** (one `{...}\n` per
  message). **No API key.** The feeder is identified by a **station ident**
  string embedded in each message (an ident like `KE-KMHR-IRIDIUM1` OR a UUID
  — UUID lets you rename on the website without relinking). Per-message lat/lon
  NOT required (coords come from the web station profile).
- **Ingest endpoints** (`feed.airframes.io`):

  | mode | proto | port |
  |---|---|---|
  | VHF ACARS (acarsdec) | UDP | 5550 |
  | VDL2 (dumpvdl2) | UDP | 5552 |
  | VDL2 (dumpvdl2) | TCP | 5553 |
  | HFDL (dumphfdl) | UDP | 5556 |
  | **Iridium ACARS (iridium-toolkit)** | **UDP/TCP** | **5590** |

  So this device feeds **two ports**: VDL2 → 5552, Iridium → 5590. (The
  Iridium port 5590 was the previously-unknown value — now confirmed.)
- **UNRESOLVED / PARKED (2026-07-31).** Exhaustive live testing never got
  airframes to attribute a single VDL2 message on *any* port or format:
  dumpvdl2 JSON → UDP **5552** (its documented port, incl. a live sustained
  unmodified-dumpvdl2 feed), → UDP **5555** (which is actually the *vdlm2dec*
  TLeconte-format port, not dumpvdl2), and → TCP **5553** — all showed **0**
  attributed, while the station read "alive." This is airframes-side: the
  legacy decoder-direct ports (5550–5556/5590) are fire-and-forget and carry
  **no feed-id**. The likely real path is the modern **`af.json.v1` JSON →
  UDP `feed.acars.io:6000`** carrying an account **feed-id UUID** (via the
  `airframes-client`/`xng` proxy) — UNTESTED (need the feed-id). So treat the
  port table above as airframes' *documented* values, but UNVERIFIED for
  actual attribution from this site. Full trail: memory
  `project_airframes_feed_port_investigation`.
- **We emit each decoder's native schema on its own port** (§2). No
  masquerading needed — `app.name` is `dumpvdl2`/`iridium-toolkit`'s own value;
  a custom name would risk the router not recognising the shape, so we mirror
  the native app names.

## 2. Message schema per band — CONFIRMED (verbatim from the two sources)

Two distinct native schemas, one formatter per band (band is fixed per boot,
so it's a boot-time branch like `s_band_vdl2`, `frame_decoder.c` L123). Both
are one-line JSON + `\n`. NOTE both carry a **wall-clock** timestamp
(dumpvdl2 `t.sec/usec` epoch; Iridium `acars.timestamp` ISO8601) → SNTP is a
hard prerequisite (Phase 2), not optional polish.

**VDL2 → 5552 (dumpvdl2-native), only the ACARS-bearing subset we produce:**
```json
{"vdl2":{
  "app":{"name":"dumpvdl2","ver":"2.6.0"},
  "t":{"sec":1785148910,"usec":428657},
  "freq":136975000,"sig_level":-11.02,"noise_level":-10.69,
  "station":"<af_id>",
  "avlc":{
    "src":{"addr":"390826","type":"Aircraft","status":"Airborne"},
    "dst":{"addr":"26B117","type":"Ground station"},
    "cr":"Command","frame_type":"I",
    "acars":{"err":false,"crc_ok":true,"more":false,"reg":".F-GCBG",
             "mode":"2","label":"2T","blk_id":"7","ack":"!","flight":"AF0000",
             "msg_num":"M06","msg_num_seq":"A","msg_text":"..."}}}}
```
(`station` is dumpvdl2's `--station-id` field. `addr` = 6-hex AVLC address;
`type`/`status` are enum strings. `more` = ACARS "more to come" bit; `ack` is
`"!"` for NAK. Fields we can't fill are simply omitted.)

**Iridium → 5590 (iridium-toolkit `reassembler.py -m acars -a json`):**
```json
{"app":{"name":"iridium-toolkit","version":"0.0.1"},
 "source":{"transport":"iridium","protocol":"acars","station_id":"<af_id>"},
 "acars":{"timestamp":"2026-01-05T10:30:00+0000","errors":0,
          "link_direction":"downlink","block_end":true,
          "mode":"2","tail":"F-GCBG","flight":"AF0000","label":"2T",
          "block_id":"7","message_number":"M06","ack":"!","text":"..."},
 "header":"<hdr-bytes-hex>"}
```
(Note: Iridium schema carries **no freq/level**; `timestamp` is ISO8601
`%Y-%m-%dT%H:%M:%S%z`; `tail` has leading dots stripped; `label` maps
`_\x7f`→`_d`; `ack` maps `\x15`→`!`; `link_direction` from the uplink bit;
`block_end` = not-continued. `header` = SBD header hex — optional, omit if not
readily available in `acars_msg_t`.)

Field mapping from `acars_msg_t` (`msg_ring.h`) + additions (§2a):

| target field | source | notes |
|---|---|---|
| `timestamp` / `t.sec/usec` | wall-clock epoch | **prerequisite: SNTP** — `timestamp_us` today is boot-relative `esp_timer` time; no SNTP exists in the firmware (grep: only `decode_survey.c` reads `time(NULL)`, unset). See Phase 2. |
| `station` / `station_id` | new `af_id` NVS key | ≥36 chars capacity (§4) |
| `freq` | LO + peak_bin offset | absolute Hz: `lo_now + (peak_bin&0xFFFF − FFT_SIZE/2)·FS_DETECT_HZ/FFT_SIZE` — same math as `reasm_freq_key_hz()` (frame_decoder.c L65) but absolute. MHz float in acarsdec-style, Hz integer in dumpvdl2-style. |
| `level` / `sig_level` | `snr_db` | flag in the JSON as SNR not dBFS (airframes tolerates either; annotate in `app` block) |
| `error` | 0 | only CRC-OK messages are fed |
| `mode/label/block_id/msgno/flight` | `mode/label/block_id/msg_num/flight_id` | direct |
| `tail` | **NOT in `acars_msg_t` today** | libacars `a->reg` is available in `acars_deliver()` (dot-stripped as `reg_nodot`, L440-443) but is dropped before `acars_msg_t` — add `reg[8]` (§2a) |
| `text` | `txt` | JSON-escape via the existing `json_escape()` (M17 triplet — this becomes a 4th copy; good moment to do the pending shared-header consolidation, or accept copy #4 kept byte-identical) |
| `ack` | not extracted | libacars has `a->ack`; add alongside `reg` if the schema requires it (acarsdec sets `!` for NACK) |
| VDL2 `avlc.src/dst` | `avlc_frame_t.src_addr/dst_addr/src_type` in `vdl2_avlc_cb` | needed for dumpvdl2-native shape; must travel `vdl2_avlc_cb → acars_deliver` (§2a) |
| `app.name/ver` | `esp_app_get_description()` | `{"name":"iridium_acars_p4","ver":app->version}` |

### 2a. `acars_msg_t` additions (Phase 1)

Extend `acars_msg_t` (`msg_ring.h`) — automatically improves `/messages`, SD
NDJSON and the existing UDP push too:

- `char reg[8]` — dot-stripped registration from `a->reg` (populate in
  `acars_deliver()` next to the existing `reg_nodot` log).
- `char ack` — `a->ack` (or 0).
- `uint32_t freq_hz` — absolute channel frequency computed at emit
  (`acars_deliver()` already receives `peak_bin`; add the LO via
  `scanner_cur_hz()`/config snapshot).
- `uint32_t src_addr; uint8_t src_type` — VDL2 only (0 for Iridium). Smallest
  honest way to carry AVLC identity: add parameters to `acars_deliver()`
  (Iridium callers pass 0) rather than a side channel.
- ring entry grows ~16 B (~0.5 KB total across MSG_RING_CAPACITY 32 — noise
  in PSRAM terms).

## 3. Transport + emit point

**Reuse `acars_push.c` — add a second target, not a second module.** The
existing queue/task already has exactly the right failure semantics (snapshot
config per message, DNS cache, socket recreate, drop-on-full). Concretely:

- Keep the single 16-deep queue and task. In `push_task()`'s loop, after the
  existing out_host send, format+send the airframes variant when
  `cfg.af_on && cfg.af_host[0] && cfg.af_port`.
- Give the airframes target its OWN socket + DNS cache + `sock_bad` flag
  (duplicate the small static trio into an `af_` set, or factor a
  `push_target_t {sock, dst, host_resolved, port_resolved, resolved_at_us,
  sock_bad}` struct used twice — the struct is the cleaner edit and keeps the
  two endpoints' failure modes isolated: local Mac down must not stall
  airframes and vice versa).
- New `format_airframes_msg(char*, size_t, const acars_msg_t*, const
  app_config_t*)` beside `format_msg()` (L142). Pure function over
  `acars_msg_t` + config — trivially host-testable if placed in a small
  common TU (recommended: `common/…/airframes_fmt.c` so tests/host can link
  it without ESP headers, mirroring how `vdl2_l2` is host-built).
- Emit stays `acars_push_emit()` at the ONE call site (`acars_deliver()`
  L471) — both bands covered automatically; PARTIAL rows structurally
  excluded (they never call `acars_push_emit`).
- UDP unicast only (esp_hosted multicast constraint is moot for a WAN target,
  but TCP would add connection-state management to a task designed around
  fire-and-forget datagrams — stay UDP unless the verified airframes ingest
  for our app requires TCP).

## 4. NVS config (global, existing idiom)

All in namespace `"iridium"` as plain global keys (NOT band-namespaced — §0).
Follow the `out_host`/`iot_log_host` pattern exactly (`app_config.h` L28-30,
`app_config.c` `nvs_get_str_or`/`set_str_field`):

| field | NVS key | type / default | notes |
|---|---|---|---|
| `af_on` | `"af_on"` | bool, **false** | explicit master gate (decode-volume gating is an operator decision, §5); belt-and-braces on top of empty-host-disables |
| `af_host[64]` | `"af_host"` | str, `""` | e.g. `feed.airframes.io`; empty = disabled (the `out_host` idiom) |
| `af_port` | `"af_port"` | u16, 0 | 0 = disabled; per-band correct port is operator-set after §1 verification |
| `af_id[40]` | `"af_id"` | str, `""` | feeder/station ident; 40 B holds a 36-char UUID + NUL (deliberately NOT reusing `station_id[32]` — too short and already used for log identity) |
| `af_lat`, `af_lon` | `"af_lat"/"af_lon"` | f32 via the `nvs_get_f32_or` u32-bitcast idiom (L111), default 0/0 | stored for the schema variants that carry position; airframes primarily takes coords from the web station profile — **verify**; harmless to store either way |

Plus: setters in `app_config.h/.c` (5 one-liners on `set_str_field` /
`commit_one_*`), fields in `nvs_save_args_t` + `config_post` form parsing +
`nvs_save_and_reboot_task` (http_server.c L89-133, ~L1440 — NVS on the
internal-stack task, never httpd), form inputs on the `/config` page
(~L825), and an `af_push:{host,port,id,enabled}` block in `/status` JSON next
to `udp_push` (~L297) — remember `HTTPD_URI_LIMIT` only if adding routes (we
aren't) and the `/status` body-buffer headroom (memory
`feedback_httpd_max_uri_handlers` / the L280 buffer comment).

## 5. Dedup / rate / volume gating / failure handling

- **Only trusted decodes:** guaranteed by construction — the hook is
  `acars_push_emit()`, which PARTIAL/dirty rows never reach, and
  `acars_deliver()` only fires it on `LA_REASM_COMPLETE/SKIPPED`. Feed
  `crc_ok==true` only: add that one check in the airframes branch (the local
  out_host debug push keeps sending everything, unchanged).
- **Dedup:** libacars reassembly already drops `LA_REASM_DUPLICATE`; airframes
  dedups across feeders server-side. No client-side dedup cache needed.
- **Rate:** worst-case decode volume today is a few messages/hour (Iridium at
  reference ceiling; VDL2 similar order at this antenna) — 3 orders of
  magnitude below anything needing throttling. The 16-deep non-blocking queue
  is the backstop; no extra limiter.
- **Volume gating (the explicit #12 condition):** keep `af_on` default OFF and
  document the enable criterion: turn on once `/status decode.rate_24h` (or
  `decode.vdl2.acars`) shows a sustained ≥~10 messages/day so the station
  isn't registered as a dead feeder. This is an operator call, not code.
- **Endpoint down:** exactly the `acars_push` model — `sendto` failure marks
  the airframes socket bad (recreated next message), stale DNS cache reused
  for 30 s, message dropped rather than queued forever. WAN outage loses
  messages by design (SD log + local push remain the archive).

## 6. Phased checklist (each small + verifiable)

**Phase 0 — verify wire contract (no code).** Confirm via airframes
docs/Discord: Iridium + VDL2 ingest host:port, accepted JSON shape(s) for a
custom app, TCP-vs-UDP, station-ident rules. Optionally run the Mac-side #11
feeder (iridium-toolkit JSON from the HydraSDR captures) to validate the
Iridium schema end-to-end before any firmware work. Gate: a test message
visible on airframes.io (or ack from staff).

**Phase 1 — schema plumbing.** `acars_msg_t` += `reg/ack/freq_hz/src_addr/
src_type`; populate in `acars_deliver()` (+ its new params from
`vdl2_avlc_cb`/`try_acars`); surface `reg`+`freq` in `/messages` JSON + SD
NDJSON + existing `format_msg()`. Gates: host — extend
`tests/host` libacars/acars-tail suites to assert the new fields; device —
Iridium RAW smoke unchanged (`Smoke-verified:` trailer, memory
`feedback_device_smoke_mandatory_after_dsp` — this touches the emit path, not
DSP, but the pre-push hook still wants the trailer), `/messages` shows reg on
a live decode.

**Phase 2 — wall clock.** Add SNTP start after WiFi-up (esp_netif SNTP,
Core-0, low prio) so `time(NULL)` is real; stamp `acars_msg_t` emit time as
epoch (keep `timestamp_us` boot-relative for the existing consumers; add
`epoch_us` alongside). Gates: host — none (device-only); device — `/status`
gains a `time_synced` bool; `decode_survey.c`'s `unix_time` (L264) becomes
non-zero for free.

**Phase 3 — formatter + second push target.** `airframes_fmt.c` (host-built,
pure) + `push_target_t` refactor + `af_*` NVS/config/status per §4. Gates:
host — byte-exact unit test of both band formatters against captured
reference JSON (an iridium-toolkit line and a dumpvdl2 line from
`~/dev/vdl2-tools` golden runs); device — point `af_host` at the Mac, `nc -ul
<port>`, confirm one datagram per live decode, then kill the listener and
confirm the device stays healthy (drop, no stall) and `/status` still serves.

**Phase 4 — go live.** Set real host:port + `af_id`, `af_on=1`, watch the
airframes station page; leave the local out_host push running in parallel to
cross-check counts for a week.
