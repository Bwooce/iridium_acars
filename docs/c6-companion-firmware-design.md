# ESP32-C6 Companion Firmware — Architecture Design

Version: 0.1 — design only, no code written
Date: 2026-05-20
Status: For review before any implementation begins

---

## 1. Hardware Topology

Board: Waveshare ESP32-P4-Nano. Two chips share a board; each has independent flash.

```
Phone / laptop browser
        |
    Wi-Fi 6 (2.4 GHz)
        |
  [ESP32-C6-MINI-1]          4 MB flash, 512 KB HP-SRAM
   Web server + captive       No PSRAM
   portal + config proxy
        |
   UART (app channel) + SDIO hardware pins present but unused by app
        |
  [ESP32-P4]                  16 MB flash, 32 MB PSRAM (octal)
   USB host + RTL-SDR          Full decode pipeline
   DSP + demod + BCH           class_driver / frame_decoder
   IDA / SBD / libacars        status_logger
        |                                                |
   USB Type-A (OTG 2.0 HS)                        RJ45 100M Ethernet
        |                                                |
  [RTL-SDR v4]                                       Local LAN
   Iridium L-band 1616–1626 MHz                   (P4 direct, no C6)
```

The board also exposes (not used by the firmware below but documented
for completeness): MIPI 2-lane display interface, MIPI 2-lane camera
interface, onboard microphone, MX1.25 speaker header, RTC battery
header (rechargeable), PoE module header (for power-over-Ethernet
deployments using the RJ45 port), and a TF (microSD) card slot wired
through SDIO 3.0.

**Confirmed inter-chip wiring (Waveshare Nano schematic):**

| Signal | P4 GPIO | Function |
|---|---|---|
| SDIO CLK | 18 | Currently esp_hosted transport — freed by design below |
| SDIO CMD | 19 | |
| SDIO D0–D3 | 14, 15, 16, 17 | |
| C6 Reset | 54 | P4 can hard-reset C6 |
| App UART TX (P4→C6) | TBD — verify from schematic | UART1 on P4 side |
| App UART RX (C6→P4) | TBD — verify from schematic | |

**SD card** (wired to P4 only): CLK=GPIO43, CMD=GPIO44, D0–D3=GPIO39–42.

---

## 2. Fundamental Architecture Decision

The Nano ships with the C6 pre-flashed as an `esp_hosted` slave — a transparent WiFi/BT modem that lets the P4 issue standard `esp_wifi_*` calls over SDIO. This design does **not** use that model. The C6 runs a **standalone IDF application** that owns WiFi natively.

Rationale:

- The P4 firmware never uses WiFi for its decode pipeline (no `esp_wifi_remote` dependency anywhere in `p4-usb-host/`). Keeping esp_hosted gains nothing for P4.
- Running `esp_hosted` slave alongside a full HTTP server and captive portal DNS on 512 KB RAM is a very tight fit and adds complexity with no benefit.
- A standalone C6 app is independently flashable (via the PROG_C6 header or C6 OTA), debuggable on C6's own UART, and has no coupling to the P4 build.
- The SDIO pins are physically present on both chips; they become available for a future use (e.g. faster bulk data transfer) if needed.

The C6 firmware is a separate IDF project under `c6-companion/` (new directory, sibling of `p4-usb-host/`). It is built and flashed independently using the same `esp-idf/` vendored IDF.

---

## 3. Captive Portal and Initial Configuration

### Entry conditions

The C6 enters soft-AP (captive portal) mode when any of the following are true at boot:

1. NVS key `wifi/ssid` is absent or empty — first boot, no credentials stored.
2. The BOOT button is held for more than 3 seconds during the C6's boot sequence. The BOOT button GPIO on the C6 side needs to be confirmed from the schematic; it is likely C6 GPIO9 (the standard ESP32-C6 strapping pin used as BOOT on most Waveshare boards).
3. P4 sends a `trigger_ap_mode()` RPC (explicit user trigger from a future CLI command or status log condition).

### Soft-AP operation

- SSID: `iridium-{last 3 bytes of C6 MAC}`, open network (no password on the AP itself — the goal is frictionless phone connection).
- DNS server (`esp_dns_server` component or a minimal custom resolver) responds to all A queries with the C6's AP IP (192.168.4.1). This causes most phones to auto-display the captive portal browser.
- HTTP server serves a single-page config form at `http://192.168.4.1/` covering: home WiFi SSID + password, LO frequency, tuner gain mode + gain dB, bias-tee toggle, output endpoint URI, station ID.
- On form submit: C6 validates inputs, stores them to its own NVS, then issues `nvs_set` RPCs to P4 for each key. On success, C6 disables AP, connects to home WiFi as a station, then sends `sdr_retune()` to P4 with the new SDR parameters.

### Captive portal frontend

Single static HTML file (< 20 KB) served from a SPIFFS partition on C6 flash. No JavaScript framework. The form posts to `/save` as `application/x-www-form-urlencoded`. No templating — the form values are pre-populated via a prior GET to `/config` that returns a JSON object, consumed by minimal inline JS.

---

## 4. Config Management and NVS Strategy

### NVS key table

All NVS keys live in namespace `iridium` on the P4. The C6 caches a copy in its own NVS (namespace `iridium_c`) for fast web UI reads.

| Key | Type | Hot-applyable? | Notes |
|---|---|---|---|
| `wifi/ssid` | string | Yes — C6 reconnects | C6 acts on this directly; P4 does not use it |
| `wifi/pass` | string | Yes — C6 reconnects | Same |
| `sdr/center_freq` | u32 (Hz) | Yes — `rtlsdr_set_center_freq()` | P4 calls immediately without restart |
| `sdr/gain_mode` | u8 (0=AGC, 1=manual) | Yes — `rtlsdr_set_tuner_gain_mode()` | |
| `sdr/gain_db` | i16 (tenths of dB) | Yes — `rtlsdr_set_tuner_gain()` | Only meaningful when gain_mode=1 |
| `sdr/bias_tee` | u8 (0/1) | Yes — requires brief USB reinit | RTL-SDR v4 supports bias-tee; toggle takes ~50 ms |
| `net/out_ep` | string | Yes — P4 reconnects to new endpoint | e.g. `mqtt://host:1883` or `udp://host:5555` |
| `station/id` | string | Yes — appended to next message | No pipeline restart needed |
| `fw/ota_url` | string | No — reboot required | Used for future OTA; no pipeline at parse time |

### Caching trade-off

The C6 keeps a shadow copy of all keys except `wifi/pass` (not served to the browser). On the web UI settings page, values come from C6's local NVS cache — no round-trip to P4 needed for display. On save, the C6 updates both its own cache and the P4's authoritative NVS via RPC. This means a P4 crash/reboot cycle produces a momentary divergence (C6 cache says new value; P4 re-reads its NVS and gets the same new value), which is benign.

The alternative (always round-trip to P4 for reads) is simpler but adds ~5–20 ms latency to every page load and fails if the UART is busy. The caching model is preferred.

### Change notification and hot-apply

When the C6 commits new SDR parameters, it sends `sdr_retune()` as a single atomic RPC rather than individual `nvs_set` calls. P4 applies them in order inside its UART handler task (Core 0, lowest priority — see section 9). The application sequence is: stop streaming → apply new parameters → restart streaming. This is a ~200 ms interruption at most.

For `wifi/ssid` and `wifi/pass`, the C6 handles reconnection itself; no P4 notification needed.

---

## 5. Web UI Architecture

Static HTML/JS files served from a SPIFFS partition on C6 flash (not FATFS — SPIFFS needs no formatting step and survives power loss mid-write better for small read-mostly partitions). No server-side templating. All dynamic content comes from REST GET requests to the C6's HTTP server.

### Pages

| Page | URL | Data rendered | Source |
|---|---|---|---|
| Settings | `/` | All NVS keys, WiFi status, P4 uptime | C6 local NVS cache + status snapshot |
| Messages | `/messages` | Ring buffer of last 200 decoded ACARS messages | C6 in-RAM ring buffer |
| Stats | `/stats` | 1 s snapshots: USB rate, DSP cap %, worker cap %, drops, burst rate, decode success rate, SNR histogram | C6 in-RAM stats array (last 60 snapshots) |

**REST endpoints (C6 HTTP server):**

| Method | Path | Response |
|---|---|---|
| GET | `/api/config` | JSON object of all non-secret NVS keys |
| POST | `/api/config` | Apply settings; returns `{ok, errors[]}` |
| GET | `/api/messages` | JSON array of last N messages; query param `?since=seq` for polling |
| GET | `/api/stats` | JSON object of latest snapshot + 60-point history |
| GET | `/api/status` | WiFi RSSI, P4 uptime, C6 uptime, link health |

The frontend polls `/api/messages?since=<last_seq>` every 5 seconds. `/api/stats` is polled every 2 seconds. No WebSocket — polling is adequate for the update rate and saves code complexity on a 512 KB RAM budget.

---

## 6. Message Ring Buffer on C6

**Sizing:** C6 HP-SRAM budget estimate after WiFi stack (~180 KB), IDF + FreeRTOS overhead (~80 KB), HTTP server + TLS heap (~50 KB), task stacks (5 tasks × ~4 KB = ~20 KB), and static BSS (~30 KB) leaves roughly 150 KB for application data. Allocating 50 KB to the message ring buffer yields capacity for 250 messages at 200 bytes each. A depth of 200 is the design target; reduce to 100 if RAM is tighter in practice.

**Persistence:** Ring buffer is in RAM only — lost on C6 reboot. This is acceptable; the purpose is "recent messages for a browser page", not a durable log. Durable logging belongs on the SD card (via P4, section 8).

**Structure:** A fixed-size circular array of `acars_entry_t` structs, indexed by a monotonically increasing sequence number. The struct holds: sequence number (u32), timestamp_us (u64), direction (u8), mode (char), label[3], flight_id[7], msg_num[5], text[160], snr_db (float), freq_hz (u32). Total: ~200 bytes per entry.

**Thread safety:** A single FreeRTOS mutex protects the ring buffer. The UART receive task holds it only during the memcpy append (~2 µs). The HTTP handler holds it during the JSON serialisation of up to 200 entries (~10 ms max). Contention is negligible given the 1 Hz update rate from P4.

---

## 7. SDR Reception Stats Forwarding

`status_logger.c` already collects a `status_snapshot_t` once per second. The hook requires one new call in `status_logger_emit()` (or a second subscriber registered via a callback pointer):

```c
// In status_logger.c, after emit():
c6_forwarder_post_snapshot(s);   // non-blocking; drops if forwarder queue full
```

`c6_forwarder` is a new thin module in `p4-usb-host/main/`: a single FreeRTOS queue (depth 2) and a low-priority task that serialises the snapshot to the inter-chip UART. Fields forwarded in the `status_snap` message: `rate_mb_s`, `dsp_cap_pct`, `worker_cap_pct`, `rb_full_drops`, `dsp_frame_count`, `ws.bursts_processed`, `s_acars_decoded` (from frame_decoder atomics). The full `status_snapshot_t` is ~200 bytes; the serialised subset is ~60 bytes — well within one UART frame.

The forwarder task runs at priority 1 (same as `status_logger_task`), pinned to Core 1. It must never block the DSP hot path.

---

## 8. SD Card Access from C6

The SD slot is wired to P4 (SDMMC, GPIOs 39–44). The 3 MB SPIFFS `storage` partition declared in `partitions.csv` is on P4's SPI flash, not the SD card — but the partition table comment notes the SD slot exists and P4 controls it.

C6 accesses the SD card via RPC: it sends file operation requests to P4, which runs the POSIX file APIs on its SD-mounted filesystem and returns results. This is the same pattern as NVS access.

**Intended SD card uses:**

- Long-term archive of decoded ACARS messages (CSV or JSONL, one file per day, appended by P4's `c6_forwarder` equivalent or a dedicated SD logger task on P4 that also listens to the `acars_msg` events).
- Configuration backup (P4 dumps NVS to `config.json` on the SD card at each `nvs_set`; survives P4 flash erase).
- Raw IQ capture (future): P4 writes burst IQ to SD for offline analysis. C6 provides the browser UI to trigger and retrieve captures.

The C6 does NOT need direct SD access for its core web UI function — message ring buffer and stats are in C6 RAM.

---

## 9. Inter-Chip Protocol

### Transport

**Application UART, 921600 baud.** The SDIO hardware is physically present but left unused by this design (see section 2 rationale). UART is sufficient: at one ACARS message per 15 seconds average and one status snapshot per second, peak bandwidth is under 1 KB/s — less than 1% of 921600 baud capacity.

The exact GPIO assignments for the application UART on both chips must be confirmed from the Waveshare Nano schematic (the schematic PDF is available at `https://files.waveshare.com/wiki/ESP32-P4-NANO/ESP32-P4-NANO-schematic.pdf` but the image-only page renders pin labels too small to read at available resolution). Likely candidates: P4 UART1 (free after USB host takes DWC OTG), C6 UART0 or UART1.

### Message framing

Length-prefixed binary frames: `[SOF:1][LEN:2 LE][TYPE:1][PAYLOAD:LEN bytes][CRC16:2]`. SOF = 0xAA. CRC16-CCITT over TYPE+PAYLOAD. Maximum payload 512 bytes (more than enough for any single message).

JSON encoding of payloads is rejected: JSON parsing on C6's 512 KB RAM is fine, but JSON encoding on P4's hot Core 0 path adds unnecessary overhead. Instead, payloads are flat packed structs with fixed-size fields. This eliminates parser dependency on P4 entirely.

### RPC API

All RPCs are initiated by C6. P4 responds with a result frame. P4-initiated pushes use type codes > 0x80 and have no response.

**C6 → P4 (requests, type < 0x80):**

| Type | Method | Payload | Response |
|---|---|---|---|
| 0x01 | `ping` | (empty) | `{uptime_ms: u64}` |
| 0x10 | `nvs_get` | `{ns: str[16], key: str[16]}` | `{val: bytes[64], type: u8, err: u8}` |
| 0x11 | `nvs_set` | `{ns: str[16], key: str[16], val: bytes[64], type: u8}` | `{err: u8}` |
| 0x20 | `sdr_retune` | `{freq_hz: u32, gain_mode: u8, gain_db: i16, bias_tee: u8}` | `{err: u8}` |
| 0x30 | `sd_list` | `{path: str[64]}` | `{n: u8, entries: str[16][n], err: u8}` |
| 0x31 | `sd_read` | `{path: str[64], offset: u32, length: u16}` | `{data: bytes[512], actual: u16, err: u8}` |
| 0x32 | `sd_write` | `{path: str[64], data: bytes[512], length: u16}` | `{written: u16, err: u8}` |

**P4 → C6 (push, type ≥ 0x80, no response):**

| Type | Method | Payload |
|---|---|---|
| 0x80 | `boot_complete` | `{fw_ver: str[16]}` |
| 0x81 | `acars_msg` | `{seq: u32, ts_us: u64, direction: u8, mode: u8, label: u8[2], flight: u8[6], msg_num: u8[4], text: u8[160], snr_db: float, freq_hz: u32}` |
| 0x82 | `status_snap` | `{rate_mb_s: float, dsp_cap: float, worker_cap: float, drops: u32, frames: u32, processed: u32, acars_decoded: u32}` |

### Backpressure

P4 pushes (0x80–0x82) are non-blocking: `c6_forwarder_post_snapshot()` and the ACARS push call `xQueueSend(..., 0)` — if the forwarder queue is full, the item is silently dropped. At 1 msg/s for status and ~0.1 msg/s for ACARS on average, queue depth 4 is more than sufficient. Backpressure only occurs if the UART is blocked (C6 not consuming), which would indicate a C6 fault — P4 continues decoding regardless.

For RPC responses (C6 waits for P4), a 2-second timeout is applied. If P4 does not respond (busy or not yet booted), the C6 logs the failure and retries once. NVS set failures block the captive portal save flow and surface an error to the browser.

---

## 10. Boot Order and Failure Modes

### Sequence

1. Power-on. Both chips start simultaneously (no hardware sequencing on the Nano).
2. C6 boots first in practice (simpler init — no USB enumeration, no PSRAM test). Within ~300 ms it either: enters soft-AP mode (no credentials) or begins STA connection. It starts its HTTP server and UART handler immediately regardless.
3. P4 boots: PSRAM test, USB host install, RTL-SDR enumeration (~3–6 s total). When `action_start_stream` completes successfully, P4 sends `boot_complete(fw_ver)` to C6.
4. C6 receives `boot_complete` and marks P4 as "ready". The stats page in the web UI shows "waiting for decoder" until this event.
5. C6 may issue `nvs_get` RPCs to prime its NVS cache immediately after `boot_complete`.

### P4 failure modes

If P4 never sends `boot_complete` (USB enumeration failure, RTL-SDR absent, panic): C6 continues serving the web UI. The stats page shows "P4 not ready" with last-known stats. The settings page still works (reads from C6 cache). Config saves that require `sdr_retune` RPCs will fail with a timeout error surfaced to the browser. This is the intended behaviour — C6 is not P4's health monitor.

If P4 sends `boot_complete` then goes silent: C6 marks P4 as stale after 5 s without a `status_snap`. Web UI shows a staleness warning. No automatic recovery by C6 — P4 owns its own watchdog.

### C6 failure modes

If C6 fails to come up, P4 continues decoding and logging all output to the P4's USB-CDC UART (`/dev/ttyACM0`). The network output path is lost. P4's `c6_forwarder` task silently drops every item from its queue (queue send returns immediately if the remote UART never ACKs). No data loss in the core pipeline. ACARS messages still appear in the P4 UART log.

### WiFi drop

C6 uses `esp_wifi`'s built-in reconnect (`WIFI_DISCONNECT_REASON_*` handling in the event loop). Reconnect backoff: immediate → 5 s → 30 s → 60 s → 60 s repeating. The web UI becomes unreachable during the gap but the message ring buffer accumulates messages from P4 over UART normally. When WiFi reconnects, the full ring buffer is immediately available to browser clients.

If WiFi cannot be recovered (credentials changed, network gone), C6 does not automatically re-enter AP mode — that would break any running browser session. A 3-second BOOT button press re-enters AP mode explicitly.

---

## 11. Build System

The C6 companion firmware lives under `c6-companion/` at repo root (sibling of `p4-usb-host/`). It is a separate IDF project with its own `CMakeLists.txt`, `sdkconfig.defaults`, and `partitions.csv`. It references `../common/` for any shared type headers (e.g. the inter-chip protocol struct definitions, which should live in `common/iridium_protocol/` to be included by both P4 and C6 builds).

Flash script: `scripts/flash_c6.sh` — mirrors `scripts/flash.sh` but targets the C6's serial port (PROG_C6 header, typically `/dev/ttyUSB0` or a second `/dev/ttyACM*`).

**C6 OTA:** The C6 can be updated over-the-air via `esp_https_ota` once connected to WiFi, without physical access. This requires an OTA partition layout in the C6's `partitions.csv` (factory + ota_0). The P4's partition table is unaffected — C6's flash is independent.

**P4 partition table changes:** None required. The current `partitions.csv` does not need a C6 image slot. The factory + SPIFFS layout is unchanged.

---

## 12. Open Questions and Deferred Decisions

1. **Application UART GPIO numbers.** The Waveshare Nano schematic PDF (1.4 MB) is available at `https://files.waveshare.com/wiki/ESP32-P4-NANO/ESP32-P4-NANO-schematic.pdf` but requires opening in a PDF viewer — the image-rendered version is too low-resolution to read pin labels. Before writing any UART driver code, confirm which P4 GPIO pair is routed to the C6-MINI-1's UART pins on the board. Likely P4 UART1 (since UART0 is the USB-CDC console) and C6 UART0 or UART1.

2. **BOOT button GPIO on C6.** The 3-second-hold for AP mode entry requires knowing which C6 GPIO is connected to the physical BOOT button. On most ESP32-C6 modules this is GPIO9 (strapping pin also used as BOOT/DL mode selector), but confirm from the Nano schematic.

3. **TLS for the web UI.** Currently designed as plain HTTP. If the device is ever used on a network where a MITM could inject malicious config, TLS becomes necessary. The tradeoff: `mbedTLS` on C6's 512 KB RAM is tight (~100 KB overhead). Defer until the threat model warrants it; captive portal and local LAN use cases do not.

4. **Output endpoint forwarding from P4.** The `net/out_ep` NVS key implies P4 sends decoded ACARS to a remote MQTT or UDP endpoint. This is listed as a Phase 6 item in the implementation plan. The RPC design above supports it but P4 does not currently implement it. When it does, the output endpoint connection lives on P4 (which has no network stack), or alternatively all remote forwarding is delegated to C6 (C6 receives `acars_msg` over UART and forwards to MQTT). The latter is architecturally cleaner and removes any network dependency from P4. Decision should be made before implementing `net/out_ep`.

5. **802.15.4 / Thread / Zigbee.** The C6 has an 802.15.4 radio. This design ignores it. If the deployment environment (aircraft hangar, airport infrastructure) happens to have a Thread or Zigbee network, the C6 could publish decoded messages over it with a thin extra task. Deferred until a concrete use case exists.

6. **Dual-transport option.** SDIO (C6 as slave, P4 as master) would give ~20 Mbps vs ~92 KB/s for UART. Given the actual bandwidth requirement (< 1 KB/s), SDIO is unnecessary overhead for the current use case. If a future use case (e.g. streaming IQ to C6 for local analysis) requires it, the SDIO pins are already wired — the slave firmware can be added to the C6 app as a FreeRTOS task alongside the WiFi stack, and the P4 side adds an SDIO master driver. This is a clean upgrade path that does not invalidate the UART design; the two channels can coexist.

---

*End of design document.*
