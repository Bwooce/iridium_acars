# P4 ↔ C6 Wi-Fi Design

Status: Implemented (commit `bba7817` — D17 redo, 2026-05-23).
Supersedes the v0.1 design (UART-based standalone C6 firmware) — see the
*History* section at the bottom for why that approach was abandoned.

---

## 1. What runs on each chip

```
   Phone / laptop browser
            |
       Wi-Fi 6 (2.4 GHz)
            |
    [ESP32-C6-MINI-1]    Factory esp_hosted slave (Wi-Fi + BT)
       512 KB HP-SRAM,   No custom firmware on the C6 in this design.
       4 MB flash
            |
       SDIO (P4 GPIO 14-19 ↔ C6 IO18-23)
            |
    [ESP32-P4]           Iridium ACARS decoder + Wi-Fi (via RPC)
       32 MB PSRAM        Wi-Fi STA/AP, HTTP server, OTA all run here.
       16 MB flash
            |
       USB Type-A (OTG 2.0 HS)            RJ45 100M Ethernet
            |                                      |
    [RTL-SDR v4]                             Local LAN
       Iridium L-band 1616-1626 MHz       (P4 native, no C6)
```

The C6 is the Wi-Fi radio. The P4 makes every Wi-Fi decision (when to scan,
which AP to join, what HTTP routes to expose), and the C6 forwards the
on-air frames it sees plus the host's transmissions.

## 2. Architecture: `esp_wifi_remote` + `esp_hosted`

Both components are vendored under `p4-usb-host/managed_components/`
(IDF Component Manager). They were added in commit `bba7817`.

- **`esp_hosted`** ships two pieces:
  - A *slave* application (already pre-flashed on the C6 by Waveshare)
    that exposes the Wi-Fi MAC/PHY plus a small RPC channel over SDIO.
  - A *host* library on the P4 side that talks to the slave: opens the
    SDIO transport, marshals RPC requests, hands raw 802.11 frames up
    to `esp_netif`.
- **`esp_wifi_remote`** intercepts the standard `esp_wifi_*()` API on
  the P4 and routes each call to the host library above. From the
  P4 application's perspective, the Wi-Fi API is the same as on a
  chip with native Wi-Fi — `esp_wifi_init()`, `esp_wifi_set_config()`,
  `esp_wifi_start()`, etc. all return as expected.

This means the P4 can use `esp_netif`, `esp_http_server`, `esp_https_ota`,
and any other IDF Wi-Fi-aware component without a special API layer.

## 3. Inter-chip wiring

Re-verified 2026-05-23 from the Waveshare ESP32-P4-NANO schematic
(full pin extract in `docs/p4-nano-board-schematic-summary.md` §2).
**Eight wires** total — six SDIO data/clock/command lines plus two
control straps:

| Signal | P4 GPIO | C6 pin / native | Notes |
|---|---|---|---|
| SDIO CMD       | 19 | 24 / IO18 | R21 = 1K series |
| SDIO CLK       | 18 | 25 / IO19 | R20 = 1K series |
| SDIO D0        | 14 | 26 / IO20 | |
| SDIO D1        | 15 | 27 / IO21 | R19 = 1K |
| SDIO D2        | 16 | 28 / IO22 | R18 = 1K |
| SDIO D3        | 17 | 29 / IO23 | R16 = 1K |
| C6 BOOT strap  |  6 |  5 / IO2  | R52 = 0Ω; drive low across reset edge → ROM bootloader |
| C6 reset (EN)  | 54 |  8 / EN   | R54 = 0Ω; pulse low to reset C6 |

C6's UART0 (`U0TXD`/`U0RXD`) and IO9 (BOOT button on C6 itself) go
**only** to the on-board USB-UART programmer chip and header P2 —
not to the P4. So:

- The P4 **can** reset the C6 (drive GPIO54 low) and force the C6's
  ROM bootloader (drive GPIO6 low while pulsing reset).
- The P4 **cannot** complete an `esp-serial-flasher` flow to the C6
  (no UART data path). SDIO-side OTA or the PROG_C6 USB cable are
  the only firmware-update paths.

The lack of UART wires killed the prior UART-based companion design
— see *History*. The BOOT + EN straps remain useful for hang
recovery and (if SDIO-side OTA is ever implemented) for forcing
the C6 into bootloader mode.

## 4. Internal-SRAM budget (the `esp_hosted` shrink)

The host side of `esp_hosted` allocates its task stacks + RPC queues +
SDIO transport buffers at boot (via `__attribute__((constructor))` so it
runs before `main()`). On a P4 already crowded by DSP code + AGC + USB
host pool, the defaults blow internal SRAM:

- WDT reset loop within ~7 s of boot, or
- `sdio_mempool_create assert (buf_mp_g)` panic.

`p4-usb-host/sdkconfig.defaults` overrides:

| Knob | Default | Project | Why |
|---|---|---|---|
| `ESP_HOSTED_MEMPOOL_PREFER_SPIRAM` | n | **y** | Transport buffers in PSRAM (P4 GDMA reaches PSRAM through cache; no perf hit for our Wi-Fi data rate). |
| `ESP_HOSTED_DFLT_TASK_FROM_SPIRAM` | n | **y** | Hosted task stacks in PSRAM (saves ~10 KB internal SRAM across the 3 Hosted tasks). |
| `ESP_HOSTED_SDIO_TX_Q_SIZE` | 20 | **8** | Queue arrays live in internal SRAM. 8 is plenty for our traffic. |
| `ESP_HOSTED_SDIO_RX_Q_SIZE` | 20 | **8** | Same. |
| `ESP_HOSTED_RPC_TASK_STACK` | 4096 | **3072** | Smaller hosted-task footprint. |
| `ESP_HOSTED_DFLT_TASK_STACK` | 3072 | **2560** | Same. |
| `ESP_HOSTED_MAX_SIMULTANEOUS_SYNC_RPC_REQUESTS` | 5 | **2** | We never issue more than 2 concurrent. |
| `ESP_HOSTED_MAX_SIMULTANEOUS_ASYNC_RPC_REQUESTS` | 5 | **2** | Same. |
| `ESP_HOSTED_MEM_MONITOR` | y | **n** | Unused. |
| `ESP_HOSTED_CLI_ENABLED` | y | **n** | Unused. |

After shrink: at smoke entry DMA-INT free = 35 KB, largest contiguous
= 27 KB (was 11 KB / 4.8 KB pre-shrink). Decode preserved at the
baseline matched=61 on the RAW_IRIDIUM fixture.

## 5. What still needs building on the P4 side

The transport is up; the application layer on top of it is still to do.
Tracked under task #36 (D17 in-progress) — the open items are:

- **NVS-driven STA join**: read `wifi_ssid`/`wifi_psk` from `APP_CFG`
  (D18 already exposes these), call `esp_wifi_set_config()` + connect.
  Fall back to soft-AP captive portal if NVS is empty.
- **HTTP server**: `esp_http_server` with a single-page config UI
  (gain, LO, bias-tee, station ID, Wi-Fi creds) and a streaming
  endpoint for ACARS frames.
- **Output**: at minimum a long-poll JSON endpoint; possibly UDP push
  to a configurable peer.
- **OTA**: `esp_https_ota` for the P4. Slave OTA for the C6 if we ever
  want to update its firmware (e.g. to enable OpenThread — see *Thread
  capability* below).

The captive portal flow that the prior design assigned to the C6 now
lives on the P4 (with the C6 simply acting as the radio).

## 6. Thread capability

The C6 silicon has an 802.15.4 radio in addition to Wi-Fi + BT, so
hardware-wise Thread/Zigbee/Matter are possible. The factory
`esp_hosted` slave is built with Wi-Fi + BT only — no
`CONFIG_OPENTHREAD_ENABLED`, no `CONFIG_IEEE802154_ENABLED`. The slave
*Kconfig* exposes `ESP_HOSTED_OT_TRANSPORT_HOSTED` for routing spinel
frames over the same SDIO transport, but enabling it requires reflashing
the C6.

That's deferred. If we ever want it, the natural moment is when the
P4-side OTA story is in place — at that point we can flash a customised
slave image from the P4 over SDIO.

## 7. History — why the standalone-C6 UART design was abandoned

The previous design (v0.1, 2026-05-20, committed in `20643e3` then
reverted in `15c8205`) assumed:

- The C6 ran a standalone IDF app (Wi-Fi STA + AP, HTTP server, captive
  portal, msg ring) totalling ~4 MB.
- The C6 talked to the P4 over an *inter-chip UART*, with a framed
  binary protocol (`common/iridium_protocol/iridium_protocol.h`) and a
  P4-side forwarder (`p4-usb-host/main/c6_forwarder.{c,h}`).

A schematic audit (memory note `project_p4_c6_pinout.md`) showed there
is **no inter-chip UART**. The C6's UART pins only go to the on-board
USB-UART programmer. The whole transport assumption was wrong.

Rather than re-route the UART (which would require board rework and
lose Waveshare's pre-flashed programming path), we deleted the
standalone-C6 design and pivoted to the SDIO + `esp_hosted` approach
documented above. The C6 is a Wi-Fi radio; everything else runs on
the P4.

## 8. References

- Memory: `project_p4_c6_pinout.md` — full pin table extracted from the schematic.
- Memory: `project_heap_position_decode_bug.md` — context for the SRAM budget pressure.
- Code: `p4-usb-host/main/idf_component.yml` — the two dependencies that bring this transport in.
- Code: `p4-usb-host/sdkconfig.defaults` — the shrink settings.
- Doc: `docs/p4-nano-board-schematic-summary.md` — the inter-chip pinout table.
- IDF: `managed_components/espressif__esp_hosted/host/port/esp/freertos/src/port_esp_hosted_host_init.c` — the constructor that asserts pre-`main()` if the mempool fails.
