# Updating the ESP32-C6 (esp_hosted slave) firmware — Waveshare ESP32-P4-NANO

The C6-MINI-1 (U1) runs the **esp_hosted *slave* firmware**, factory-flashed by
Waveshare. The P4 host uses `espressif/esp_hosted` (≥2.11 — see
`p4-usb-host/dependencies.lock` for the resolved version). A **stale slave**
(the 0.0.0-vs-host version mismatch) is the root cause of the SDIO-RPC
throttle *and* the broken outbound multicast (why `iot_log`/mDNS don't work —
see `memory/project_esp_hosted_multicast_tx_broken`). The goal is to flash a
C6 slave whose version matches the host esp_hosted.

## The hard constraint

The P4↔C6 link is **SDIO only — no UART data wires between the two chips**
(`docs/c6-companion-firmware-design.md` §3, `docs/p4-nano-board-schematic-summary.md` §2).
So the P4 **cannot** `esp-serial-flasher` the C6. The two real update paths:

- **Method A — PROG_C6 UART flash.** Works today; independent of the (flaky) SDIO link.
- **Method B — esp_hosted SDIO slave-OTA.** Cable-free, host-driven; the API + a
  working example already ship in the component, but it isn't wired into *our* P4
  app yet, and it depends on the SDIO link (chicken-and-egg with the mismatch).

Do **Method A first** to escape the stale version; once the C6 is on a modern
esp_hosted, Method B becomes the clean path for future updates.

---

## Current state (2026-07-11)

- **C6 is now on esp_hosted 2.12.10** (version-matched to the host), flashed via
  **Method B (SDIO slave-OTA) — now proven end-to-end** (`POST /c6ota?url=…` then
  `?activate=1`; wired into the P4 app as `c6_ota.c`). Method A is no longer needed
  for routine updates, only for recovery if the SDIO link is ever broken.
- **SDIO transport checksum is ON** on both ends: host `CONFIG_ESP_HOSTED_SDIO_CHECKSUM=y`
  + slave `CONFIG_ESP_SDIO_CHECKSUM=y`. **These MUST match.** The header layout is
  fixed regardless, so the safe rollout order is **slave-first** (a checksum-on slave
  + checksum-off host works — host ignores the field; the reverse drops every frame).
  Build the slave with the symbol in `slave/sdkconfig.defaults.esp32c6`.
- **Multicast TX is still broken at 2.12.10** — the version bump did NOT fix it; the
  drop is in the C6's closed esp_wifi blob, not the slave version. Confirmed live
  (`nettest`: 0/10 mcast, 5/5 unicast). Remote telemetry must use unicast or HTTP
  `/status` — see `memory/project_esp_hosted_multicast_tx_broken`.
- **Unrelated: P4 app OTA (`POST /ota`) verify-fails on this board** — upstream
  esp-idf#17855 (ESP32-P4 + PSRAM: `esp_image_verify` reads the just-written image
  back through a stale flash mmap cache → false "New image failed verification"; the
  write itself is correct, download-mode ROM read-back is byte-identical). Fails safe
  (rejected before any partition switch, no brick). **Until upstream fixes it, update
  the P4 app over USB** (`scripts/flash.sh`), not OTA.

---

## Method A — flash the C6 over its UART (reliable)

1. **Get the version-matched slave binary.** From Espressif's `esp_hosted` repo,
   the ESP32-C6 slave (`network_adapter`) built at the **same version as the host**
   (`dependencies.lock`). Prebuilt binaries ship in the esp_hosted releases, or
   build from source: clone at the matching tag → `slave/` → `idf.py set-target
   esp32c6` → `idf.py build`. Match the SDIO transport + any non-default
   `ESP_HOSTED_*` Kconfig the host expects.

2. **Reach the C6 UART.** `C6_U0TXD`/`C6_U0RXD` go to the board's **PROG_C6
   console (on-board USB-UART bridge)** and header **P2**. ⚠️ Which USB
   port/jumper selects the C6 (vs the P4 console) is board-specific — get it from
   the **[Waveshare ESP32-P4-NANO wiki](https://www.waveshare.com/wiki/ESP32-P4-NANO)**
   (they document the PROG_C6 procedure). Via P2 directly: C6 `U0TXD`→adapter RX,
   `U0RXD`→adapter TX, GND→GND, **3.3 V logic, no power from the adapter**.

3. **Enter download mode.** C6 BOOT = `IO9` (on P2); EN=`GPIO54`, IO2 strap=`GPIO6`
   are P4-controlled. Cleanest: **hold the P4 in reset** (RESET button) so it
   releases the C6 lines, then let the PROG_C6 tool auto-reset (or manually strap
   `IO9` low across an EN pulse).

4. **Flash:**
   ```
   esptool.py --chip esp32c6 -p <C6_PORT> -b 460800 write_flash \
     0x0 bootloader.bin 0x8000 partition-table.bin 0x10000 network_adapter.bin
   ```
   (offsets per the slave build's `flash_args`; or `idf.py -p <C6_PORT> flash`).

5. **Verify.** Reboot; in the **P4** boot serial the esp_hosted transport-version
   handshake mismatch should be gone, and the SDIO throttle / multicast behavior
   should improve.

---

## Method B — esp_hosted host-driven SDIO slave-OTA (cable-free)

The `espressif/esp_hosted` component we already depend on ships this. The host
(P4) streams the new C6 firmware over the SDIO RPC link; the C6 writes it to its
OTA partition and boots into it. **No UART, no cable, no physical access.**

**API** (`managed_components/espressif__esp_hosted/host/api/.../esp_hosted_ota_api.c`):
- `esp_hosted_slave_ota_begin()` — start the session (RPC `rpc_ota_begin`)
- `esp_hosted_slave_ota_write(chunk, len)` — stream firmware chunks
- `esp_hosted_slave_ota_end()` — finalize
- `esp_hosted_slave_ota_activate()` — boot the new image (**requires current slave
  FW > v2.5.x** — see catch-22 below)

**Example** (`managed_components/espressif__esp_hosted/examples/host_performs_slave_ota/`)
shows three ways to *source* the C6 binary, all ending in begin→write→end:
- **HTTPS** — host downloads from a URL (`CONFIG_OTA_SERVER_URL`)
- **partition** — from a dedicated partition on the P4 flash (`CONFIG_OTA_PARTITION_LABEL`)
- **littlefs** — from a littlefs partition on the P4 flash

(The header notes the low-level API is being superseded by newer
`examples/host_slave_ota/` + `host_self_ota/`; same begin/write/end primitives.)

**To use it here** we'd add to the P4 app: a trigger (e.g. `POST /c6ota?url=…`,
mirroring our existing `/ota` for the P4), one of the source components above,
and the begin→write→end→activate loop. Modest — the primitives are done.

### Why NOT to rely on Method B for the *first* update — two catch-22s

1. **`activate()` needs slave FW > v2.5.x.** The C6 is at 0.0.0. The stale slave
   may not support the activate step (or the OTA RPC protocol at all). You could
   land the write but not switch to it.
2. **It rides the SDIO RPC link — the very thing the mismatch degrades.** OTA is a
   long bulk transfer over the same throttled/timing-out RPC channel. It *might*
   work (the throttle is intermittent), but a broken-enough link can't reliably
   complete it. Bootstrapping a working slave over a link the broken slave makes
   flaky is fragile.

So: **Method A once** (UART, independent of SDIO) to reach a known-good modern
version, then **Method B** is viable and preferred for every update after —
including once the device is deployed outside with no USB access.

---

## Critical caveats (both methods)

- **Match the version.** A slave newer/older than the host esp_hosted can be worse
  than the current stale one. Pin both to the same release.
- **Coordinate reset (Method A).** The P4 controls the C6 (`GPIO54`/`GPIO6`); flash
  with the P4 held in reset (or off) so it isn't fighting the download straps.
- **Recoverable, not one-shot.** A failed flash re-flashes over the same path — but
  you'll have **no Wi-Fi (and no remote monitoring)** until it's back. Do the first
  update while USB is accessible, **before** the device is mounted outside.
- **Waveshare wiki is authoritative** for the board-specific bits (PROG_C6 port/jumper).
