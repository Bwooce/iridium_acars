# Waveshare ESP32-P4-NANO schematic reference

Source: `e6486a2a-ESP32P4NANOschematic.pdf` (Altium export, dated 2024-10-25, single
page composed of ~12 sheet blocks). All net names, refdes and pin labels below are
taken verbatim from that PDF unless noted otherwise.

Two MCUs on one PCB:

- **U7**: ESP32-P4 main SoC (104-pin QFN). All on-board peripherals except Wi-Fi/BT
  attach to this die.
- **U1**: ESP32-C6-MINI-1-N4 radio companion (Wi-Fi 6 / BT-LE / 802.15.4 + 4 MB
  flash). Acts as an SDIO/UART slave to the P4.

Primary 5 V rail = `VCC_5V` (sourced from USB-C VBUS via Q2 high-side P-FET,
optionally from PoE — see §8). 3.3 V system rail = `ESP_3V3` from U3
(MP1658GTF-Z buck, 3 A). P4 core (1.2 V `ESP_VDD_HP`) from U4 (MP1605GTF-Z).
PHY/analog 3.3 V `PHY_3V3`/`A3V3` from U9 (RT9193-33PB LDO).

---

## 1. Peripheral-to-MCU mapping

| Peripheral | Part | Wired to | Bus / pin block | P4 GPIOs (or C6) | Notes |
|---|---|---|---|---|---|
| External SPI flash | U8 GD25Q128ESIG (128 Mbit / 16 MB) | **P4** | dedicated SPI0 flash pins | `FLASH_CS/CK/D/Q/HD/WP` (P4 pins 30-36) | P4 boots from this; `VDD_FLASH` = `ESP_LDO_VO1` (internal LDO out) |
| C6 internal flash | inside ESP32-C6-MINI-1-N4 | **C6** | — | — | 4 MB, used by C6 firmware only |
| USB-A host port | H3 (USB type-A jack), J2 in some sheets | **P4** | USB-OTG / USB 2.0 HS | `DM` / `DP` (P4 pins 50/51 = `USBD_N`/`USBD_P`) | VBUS = `VBUS_OUT`, gated by U2 (DIO7003HEST5). **See §3 — VBUS is NOT GPIO-controllable.** |
| USB-C (UART debug / power-in) | H5 USB-C, U5 CH343P USB-to-UART bridge | **P4** (UART), board (power) | UART via CH343P; VBUS feeds VCC_5V | CH343P UART → P4 UART0 (`RTS`/`DTR` also wired to reset/boot circuit) | Type-C VBUS = `USB0_5V` → through Q2 (AO3401 P-FET, driven by U13 LMBT3906DW1T1G) → `VCC_5V` |
| µSD slot | SD1 (microSD card) | **P4** | SDIO 1/4-bit | `GPIO39..GPIO44` (D0/D1/D2/D3/CLK/CMD); `SD1_VDD` gated by Q1 (AO3401) driven by **GPIO45** | Card-power switchable from P4 (`GPIO45` high → SD_VDD off because Q1 is P-FET, low → SD_VDD on) |
| MIPI-DSI display connector | J1 (15-pin RPi-Pi4B ribbon) | **P4** | MIPI D-PHY | `DSI_D0_P/N`, `DSI_D1_P/N`, `DSI_CLK_P/N` (P4 pins 38-43), `DSI_REXT` | I²C SDA/SCL on connector pins 11/12 = `ESP_I2C_SDA`/`ESP_I2C_SCL` = **GPIO7/GPIO8** (shared with codec + CSI) |
| MIPI-CSI camera connector | J3 (15-pin RPi cam ribbon) | **P4** | MIPI D-PHY | `CSI_D0_P/N`, `CSI_D1_P/N`, `CSI_CLK_P/N` (P4 pins 44-49), `CSI_REXT` | I²C shared with DSI (GPIO7/GPIO8). `CSI_IO0` pulled up 10K |
| Audio codec | U10 ES8311 (mono I²S) | **P4** | I²C control + I²S audio | I²C: GPIO7/GPIO8; I²S MCLK/SCLK/ASDOUT/LRCK/DSDIN = **GPIO13/12/11/10/9** | Codec CE strapped via R53 10K to AGND |
| Audio PA | U12 NS4150B mono Class-D | **P4** | enable line | `PA_CTRL` = **GPIO53** (R71 0R series) | Drives 2-pin speaker header H4 |
| On-board MIC | MIC1 (analog) | **P4** | via codec U10 | — | ES8311 MIC_P/N input |
| Ethernet PHY | U11 IP101GRI (10/100M) | **P4** | **RMII direct** (see §4) | RMII data + MDIO on GPIO~17/27-31/49-52 area; PHY 25 MHz xtal Y3 | RJ45 = HBJ-6117ANL (mag-jack with PoE pins broken out on header P4) |
| PoE pass-through header | POUT1 (2-pin) + P4 4-pin header | passive | — | — | Spare RJ45 pairs (RJ12, RJ36, RJ45-pin8, RJ78) brought out. **No PoE-PD/regulator on the board itself**; needs external module wired to `VCC_5V` |
| RTC battery | 1-cell coin via Schottky D2 (B5819WS) | **P4** | `ESP_VBAT` rail | — | Maintains P4 RTC domain when main 3V3 is off |
| 32.768 kHz crystal | Y2 (3225 SMD) | **P4** | XTAL32K | **GPIO0** / **GPIO1** via 0R R32/R36 | These GPIOs are sacrificed to RTC — do not use as digital I/O |
| 40 MHz main crystal | Y1 (3225 SMD) | **P4** | `XTAL_P`/`XTAL_N` | — | Standard main clock |
| BOOT button | Key1 (with ED1 ESD diode) | **P4** | strap | **GPIO35** | Active-low to GND, no on-board pull-up shown (P4 internal pull-up assumed) |
| RESET button | Key2 (with ED2 ESD diode) | **P4** | reset | `ESP_EN` (CHIP_PU) | R84 = 10K pull-up to `ESP_VBAT`; C121 = 100 nF debounce |
| Status LEDs | LED1 (USB-host overcurrent flag), D2 (on `RTC BAT` sheet — power good), plus PHY link LEDs on RJ45 | various | — | LED1 ← U2 `FLG` only; PHY link/activity from PHY_AD0/AD3 strapping pins | No software-controllable user LED on a P4 GPIO is visible in the schematic |
| User GPIO headers | P1, P2 (2×13, 2.54 mm, "Black straight needle") | **P4** + a few C6 lines | — | P1 breaks out ~24 P4 GPIOs (incl. GPIO2-8, GPIO20-27, GPIO32-38, GPIO51-54). P2 carries GPIO0/1/3/45-48/53/54 **plus** `C6_U0RXD`, `C6_U0TXD`, `C6_IO9`, `C6_IO12`, `C6_IO13` | C6 UART **and** spare C6 GPIOs are externally probable/floatable through P2 |
| Test/programming pads | LSK1..LSK4 (0R headers) | unknown | — | — | Unpopulated 0R pads; usage from silkscreen unclear |
| JTAG | not present as a dedicated connector | — | — | — | No JTAG header in the schematic. USB-OTG/USB-C debug only |

> The schematic does **not** show an on-board SDR or any extra USB peripheral —
> the USB-A jack (H3) is the sole host port. Any SDR/dongle is external.

---

## 2. P4 ↔ C6 interconnect

Re-verified 2026-05-23 from the schematic PDF via `pdftohtml -xml` and
spatial net-label alignment. There are **eight wires** between the
P4 and the C6, and **no UART** wires between them.

| Function | Signal on schematic | C6 pin | C6 native | P4 GPIO | Series R | Notes |
|---|---|---|---|---|---|---|
| C6 reset (active-high enable) | `C6_CHIP_PU` | 8  | EN     | **GPIO54** | R54 = 0Ω | P4 holds C6 in reset by driving GPIO54 low. |
| C6 boot strap | `C6_IO2`     | 5  | IO2    | **GPIO6**  | R52 = 0Ω | Drive low across a GPIO54 reset edge to force ROM bootloader. |
| SDIO CMD | `GPIO19`         | 24 | IO18   | **GPIO19** | R21 = 1K | |
| SDIO CLK | `GPIO18`         | 25 | IO19   | **GPIO18** | R20 = 1K | |
| SDIO D0  | `GPIO14`         | 26 | IO20   | **GPIO14** | (1K)\*   | |
| SDIO D1  | `GPIO15`         | 27 | IO21   | **GPIO15** | R19 = 1K | |
| SDIO D2  | `GPIO16`         | 28 | IO22   | **GPIO16** | R18 = 1K | |
| SDIO D3  | `GPIO17`         | 29 | IO23   | **GPIO17** | R16 = 1K | |

All six SDIO lines also have 51K pull-ups to `ESP_3V3` near the C6 module.

\* GPIO14's series resistor designator isn't clearly readable in the
PDF spatial dump; the 1K value matches the rest of the bus and the
empirical link is stable.

### NOT wired between P4 and C6

| Signal | C6 pin | Goes to (NOT the P4) |
|---|---|---|
| `C6_U0TXD` | 31 (IO16) | Header P2 + on-board USB-UART bridge (PROG_C6 console). R15 = 1K is in series on the U0RXD line toward the bridge; an earlier read of this doc that placed R15 on a P4 GPIO was incorrect. |
| `C6_U0RXD` | 30 (IO17) | Same path. |
| `C6_IO9`   | —         | C6's own BOOT button + header P2 only — not connected to any P4 GPIO. |
| `C6_IO8, IO12, IO13` | — | Header P2 only. |

This matches `p4-usb-host/sdkconfig`:

```
CONFIG_ESP_HOSTED_SDIO_PIN_CMD=19
CONFIG_ESP_HOSTED_SDIO_PIN_CLK=18
CONFIG_ESP_HOSTED_SDIO_PIN_D0=14
CONFIG_ESP_HOSTED_SDIO_PIN_D1=15
CONFIG_ESP_HOSTED_SDIO_PIN_D2=16
CONFIG_ESP_HOSTED_SDIO_PIN_D3=17
```

…and the empirically working `esp_hosted` + `esp_wifi_remote`
transport on commit `bba7817` onwards.

Mapping summary: this is the **standard Espressif ESP-Hosted-MCU C6
SDIO-slave topology** — P4 = host, C6 = SDIO slave at native voltage
(3.3 V). The two 0Ω strap resistors (R52/R54) let the P4 reset the
C6 and force ROM bootloader entry, though without UART pins the P4
**cannot** complete an `esp-serial-flasher` flow to the C6; SDIO-side
slave OTA or the PROG_C6 USB cable are the only firmware-update
paths.

**Boot/power sequencing.** The C6 starts when:

1. `VCC_5V` comes up → U3 produces `ESP_3V3`.
2. P4 boots from internal/external flash and runs application code.
3. P4 application releases `GPIO54` high (or never drives it low) →
   `C6_CHIP_PU` released → C6 boots from its own internal flash and
   the factory `esp_hosted` slave runs.

If the P4 hangs with `GPIO54` low, the C6 stays in reset. Conversely
the P4's own CHIP_PU (`ESP_EN`) is driven only by the RESET button
and the CH343P's RTS/DTR (USB-CDC reset glue, via the discrete U6
transistor on the USB-to-UART sheet); the C6 has no hardware path
to reset the P4.

---

## 3. USB-A host VBUS — control path

**Load-bearing answer:** the USB-A host VBUS rail (`VBUS_OUT`, on USB-A jack
H3 VBUS pin) is driven by **U2 = DIO7003HEST5**, a Diodes Inc. USB power
switch with current-limit and `FLG` output. Its pinout in this schematic:

| U2 pin | Net | Notes |
|---|---|---|
| IN | `VCC_5V` | Board 5V rail |
| OUT | `VBUS_OUT` | Direct to USB-A jack VBUS |
| EN | **pulled up to `VCC_5V` via R2 = 10K** | No GPIO is wired here |
| FLG | `VCC_5V` via R1 = 5.1K, lights LED1 | Open-drain fault indicator; **not wired to any GPIO** |
| GND | GND | |

Two near-by passives that look like they might enable software control:

- **R11**: marked `NC` (depopulated) — would be a pull-down on EN if fitted.
- **R12**: marked `NC/0R` (build-time option, currently NC) — would tie EN to
  OUT, i.e. self-latching. As shipped this position is unpopulated.

**Conclusion:** as built, the USB-A host VBUS is **hard-enabled the moment
`VCC_5V` is present**. There is **no GPIO that gates U2/EN**, and `FLG` is
LED-only — it is not wired back to the P4 either. Consequences for firmware:

- `usb_host_lib_set_root_port_power(false)` (or equivalent USB-OTG-internal
  VBUS bit clear) will only signal the P4's USB controller; the
  downstream device (e.g. an RTL-SDR dongle plugged into H3) **will keep
  receiving 5 V** from U2.
- The only way to cut power to the dongle in software is to remove power
  from the entire `VCC_5V` rail, which is not feasible while the board is
  self-powered.
- A hardware mod is needed if you require software VBUS cycling: depopulate
  R2, fit a pull-down at R11, and wire EN to a free P4 GPIO via a series
  resistor. There is no copper-side flex to do this without modification.
- U2 will still current-limit (DIO7003 typical limit ≈ 1 A) and assert FLG
  → LED1 on a fault, but the P4 has no visibility of this.

---

## 4. Ethernet PHY

- **U11 = IP101GRI** (IC+/ICplus 10/100M PHY, RMII interface)
- **RJ45 = HBJ-6117ANL** integrated magnetics jack with PoE bob-smith
  termination, mounted on the RJ45 sheet. Spare pairs (RJ12 / RJ36 /
  RJ45 / RJ78) broken out on header P4 + POUT1 — no on-board PoE-PD
  controller.
- **25 MHz clock**: crystal Y3 (3225 SMD) on `25M_XI`/`25M_XO` directly to
  U11.
- ESD: D3 = RClamp0524PATCT on the four TD/RD differential pairs.

### Drive path — P4 direct RMII

The PHY is driven by the **ESP32-P4 via RMII**, not through the C6 and not
over Wi-Fi. (This corrects a previous note that suggested a C6 bridge.)
RMII signals on P4 GPIO:

| RMII line | U11 pin | P4 net |
|---|---|---|
| TXD0 | 26 | GPIO34-ish (label `TXD0` on the PHY side, routed to P4 RMII pins) |
| TXD1 | 25 | (P4 RMII) |
| TXEN | 5 | `TXEN` net → P4 |
| RXD0 | 18 | `RXD0` → P4 |
| RXD1 | 17 | `RXD1` → P4 |
| RXER | 20 | `RXER` → P4 GPIO (5.1K pull-up to PHY_3V3 via R62) |
| CRS_DV | 21 | `CRS` → P4 |
| 50M_CLKO (REF_CLK out) | 24 | drives P4 EMAC_RX_CLK / REF_CLK input |
| MDIO | 22 | `MDIO` → P4 |
| MDC | 23 | `MDC` → P4 |
| COL | 4 | `COL` → P4 |
| PHY_AD0/AD3 | 11/12 | LED + strap pins; pulled by R63/R64 etc. (set PHY MDIO address) |
| ISET | 13 | R73 6.2K ±1% to AGND |

The PHY's `REF_CLK_OUT` (50 MHz / 50M_CLKO) drives the P4 EMAC, i.e. the
**PHY is the REF_CLK master** (clock-out mode). The P4 must therefore be
configured as RMII slave for the reference clock. The exact P4 GPIO
numbers for each RMII line are partly inferred from net-label routing in
the dump; confirm against the Waveshare board-support package's
`emac_init()` GPIO map before relying on specific numbers in firmware.

---

## 5. Power & rail summary

| Rail | Source | Approx. budget | Notes |
|---|---|---|---|
| `USB0_5V` | USB-C VBUS (H5) | 1.5 A USB-C | Through LTVS16H5.0ET5G ESD |
| `VCC_5V` | `USB0_5V` via Q2 (AO3401 P-FET, driven by U13 LMBT3906DW1T1G + R82/R83 sense network) | 5 V system, ~3 A | OR-ing diode-OR replacement; auto-enables when USB-C plugged in. Also reachable from PoE header for external mods. |
| `ESP_3V3` | U3 MP1658GTF-Z buck (3 A max) | 3.3 V system | Main digital rail; FB divider sets exactly 3.3 V |
| `ESP_VDD_HP` | U4 MP1605GTF-Z buck (3.5 A max) | 1.2 V core | Enabled by P4's own `EN_DCDC` pin (no GPIO); FB ~470K÷470K |
| `PHY_3V3` / `A3V3` | U9 RT9193-33PB LDO | 3.3 V, low-noise | Powers PHY and audio analog |
| `SD1_VDD` | `ESP_LDO_VO4` (P4 internal LDO) via Q1 AO3401 gated by GPIO45 | 3.3 V SD card | Switchable from firmware |
| `VDD_FLASH` | `ESP_LDO_VO1` (P4 internal LDO) | 3.3 V | Powers the GD25Q128 SPI flash |
| `ESP_VBAT` | RTC battery via D2 (B5819WS Schottky), with `ESP_3V3` also OR'd in | ~3.0–3.3 V | Maintains P4 RTC when main 3V3 off |

---

## 6. Strapping / boot pins worth knowing

- **P4 BOOT** = GPIO35 (Key1). Pull low at reset to enter ROM downloader.
- **P4 RESET** = ESP_EN (Key2). Active-low; also driven by CH343P's auto-reset
  glue (DTR/RTS through U6 LMBT3906 transistor) so `esptool` reset works.
- **C6 BOOT** = C6_IO2 = P4 GPIO6. Pull low while pulsing C6_CHIP_PU
  (P4 GPIO54) low → high to push the C6 into ROM bootloader for direct
  flashing.
- **PHY MDIO address** = PHY_AD0/AD3 strapped via R63 (5.1K) / R64 (NC, so
  effectively pulled to PHY_3V3 by R63) — gives PHY MDIO address 0
  (assuming AD0=low, AD3=low at strap-latch time; verify on the assembled
  board because some R-positions are marked NC/5.1K alternates).

---

## 7. Headers (P1, P2, P4) — summary

- **P1** (2×13, 2.54 mm, left): pin 1=`ESP_3V3`, pin 2=`VCC_5V`. Remaining 24
  pins are all **P4 GPIOs** (no C6 lines); roughly GPIO2-8, GPIO20-27,
  GPIO32-38, GPIO51-52. For the exact pin-to-GPIO list, read the on-board
  silkscreen — the PDF labels overlap and aren't reliable.
- **P2** (2×13, 2.54 mm, right): pin 1=`ESP_3V3`/`VCC_5V`, pin 2=`ESP_LDO_VO4`.
  Carries P4 GPIO0/1/3/45-48/53/54 **plus C6 lines** `C6_U0RXD`, `C6_U0TXD`,
  `C6_IO9`, `C6_IO12`, `C6_IO13` — so the C6's debug UART is externally
  reachable without going through the P4.
- **P4** (1×4 yellow header near RJ45): RJ12, RJ36, RJ45-spare, RJ78 (RJ45
  unused pairs). **No PoE-PD on board** — external module required.

---

## 8. Non-obvious gotchas

1. **GPIO0 and GPIO1 are consumed by the 32.768 kHz RTC crystal** (Y2 via 0R
   R32/R36). Treat them as unusable for GPIO unless you depopulate R32/R36
   and accept loss of RTC slow-clock accuracy.
2. **ESP_I2C bus is shared four ways**: codec U10, DSI connector J1
   (pins 11/12), CSI connector J3 (pins 13/14), and broken out on P1.
   With three 2.2K pull-ups in parallel (R48/R50 + codec internal), bus
   capacitance is already non-trivial; adding more I²C devices on P1
   warrants checking rise-time.
3. **The USB-C jack H5 is power-in only for normal use**, but the CC1/CC2
   resistors (R3/R4 = 5.1K each to GND) make it advertise as a USB **sink**
   (UFP). It cannot supply VBUS upstream. The USB-C data lines go to the
   **CH343P UART bridge (U5)**, not to the P4 directly — the P4's HS USB-OTG
   PHY is wired only to the USB-A jack H3. So the P4 cannot do USB device
   mode via USB-C; for USB device mode you'd need to repurpose the H3 jack
   in OTG mode (which still goes through U2's always-on VBUS switch).
4. **PA enable on GPIO53**: leave it low at boot to avoid speaker pop on
   power-up. The codec also has its own muting via I²C registers, but
   `PA_CTRL` is the hardware-level mute.
5. **No JTAG header**. Debug is via USB-Serial-JTAG over the P4's USB-OTG
   pins (the USB-A jack), or via the CH343P UART. Both routes share the
   single USB-C / USB-A pair on the front of the board.
6. **No POE PD controller** despite the PoE-friendly mag-jack. The
   `POUT1` 2-pin header and `P4` 4-pin header expose the spare RJ45 pairs
   for an external PoE module to inject VBUS — they do **not** feed
   `VCC_5V` directly without that external module.
7. **PHY 50 MHz reference clock direction**: U11 is configured as REF_CLK
   *master* (PHY drives REF_CLK *out*). P4 EMAC must be configured for
   external REF_CLK (`emac_rmii_clock_mode = EMAC_CLK_EXT_IN` in ESP-IDF
   terms). Driving REF_CLK from the P4 instead will brick the link.
8. **RTC battery rail (`ESP_VBAT`)** is OR'd from both the coin cell and
   `ESP_3V3` via the Schottky D2 — so the battery is **trickle-floated**
   whenever the board is powered, with no proper charge control. Use a
   non-rechargeable coin cell, or fit a current-limit resistor in series
   with the battery holder.
