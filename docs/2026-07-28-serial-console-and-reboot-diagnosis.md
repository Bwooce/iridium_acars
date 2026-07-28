# Reading the P4 serial console on macOS + diagnosing reboots — 2026-07-28

Operational notes from a session that lost hours to a self-inflicted "serial
console is garbage" scare. Two independent things bit us; both are documented
here so the next person (or agent) skips the detour. The device was healthy the
whole time.

## 1. The console is 115200 — everywhere

`p4-usb-host/sdkconfig.defaults` pins `CONFIG_ESP_CONSOLE_UART_BAUDRATE=115200`
and `serial_cmd.c` sets UART0 to 115200 at init (both in lockstep, on purpose).
The old #18 "console at 921600" experiment was reverted — there is no 921600 in
committed source. Read at **115200**.

## 2. macOS: `stty` + `cat` does NOT hold the baud — use pyserial

The trap: `stty -f /dev/cu.usbmodem… <baud>` followed by a **separate** `cat`
does not persist the baud on macOS. The port reverts to **9600** the moment the
`stty` fd closes, so `cat` reads at 9600 and every line garbles **regardless of
the baud you asked for**. Proof: after `stty … 115200` (or 921600),
`stty … -a` reports `speed 9600`.

This is NOT a CH34x/CH343 framing limit (an old memory claimed it was — false).
pyserial, which sets the baud inside the same fd it reads from, reads 115200
cleanly (and 921600 too).

**Do this:**
```
PORT=/dev/cu.usbmodem5B5E1316491 scripts/serial_logger.sh        # pyserial @115200 on macOS
scripts/serial_logger.sh status         # confirm it's up + log path
scripts/serial_logger.sh stop
```
`serial_logger.sh` was fixed (commit 46e2116) to use pyserial on Darwin and
`stty`+`cat` on Linux (where settings DO hold across the `cat` open). Ensure
**one reader only** — two readers split the byte stream into mutual garbage;
`lsof "$PORT"` to check.

**Muting:** `uart_log` defaults AUTO and mutes `ESP_LOG` to a no-op once WiFi is
up, so a live console looks nearly idle. Un-mute without a reboot:
`POST /uartlog?on=1` (or serial `set uart_log 1`); `?auto=1` restores.

## 3. sdkconfig drift pitfall (the second half of the scare)

`p4-usb-host/sdkconfig` is gitignored and **persistent**: sdkconfig-defaults only
seed a *fresh* config, so a stale value survives forever. Ours had drifted to
`CONFIG_ESP_CONSOLE_UART_BAUDRATE=921600` (leftover from #18) while committed
defaults said 115200 — so flashed images booted their **bootloader / early
console at 921600** (runtime is still 115200 because `serial_cmd` forces it).
Committed `sdkconfig.defaults` is the source of truth; if the local `sdkconfig`
drifts, realign it (set the value, or delete the line and reconfigure) and
reflash. Fixed + reflashed 2026-07-28.

## 4. Diagnosing reboots — check reset_reason, don't guess

There is a persistent iot_log receiver on the Mac
(`~/dev/esp-iot-log/python/esp_iot_log_receiver.py → /tmp/iot_log.log`, UDP
:4210). It logs a `BOOT reset_reason=…` line on **every** boot. Check it before
theorising (in particular, do NOT reach for the stale "~24.7-min autotune panic"
story):

```
grep "BOOT reset_reason" /tmp/iot_log.log | tail
```

Reading the reason:
- **SW (3)** — deliberate `esp_restart()`: health-watchdog recovery, `POST /reboot`,
  or an OTA. Not a crash.
- **POWERON (1)** — power / EN-pin reset. A DTR/RTS→EN pulse from a serial
  port-opener counts (this is exactly how `flash.sh` auto-resets), as does an
  actual power blip.
- **PANIC (4)** — a real firmware crash; there will be a guru-meditation /
  backtrace / abort nearby. These have happened historically but are rare now.

The reboots seen this session were flash-cycle resets (POWERON) plus one SW
restart — no crashes.

## 5. Post-flash dongle miss (unrelated but recurring)

After a flash/reset the RTL-SDR sometimes does not re-enumerate — `/status`
shows `usb.completed=0`, `stream_live=false`. Firmware cannot power-cycle the
dongle (no switchable VBUS), so the fix is a clean `POST /reboot`; the stream
comes back (`usb.completed` climbs, `stream_live=true`). If it persists across
reboots, reseat the dongle.
