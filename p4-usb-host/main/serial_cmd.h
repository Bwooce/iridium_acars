#pragma once

// Minimal UART0 NVS command interface.  Call once from app_main() to
// start a background task that reads lines from the CH343 serial port
// and dispatches set/get commands to app_config.  Useful when the
// board is in AP-mode or WiFi credentials have been lost.
//
// Commands (one per line, \n or \r\n):
//   set <key> <value>  — write key to NVS and update live config
//   get <key>          — print current value
//   config             — dump all keys
//   reboot             — esp_restart()
//
// Keys: wifi_ssid  wifi_psk  out_host  out_port  station_id
//       lo_hz  rate_hz  gain_mode  gain_dbx10  bias_tee  tag_thr  ota_url

void serial_cmd_init(void);
