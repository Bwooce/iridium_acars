#pragma once
#include <stdint.h>
#include <stdbool.h>

// SNTP wall-clock helper.
//
// The rest of the firmware runs on boot-relative esp_timer time (acars_msg_t
// timestamp_us). The airframes.io feed needs a REAL epoch timestamp — the
// dumpvdl2 schema carries t.sec/usec (Unix epoch) and the iridium-toolkit
// schema an ISO8601 string — so we run SNTP once the STA has an IP.
//
// Until the first sync completes, net_time_synced() is false and callers MUST
// omit the timestamp rather than emit a boot-relative (wrong) one: a wrong
// wall-clock is worse than an absent one for a downstream aggregator.

// Start SNTP. Idempotent and cheap to call on every IP_EVENT_STA_GOT_IP —
// only the first call initialises the client; later calls are no-ops.
void net_time_start(void);

// True once SNTP has set the system clock at least once this boot.
bool net_time_synced(void);

// Wall-clock microseconds since the Unix epoch, or 0 if not yet synced.
int64_t net_time_epoch_us(void);
