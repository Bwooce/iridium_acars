#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

// Ring buffer of recent decoded ACARS messages. Capacity is fixed at
// MSG_RING_CAPACITY; oldest entry is overwritten when full. Producer
// is frame_decoder's try_acars() (called on Core 1); consumer is the
// HTTP /messages handler (any core). A single mutex guards both.
//
// `id` is a monotonically-increasing message counter. Clients can poll
// /messages?since=ID to get only newer entries.

#define MSG_RING_CAPACITY  32          // ~9 KB at 280 B/entry (PSRAM-allocated)
#define MSG_RING_TXT_MAX   256         // ACARS message text cap

typedef struct {
    uint64_t id;                       // monotonic, 1-based; 0 = empty slot
    uint64_t timestamp_us;             // boot-relative microseconds (esp_timer_get_time)
    bool     uplink;                   // direction (true = GND→AIR)
    char     mode;                     // ACARS mode byte (printable or '?')
    char     label[3];                 // 2-char label + NUL
    char     block_id;                 // single block-id byte or '?'
    char     msg_num[5];               // 4-char msgnum + NUL
    char     flight_id[7];             // 6-char flight ID + NUL
    bool     crc_ok;                   // libacars-reported CRC status
    int32_t  peak_bin;                 // tagger peak bin (informational)
    float    snr_db;                   // tagger SNR at detection (informational)
    char     txt[MSG_RING_TXT_MAX];    // NUL-terminated payload text; may be empty
} acars_msg_t;

// One-shot init. Allocates the ring in PSRAM. Idempotent.
void msg_ring_init(void);

// Push a fully-populated message. The producer fills the acars_msg_t
// on the caller's stack and msg_ring assigns the id atomically inside.
void msg_ring_push(const acars_msg_t *m);

// Snapshot: copy up to `cap` entries newer than `since_id` into `out`.
// Returns the count written. Entries are written oldest-first. Use
// returned[count-1].id as the next `since_id`.
size_t msg_ring_snapshot(uint64_t since_id, acars_msg_t *out, size_t cap);

// Total messages observed since boot (== last id assigned). 0 if none yet.
uint64_t msg_ring_total(void);
