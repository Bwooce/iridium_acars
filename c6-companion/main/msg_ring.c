#include "msg_ring.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

static SemaphoreHandle_t s_mu = NULL;

static irp_acars_msg_t s_acars[MSG_RING_CAP];
static int             s_acars_head = 0;   // next write index
static int             s_acars_count = 0;  // entries currently held (≤ CAP)
static uint32_t        s_acars_total = 0;
static uint32_t        s_acars_last_seq = 0;

static irp_status_snap_t s_status[STATS_RING_CAP];
static int               s_status_head = 0;
static int               s_status_count = 0;
static uint32_t          s_status_total = 0;

static uint64_t s_boot_at_ms = 0;

void msg_ring_init(void)
{
    if (s_mu) return;
    s_mu = xSemaphoreCreateMutex();
    s_boot_at_ms = (uint64_t)(esp_timer_get_time() / 1000);
}

void msg_ring_push_acars(const irp_acars_msg_t *m)
{
    if (!s_mu || !m) return;
    xSemaphoreTake(s_mu, portMAX_DELAY);
    s_acars[s_acars_head] = *m;
    s_acars_head = (s_acars_head + 1) % MSG_RING_CAP;
    if (s_acars_count < MSG_RING_CAP) s_acars_count++;
    s_acars_total++;
    s_acars_last_seq = m->seq;
    xSemaphoreGive(s_mu);
}

void msg_ring_push_status(const irp_status_snap_t *s)
{
    if (!s_mu || !s) return;
    xSemaphoreTake(s_mu, portMAX_DELAY);
    s_status[s_status_head] = *s;
    s_status_head = (s_status_head + 1) % STATS_RING_CAP;
    if (s_status_count < STATS_RING_CAP) s_status_count++;
    s_status_total++;
    xSemaphoreGive(s_mu);
}

int msg_ring_snapshot_acars(irp_acars_msg_t *out, int max, uint32_t since_seq)
{
    if (!s_mu || !out || max <= 0) return 0;
    int n = 0;
    xSemaphoreTake(s_mu, portMAX_DELAY);
    // Walk OLDEST → NEWEST and pick those with seq > since_seq.
    int idx = (s_acars_head - s_acars_count + MSG_RING_CAP) % MSG_RING_CAP;
    for (int i = 0; i < s_acars_count && n < max; i++) {
        if (s_acars[idx].seq > since_seq) {
            out[n++] = s_acars[idx];
        }
        idx = (idx + 1) % MSG_RING_CAP;
    }
    xSemaphoreGive(s_mu);
    return n;
}

int msg_ring_snapshot_status(irp_status_snap_t *out, int max)
{
    if (!s_mu || !out || max <= 0) return 0;
    int n = 0;
    xSemaphoreTake(s_mu, portMAX_DELAY);
    // Newest first: walk backwards from head.
    int idx = (s_status_head - 1 + STATS_RING_CAP) % STATS_RING_CAP;
    for (int i = 0; i < s_status_count && n < max; i++) {
        out[n++] = s_status[idx];
        idx = (idx - 1 + STATS_RING_CAP) % STATS_RING_CAP;
    }
    xSemaphoreGive(s_mu);
    return n;
}

bool msg_ring_get_latest_status(irp_status_snap_t *out)
{
    if (!s_mu || !out) return false;
    xSemaphoreTake(s_mu, portMAX_DELAY);
    bool has = (s_status_count > 0);
    if (has) {
        int idx = (s_status_head - 1 + STATS_RING_CAP) % STATS_RING_CAP;
        *out = s_status[idx];
    }
    xSemaphoreGive(s_mu);
    return has;
}

void msg_ring_get_stats(msg_ring_stats_t *out)
{
    if (!out) return;
    xSemaphoreTake(s_mu, portMAX_DELAY);
    out->acars_total    = s_acars_total;
    out->status_total   = s_status_total;
    out->last_acars_seq = s_acars_last_seq;
    out->boot_at_ms     = s_boot_at_ms;
    xSemaphoreGive(s_mu);
}
