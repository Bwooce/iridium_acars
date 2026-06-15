#include "fault_inject.h"

#if CONFIG_FAULT_INJECT

#include <string.h>
#include "esp_log.h"

static const char *TAG = "FI";

// One pending-failure counter per site. volatile: written by the
// requesting task, read+decremented by hot paths incl. the URB-submit
// ISR. A racing decrement at worst fires one extra/fewer synthetic
// failure, which is harmless for a test harness.
static volatile uint32_t s_remaining[FI_SITE_COUNT];

static const char *const s_names[FI_SITE_COUNT] = {
    [FI_SITE_DMA_SUBMIT]      = "dma_submit",
    [FI_SITE_DMA_SUBMIT_WRAP] = "dma_submit_wrap",
    [FI_SITE_DISPATCH_QUEUE]  = "dispatch_queue",
    [FI_SITE_TAKE_CONVERTED]  = "take_converted",
    [FI_SITE_URB_SUBMIT]      = "urb_submit",
};

void fault_inject_request(fi_site_t site, uint32_t count)
{
    if (site < 0 || site >= FI_SITE_COUNT) return;
    s_remaining[site] += count;
    ESP_LOGW(TAG, "scheduled %u failure(s) at %s (pending=%u)",
             (unsigned)count, s_names[site], (unsigned)s_remaining[site]);
}

bool fault_inject_should_fail(fi_site_t site)
{
    if (site < 0 || site >= FI_SITE_COUNT) return false;
    if (s_remaining[site] == 0) return false;
    s_remaining[site]--;
    return true;
}

uint32_t fault_inject_remaining(fi_site_t site)
{
    if (site < 0 || site >= FI_SITE_COUNT) return 0;
    return s_remaining[site];
}

fi_site_t fault_inject_site_from_name(const char *name)
{
    if (!name) return FI_SITE_COUNT;
    for (int i = 0; i < FI_SITE_COUNT; i++) {
        if (s_names[i] && strcmp(name, s_names[i]) == 0) return (fi_site_t)i;
    }
    return FI_SITE_COUNT;
}

const char *fault_inject_site_name(fi_site_t site)
{
    if (site < 0 || site >= FI_SITE_COUNT) return 0;
    return s_names[site];
}

#endif // CONFIG_FAULT_INJECT
