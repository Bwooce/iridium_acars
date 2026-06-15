// Synthetic fault injection for exercising the recovery-counter paths
// (#122). Lets a test schedule N failures at a named site; the site's
// hot path calls fault_inject_should_fail() and takes its error branch
// when a synthetic failure is pending.
//
// Gated by CONFIG_FAULT_INJECT (Kconfig, default off). When off, the
// hooks are `static inline` no-ops that the optimiser folds away, so
// production builds pay exactly zero cost and the DSP path is bit-identical.
//
// Backing storage is a volatile uint32_t per site — safe to decrement
// from ISR context (the URB-submit site fires in the USB completion ISR).

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FI_SITE_DMA_SUBMIT = 0,  // signal_buffer_push esp_async_memcpy (simple path)
    FI_SITE_DMA_SUBMIT_WRAP, // signal_buffer_push esp_async_memcpy (wrap-path first call, #107)
    FI_SITE_DISPATCH_QUEUE,  // ingest_core1 dispatch xQueueSend (#110/#106 backpressure)
    FI_SITE_TAKE_CONVERTED,  // ingest_core1_take_converted semaphore wait (#110)
    FI_SITE_URB_SUBMIT,      // esp_libusb stream resubmit (#124 pool-lost)
    FI_SITE_COUNT,
} fi_site_t;

#if CONFIG_FAULT_INJECT

// Schedule `count` synthetic failures at `site` (accumulates with any
// already pending). Safe to call from any task.
void fault_inject_request(fi_site_t site, uint32_t count);

// Hot-path check: returns true (and decrements the pending count) when a
// synthetic failure is scheduled for `site`, else false. ISR-safe.
bool fault_inject_should_fail(fi_site_t site);

// Pending count for `site` (0 if site out of range). For the endpoint.
uint32_t fault_inject_remaining(fi_site_t site);

// Map a site name ("dma_submit", "dispatch_queue", ...) to its enum, or
// FI_SITE_COUNT if unknown. Names match the enum minus the FI_SITE_ prefix,
// lowercased.
fi_site_t fault_inject_site_from_name(const char *name);

// Canonical lowercase name for a site (NULL if out of range).
const char *fault_inject_site_name(fi_site_t site);

#else // CONFIG_FAULT_INJECT off — zero-cost stubs

static inline void fault_inject_request(fi_site_t site, uint32_t count)
{
    (void)site;
    (void)count;
}
static inline bool fault_inject_should_fail(fi_site_t site)
{
    (void)site;
    return false;
}
static inline uint32_t fault_inject_remaining(fi_site_t site)
{
    (void)site;
    return 0;
}
static inline fi_site_t fault_inject_site_from_name(const char *name)
{
    (void)name;
    return FI_SITE_COUNT;
}
static inline const char *fault_inject_site_name(fi_site_t site)
{
    (void)site;
    return 0;
}

#endif // CONFIG_FAULT_INJECT
