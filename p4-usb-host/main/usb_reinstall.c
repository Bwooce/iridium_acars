// See usb_reinstall.h. Shared request/result state between the HTTP handler
// (POST /usbreinstall), the daemon orchestrator (usb_host_lib_main.c), and the
// class_driver quiesce path.
#include "usb_reinstall.h"

#include <stdatomic.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static atomic_bool            s_pending  = false;
static usb_reinstall_result_t s_result;
static SemaphoreHandle_t      s_done_sem = NULL;

// Lazily create the completion semaphore. Called from the HTTP task (request/
// wait) before the daemon ever touches it, so there's no create race in
// practice; guarded anyway.
static void ensure_done_sem(void)
{
    if (!s_done_sem) {
        s_done_sem = xSemaphoreCreateBinary();
    }
}

void usb_reinstall_request(void)
{
    ensure_done_sem();
    if (s_done_sem) (void)xSemaphoreTake(s_done_sem, 0); // drop any stale completion
    atomic_store(&s_pending, true);
}

bool usb_reinstall_pending(void)
{
    return atomic_load(&s_pending);
}

void usb_reinstall_clear_pending(void)
{
    atomic_store(&s_pending, false);
}

void usb_reinstall_report(const usb_reinstall_result_t *res)
{
    if (res) s_result = *res;
    if (s_done_sem) xSemaphoreGive(s_done_sem);
}

bool usb_reinstall_wait(usb_reinstall_result_t *out, uint32_t timeout_ms)
{
    ensure_done_sem();
    if (!s_done_sem) return false;
    if (xSemaphoreTake(s_done_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) return false;
    if (out) *out = s_result;
    return true;
}
