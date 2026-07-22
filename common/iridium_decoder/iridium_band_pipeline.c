// See iridium_band_pipeline.h. Pure adapter — no state, no processing.

#include "iridium_band_pipeline.h"

#include <stddef.h>
#include "burst_pipeline.h"
#include "burst_prefilter.h"
#include "qpsk_demod.h"
#include "uw_correlator.h"
#include "esp_log.h" // host builds pick up tests/host/esp_log.h stub

// unused attr: the host esp_log stub compiles ESP_LOGD away entirely.
static const char *TAG __attribute__((unused)) = "IR_BANDP";

static bool ir_prefilter(const int16_t *iq250, int n_complex, int width_bins,
                         band_prefilter_verdict_t *v)
{
    burst_prefilter_result_t pf;
    bool accept = burst_prefilter(iq250, n_complex, width_bins, &pf);
    v->accept   = pf.accept;
    v->width_ok = pf.width_ok;
    v->dur_ok   = pf.dur_ok;
    v->snr_ok   = pf.snr_ok;
    return accept;
}

// Shim translating burst_pipeline's per-frame callback into the generic
// band_frame_t one. Ownership of frame.bits / frame.soft_bits passes
// straight through to the outer callback (band_pipeline.h contract ==
// burst_pipeline.h contract), so nothing is freed here.
typedef struct {
    band_frame_cb_t cb;
    void           *ctx;
} ir_shim_ctx_t;

static void ir_shim_cb(burst_pipeline_result_t *res, void *vctx)
{
    ir_shim_ctx_t *s = (ir_shim_ctx_t *)vctx;

    // Per-frame D13/UW diagnostic (was worker_core1.c's worker_emit_frame
    // preamble; lives with the Iridium result type now that the worker is
    // band-generic). Debug level — compiled/filtered out in production.
    ESP_LOGD(TAG, "D13 start=%d  UW dir=%s off=%d corr=%.3f SNR=%.1f omega=%.3f",
             res->burst_start,
             res->uw_res.direction == UW_DIR_DOWNLINK ? "DL" : res->uw_res.direction == UW_DIR_UPLINK ? "UL"
                                                                                                      : "??",
             res->uw_res.uw_offset,
             (double)res->uw_res.correction,
             (double)res->uw_res.snr_estimate_db,
             (double)res->uw_res.omega_per_sym);

    band_frame_t f = {
        .bits        = res->frame.bits,
        .n_bits      = res->frame.n_bits,
        .soft_bits   = res->frame.soft_bits,
        .direction   = (int)res->frame.direction,
        .demod_ok    = res->demod_ok,
        .snr_db      = res->frame.snr_db,
        .band_detail = res,
    };
    s->cb(&f, s->ctx);
}

static int ir_process_burst(int16_t *iq250, int n_complex,
                            band_frame_cb_t cb, void *ctx)
{
    ir_shim_ctx_t shim = {.cb = cb, .ctx = ctx};
    return burst_pipeline_process_burst(iq250, n_complex, ir_shim_cb, &shim);
}

const band_pipeline_t *iridium_band_pipeline(void)
{
    static const band_pipeline_t p = {
        .name          = "iridium",
        .prefilter     = ir_prefilter,
        .process_burst = ir_process_burst,
    };
    return &p;
}
