// rotate_to_dc.c — see header for design and why this lives in
// common/iridium_decoder/ rather than as two local copies.

#include "rotate_to_dc.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void rotate_to_dc(int16_t *iq, int n_complex, double phase_step)
{
    // float (single-precision) cosf/sinf: P4 has only a single-
    // precision FPU (RV-32IMAFC), so double trig would be soft-
    // emulated. Single-precision epsilon (~1e-7) is well below the
    // Q15 quantisation floor (~3e-5 for amplitude 32767), so the
    // numerical accuracy difference vs the host's prior double
    // implementation is negligible. NMSE re-validated against gri at
    // step rebuild — see tests/host/test_pipeline_wideband_albq.
    const float dphi_f = (float)phase_step;
    for (int k = 0; k < n_complex; k++) {
        float phase = dphi_f * (float)k;
        float cs = cosf(phase);
        float ss = sinf(phase);
        int32_t r = iq[k * 2 + 0];
        int32_t v = iq[k * 2 + 1];
        float nr = (float)r * cs - (float)v * ss;
        float ni = (float)r * ss + (float)v * cs;
        if (nr >  32767.0f) nr =  32767.0f;
        if (nr < -32768.0f) nr = -32768.0f;
        if (ni >  32767.0f) ni =  32767.0f;
        if (ni < -32768.0f) ni = -32768.0f;
        iq[k * 2 + 0] = (int16_t)lrintf(nr);
        iq[k * 2 + 1] = (int16_t)lrintf(ni);
    }
}

double rotate_to_dc_phase_step_from_bin(int center_bin, int fft_size)
{
    double rel_f = ((double)center_bin - (double)fft_size / 2.0)
                   / (double)fft_size;
    return -2.0 * M_PI * rel_f;
}
