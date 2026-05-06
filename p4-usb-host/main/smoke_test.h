#ifndef SMOKE_TEST_H
#define SMOKE_TEST_H

// Target-side functional smoke test. Bypasses USB ingestion and feeds
// synthetic noise+tone+noise IQ through the same convert -> ingest ->
// dsp_processor path the production code uses, then asserts a single
// burst was detected at a non-DC, non-edge FFT bin with sensible SNR.
//
// Runs in place of normal app behaviour when CONFIG_SMOKE_TEST_MODE=y.
// Logs SMOKE_PASS or SMOKE_FAIL at the end and parks the CPU.
//
// The test does NOT require any USB device to be attached.
void smoke_test_run(void);

#endif
