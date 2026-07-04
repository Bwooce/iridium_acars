#ifndef SD_CAPTURE_FRAMING_H
#define SD_CAPTURE_FRAMING_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Atomic burst-record framing decision (T4, commit fixing
// sd_capture_record_burst_begin's DEFECT-A-style partial-enqueue risk
// for burst mode). Each burst record is `hdr_size` bytes of header
// followed by `length_samples * 2 * sizeof(int16_t)` bytes of
// interleaved int16 IQ. xStreamBufferSend() can partially enqueue
// under back-pressure, and a torn header or a truncated IQ tail would
// desync the "header + length_samples*4 bytes" framing for every
// burst after it in the file -- a parser has no way to tell how much
// of a promised payload actually arrived. The fix is all-or-nothing:
// either the whole record fits in the stream buffer's free space right
// now, or the whole record is dropped (zero bytes enqueued) and the
// next burst starts clean.
//
// Pulled out as pure, dependency-free arithmetic (no ESP/FreeRTOS
// headers) so it can be exercised by a host-side unit test even though
// sd_capture.c itself only builds on-device (FreeRTOS stream buffers /
// SDMMC). sd_capture_record_burst_begin() calls these two functions
// for its size computation and fit decision; do not reintroduce the
// arithmetic inline there.

// Total bytes a burst record of `length_samples` complex samples
// occupies in the stream: header + interleaved int16 I/Q.
static inline size_t sd_capture_burst_record_bytes(size_t   hdr_size,
                                                   uint32_t length_samples)
{
    return hdr_size + (size_t)length_samples * 2 * sizeof(int16_t);
}

// Whether a record of `hdr_size` + `length_samples` worth of IQ fits
// WHOLE in `free_space` bytes of stream-buffer headroom. Returns
// false if the record — even by a single byte — exceeds what's
// currently free, so the caller can drop it atomically rather than
// send a partial record.
static inline bool sd_capture_burst_fits(size_t free_space, size_t hdr_size,
                                         uint32_t length_samples)
{
    return free_space >= sd_capture_burst_record_bytes(hdr_size, length_samples);
}

#endif // SD_CAPTURE_FRAMING_H
