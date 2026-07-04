#ifndef SIGNAL_BUFFER_RING_H
#define SIGNAL_BUFFER_RING_H

#include <stdint.h>

// Ring index invariant (T2, commit 933ab38: "signal_buffer: preserve ring
// index invariant on wrap-path DMA drop"). `head` MUST advance by
// aligned_count on every signal_buffer_push() exit path — including the
// wrap-path DMA-submit-failure recovery branches — because downstream
// code (signal_buffer_read_chunk, signal_buffer_burst_valid, and the
// worker's mapping of a tagger's cumulative start_sample_idx into a ring
// offset) all compute `some_cumulative_index % total_cap` against this
// same head. An exit path that skips the advance leaves head behind the
// producer's true cumulative sample count forever — a permanent desync,
// not a one-burst glitch.
//
// Pulled out as a pure, dependency-free function (no ESP/FreeRTOS
// headers) so it can be exercised by a host-side unit test even though
// signal_buffer.c itself only builds on-device. signal_buffer.c's single
// head-update exit site calls this function; do not reintroduce a
// bare `head = (head + n) % total_cap` computed inline anywhere else, or
// a future early-return exit path could once again skip the advance.
static inline uint32_t signal_buffer_next_head(uint32_t head,
                                               uint32_t aligned_count,
                                               uint32_t total_cap)
{
    return (head + aligned_count) % total_cap;
}

#endif // SIGNAL_BUFFER_RING_H
