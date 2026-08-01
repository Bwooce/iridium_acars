// dsp_processor — wideband burst detector. Thin shim around
// fft_burst_tagger (common/iridium_decoder/), which runs the gr-iridium
// wideband spectrogram detector against the full 2.5 MSPS input
// subband. The Phase 3.6.M cutover (commit bf0bb9f) replaced an
// earlier single-FFT detector and a polyphase channelizer with this
// gri-aligned path; both predecessors were removed from the tree.
// Each tagged burst carries an exact relative-frequency tag (FFT
// bin → Hz), so the worker's rotation is sub-bin precise and the
// downstream burst_pipeline PLL only tracks residual scatter.
//
// #120: state moved from file-scope statics into a heap-allocated
// dsp_processor_t handle so the module is reentrant. The current
// firmware creates exactly one and threads the handle through
// feed/flush/stats; cross-task diagnostic readers use
// dsp_processor_default(). No PIE buffers live here (the tagger owns
// its own FFT scratch), so the handle is heap-safe — none of the
// project_heap_position_decode_bug placement concerns apply.

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdatomic.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "dsp_processor.h"
#include "fft_burst_tagger.h"
#include "worker_core1.h"
#include "app_config.h"
#include "band_profile.h" // per-band tagger parameters (VHF/VDL2 foundation)
#include "band_select.h"  // band_runtime_resolve — resolve band once (phase 3)
#include "poa_frontend.h" // POA CHANNELIZED front end (tagger-bypass, P2)
#include "frame_decoder.h" // frame_decoder_push_poa — POA block -> decoder task (P3)

static const char *TAG = "DSP_PROC";

// Tagger threshold over the EMA baseline.
//
// Our threshold scale sits ~4 dB BELOW gr-iridium's for the same
// detection behaviour (ENBW/normalisation difference in our tagger;
// see fft_burst_tagger.c:131-141). Measured 2026-07-06 on the ALBQ
// fixture, where gri's reference 65 bursts were produced at gri-18 dB
// (apps/iridium-extractor:130 default):
//   ours-14 -> 57/65 decoded;  ours-18 -> 38/65 (over-suppressed).
// So gri-parity default = 14 on our scale.
//
// History: this was 10.0f from 2026-05-24 to 2026-07-06, chosen by a
// host fixture-recall sweep (10 dB scored 60/65 vs 57/65 at 14). That
// +3-frame fixture gain violated the "every numeric comes from gri"
// rule and cost a ~3000x noise false-positive rate live: ~730 junk
// bursts/s on bench RF, saturating the worker (>=99.9% of all bursts
// shed, ~0.66/s processed) and engaging the frozen-baseline latch.
// The 2026-07-06 live A/B (P1 squelch/force-close + threshold) is in
// docs/superpowers/ANALYSIS-2026-07-06-path-to-first-acars.md.
// NVS `tag_thr` overrides this default at detector create.
#define FBT_THRESHOLD_DB 14.0f

// Burst window padding in INPUT samples (at FS_DETECT_HZ). gri's
// defaults: pre = 2*fft_size = 4096, post = sample_rate * 16e-3 =
// 40000. Same numbers work here because FS_DETECT_HZ matches gri's
// nominal 2.5 MSPS.
#define FBT_BURST_PRE_LEN (2 * FBT_FFT_SIZE)
#define FBT_BURST_POST_LEN 40000

// Burst width in FFT bins (= half an Iridium channel). 40 kHz /
// (2.5 MSPS / 2048) ≈ 32 bins.
#define FBT_BURST_WIDTH 32

#define FBT_NEW_BUF_SIZE 16
#define FBT_GONE_BUF_SIZE 16

// VHF/VDL2 foundation: the constants above are now the IRIDIUM band
// profile's values (band_profile.c); dsp_processor_create pulls them
// from the profile selected by the NVS band switch. These asserts are
// the compile-time proof that band=iridium (the default) configures the
// tagger bit-identically to the historical hard-coded path.
_Static_assert(BAND_IRIDIUM_FBT_PRE_LEN == FBT_BURST_PRE_LEN,
               "iridium band profile pre-pad must match FBT_BURST_PRE_LEN");
_Static_assert(BAND_IRIDIUM_FBT_POST_LEN == FBT_BURST_POST_LEN,
               "iridium band profile post-pad must match FBT_BURST_POST_LEN");
_Static_assert(BAND_IRIDIUM_FBT_WIDTH_BINS == FBT_BURST_WIDTH,
               "iridium band profile burst width must match FBT_BURST_WIDTH");
_Static_assert(BAND_IRIDIUM_FS_HZ == FS_DETECT_HZ,
               "iridium band profile fs must match FS_DETECT_HZ");
// (The float FBT_THRESHOLD_DB == BAND_IRIDIUM_TAG_THR_DB identity can't
// be a _Static_assert — not an integer constant expression; it is pinned
// by tests/host/test_band_profile.c instead.)

// Detector instance. Formerly a bag of file-scope statics (#120).
struct dsp_processor {
    fft_burst_tagger_t *tagger;
    int32_t            *baseline_history; // PSRAM, 4 MB
    burst_detected_cb_t user_cb;

    // POA (BAND_FE_CHANNELIZED): when set, the tagger/baseline_history are
    // NULL and feed() routes to this always-on channelizer+demod instead of
    // the burst path (docs/2026-08-01-poa-onband-plan.md §1). Mutually
    // exclusive with `tagger`.
    poa_frontend_t     *poa_fe;
    _Atomic uint32_t    poa_blocks; // ACARS blocks decoded (P2 counter; P3 delivers)

    // Detect-path sample rate from the selected band profile. Both
    // current bands run 2.5 MSPS (== FS_DETECT_HZ, statically asserted
    // for iridium above), so today this only feeds the bin→Hz mapping
    // and the create log; a future band at a different rate changes it
    // here without touching the callers.
    uint32_t fs_hz;

    // Accumulator for chunks smaller than FBT_FFT_SIZE complex samples.
    // dsp_processor_feed receives variable-length buffers from class_driver
    // (typically ~8000 complex after the 125/128 resample of a 16 KB USB
    // transfer), so we batch into FFT_SIZE-aligned chunks.
    int16_t accum[2 * FBT_FFT_SIZE] __attribute__((aligned(16)));
    int     accum_n; // complex samples currently in accum

    // Absolute sample index of the NEXT chunk we'll feed to the tagger.
    // Matches signal_buffer's head modulo SIGNAL_BUF cap.
    uint64_t next_sample_idx;

    // Diagnostic accumulators (reset by get_stage_stats once per second).
    // T48: dsp_processor_feed()/process_chunk() (the writers) now run on
    // the dsp_feed task while dsp_processor_get_stage_stats() (the
    // reader+resetter) runs on usb_pump — before the split both were the
    // same task, so plain volatile was safe w.r.t. this specific
    // reader/writer pair. Now that a higher-priority task (pump, prio
    // pump_prio) can preempt the writer (dsp_feed, prio pump_prio-1)
    // mid read-modify-write, a 64-bit accumulator could tear on this
    // 32-bit core. _Atomic + relaxed ordering (matches usbring.c /
    // sd_capture.c's existing idiom) costs nothing on the decode path —
    // these are diagnostics-only — and removes the tear.
    _Atomic(uint64_t) acc_step_us;
    _Atomic(uint32_t) acc_input_samples;
    _Atomic(uint32_t) acc_new_bursts;
    _Atomic(uint32_t) acc_gone_bursts;

    // Non-resetting cumulative counter for callers that compute their own
    // deltas (e.g. /diag/dsp_health's 2-second window — #127). Readers
    // snapshot at t0/t1 and subtract. Same cross-task tearing concern as
    // above post-T48; _Atomic removes it.
    _Atomic(uint64_t) total_input_samples;

    // Scanner density counters (Phase 1). Tapped in dispatch_gone_burst,
    // read-and-reset by dsp_processor_read_reset_density from the serial_cmd
    // task. Same relaxed-atomic idiom as the diagnostics above.
    _Atomic(uint32_t) acc_nb_bursts;   // width <= DSP_NARROWBAND_MAX_BINS
    _Atomic(uint32_t) acc_dens_bursts; // all gone bursts in the window
    _Atomic(uint64_t) acc_snr_milli;   // sum of SNR(dB) * 1000

    // P1.5 same-instant gone-burst coalescer (NON-GRI heuristic; see
    // dispatch_gone_batch). 0/1 = disabled (default). Snapshot of the
    // NVS "coal_n" config value at create.
    uint8_t           coalesce_min;
    _Atomic(uint32_t) acc_coalesced; // gone bursts suppressed by the coalescer
};

// Process "default" instance for cross-task diagnostic getters that
// can't be handed the owner's handle (httpd /diag/dsp_health). Set by
// create(). The firmware only ever makes one detector.
static dsp_processor_t *s_default = NULL;

// Push one gone burst out to the user callback. Converts fbt_burst_t
// (FFT bin space, uint64 sample idx) into detected_burst_t (signed
// rel_freq_hz, uint32 sample idx for signal_buffer).
//
// Dispatch fires on GONE events (not new) to match gr-iridium's
// tagged_burst_to_pdu_impl.cc:205-227 semantics. The gone event
// carries `stop = last_active + burst_post_len`, so the burst
// length passed to the worker is variable — long enough for
// multi-frame bursts where gri's handle_multiple_frames_per_burst
// would publish the whole PDU.
static void dispatch_gone_burst(dsp_processor_t *p, const fbt_burst_t *b)
{
    if (!p->user_cb) return;

    int   signed_bin  = b->center_bin - FBT_FFT_SIZE / 2;
    float rel_freq_hz = (float)signed_bin * (float)p->fs_hz / (float)FBT_FFT_SIZE;

    // length = stop - start, variable per burst. Clamp at uint32 max
    // to be safe; the worker further clamps to WB_EXTRACT_MAX.
    uint64_t length     = b->stop - b->start;
    uint32_t length_u32 = length > 0xFFFFFFFFu
                              ? 0xFFFFFFFFu
                              : (uint32_t)length;

    // fft_burst_tagger's magnitude_db is ALREADY the SNR
    // (10·log10(mag² · HISTORY / baseline_sum) — see
    // fft_burst_tagger.c). Don't subtract noise_db.
    detected_burst_t out = {
        .start_sample_idx = b->start, // T44: keep full 64-bit cumulative index
        .length_samples   = length_u32,
        .rel_freq_hz      = rel_freq_hz,
        .peak_snr_db      = b->magnitude_db,
        .magnitude_db     = b->magnitude_db,
        .noise_db         = b->noise_db,
        // T60: pack center_bin (low 16) + width_bins (high 16) — keeps
        // detected_burst_t byte-identical (no PIE-position perturbation).
        .peak_bin = BURST_PACK_BIN_WIDTH(b->center_bin, b->width_bins),
    };

    atomic_fetch_add_explicit(&p->acc_dens_bursts, 1u, memory_order_relaxed);
    if (b->width_bins > 0 && b->width_bins <= DSP_NARROWBAND_MAX_BINS)
        atomic_fetch_add_explicit(&p->acc_nb_bursts, 1u, memory_order_relaxed);
    atomic_fetch_add_explicit(&p->acc_snr_milli,
                              (uint64_t)(int64_t)(b->magnitude_db * 1000.0f),
                              memory_order_relaxed);

    p->user_cb(&out);
}

// Dispatch a batch of gone bursts (one tagger step's worth, or the
// flush residue), optionally coalescing same-instant multi-bin events
// first.
//
// P1.5 companion heuristic — NON-GRI: gr-iridium has no equivalent
// (its tagged_burst_to_pdu dispatches every gone burst). Bench soak
// 2026-07-06/07: the junk load is impulsive and band-wide — ONE
// impulse tags MANY narrow bursts with near-identical start times
// across the band, each of which costs the worker a triage pass.
// When >= coalesce_min gone bursts share a start within one FFT step
// (FBT_FFT_SIZE = 2048 samples), keep only the strongest
// (magnitude_db) and drop the rest, counting them in acc_coalesced
// (`fbt: coal=` in the status log). A real Iridium burst occupies ONE
// channel, so a same-instant many-bin cluster is interference by
// construction — but simultaneous bursts on different channels from
// different satellites ARE physically possible, which is why this
// ships DEFAULT OFF (NVS coal_n = 0) for an explicit A/B.
//
// Scope limit: within one gone batch only (stateless). Cluster
// members whose gone events straddle a step boundary are not
// coalesced — acceptable for an off-by-default load-shedder aimed at
// single-impulse events whose members share one last_active step.
static void dispatch_gone_batch(dsp_processor_t *p, const fbt_burst_t *b, int n)
{
    if (p->coalesce_min >= 2 && n >= (int)p->coalesce_min) {
        bool drop[FBT_GONE_BUF_SIZE] = {false};
        for (int i = 0; i < n; i++) {
            if (drop[i]) continue;
            int idx[FBT_GONE_BUF_SIZE];
            int m = 0;
            for (int j = 0; j < n; j++) {
                if (drop[j]) continue;
                uint64_t d = b[i].start > b[j].start ? b[i].start - b[j].start
                                                     : b[j].start - b[i].start;
                if (d < FBT_FFT_SIZE) idx[m++] = j; // includes j == i
            }
            if (m < (int)p->coalesce_min) continue;
            int best = idx[0];
            for (int k = 1; k < m; k++) {
                if (b[idx[k]].magnitude_db > b[best].magnitude_db) best = idx[k];
            }
            for (int k = 0; k < m; k++) {
                if (idx[k] != best) drop[idx[k]] = true;
            }
        }
        for (int i = 0; i < n; i++) {
            if (drop[i]) {
                atomic_fetch_add_explicit(&p->acc_coalesced, 1u,
                                          memory_order_relaxed);
                continue;
            }
            dispatch_gone_burst(p, &b[i]);
        }
        return;
    }
    for (int i = 0; i < n; i++)
        dispatch_gone_burst(p, &b[i]);
}

// Process one FFT-aligned chunk: hand it to the tagger, advance the
// sample index, dispatch gone-burst events.
static void process_chunk(dsp_processor_t *p, const int16_t *chunk_iq)
{
    fbt_burst_t new_bursts[FBT_NEW_BUF_SIZE];
    fbt_burst_t gone_bursts[FBT_GONE_BUF_SIZE];
    int         n_new  = FBT_NEW_BUF_SIZE;
    int         n_gone = FBT_GONE_BUF_SIZE;

    int64_t t0 = esp_timer_get_time();
    bool    ok = fft_burst_tagger_step(p->tagger, chunk_iq, NULL,
                                       new_bursts, &n_new,
                                       gone_bursts, &n_gone);
    int64_t t1 = esp_timer_get_time();
    atomic_fetch_add_explicit(&p->acc_step_us, (uint64_t)(t1 - t0), memory_order_relaxed);

    if (ok) {
        dispatch_gone_batch(p, gone_bursts, n_gone);
        atomic_fetch_add_explicit(&p->acc_new_bursts, (uint32_t)n_new, memory_order_relaxed);
        atomic_fetch_add_explicit(&p->acc_gone_bursts, (uint32_t)n_gone, memory_order_relaxed);
    }

    p->next_sample_idx += FBT_FFT_SIZE;
}

// End-of-stream flush. Forces any still-active bursts to emit their
// gone-event with stop = current d_index. Use at end of an offline
// fixture or when the SDR source closes; on a live RF feed, bursts
// naturally time out and reach the gone callback in the steady-state
// step loop — no flush needed.
void dsp_processor_flush(dsp_processor_t *p)
{
    if (!p || !p->tagger) return;
    fbt_burst_t flushed[FBT_GONE_BUF_SIZE];
    int         n = FBT_GONE_BUF_SIZE;
    fft_burst_tagger_flush(p->tagger, flushed, &n);
    dispatch_gone_batch(p, flushed, n);
    atomic_fetch_add_explicit(&p->acc_gone_bursts, (uint32_t)n, memory_order_relaxed);
    ESP_LOGI(TAG, "fbt flush: emitted %d residual bursts", n);
}

// POA channel set — P4 reads these from NVS `po_chans`; hardcoded for now
// (docs/2026-08-01-poa-onband-plan.md §5, LO 130.8 MHz). Index == poa_block chn.
static const uint32_t k_poa_chans[] = {131550000u, 130450000u, 130425000u, 130025000u};

// POA (CHANNELIZED) block callback — fires on the Core-0 feed task per decoded
// ACARS block (P3). Hands the block+CRC to the Core-1 decoder task via
// frame_decoder_push_poa (that task owns s_reasm_ctx/msg_ring, so acars_deliver
// must run there, not here). Also counts for the /status POA funnel.
static void poa_on_block(const poa_block_t *b, void *user)
{
    dsp_processor_t *p = (dsp_processor_t *)user;
    atomic_fetch_add_explicit(&p->poa_blocks, 1, memory_order_relaxed);
    uint32_t freq = (b->chn >= 0 &&
                     b->chn < (int)(sizeof(k_poa_chans) / sizeof(k_poa_chans[0])))
                        ? k_poa_chans[b->chn]
                        : 0;
    frame_decoder_push_poa(b->txt, b->len, b->crc, freq, /*stamp now=*/0, b->level_db);
}

static dsp_processor_t *dsp_create_channelized(burst_detected_cb_t cb,
                                               const band_profile_t *bp)
{
    dsp_processor_t *p = heap_caps_calloc(1, sizeof(*p), MALLOC_CAP_SPIRAM);
    if (!p) { ESP_LOGE(TAG, "dsp_processor (POA) handle alloc failed"); return NULL; }
    p->user_cb = cb; // unused under POA (no burst path)
    p->fs_hz   = bp->detect_fs_hz;
    int nch = (int)(sizeof(k_poa_chans) / sizeof(k_poa_chans[0]));
    p->poa_fe = poa_frontend_create(bp->detect_fs_hz, bp->default_lo_hz,
                                    k_poa_chans, nch, poa_on_block, p);
    if (!p->poa_fe) {
        ESP_LOGE(TAG, "poa_frontend_create failed");
        heap_caps_free(p);
        return NULL;
    }
    ESP_LOGI(TAG,
             "Creating POA channelized front end (band=%s, fs=%u Hz, lo=%u Hz, %d channels) — no tagger",
             bp->name, (unsigned)bp->detect_fs_hz, (unsigned)bp->default_lo_hz, nch);
    s_default = p; // publish for cross-task diagnostic readers
    return p;
}

dsp_processor_t *dsp_processor_create(burst_detected_cb_t cb)
{
    // Pull threshold from NVS-backed config (D18). Falls back to the
    // compile-time default if app_config wasn't initialised.
    app_config_t cfg;
    app_config_snapshot(&cfg);
    // Band profile (VHF/VDL2 foundation): tagger window/threshold
    // parameters come from the selected band. For band=iridium (the
    // default) every value equals the historical constants (see the
    // _Static_asserts above) — bit-identical behaviour.
    const band_runtime_t *rt  = band_runtime_resolve((band_id_t)cfg.band); // resolve once (phase 3)
    const band_profile_t *bp  = rt->profile;
    // POA and any future CHANNELIZED band bypass the burst tagger entirely
    // (no PSK bursts to detect) — build the always-on channelizer instead.
    if (bp->frontend == BAND_FE_CHANNELIZED)
        return dsp_create_channelized(cb, bp);
    float                 thr = cfg.tagger_threshold_db;
    if (thr <= 0.0f || thr > 30.0f) thr = bp->tagger_threshold_db; // sanity
    ESP_LOGI(TAG,
             "Creating wideband fft_burst_tagger (band=%s, N=%d, fs=%u Hz, thr=%.1f dB, coal_n=%u)",
             bp->name, FBT_FFT_SIZE, (unsigned)bp->detect_fs_hz, (double)thr,
             (unsigned)cfg.coalesce_min_bursts);

    // 4 MB baseline_history in PSRAM. Internal SRAM doesn't have room
    // (int32 × FFT_SIZE × HISTORY_SIZE = 2048 × 512 × 4 = 4 MB).
    size_t   bytes            = (size_t)FBT_FFT_SIZE * FBT_HISTORY_SIZE * sizeof(int32_t);
    int32_t *baseline_history = (int32_t *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
    if (!baseline_history) {
        ESP_LOGE(TAG, "baseline_history alloc %zu bytes (PSRAM) failed", bytes);
        return NULL;
    }

    // Allocate the 66 KB tagger struct BEFORE the ~8 KB handle: the handle
    // carved from the largest internal block leaves it just under the
    // tagger's size when the budget is tight (seen in the smoke build:
    // largest 73728 -> 65536 < 66672 -> create failed). Biggest-first
    // avoids the fragmentation; the handle fits any smaller fragment.
    fft_burst_tagger_t *tagger = fft_burst_tagger_init(bp->fbt_pre_len, bp->fbt_post_len,
                                                       bp->fbt_width_bins, thr,
                                                       baseline_history);
    if (!tagger) {
        ESP_LOGE(TAG, "fft_burst_tagger_init failed");
        heap_caps_free(baseline_history);
        return NULL;
    }

    // Handle (incl. accum[]) in PSRAM: reclaims ~8 KB DMA-INT so the USB
    // transfer pool can allocate. The 256 KB L2 cache caps the internal DMA
    // reserve at ~144 KB, which tagger(66KB)+init+pool overran (rate=0).
    // Placement is perf-only, not correctness — holds no PIE buffers; accum[]
    // is CPU-touched then memcpy'd, not PIE-read (~125 chunks/s). 2026-07-19.
    dsp_processor_t *p = heap_caps_calloc(1, sizeof(*p), MALLOC_CAP_SPIRAM);
    if (!p) {
        ESP_LOGE(TAG, "dsp_processor handle alloc (%zu bytes internal) failed",
                 sizeof(*p));
        fft_burst_tagger_destroy(tagger);
        heap_caps_free(baseline_history);
        return NULL;
    }
    p->user_cb          = cb;
    p->baseline_history = baseline_history;
    p->tagger           = tagger;
    p->fs_hz            = bp->detect_fs_hz;
    p->coalesce_min     = cfg.coalesce_min_bursts; // 0/1 = off (default)

    fft_burst_tagger_set_start(p->tagger, 0);
    fft_burst_tagger_set_dc_mask(p->tagger, cfg.dcmask_lo, cfg.dcmask_hi);
    // Measure-first VDL2 burst-tagger trace (/diag/tagger_trace). Armed ONLY
    // for band==vdl2 — the entire trace path self-gates on this flag, so
    // band==iridium is byte-identical and pays nothing.
    fft_burst_tagger_set_trace_enabled(rt->band == BAND_VDL2);
    s_default = p; // publish for cross-task diagnostic readers
    return p;
}

dsp_processor_t *dsp_processor_default(void)
{
    return s_default;
}

// Cross-task write of two int globals inside fft_burst_tagger (s_dc_mask_lo/
// s_dc_mask_hi), read once per FFT step by the Core-0 scan loop. Not mutex
// guarded: worst case one FFT step observes a half-updated window (one bound
// old, one new) before the next step sees both new values — harmless, since
// the window only gates new-burst declaration, not the noise-floor EMA.
void dsp_processor_apply_dc_mask(int16_t lo, int16_t hi)
{
    fft_burst_tagger_set_dc_mask(s_default ? s_default->tagger : NULL, lo, hi);
}

void dsp_processor_feed(dsp_processor_t *p, const int16_t *samples, size_t n_samples)
{
    // n_samples is complex IQ pairs. At FS_DETECT_HZ the typical USB
    // transfer (16 KB raw / 2 bytes per complex) is ~8000 complex
    // after the 125/128 resample in ingest_core1.
    if (!p) return;

    atomic_fetch_add_explicit(&p->acc_input_samples, (uint32_t)n_samples, memory_order_relaxed);
    atomic_fetch_add_explicit(&p->total_input_samples, (uint64_t)n_samples, memory_order_relaxed); // #127, never reset

    // POA CHANNELIZED path: convert int16 IQ -> float interleaved and drive
    // the channelizer (no tagger). Single-writer task, so a static scratch
    // buffer is safe. Chunked to bound stack/scratch.
    if (p->poa_fe) {
        static float fbuf[2 * 4096];
        size_t off = 0;
        while (off < n_samples) {
            size_t c = n_samples - off;
            if (c > 4096) c = 4096;
            for (size_t i = 0; i < c * 2; i++) fbuf[i] = (float)samples[off * 2 + i];
            poa_frontend_feed(p->poa_fe, fbuf, (int)c);
            off += c;
        }
        return;
    }
    if (!p->tagger) return;

    size_t off = 0;
    while (off < n_samples) {
        size_t space   = FBT_FFT_SIZE - (size_t)p->accum_n;
        size_t to_copy = n_samples - off;
        if (to_copy > space) to_copy = space;

        memcpy(p->accum + (size_t)p->accum_n * 2,
               samples + off * 2,
               to_copy * 2 * sizeof(int16_t));
        p->accum_n += (int)to_copy;
        off += to_copy;

        if (p->accum_n == FBT_FFT_SIZE) {
            process_chunk(p, p->accum);
            p->accum_n = 0;
        }
    }
}

uint32_t dsp_processor_get_poa_blocks(const dsp_processor_t *p)
{
    return p ? atomic_load_explicit(&p->poa_blocks, memory_order_relaxed) : 0;
}

void dsp_processor_get_stage_stats(dsp_processor_t *p, dsp_stage_stats_t *out)
{
    if (!p) {
        memset(out, 0, sizeof(*out));
        return;
    }
    // T48: read-and-reset each accumulator in one atomic_exchange so
    // there's no load-then-clear window a concurrent dsp_feed increment
    // could fall into (this getter runs on usb_pump; the increments run
    // on dsp_feed — see the struct's field comments). Snapshot every
    // value up front and derive the rest from the snapshot, matching the
    // original same-task semantics exactly.
    uint64_t acc_step_us_snap = atomic_exchange_explicit(&p->acc_step_us, 0, memory_order_relaxed);
    uint32_t acc_input_snap   = atomic_exchange_explicit(&p->acc_input_samples, 0, memory_order_relaxed);
    uint32_t acc_new_snap     = atomic_exchange_explicit(&p->acc_new_bursts, 0, memory_order_relaxed);
    uint32_t acc_gone_snap    = atomic_exchange_explicit(&p->acc_gone_bursts, 0, memory_order_relaxed);

    uint32_t frames          = acc_input_snap / FBT_FFT_SIZE;
    uint64_t tag_stage_us[5] = {0};
    uint32_t tag_steps       = 0;
    fft_burst_tagger_get_stage_us(tag_stage_us, &tag_steps);
    // Per-stage uncontended min (µs/step). Read-and-reset unconditionally
    // so the accumulator drains even in the frames==0 window; these are
    // already per-step floors — do NOT divide by frame/step count.
    uint64_t tag_min_us[5] = {0};
    fft_burst_tagger_get_stage_min_us(tag_min_us);

    if (frames == 0) {
        memset(out, 0, sizeof(*out));
    } else {
        float fn             = (float)frames;
        float ts             = (tag_steps > 0) ? (float)tag_steps : 1.0f;
        out->frames          = frames;
        out->wind_us         = (float)tag_stage_us[0] / ts;
        out->fft_us          = (float)tag_stage_us[1] / ts;
        out->mag_us          = (float)tag_stage_us[2] / ts;
        out->detect_us       = (float)tag_stage_us[3] / ts;
        out->baseline_us     = (float)tag_stage_us[4] / ts;
        out->total_us        = (float)acc_step_us_snap / fn;
        out->wind_min_us     = (float)tag_min_us[0];
        out->fft_min_us      = (float)tag_min_us[1];
        out->mag_min_us      = (float)tag_min_us[2];
        out->detect_min_us   = (float)tag_min_us[3];
        out->baseline_min_us = (float)tag_min_us[4];
    }
    // Raw accumulators for the `fbt:` line. No ESP_LOGI here: this
    // getter runs on usb_pump's 1 Hz snapshot and log formatting belongs
    // on Core 1 — status_logger emits the line from these fields.
    out->new_bursts  = acc_new_snap;
    out->gone_bursts = acc_gone_snap;
    out->step_us     = (uint32_t)acc_step_us_snap;
    out->tag_steps   = tag_steps;
    out->coalesced   = atomic_exchange_explicit(&p->acc_coalesced, 0, memory_order_relaxed);
    fft_burst_tagger_get_squelch_stats(&out->squelch_events,
                                       &out->squelch_dropped,
                                       &out->noise_resets);
}

void dsp_processor_read_reset_density(dsp_processor_t *p, dsp_density_t *out)
{
    if (!p || !out) return;
    uint32_t nb            = atomic_exchange_explicit(&p->acc_nb_bursts, 0u, memory_order_relaxed);
    uint32_t all           = atomic_exchange_explicit(&p->acc_dens_bursts, 0u, memory_order_relaxed);
    uint64_t smil          = atomic_exchange_explicit(&p->acc_snr_milli, 0u, memory_order_relaxed);
    out->narrowband_bursts = nb;
    out->all_bursts        = all;
    out->mean_snr_db       = all ? (float)((double)smil / 1000.0 / (double)all) : 0.0f;
}

void dsp_processor_reset_tagger_baseline(dsp_processor_t *p)
{
    if (!p || !p->tagger) return;
    fft_burst_tagger_reset_baseline(p->tagger);
}

// #127: race-free cumulative FFT-frames count. Callers that compute
// their own delta over a wall-clock window (e.g. /diag/dsp_health)
// must use this instead of dsp_processor_get_stage_stats — the latter
// resets its accumulator on every call, so any second reader (e.g.
// status_logger at 1 Hz) destroys the snapshot.
uint64_t dsp_processor_get_total_fft_frames(dsp_processor_t *p)
{
    if (!p) return 0;
    return atomic_load_explicit(&p->total_input_samples, memory_order_relaxed) / FBT_FFT_SIZE;
}
