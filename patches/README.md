# IDF patches

Patches against the vendored ESP-IDF (gitignored at `esp-idf/`).
Apply after cloning / updating the IDF:

```sh
cd esp-idf
git apply ../patches/0001-esp_dma_utils-defer-stash-alloc-until-overflow-confirmed.patch
```

## 0001 — async_memcpy_gdma: place link list descriptors in PSRAM

**File:** `components/esp_driver_dma/src/async_memcpy_gdma.c`
**IDF version:** v6.1 — vendored checkout tracks `release/v6.1` (was master snapshot; upgraded 2026-07-04). Re-verify this patch applies on IDF updates.

`gdma_new_link_list` was called with `items_in_ext_mem = false`, placing
GDMA descriptor arrays in the DMA-INT heap (`MALLOC_CAP_DMA |
MALLOC_CAP_INTERNAL`). That heap is only ~5 KB on ESP32-P4. Combined
with the unconditional stash buffer allocation in `esp_dma_split_rx_buffer_to_cache_aligned`
(128 bytes per call, also from DMA-INT), the heap was exhausted and
fragmented after ~6 hours at 625 DMA transfers/sec, producing
`stash_alloc_fails` and `audio_dropped` events in `signal_buffer.c`.

Fix: set `items_in_ext_mem = true` for both TX and RX link lists.
The ESP32-P4's AXI-GDMA can fetch descriptors from PSRAM over the AXI
bus — that is the design intent of the AXI-GDMA (vs the AHB-GDMA which
is SRAM-only). The IDF code already has the full SPIRAM allocation +
L2-cache-sync + non-cached-address path for this case; the `false`
setting was a conservative TODO left by Espressif.

This moves descriptor arrays to PSRAM (unlimited headroom), leaving
the 5 KB DMA-INT heap exclusively for the stash buffer (128 bytes per
unaligned transfer, rare for our cache-line-aligned paths).
