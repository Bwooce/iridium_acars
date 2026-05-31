# IDF patches

Patches against the vendored ESP-IDF (gitignored at `esp-idf/`).
Apply after cloning / updating the IDF:

```sh
cd esp-idf
git apply ../patches/0001-esp_dma_utils-defer-stash-alloc-until-overflow-confirmed.patch
```

## 0001 — esp_dma_utils: defer stash alloc until overflow is confirmed

**File:** `components/esp_driver_dma/src/esp_dma_utils.c`
**IDF version:** v6.1 (commit the vendored copy tracks)

`esp_dma_split_rx_buffer_to_cache_aligned()` allocated 128 bytes from
the DMA-INT heap (`MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL`) unconditionally
on every call, even when src/dst/len were already cache-line aligned and
the stash buffer would never be written to.

At 625 transfers/second (our 4.88 MB/s at 8 KB/transfer) this fragmented
the tiny (~5 KB) DMA-INT heap over ~6 hours until `heap_caps_calloc`
could no longer satisfy a 128-byte request, producing `stash_alloc_fails`
and `audio_dropped` events in `signal_buffer.c`.

Fix: compute head/tail overflow lengths before the allocation decision.
The stash is only allocated when at least one overflow is non-zero.
For aligned buffers the DMA-INT heap is never touched.
