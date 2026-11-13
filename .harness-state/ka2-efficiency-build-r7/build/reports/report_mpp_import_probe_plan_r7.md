# One-buffer DMA-BUF import probe plan — r7

**Purpose:** Determine whether the exact MPP runtime accepts one genuine synthetic DMA-BUF as `MPP_BUFFER_TYPE_EXT_DMA`. This is a prerequisite probe only; it does not encode a frame or measure performance.

## Target and environment fingerprint

- Board/image: Firefly AIO-3588Q HDMI / RK3588, Ubuntu 24.04.4 LTS, Linux `5.10.198`, `aarch64`.
- Merged candidate binary for later same-binary comparison: `/tmp/ka2-efficiency-r7-mergecheck/system/loggerd/encoderd`, SHA-256 `c30b920d13b072d271122408822192e6e427b8ecac4ed289e3cecd64b7d37b47`, Build ID `a956b9960bf8e8ebb08b7e560570ff4c85449b1d`.
- Runtime library: `/lib/aarch64-linux-gnu/librockchip_mpp.so.1` → `/usr/lib/aarch64-linux-gnu/librockchip_mpp.so.0`, SHA-256 `ea762b7da71b63a73cdf14d053d84b091654aee5e5c4f26ceb1449b8fb8d92f9`, Build ID `b333b160bf1f80ad4f935d9794d2b8956b90bb14`. `LD_LIBRARY_PATH` and `LD_PRELOAD` are unset for the run.
- Readiness status: fresh idle/device checks pass; init-only HEVC context init/destroy passed. `MemAvailable` is about 2.03 GB, but `CmaFree=0` and `nr_free_cma=0`; do not use the CMA heap or start the A/B on this evidence alone.
- The probe binary will be compiled as a small C++ harness against `/usr/local/include/rockchip/rk_mpi.h` and explicitly linked to the exact `/lib/...so.1` library; `ldd -r` must confirm that exact runtime before any execution. A header/runtime version skew remains a limitation.

## Single probe workload

1. Recheck the live process list and `fuser` on MPP/render/video/V4L2 nodes immediately before execution. Proceed only if there is no competing camera, loggerd capture, encoderd, sensor, MPP, or user-space RGA activity; ignore only kernel `irq/*` workers with empty cmdlines and known non-recording deleter/uploader services.
2. Open `/dev/dma_heap/system` and allocate exactly one NV12 buffer for 1920×1200 (`1920 × 1200 × 3/2 = 3,456,000` bytes; page-rounded allocation must stay below 4 MiB). Do not fall back to `/dev/dma_heap/cma` while CMA free is zero.
3. Map the buffer and fill it with a deterministic changing byte pattern. Prove the fd is a DMA-BUF using its `/proc/self/fd` identity and a successful `DMA_BUF_IOCTL_SYNC`; do not use the ordinary VisionIPC memfd (`/dev/shm/msgq_visionbuf_*`).
4. Create/init one HEVC MPP context using the exact loaded library. Call `mpp_buffer_import` once with `MPP_BUFFER_TYPE_EXT_DMA`, fd, and size. If that fails, make at most the production-compatible single retry with `info.ptr` set. Record both return codes and whether an imported handle was returned.
5. If import succeeds, call `mpp_buffer_put`, destroy the MPP context, unmap and close the heap fd. Submit no frame, create no output stream/file, and do not start encoderd/loggerd/camerad/sensord or IPC.

## Explicit ceilings and abort conditions

- Exactly one allocation, no larger than 4 MiB; at most two import calls (fd/size attempt plus one pointer retry); zero encode frames; zero output bytes; no Params writes or media/production-root writes.
- One process with a 10-second wall-clock ceiling, then TERM and a 2-second KILL grace. Run once only.
- Abort before start if any prohibited process/device user appears, `/dev/dma_heap/system` is absent, the heap allocation would exceed 4 MiB, `MemAvailable < 1.5 GB`, `/data` free space <1 GB, or the exact loaded library check differs.
- Abort immediately on DMA-BUF identity/sync failure, either MPP context init failure, both import attempts failing, timeout, unexpected kernel/MPP error, cleanup failure, or a new competing process/device user after the probe. Preserve logs; do not retry or widen the workload in this turn.
- `CmaFree=0` is recorded as a full-A/B limitation. This one small `system`-heap import probe is permitted under the 4 MiB and 1.5 GB MemAvailable ceilings; it does not approve A/B.

## Decision after probe

- **Import fails:** stop before any A/B. Report the exact DMA-BUF type, heap, library and return codes. The candidate's zero-copy import is not usable on this synthetic input path; keep copy default and return the A/B question to build-management/Admiral.
- **Import succeeds:** this proves only that the exact library can wrap one `system`-heap DMA-BUF. It does not prove encoder acceptance, DMA completion lifetime, decoded content, resource headroom, or a speedup. Save a separate matched A/B plan with fixed workload, warm-up/pairing/sample counts, output/resource ceilings and abort criteria, and verify its actual input fd type/import-success counters before its first speed sample. Continue to keep `ENCODER_ZERO_COPY=0` default.
