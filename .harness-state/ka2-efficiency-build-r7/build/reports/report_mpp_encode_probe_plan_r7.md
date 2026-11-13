# One-frame MPP encode/output probe plan — r7

**Purpose:** Verify that the exact MPP runtime can encode one deterministic NV12 frame read from a genuine system-heap DMA-BUF, return a non-empty HEVC packet, and produce a stream that a software decoder can decode. This is readiness evidence only; it is not a performance sample and does not exercise camera, VisionIPC, encoderd, or production logging.

## Pinned target and frame layout

- Board: `kommu-870ce22d4093b701`, Firefly AIO-3588Q / RK3588, Ubuntu 24.04.4, Linux `5.10.198`, aarch64.
- Production comparison binary: merged-overlay `encoderd` SHA-256 `c30b920d13b072d271122408822192e6e427b8ecac4ed289e3cecd64b7d37b47`, Build ID `a956b9960bf8e8ebb08b7e560570ff4c85449b1d`.
- Runtime MPP: `/lib/aarch64-linux-gnu/librockchip_mpp.so.1`, SHA-256 `ea762b7da71b63a73cdf14d053d84b091654aee5e5c4f26ceb1449b8fb8d92f9`, Build ID `b333b160bf1f80ad4f935d9794d2b8956b90bb14`; no `LD_LIBRARY_PATH` or `LD_PRELOAD`.
- Geometry is pinned from `system/camerad/camera/common.cc` and `system/camerad/camera/rk.cc`: 1920×1200 NV12, Y and UV row strides 1920, UV offset 2,304,000 bytes, allocation size 3,456,000 bytes. This is already 16-pixel aligned, matching `MppEncoder`'s production alignment.
- Encoder settings mirror the production full-resolution HEVC road path: 1920×1200, NV12 (`MPP_FMT_YUV420SP`), 20 fps input/output, CBR target 10,000,000 bps, max 10,100,000 bps, min 9,900,000 bps, GOP 90, QP init/max/min 26/51/10, I-frame max/min 46/10, and QP IP 2.

## Single-probe procedure and ceilings

1. Recheck no camera, sensor, loggerd capture, encoderd, or competing user-space MPP/RGA process/device user exists. Ignore only known non-recording deleter/uploader services and empty-cmdline `irq/*` workers.
2. Require `MemAvailable >= 1.5 GiB`, `/data` free space `>= 1 GiB`, the exact merged binary/library hashes above, and the loaded library path above. Record `CmaFree`; it is currently zero and remains a stop condition for any CMA-heap use.
3. Allocate exactly one 3,456,000-byte buffer from `/dev/dma_heap/system` (never `/dev/dma_heap/cma`), map it, fill one deterministic NV12 image, and complete both DMA-BUF sync ioctls. Import via `MPP_BUFFER_TYPE_EXT_DMA`; at most one pointer retry is allowed, as in production.
4. Configure one HEVC MPP context to the pinned production settings, submit one image and one EOS marker to drain it, and require a non-empty packet. Keep the input DMA-BUF, map, and MPP handle alive through output collection.
5. Write VPS/SPS/PPS plus the single packet to one `/dev/shm` temporary stream, capped at 8 MiB. Require ffprobe to report exactly one decoded frame and ffmpeg to decode one frame with `-xerror`. Hash the temporary stream and record packet size, return codes, and decoder result; delete the stream immediately afterward.

Hard limits: one allocation under 4 MiB, one MPP context, one input frame, one EOS marker, at most two import calls, no more than 8 MiB temporary output, 10 seconds for the MPP probe and a 30-second total wall-clock ceiling including decode. No persistent video output, Params writes, IPC, camera/sensor access, or production-root writes. Do not retry at the same load after any allocation, import, encode, output, or decode failure.

Abort immediately if a prohibited process/device user appears, either resource threshold is crossed, the runtime/binary fingerprint differs, a DMA-BUF sync/import/config/submit call fails, no packet arrives within one second, output is empty or exceeds 8 MiB, the decoder does not report exactly one frame, or cleanup fails. Preserve logs and do not proceed to performance samples if any guard fails.

## Interpretation

A pass establishes only that one synthetic, system-heap DMA-BUF can reach the configured MPP HEVC encoder and yield one decodable frame on this board. It does not establish imported-buffer lifetime safety under concurrency, equivalence to a camera-exported buffer, correctness of the entire `MppEncoder`/encoderd path, loss rate, sustained behavior, or a speedup. A later same-binary A/B still needs a separate review of synthetic input delivery, production-path counters, decoded-content comparison against copy mode, resource ceilings, and the approved sample schedule.
