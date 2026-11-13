# One-frame synthetic HEVC encode/output probe — r7

## Outcome

The bounded probe passed. A synthetic 1920×1200 NV12 image in a genuine `/dev/dma_heap/system` DMA-BUF was imported by the exact board MPP runtime, submitted to one HEVC encoder context using the production full-resolution road settings, and drained through EOS. MPP returned a non-empty 11,617-byte picture packet plus 83 bytes of VPS/SPS/PPS. `ffprobe` counted exactly one frame and `ffmpeg -xerror` decoded one frame successfully. MPP cleanup succeeded, the transient 11,700-byte stream was deleted from `/dev/shm`, and no MPP/DRM/DMA device users remained afterward.

This is a readiness check, not a performance measurement. It exercises the MPP API directly with settings copied from the production encoder. It does not execute the merged `encoderd` binary or prove `MppEncoder` integration, imported-buffer lifetime under concurrent streams, camera-export buffer compatibility, frame-loss behavior, or sustained zero-copy performance.

## Exact target and configuration

- Merged comparison target: `encoderd` SHA-256 `c30b920d13b072d271122408822192e6e427b8ecac4ed289e3cecd64b7d37b47`, Build ID `a956b9960bf8e8ebb08b7e560570ff4c85449b1d`.
- MPP runtime: `/lib/aarch64-linux-gnu/librockchip_mpp.so.1`, SHA-256 `ea762b7da71b63a73cdf14d053d84b091654aee5e5c4f26ceb1449b8fb8d92f9`, Build ID `b333b160bf1f80ad4f935d9794d2b8956b90bb14`.
- Synthetic frame: 1920×1200 NV12, 1920×1200 strides, UV offset 2,304,000 bytes, 3,456,000-byte allocation from `/dev/dma_heap/system`; no CMA heap was used.
- Config: HEVC, YUV420SP, 20 fps input/output, CBR target/max/min 10.0/10.1/9.9 Mbps, GOP 90, production QP bounds, one image plus one EOS marker.
- Import: DMA-BUF sync start/end succeeded; MPP fd/size import returned `0` and a handle on the first attempt; pointer retry was not needed.
- Resource observations: before probe `MemAvailable=2,024,236 kB`, `CmaFree=0`; after probe `MemAvailable=2,001,264 kB`, `CmaFree=0`; `/data` free was approximately 1.82 GB and `/dev/shm` returned to its prior free space after cleanup.

## Execution record and duplicate

The first run completed at 2026-10-30 18:35:39 UTC under the saved 10-second MPP and 30-second total ceilings. The raw executed log is `build/evidence/g0-mpp-encode-probe-run.log`, SHA-256 `577dc476e8f556cddb805cd14971c3b18322d3ec64316394e7fbbde801f4d2f6`.

The root agent reports that it accidentally executed the same saved one-shot script again at 18:35:52 UTC after the first run had already completed. It reports the same successful import, one-frame output and software decode, with cleanup and no device users. This duplicate was unplanned, produced no performance data, and made no persistent video output; no third run is authorized. The root-reported duplicate has no separate raw log attached to this report.

## Evidence hashes

| Artifact | SHA-256 |
| --- | --- |
| Probe plan | `91784856987baf5bb5c193f5b16ce40f91cf833dd00e4ae094c36f5f009ad523` |
| Probe source | `139dbdf059805fd226dbc6dfe843d037f5d308437f218b04c848f48a06e66332` |
| Probe binary | `5474d5f501d57c7bfa3ee9101c5379cefe2cd56f00792f44e8c861eac9ad0b85` |
| Build log | `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` |
| Immediate preflight log | `6d73f82d6ae692a786675fc5f14c3e8a20b0df7f4f56bcae96258b8044179006` |
| Run script | `716b269ad63326c306fa67d25ba31e20d96e9bc1da00a8386e750c6b2e3eacde` |
| Raw first-run log | `577dc476e8f556cddb805cd14971c3b18322d3ec64316394e7fbbde801f4d2f6` |

## Next gate

Do not begin the longer A/B from this direct API result alone. First prove that a synthetic sender can pass real DMA-BUF fds through isolated VisionIPC into the unchanged comparison binary or a direct harness around the candidate `MppEncoder` class. Verify the received fd type and import/fallback counters, capture output packets from every arm, compare decoded frame counts and content against copy mode, and retain the existing time, memory, output-volume, and abort ceilings. If that production-path input seam cannot be built and shown to use DMA-BUF without copy fallback, report the performance result as unavailable.
