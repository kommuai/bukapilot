---
run_id: sidecamerad-mvp-20261113
submission_id: sidecamerad-mvp-20261113_investigation-ac4-timing-r1
revision: "1"
---

# Investigation Package — SideCamera AC-4 Timing Jitter

**Boundary:** `investigation-review` · **Owning phase:** BUILD · **Build input revision:** 68 · **Investigation revision:** 1

## Scope and impact

The strict single-camera AC-4 cadence check fails on the assigned front-left UVC feed. The investigation covers only the source-to-publication timing path for one 1280×720 MJPEG camera requested at 20 fps. AC-6 physical reconnect/sibling continuity and AC-8 four-camera performance/CPU soak remain outside scope because only one decoder is available. Build source was not changed.

AC-4 requires at least 6,000 complete published frames over five steady-state minutes, mean cadence of at least 20 fps, no EOF-to-EOF timestamp interval above 60 ms, and matching cereal/VisionIPC frame IDs.

## Reproduction

The current-source five-minute failure was reproduced in the assigned runtime capture. It produced 6,050 selected matching IDs over 302.506666 seconds; the 6,049 EOF intervals averaged 49.998692 ms (20.000523 fps), the largest was 89.163715 ms, and 753 intervals exceeded 60 ms. IDs were contiguous and cereal/VisionIPC timestamps matched. The earlier five-minute capture also failed, with 19.999396661241942 fps mean, 123.185415 ms maximum gap, and 743 of 5,999 intervals above 60 ms.

The copied executed log is `evidence/reproduction.log` (same bytes as the build runtime log, SHA-256 `de804d35326c6925862d2c8b36708d4a14a10b05120076924f523397976e8c55`). Its typed record and source hashes are bound in `manifest.json`. The failure is a max-gap failure; this package does not claim AC-4 passes.

## Evidence chain and mechanism

The same sidecamerad run shows bursty arrival timing at the producer and at the readers. VisionIPC receive intervals averaged 49.998402 ms, maxed at 89.389179 ms, and had 747 of 6,049 intervals above 60 ms. Median VisionIPC receive lag behind the producer EOF timestamp was 0.881836 ms (maximum 7.017679 ms). There were no frame-ID gaps or cereal/VisionIPC timestamp mismatches. This does not support subscriber backlog or IPC transport as the source.

A separate direct V4L2 capture, with no FFmpeg or sidecamerad, used the same device mode confirmed by a read-only format query and sent 1,200 mmap buffers to `/dev/null`. Kernel timestamps were monotonic `ts-src-soe`; sequence numbers ran 0 through 1199 without loss. The first startup interval was 167.987 ms. Excluding that interval, 1,198 intervals averaged 49.995553 ms, ranged from 47.672 to 52.287 ms, and none exceeded 60 ms. The capture completed in 60.22 seconds including device startup; no image bytes were saved.

A separate FFmpeg-to-raw-pipe timing capture used the same V4L2/MJPEG-to-NV12 command and Python `poll`/`readv` completion timestamp, but no sidecamerad, IPC publisher, subscribers, or recording. Of 1,090 complete frames, 1,089 intervals averaged 49.991474 ms (20.003411 fps), ranged from 10.275 to 88.842 ms, and 133 exceeded 60 ms while 147 were below 40 ms. The nine current package source hashes remained unchanged. This reproduces the long gaps before SideCamera IPC work.

Source inspection confirms the worker assigns EOF time with `time.monotonic_ns()` only after reading a complete rawvideo frame. The present rawvideo pipe carries pixel bytes; the worker does not receive the V4L2 source timestamp. Direct V4L2 timing is regular, but there is no simultaneous mapping between those kernel timestamps and the FFmpeg decoded output frames.

**Mechanism, medium confidence:** the reported EOF jitter is introduced after regular V4L2 source capture and before IPC publication, in the FFmpeg raw-output / pipe-read completion path. The capture worker's completion-time timestamp exposes that software-path jitter. IPC backlog, frame loss, and irregular V4L2 source cadence are not supported by the observed data. The evidence does not distinguish FFmpeg decode/output timing from host/Python scheduling around pipe delivery and full-frame reads.

The full timestamped evidence trace is `evidence/evidence-chain.log` (SHA-256 `e6fa572cd107e21422cdb17f9d3bbf8c982cefb148afff8d782185f8842e8905`). It names every observation, its capture, and what it does and does not establish.

## Bounded fix path

**Owner:** build implementation in the BUILD phase, routed through `build/bob-the-builder` after the source read-only guard is formally lifted through Admiral.

Carry and validate a per-frame monotonic V4L2/FFmpeg source timestamp alongside each decoded NV12 frame, then use the verified source EOF time for the published timestamp instead of sampling the clock only after the full rawvideo pipe read. Measure and retain the real frame-read/publication latency separately so late delivery remains visible; do not smooth timestamp intervals or synthesize a regular 20 fps schedule. Keep frame IDs and cereal/VisionIPC publication paired.

Verify frame-to-source-timestamp alignment first, then repeat the exact five-minute AC-4 capture. Pass only with at least 6,000 complete published frames, mean cadence >=20.0 fps, every EOF interval <=60 ms, and matching IDs. If source timestamp alignment is unavailable or the published stream still misses the limit, the build owner should investigate FFmpeg output/host scheduling rather than treating timestamp relabeling as a fix.

No implementation change is authorized or included in this investigation package.

## Residual uncertainty

- The direct V4L2 and FFmpeg-only probes were sequential, not simultaneous; their timestamps were not correlated on the same individual frame.
- FFmpeg decode/output timing and host/Python scheduling cannot be separated by the available captures. CPU pressure and load average are aggregate host observations without per-process attribution.
- It is not yet established whether the current FFmpeg input path exposes the exact monotonic V4L2 source timestamp on each decoded frame or whether another capture interface is needed.
- The direct V4L2 diagnostic took 60.22 seconds including startup, 0.22 seconds above a nominal 60-second cap; it captured exactly 1,200 frames and wrote no image data. The FFmpeg-only diagnostic ended by SIGKILL after a one-second SIGTERM wait; the process was reaped and no child remained.
- AC-6 physical reconnect/sibling continuity and AC-8 four-camera 30-minute soak/CPU headroom remain unverified and are not addressed here.

The observation that would change the mechanism conclusion is a same-frame correlation of V4L2 capture timestamps, FFmpeg output PTS, and post-read completion timestamps with scheduler delay captured. That measurement would separate decoder/output timing from process scheduling and show whether source timestamps can safely drive the published EOF fields.
