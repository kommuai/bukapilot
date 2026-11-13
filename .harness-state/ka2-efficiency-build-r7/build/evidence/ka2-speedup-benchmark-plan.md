# KA2 encoder speedup measurement plan

**Run:** `loggerd-ka2-efficiency-20261029`  
**Target:** Firefly AIO-3588Q / RK3588 board `kommu-870ce22d4093b701`  
**Authorization:** User explicitly authorized board testing in this conversation on 2026-10-30.  
**Scope:** Synthetic input only. Do not start or use live cameras, sensors, road data, or production log roots.

## Comparison

- **Production comparison target:** merged-overlay `encoderd` binary SHA-256 `c30b920d13b072d271122408822192e6e427b8ecac4ed289e3cecd64b7d37b47` (Build ID `a956b9960bf8e8ebb08b7e560570ff4c85449b1d`); it remains unchanged and will not be started for this pilot.
- **Pilot implementation:** a temporary direct harness around the candidate `MppEncoder` object from `/tmp/ka2-efficiency-r7-mergecheck/system/loggerd/liblogger.a`, SHA-256 `991024bd6c713f4bd3910df1ed3f904e9dcc356d82163bb44191ad24e81e3801` (the object is `mpp_encoder.o`, SHA-256 `b9934b105ebb85c1347e1bb8b438bf122fbf3e7e13b30ee8567e7eac31e3bf44`). Use the same harness binary and libraries for both arms, setting only `ENCODER_ZERO_COPY=0` or `1`. This is a class-level pilot; it does not measure encoderd IPC or its process overhead.
- **Environment:** aarch64, Ubuntu 24.04.4, kernel `5.10.198`; loaded MPP library `/lib/aarch64-linux-gnu/librockchip_mpp.so.1` (SHA/build ID recorded in probe evidence); `CmaTotal=262144 kB`, `CmaFree=0 kB`, `MemAvailable≈2.0 GB`. No user-space camera, encoder, sensor, or MPP device user was present at the fresh G0 check. Existing manager/uploader services remain running; no host settings or services will be changed.

## Workload and measurements

The first pilot exercises one full-resolution road encoder plus qRoad, which consumes the same road input and downsizes to 526×330 as the copy/downscale control. Use deterministic synthetic NV12 frames in genuine DMA-BUF-backed buffers, with the same input bytes, dimensions, timestamps, codec settings, and frame cadence for both arms. Board camera configuration pins input to 1920×1200 NV12 with Y/UV row strides 1920 and UV offset 2,304,000 bytes; the layout is already 16-pixel aligned. Feed each arm at a fixed 20 frames/s, one MPP process at a time. Use a 24-buffer system-heap DMA-BUF ring (about 82.9 MiB input allocation) so an in-flight frame is not overwritten. The stock Python VisionIPC SHM/memfd sender is excluded. Expansion to wide-road and driver requires the pilot to pass and another bounded plan checkpoint.

Record per arm and per stream: accepted/emitted/missing frames, non-empty packet count and bytes, import/fallback status, per-frame submit latency, encoder process CPU time, output decode validity, decoded-frame count, and per-frame content hashes; when hashes differ, report decoded pixel error (MAE/PSNR) against copy mode. Also record wall time, process peak RSS, and before/after CMA and memory counters. Use three discarded 5-second warm-up intervals per arm (six intervals total). Then run five paired 5-second pilot samples with arm order alternating; the pilot is exploratory and cannot support a stable speedup claim. Estimate paired variance from the pilot and size a final paired sample set up to 30 pairs, alternating arm order, within the 15-minute total ceiling. Six warm-up intervals + five pilot pairs + 30 final pairs require about 380 seconds of sample time, plus process startup and decode. Report median, p95, p99, sample count, and 95% bootstrap confidence intervals for submit latency and CPU time. A speedup claim requires successful imports, valid output, content comparison, and the final-set confidence interval excluding zero. If validity fails, only the pilot fits, or variance requires more than 30 pairs, report the results as exploratory without a stable speedup claim. This one-stream pilot does not support a three-stream or end-to-end encoderd claim. Do not proceed if the synthetic input is not DMA-BUF, if import/fallback evidence shows copy mode, or if the candidate cannot maintain valid packets.

## Preflight gate and ceiling

The single-frame import and encode/output readiness probes have passed and are not performance samples. Before the pilot, verify the direct harness binary resolves the pinned MPP library, configure isolated `OPENPILOT_PREFIX`/`PARAMS_ROOT` and MsgQ sockets, and check that its input fds are genuine DMA-BUFs. During the zero-copy arm, require no fallback warning, one non-empty packet per accepted input, and a decodable output stream.

For this pilot/final sequence: peak rate is 20 input frames/s to road plus qRoad; peak concurrency is one harness process with at most two MPP contexts; total board test time is at most 15 minutes including warm-up, pilot, final samples, and decode; temporary input/output data is capped at 256 MiB, and encoded streams are removed after decode/content checks. Do not claim a stable benchmark if the ceiling or output losses prevent enough final paired samples.

Abort immediately on any live camera/sensor or production logger path appearing, another MPP user starting, encoder/import/output failure, any missing or undecodable frame, output stall over 1 second, `MemAvailable` falling below 1.5 GB, temporary data reaching 256 MiB, target restart, thermal throttle, or any other workload showing impact. Because CMA is already at zero, any allocation failure or further inability to allocate/import is an immediate stop; do not retry at the same load.

## Restore register

No services, CPU governor/frequency, thermal settings, kernel settings, or board defaults will be changed. Temporary probe files will be removed after hashes/results are captured. There are no planned host state changes to restore.
