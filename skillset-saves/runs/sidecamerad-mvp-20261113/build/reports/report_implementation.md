# Implementation report — sidecamerad MVP

## Status

The approved opt-in source slice is implemented. The build-to-review package is not complete: no camera capture, automated tests, runtime-health smoke, target-hardware soak, or security sign-off was run in this pass. Four-camera cadence and CPU headroom remain unmeasured; the approved design requires a 30-minute target-hardware soak before performance sign-off.

Approved design: revision `r1`, cross-stage verdict SHA-256 `36b70ad825fe8f036460fb8d7eb994b40195d3d1661e548f09610d4d81275832`.

## Implementation

- Added `system.sidecamerad` to manager configuration behind `SIDECAMERAD=1`, using the existing `driverview` lifecycle. Native `camerad` configuration and source are unchanged.
- Added four optional device paths: `SIDECAM_FRONT_LEFT`, `SIDECAM_FRONT_RIGHT`, `SIDECAM_BACK_LEFT`, and `SIDECAM_BACK_RIGHT`. Missing paths leave those feeds absent; each supplied path is handled by an independent worker.
- Added four non-logging 20 Hz services and Event arms: `frontLeftCameraState`, `frontRightCameraState`, `backLeftCameraState`, and `backRightCameraState`.
- Added VisionIPC streams 7–10 and allowed the `sidecamerad` server name in the ZMQ endpoint check.
- Added an FFmpeg V4L2 MJPEG pipeline that requests 1280×720 at 20 fps and emits NV12. It reads complete 1,382,400-byte frames into one reusable scratch buffer per camera, publishes through three VisionIPC buffers per camera, and sends state timestamps using monotonic time with SOF exactly 50 ms before EOF.
- Uses direct argument arrays, validates `/dev/videoN` character devices, limits FFmpeg decoder/filter thread settings, sends FFmpeg stderr to `/dev/null`, and retries failed workers with bounded exponential delays up to 30 seconds. No package was installed.

At four configured cameras, the fixed image-buffer payload is `(3 shared + 1 scratch) × 4 × 1,382,400 = 22,118,400 bytes` (21.09 MiB), excluding FFmpeg internals and pipe buffers. No enforceable child CPU or memory limit is established; this remains an open resource-control risk.

## Changed source artifacts

| Path | SHA-256 |
|---|---|
| `cereal/log.capnp` | `d5c0372d714e65461942874c91c522b205ce208261b382283839ed03b9731f64` |
| `cereal/services.py` | `cacd4b749d9a0a3e19e3c466784dcf34925f263fdca3c26af72578c47d740a6b` |
| `system/manager/process_config.py` | `63dfdd352cb7776cf804d17449da0f7aac72f2d56efb9dc01e550a1d501b7828` |
| `system/sidecamerad.py` | `3b4881aa1e215e266545fa318de65a8af69b0b87afa35b83178c269b6a21170d` |
| `msgq_repo/msgq/visionipc/visionbuf.h` | `c971a1afa601f5bb8f560de6fb1dd78c2121cee61ea41c169f00a4154566e870` |
| `msgq_repo/msgq/visionipc/visionipc_server.cc` | `36b383bfcaf10eaa69f6844a579c86204b4a07464a889208fe059a6c1202d198` |
| `msgq_repo/msgq/visionipc/visionipc_pyx.pyx` | `36247e353901eb80e52735c91f466151b3026e77152dde9ad9e3bcd682c66f32` |

`msgq_repo` is the KommuAI-owned submodule at commit `dc1799e42ebfb8642a9b1f677a3d1861c3e59bda`; these source edits are in the approved change list. No generated or third-party package content was edited. The change-set patch is `build/artifacts/implementation-change-set.patch`, SHA-256 `82a35f9b4ea1abbaf9006bebf9345d6650ed98e45a64e8dfdea13534579260ef`.

## Checks recorded

- Python in-memory syntax compilation passed for the daemon, manager configuration, and service registry: `build/evidence/source-syntax.log`.
- `git diff --check` passed for the main repository and `msgq_repo`: `build/evidence/diff-check.log`.
- `scons msgq_repo/msgq/visionipc/visionipc_pyx.so` exited 0 and reported the target up to date: `build/evidence/native-binding-build.log`.
- No package installation, camera capture, automated test, runtime-health smoke, or performance measurement was performed.

## Trial setup and limits

Set these variables in the environment inherited by the manager before it starts. The manager starts `sidecamerad` only while `driverview` is active (onroad or driver view enabled).

```sh
SIDECAMERAD=1
SIDECAM_FRONT_LEFT=/dev/videoN
```

Add the other `SIDECAM_*` paths for additional decoders. Starting with one camera is supported. The new services and VisionIPC buffers are published, but no model, UI, logger, or encoder consumer is wired to them yet. Actual cadence depends on the decoder and target CPU; the configured 20 fps is not a measured guarantee.

Unrelated existing workspace changes in `common/params_keys.h`, `common/params_pyx.so`, and `opendbc_repo` were left untouched.
