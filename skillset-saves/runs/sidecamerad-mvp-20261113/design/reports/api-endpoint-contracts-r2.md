---
revision: r2
---

# Internal endpoint contracts — sidecamerad availability MVP

**Architecture basis:** [report_architecture-r2.md](report_architecture-r2.md), [report_interfaces-r2.md](report_interfaces-r2.md). The MVP adds local process, Params, cereal, and VisionIPC contracts; it adds no HTTP endpoint, remote RPC, or routable listener.

## Endpoint and configuration inventory

| Interface | Purpose and contract | Access / state |
|---|---|---|
| `VisionIPC SUB` — server `sidecamerad`, stream IDs 7–10 (ZMQ ports 9007–9010 when using the ZMQ backend) | Read the position-mapped 1280×720 NV12 frame (1,382,400 bytes). IDs map front-left, front-right, back-left, back-right. Three buffers per configured stream; image data is not duplicated in cereal. | Existing host-local IPC transport and OS permissions; no remote listener. Read-only subscription. |
| `cereal SUB` — `frontLeftCameraState`, `frontRightCameraState`, `backLeftCameraState`, `backRightCameraState` | Read each position's 20 Hz `FrameData` metadata, including `frameId`, `timestampSof`, and `timestampEof`. Match `frameId` to the position's VisionIPC frame. Event arms use ordinals 150–153 after `audioFeedback @149`; service declarations are appended at the end for minimal ordering only. | Existing local cereal transport; `should_log=False`; no image bytes. Read-only subscription. |
| `Params GET` — `SideCameraAvailability` | Read one Python dictionary with exactly four stable service-name keys and boolean values; its startup value serializes as `{"frontLeftCameraState":false,"frontRightCameraState":false,"backLeftCameraState":false,"backRightCameraState":false}`. Type is `JSON`; flag is `CLEAR_ON_MANAGER_START`; the compact all-false JSON literal is the C++ declared default. Manager initialization restores it after clear. Python manager/sidecar calls pass dictionaries directly to `Params.put()` (never JSON text strings); Python consumers read dictionaries. Sidecar writes all false before camera work. | Local Params store. `Params.put()` is synchronous; sidecar updates only on transitions under a shared lock to avoid per-frame blocking. True follows the same frame's completed VisionIPC and cereal send calls; false follows capture error/loss/disconnect or orderly shutdown. This is process readiness state, not per-subscriber delivery acknowledgement. |
| `ENV CONFIG` — `SIDECAMERAD` | Unset means enabled by default under the existing `driverview` condition. `SIDECAMERAD=0` is an optional explicit off switch. Native `camerad` retains its existing registration and manager policy. | Local manager environment; selects whether the sidecar process is enabled. |
| `ENV CONFIG` — `SIDECAM_FRONT_LEFT`, `SIDECAM_FRONT_RIGHT`, `SIDECAM_BACK_LEFT`, `SIDECAM_BACK_RIGHT` | Each variable explicitly maps a position to one image device. Recommend `/dev/v4l/by-path/<physical-port>-video-index0` for physical-port stability; continue accepting `/dev/videoN`. Validate the resolved V4L2 character device under `/dev`; reject metadata-only `video-index1`. Never infer a side from USB enumeration order. | Local manager environment; selects configured workers only. A configured missing path remains in bounded worker retry and may recover when the same by-path node reappears. |

The VisionIPC and cereal channels are independent one-way publications, not a transaction. Sidecar sends both forms with the same per-position frame ID and only then marks that position available in Params. Consumers may observe one channel before the other. Frame IDs increase per position for the life of the process and are not persisted across restart.

## Empty configuration, errors, and privacy

With no `SIDECAM_*` path configured, the default-enabled process logs exactly one missing-feed warning for each unconfigured service, leaves all four Params fields false, creates no capture workers or VisionIPC buffers, and remains alive in bounded `stop_event.wait(60)` intervals until SIGINT or SIGTERM. It must not return normally while the manager expects the default-enabled process to run.

A configured unavailable device, FFmpeg exit, short frame, or capture timeout affects only that position: mark it unavailable if it had been available, close/reap the child, then retry at 1, 2, 4, 8, 16, and at most 30 seconds. Siblings and native `camerad` continue. On orderly shutdown, stop/join workers and publish all false. Abrupt uncatchable termination cannot run that final write; manager startup resets the default.

Device paths and MJPEG input are treated as untrusted local input. Use a fixed FFmpeg executable and argv without a shell or environment-supplied options. Keep image bytes in ephemeral VisionIPC buffers; do not persist or log them. Preserve existing host-local endpoint binding and permissions.

## Compatibility and acceptance

Append cereal service declarations after existing entries as an ordering choice only: the enumeration index is unused and the C++ registry is keyed by service name, so there is no numeric service-ID or ABI shift. Append event arms and VisionIPC IDs without changing existing ordinals/IDs 0–6 or native `camerad` behavior. No new package, remote endpoint, or image persistence is introduced.

Acceptance includes default-on manager behavior with the flag unset, unchanged native `camerad`, all-false no-path startup and signal-wait lifecycle, and Param false→true→false transitions on availability changes. With the single attached decoder, run one explicit `video-index0` path through V4L2 capture and verify both a cereal state and nonzero VisionIPC payload arrive with at least one matching frame ID. Reconnect and four-camera performance remain target-hardware checks; the one-camera evidence does not prove four-camera throughput or cadence.
