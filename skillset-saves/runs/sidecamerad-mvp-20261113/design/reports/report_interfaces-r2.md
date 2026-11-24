# Interface contracts — sidecamerad default-on availability MVP

**Run/revision:** `sidecamerad-mvp-20261113` / r2  
**Requirements:** [requirements-brief-r2.md](requirements-brief-r2.md) · SHA-256 `bff3dc6825e257d653772d43d64a3d7f6221ed91bea0dff4c08ce50b82d7f748`  
**Architecture:** [report_architecture-r2.md](report_architecture-r2.md)  
**Change:** Additive/default-on local process behavior, explicit by-path-capable device inputs, and one Params availability value. No HTTP, remote RPC, or routable listener.

## Interface inventory

| ID | Interface; caller → callee | Contract and error behavior |
|---|---|---|
| IF-1 | Manager → `sidecamerad` | Start by default under the existing `driverview` condition. `SIDECAMERAD=0` is the explicit opt-out; when unset, the process is enabled. Keep native `camerad` registration and lifecycle untouched. Since the manager does not restart a normal exit, the empty-config path must remain alive until a signal. |
| IF-2 | Operator environment → sidecar | Position variables are `SIDECAM_FRONT_LEFT`, `SIDECAM_FRONT_RIGHT`, `SIDECAM_BACK_LEFT`, and `SIDECAM_BACK_RIGHT`. Each explicitly names its image node. Recommend `/dev/v4l/by-path/<physical-port>-video-index0`; accept `/dev/videoN` for existing setups. Resolve under `/dev`, validate as a V4L2 character device, and reject metadata-only `video-index1`. No USB enumeration order or guessed side mapping. An empty variable leaves that feed unconfigured; a configured-but-missing path retries within that position's worker. |
| IF-3 | Sidecar → Params consumers | Exactly one key: `SideCameraAvailability`, type `JSON`, flag `CLEAR_ON_MANAGER_START`, with the compact all-false JSON literal `{"frontLeftCameraState":false,"frontRightCameraState":false,"backLeftCameraState":false,"backRightCameraState":false}` as its C++ declared default. Manager startup clears then writes this default; the Python manager and sidecar pass Python dictionaries directly to `Params.put()` (never a JSON text string), and Python consumers read a dictionary. The sidecar writes the all-false dictionary before capture. Object fields are stable and always present, keyed by exact cereal service name, with boolean values. One mutex protects the shared map and synchronous `Params.put(dict)` transition; only changed fields trigger a write. `Params.put()` is synchronous, so transition-only writes avoid per-frame blocking. Set a field true only after the same full frame's VisionIPC send and cereal PubMaster send complete; set false on capture failure/loss/disconnect/stop and final orderly shutdown. |
| IF-4 | Capture worker → fixed FFmpeg child | Input: one validated explicit V4L2 device path, MJPEG 1280×720 at 20 fps. Output: raw NV12 bytes via stdout, one FFmpeg thread, fixed executable/arguments and no shell. Accept exactly 1,382,400 bytes per frame into a reusable buffer. On EOF, short read, decoder exit, startup/stall timeout, or device error: mark that position unavailable if it was available, close/reap the child, log bounded lifecycle detail, and retry with 1, 2, 4, 8, 16, then 30 seconds maximum. |
| IF-5 | Sidecar → VisionIPC local clients | Server/control name `sidecamerad`; add it to the existing endpoint-name allowlist. Three buffers per configured stream at 1280×720 NV12. Fixed mapping: stream IDs 7, 8, 9, 10 for front-left, front-right, back-left, back-right. Preserve all IDs 0–6; mirror enum values in C++ and Cython. VisionIPC carries pixels; no cereal image duplication. |
| IF-6 | Sidecar → cereal subscribers | Append `FrameData` arms `frontLeftCameraState @150`, `frontRightCameraState @151`, `backLeftCameraState @152`, `backRightCameraState @153` after `audioFeedback @149`. Append their four non-logged 20 Hz service declarations after existing entries. The ordered declaration is for minimal additive diff only: the list index is unused, C++ lookup is by service name, and no numeric service-port ID changes. For one camera, cereal's `frameId` and the position-matched VisionIPC frame ID are equal; `timestampEof` is host monotonic and nominal `timestampSof = timestampEof - 50,000,000 ns`. No cross-camera synchronization or atomic cross-channel delivery is promised. |
| IF-7 | Sidecar lifecycle → local observers | With no path variables configured, emit exactly one missing-feed warning per unconfigured service, keep `SideCameraAvailability` all false, create no capture workers/buffers, install SIGINT/SIGTERM handlers, and wait in bounded `stop_event.wait(60)` intervals that wake only on timeout or signal. With configured but unavailable paths, keep workers in bounded retry. On orderly shutdown, stop/join workers and write all-false state. Manager startup restores the declared default after process-manager restart. |

## State and error semantics

The Param reports only whether each position has produced a full frame to **both** interfaces during the current sidecar process. All four keys exist in every value. A missing path or a configured path that has never yielded a full dual-published frame is `false`. An available worker changes to `false` as soon as capture loss, decoder failure, a complete-frame timeout, disconnect, or shutdown is observed; a later successful dual publication changes it back to `true`. Same-port replug using an explicit by-path path is expected to be recoverable through that worker's retry loop, but reconnect was not injected in the available hardware evidence.

The process sends the VisionIPC frame and cereal `FrameData` with the same per-position frame ID. No transaction spans the two transports, so the Param becomes true only after both send calls complete; it is a readiness summary rather than a delivery acknowledgement to every subscriber. Frame IDs are monotonic per position while the sidecar runs and are not persisted across process restarts.

The Params value is local status and contains no image bytes or path. Camera bytes remain in ephemeral VisionIPC buffers; no logs or persistence include pixels. Existing host-local IPC permissions and endpoint binding remain in effect.

## Required implementation checks (acceptance contract)

- Manager default-on behavior with the flag unset; native `camerad` still follows its prior policy.
- No-path idle behavior stays alive, warns once for each of four feeds, and leaves all fields false until SIGINT/SIGTERM.
- One-camera V4L2 capture using an explicit `video-index0` by-path value publishes a valid cereal state and NV12 VisionIPC payload at 1,382,400 bytes, with at least one matching frame ID across the two interfaces.
- Param behavior is all-false at manager/sidecar startup, false→true after first dual-published frame, true→false after disconnect/failure, and all false after orderly shutdown; updates occur only on transitions.
- Configured unavailable path retries without affecting sibling feeds or native `camerad`; reconnect behavior is exercised on target hardware.
- Preserve stream IDs 0–6, existing cereal ordinals/services and native process behavior; no package is added.

## UI handoff

Not applicable: this MVP exposes local Params/cereal/VisionIPC interfaces only and adds no user-facing surface.
