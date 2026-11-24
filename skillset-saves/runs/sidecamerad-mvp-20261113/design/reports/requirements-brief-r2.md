# Requirements Brief — sidecamerad default startup and availability

**Run:** `sidecamerad-mvp-20261113` · **Revision:** r2  
**Prepared for:** `design/architect`  
**Prior approved design:** r1, `design-to-build` cross-stage verdict SHA-256 `36b70ad825fe8f036460fb8d7eb994b40195d3d1661e548f09610d4d81275832`.

**Sources mined:** latest user request recorded in `skillset-saves/runs/sidecamerad-mvp-20261113/intake/report_scope-change.md`; prior brief `design/reports/requirements-brief.md`; `system/manager/process_config.py`; `system/sidecamerad.py`; `common/params_keys.h`; `common/params.h`; `cereal/log.capnp`; `cereal/services.py`; `msgq_repo/msgq/visionipc/visionbuf.h`; `msgq_repo/msgq/visionipc/visionipc_server.cc`; observed host capture in `build/evidence/usb-camera-baseline.log`; observed V4L2 topology under `/dev/v4l/by-path/`.

## Actors and jobs

| Actor | Job to be done | Success looks like | Frequency / volume |
|---|---|---|---|
| Device operator / integrator | Start side-camera capture with the openpilot camera lifecycle and see when a feed is absent | The sidecar is enabled by default, each position reports its current feed availability, and missing feeds are acknowledged without stopping native `camerad` | Startup and connection changes; up to four cameras |
| Messaging / VisionIPC consumer | Pair a position's frame metadata with its pixels | The named cereal state and corresponding VisionIPC stream publish matching frame IDs | Continuous; target 20 frames/s per connected camera |

## Requirements

| # | Requirement | Source | Confidence | Affects | Open assumption |
|---|---|---|---|---|---|
| R1 | The manager enables `sidecamerad` by default whenever its existing `driverview` condition is true. | User request; `intake/report_scope-change.md`; prior `process_config.py` | reported | Manager registration and process startup | This changes r1's opt-in default. Verify the manager process entry is enabled without `SIDECAMERAD=1`; the feature owner can settle any later policy change. |
| R2 | Enabling `sidecamerad` leaves native `camerad` under its existing manager policy and does not stop, replace, or reconfigure it. | User request; `intake/report_scope-change.md`; `system/manager/process_config.py` | reported / observed | Process ownership and lifecycle | No change is authorized to `camerad`; verify both entries remain independently managed. |
| R3 | The four positions remain explicitly assigned through `SIDECAM_FRONT_LEFT`, `SIDECAM_FRONT_RIGHT`, `SIDECAM_BACK_LEFT`, and `SIDECAM_BACK_RIGHT`; each value may be `/dev/videoN` or a `/dev/v4l/by-path/...-video-index0` symlink. The implementation does not infer side labels from unlabelled USB-device order. | Prior approved r1 brief and implementation; latest user request; observed `/dev/v4l/by-path` topology | reported / observed | Device-to-position mapping and path validation | Prefer the by-path form to retry the same physical port after replug. The operator still assigns that port to a side explicitly. |
| R4 | Each configured position publishes its matching `frontLeftCameraState`, `frontRightCameraState`, `backLeftCameraState`, or `backRightCameraState` and its separate VisionIPC stream; partial configurations remain valid. | Prior approved r1 brief and implementation | reported | Event arms, service registry, stream mapping, and worker isolation | A missing position emits no frame; consumers use availability state to distinguish it from a live feed. |
| R5 | One Params key named `SideCameraAvailability` reports all four positions as JSON booleans keyed by the four camera-state service names. | User request; `intake/report_scope-change.md`; interface proposal for r2 | reported | `common/params_keys.h`, Params interface, and availability writer | The proposed value is `{"frontLeftCameraState":false,"frontRightCameraState":false,"backLeftCameraState":false,"backRightCameraState":false}` at manager startup. Confirm the exact shape in the architecture/interface artifact before build. |
| R6 | A position reports unavailable until its first complete frame is published, available after that publication, and unavailable again when capture disconnects or fails. | User request; `intake/report_scope-change.md` | reported | Worker state transitions and Params updates | The implementation should update the one JSON key only when a position changes state; verify disconnect handling in build. |
| R7 | Missing or unmapped feeds are acknowledged once per position at startup; with no camera paths configured, the default-enabled process stays alive in a signal-wait loop with all positions unavailable. | User request; parent-approved lifecycle interpretation in the active design handoff | reported | Empty-configuration behavior, logging, manager health | The parent design handoff selected a bounded SIGINT/SIGTERM wait to avoid a dead `shouldBeRunning` manager process. Build must verify this branch. |
| R8 | A configured but unavailable device remains isolated to its worker and follows the existing bounded retry policy; it does not stop sibling feeds or native `camerad`. | Prior approved r1 architecture and implementation; `system/sidecamerad.py` | reported / observed | Error recovery and process isolation | Recommend a configured by-path symlink for replug recovery. The current baseline did not inject a disconnect; false-on-loss and recovery remain build verification items. |
| R9 | The 1280×720 MJPEG-to-NV12 path and 20 fps capture request remain the target, with no added package and all product-source changes inside `/data/openpilot`. | Prior approved r1 scope; user constraint in `intake/report_scope-change.md` | reported | Capture configuration, stack lock, and repository scope | V4L2 nominal mode is observed for one connected decoder only; actual sustained four-camera cadence remains unmeasured. |
| R10 | Build validation checks the manager-run sidecar against a connected device and observes both the cereal state and VisionIPC image payload, including at least one matching frame ID. | User's explicit validation request in the latest turn and `intake/report_scope-change.md` | reported | Build acceptance evidence | The r1 baseline is not proof for changed r2 code; rerun after implementation. |

## Constraints

| # | Constraint | Kind | Source | Confidence | Affects |
|---|---|---|---|---|---|
| C1 | Keep the native `camerad` process and behavior unchanged; side-camera failure must remain position-local. | Operational | User request; prior approved r1 package | reported | Process configuration and failure containment |
| C2 | Add no package and retain the existing FFmpeg/V4L2/MJPEG/NV12 stack. | Technical | User request; `intake/report_scope-change.md` | reported | Architecture, build inputs, and deployment |
| C3 | Preserve explicit per-position device paths; accept a `/dev/v4l/by-path/...-video-index0` symlink (recommended) or a `/dev/videoN` node, and verify the resolved path is a V4L2 character device under `/dev`. Do not bind an unlabelled discovered UVC node to a side by guess. | Operational | Prior approved r1 contract; observed host has no unique decoder serial; observed by-path symlink | reported / observed | Device assignment, reconnect behavior, and availability semantics |
| C4 | Keep the Param default/current-start value false for every position until a first frame is successfully published. | Technical | User request; r2 interface proposal | reported | Startup status and consumer interpretation |
| C5 | Append new cereal service declarations after existing entries as a minimal additive ordering choice. The current `SERVICE_LIST` enumeration index is unused and the generated C++ registry is keyed by service name; this is not an ABI or numeric-port requirement. | Technical | `cereal/services.py`; parent source audit | observed | Service registry diff organization |

## Non-functional targets

| Target | Number | Source | Confidence | Why this number / limitation |
|---|---:|---|---|---|
| Requested capture mode | 1280×720 MJPEG input and NV12 output | Prior approved r1 brief; V4L2 baseline | reported / observed | The connected device reports this format and 20 fps in `usb-camera-baseline.log`. |
| Per-camera frame rate target | 20 frames/s | Prior user request and approved r1 brief | reported | One-camera nominal V4L2 mode is observed; sustained four-camera performance is not measured. |
| Four-camera image-payload ceiling | 27,648,000 bytes (26.37 MiB) for the r1 ceiling; current three-shared-plus-one-scratch design uses 22,118,400 bytes (21.09 MiB) | Prior r1 brief and `visionipc_server.cc` allocation rule | inferred | Pixel payload only; excludes FFmpeg, V4L2/kernel buffers, message metadata, and other process RSS. |
| End-to-end publication check | At least one frame received on both interfaces with matching frame IDs and a 1,382,400-byte NV12 buffer | User validation request; observed r1 baseline | reported | Baseline proves the earlier source revision on one device only; r2 must be rerun. |

## Observed host evidence

| Evidence | Source | Confidence | Affects | Limitation |
|---|---|---|---|---|
| `/dev/video60` is one USB UVC capture node, VID:PID `534d:2109`; it reports MJPEG 1280×720 at 20 fps. `/dev/video61` is the same device's metadata interface, not another image feed. | `build/evidence/usb-camera-baseline.log` | observed | V4L2 path validation and capture proof | Only one decoder is attached; no four-camera behavior is established. |
| `/dev/v4l/by-path/platform-xhci-hcd.11.auto-usb-0:1:1.0-video-index0` resolves through `../../video60` to the USB image node; the corresponding `video-index1` resolves to metadata node `/dev/video61`. | Host query: `readlink -f` and `readlink` on the two by-path entries | observed; same-port replug recovery is inferred from the physical-topology path | Stable per-port path choice and exclusion of metadata nodes | The symlink names a USB port, not a camera side. Replug recovery was not injected in the baseline and must be checked in build; explicit operator assignment remains required. |
| The r1 sidecar, run with explicit `SIDECAM_FRONT_LEFT=/dev/video60`, published a valid `frontLeftCameraState` and VisionIPC frames. The observed state frame ID 1 matched a VisionIPC frame ID 1; the image buffer was 1,382,400 bytes with nonzero sample data. Shutdown exited 0 and removed the server socket. | `build/evidence/usb-camera-baseline.log` | observed | Confirms the existing topic/stream path can receive one live device frame | This run predates r2; it does not validate default startup, the new Params key, missing-device behavior, or four-camera throughput. |

## Non-goals

- Do not replace, disable, or reconfigure native `camerad`.
- Do not add USB auto-discovery or persistent side assignment; reopen only after the physical port-to-side mapping is explicitly decided.
- Do not add model, UI, driving, encoder, logging, network, or persistent-image consumers.
- Do not claim four-camera cadence, synchronization, transforms, or resource headroom from the one-device probe.

## Prior art and comparable flows

| Source | What it shows | Why it transfers (or does not) |
|---|---|---|
| Existing r1 implementation and `build/evidence/usb-camera-baseline.log` | One configured USB decoder can feed the cereal state and side-specific VisionIPC stream using the current source revision. | It directly exercises the same device and interfaces, but predates default-on and availability changes and covers only one camera. |
| `common/params_keys.h` and `common/params.h` | Params keys declare a storage type and lifecycle flags; manager startup clears selected keys and initializes declared defaults. | This is the repository contract for choosing the new key type and startup default; build must compile the Params extension and verify the JSON value round-trips. |
| `/dev/v4l/by-path/*-video-index0` | The observed UVC capture node has a physical-topology symlink distinct from its metadata `video-index1` entry. | Use an explicit per-position by-path value to target the same USB port across replug; this does not identify front/back or left/right. |

## Conflicts preserved

No conflict remains between the user's latest request and r2's explicit device-path contract: default-on refers to process enablement, while camera-to-position assignment remains explicit. The earlier opt-in flag requirement is superseded by the latest user instruction.

## Open questions

| Question | Blocks | Owner | Latest safe decision point |
|---|---|---|---|
| Should a later revision auto-map USB hub ports to side names? | Only auto-discovery; it does not block r2 because r2 retains explicit paths. | Feature owner | Before any future auto-discovery implementation. |
| What CPU/headroom and sustained cadence do four simultaneous streams achieve on the target device? | Performance sign-off only; no four-camera claim is permitted until measured. | Feature owner with target hardware | Before four-camera performance sign-off; use the approved 30-minute soak. |
| Can the target safely enforce FFmpeg child CPU/memory limits without `preexec_fn` or new dependencies? | Security/resource acceptance, not the current source-scope decision. | Build/security owner | Before build-to-review security sign-off. |

## Limitations

- Only one USB decoder is available on this host. The new four-position availability behavior and four-camera load need later build/runtime evidence.
- The existing one-camera integration result is baseline evidence for r1. The r2 build must repeat the test after implementing default enablement and `SideCameraAvailability`.
- No four-camera performance result or disconnect-injection result is available yet.
