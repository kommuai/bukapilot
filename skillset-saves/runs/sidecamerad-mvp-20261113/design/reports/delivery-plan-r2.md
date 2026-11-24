# Delivery Plan — sidecamerad default-on availability MVP

**Run:** sidecamerad-mvp-20261113 · **Revision:** r2  
**Architecture:** reports/report_architecture-r2.md @ 9f4cb41838303bd661048d62ddbcfea0ced765d4fc10ec228ef2c453e1a94991  
**Interfaces:** reports/report_interfaces-r2.md @ 2ae1cfbe0746ae3a03df5a682e2dbe02d11337270809f3ace4dd220a6b0c05ad  
**Requirements:** reports/requirements-brief-r2.md @ bff3dc6825e257d653772d43d64a3d7f6221ed91bea0dff4c08ce50b82d7f748  
**Security seed:** reports/security-seed-r2.md (produced from this architecture revision; register its final hash in the r2 manifest).

## Objective and scope

Deliver one additive sidecamerad slice alongside native camerad. Under the existing driverview condition, an unset SIDECAMERAD enables the process by default; SIDECAMERAD=0 is the explicit opt-out. Native camerad keeps its current registration and behavior.

Each feed uses its explicit SIDECAM_* position variable. Recommend a /dev/v4l/by-path/...-video-index0 value for stable physical-port identity, while continuing to accept /dev/videoN. The implementation must not infer side names from unlabelled USB enumeration order. Four partial or missing camera paths are valid: the one Params key SideCameraAvailability reports all four stable service-name booleans, all false at startup. Declare it as JSON with CLEAR_ON_MANAGER_START and the compact all-false JSON literal in the C++ key registry. In sidecamerad, Params.put receives a Python dict directly and Params.get returns a dict; do not pass a JSON string to the Python Params API. A field turns true only after the same complete frame is sent to both VisionIPC and cereal, and turns false on capture loss, failure, disconnect, or orderly shutdown.

With no configured paths the default-enabled process warns once per missing feed, leaves all fields false, and waits for SIGINT/SIGTERM in bounded intervals. A configured but unavailable device remains in its worker's bounded retry path. Capture continues to use the existing FFmpeg executable for 1280×720 MJPEG-to-NV12 at a requested 20 fps. Add no packages, BGR conversion, application frame queue, persistent image sink, or network listener.

## Milestone path

| # | Milestone | Delivery slice | Depends on | Decision gate | Exit condition |
|---|---|---|---|---|---|
| 1 | Default-on side-camera capture and availability | S1 — one additive MVP slice | Approved r2 process, Params, cereal, VisionIPC, and failure contracts; existing FFmpeg; explicitly assigned V4L2 paths | Confirm the manager's unset/0 behavior and preserve native camerad; keep existing stream IDs 0–6 and cereal ordinals through 149; confirm one key and exact JSON fields | No-path process stays alive with all fields false; one-camera capture publishes matching cereal/VisionIPC frame IDs and passes the five-minute smoke when the attached camera is available |
| 2 | Four-camera performance sign-off | Verification gate for S1, not a separate source slice | S1 functional acceptance; target hardware and all four explicit camera paths | The four-camera result is not claimed until the concurrent soak and CPU report exist | Four feeds each sustain the 20 fps target over 30 minutes, with per-camera gaps and CPU headroom recorded |

## Ordered work and dependencies

| Order | Work | Depends on | Why |
|---|---|---|---|
| 1 | Add SideCameraAvailability to the C++ Params registry as JSON, CLEAR_ON_MANAGER_START, with the compact all-false default literal; manager startup clears the key then writes that declared default. In sidecamerad, pass Python dicts to Params.put and consume dicts from Params.get. Append the four service declarations after existing services for an additive diff. Preserve the four FrameData ordinals 150–153 and VisionIPC stream IDs 7–10. | Existing Params, cereal, and VisionIPC contracts | Consumers and publishers need a stable shape and mapping. The service-list index is unused and C++ lookup is keyed by service name; appending does not address a numeric-ID or ABI shift. |
| 2 | Register sidecamerad under its existing driverview lifecycle, enabled when SIDECAMERAD is unset and disabled only by explicit 0. Leave the camerad entry alone. Write all-false availability before starting workers. | Order 1; IF-1 and IF-3 | Default enablement must not make an empty camera configuration exit normally or make side-camera failure affect native capture. |
| 3 | Validate each explicit path, including resolution of a recommended video-index0 symlink to a V4L2 character device; reject metadata-only video-index1. Start only configured workers with fixed FFmpeg argv, one FFmpeg thread, a reusable NV12 buffer, three VisionIPC buffers, and bounded retries. | Order 2; IF-2 and IF-4/IF-5 | Physical port paths support same-port retries without guessing which side an unlabelled USB device represents. Fixed buffers and worker limits maintain the existing resource design. |
| 4 | Publish each complete frame with one per-position frame ID to VisionIPC and cereal. Update the four-field Python dict and pass it to Params.put only on availability transitions; set true after both sends complete and false on failure or orderly shutdown. | Order 3; IF-3, IF-5, and IF-6 | The Param summarizes live dual-publication state without a per-frame disk write. |
| 5 | Exercise default/empty startup, one-camera capture and publication, Param transitions, and the failure/retry path. Run the four-camera soak only on a target with four assigned devices. | Orders 1–4; one image device for the smoke; four devices for performance evidence | The current host has only one decoder, so one-camera acceptance cannot establish aggregate throughput or CPU headroom. |

## Acceptance criteria

| ID | Slice | Observable condition | Check that proves it | Acceptor |
|---|---|---|---|---|
| AC-1 | S1 — default-on capture | With SIDECAMERAD unset and the existing driverview condition true, the manager starts sidecamerad; explicit SIDECAMERAD=0 disables it. Native camerad retains its previous manager entry, condition, and behavior in both cases. | On the target build, inspect process configuration and manager status with the variable unset and at 0; compare the camerad entry and state to the pre-change behavior. | Side-camera feature owner |
| AC-2 | S1 — empty configuration | With all four SIDECAM_* variables absent, the process logs exactly one missing-feed warning for each service, writes the exact all-false JSON, creates no capture workers/buffers, and remains alive in bounded waits until SIGINT or SIGTERM. | Start the manager with no side-camera variables; inspect the four warnings and Param bytes, confirm process liveness past one wait interval, then signal it and confirm orderly exit. | Side-camera feature owner |
| AC-3 | S1 — availability state | SideCameraAvailability is a JSON object with exactly the four stable cereal service-name keys and boolean values. The C++ key default is the compact all-false literal; sidecamerad writes Python dict values through Params.put and receives dict values from Params.get. It begins all false, changes a feed to true only after one full frame with matching IDs is sent to VisionIPC and cereal, changes true→false when that feed fails or disconnects, and is all false after orderly shutdown. Param writes occur only at initial state and transitions. | Read the Param as a Python dict before capture, after the first dual-published frame, after a controlled disconnect/failure, and after shutdown; compare its fields and frame IDs, and inspect writes or logs to confirm there is no per-frame update. | Side-camera feature owner |
| AC-4 | S1 — one-camera capture and publication | With one attached camera explicitly mapped to a service using its video-index0 by-path path, the sidecar captures 1280×720 MJPEG at requested 20 fps and publishes complete 1,382,400-byte NV12 payloads with nonzero data. At least one cereal frame ID matches a received VisionIPC frame ID. During five steady-state minutes, at least 6,000 frames publish, mean cadence is 20.0 fps, and no EOF-to-EOF interval exceeds 60 ms. | Run a five-minute capture on the available image node; record service, stream, frame IDs, payload size/content sample, timestamps, count, mean cadence, maximum interval, and decode errors. Treat the existing r1 capture as baseline only and rerun on r2 code. | Side-camera feature owner |
| AC-5 | S1 — explicit mapping and interface compatibility | Each position variable maps to its matching service and stream (7–10); by-path video-index0 and /dev/videoN are accepted, metadata video-index1 is rejected, and no position is derived from USB enumeration. Existing stream IDs 0–6 and cereal ordinals through 149 remain unchanged; new service declarations are appended for ordering only, with no claimed numeric service ID. | Configure one position at a time and inspect process arguments, emitted service/stream names, and generated registry. Verify by-path resolution targets the intended image node and not the metadata node. | Side-camera feature owner |
| AC-6 | S1 — worker isolation and reconnect | A configured unavailable path remains unavailable and retries at 1, 2, 4, 8, 16, then at most 30 seconds. Failure and reconnect of one path do not stop configured siblings or native camerad; a reconnected explicit by-path camera can return to true after a subsequent dual-published frame. | Run with two explicit paths, disconnect or stop one, inspect retry/backoff and false→true state after reconnect, and confirm sibling frames and camerad continue. The current one-device host can validate only the configured-missing-path branch unless a second device is attached. | Side-camera feature owner |
| AC-7 | S1 — resource envelope | Four configured feeds allocate three 1,382,400-byte VisionIPC buffers plus one scratch frame per feed: 16 image frames and 22,118,400 bytes (21.09 MiB) total application image payload. FFmpeg remains one thread per worker; no BGR copy or application queue is added. | Inspect buffer dimensions/counts and worker setup; verify (3 + 1) × 4 × 1,382,400 = 22,118,400 bytes. Exclude decoder internals, kernel/V4L2 buffers, metadata, and other process RSS. | Side-camera feature owner |
| AC-8 | S1 — four-camera performance gate | Before performance sign-off, four assigned cameras sustain requested 20 fps each for 30 minutes: at least 36,000 successfully published frames per camera and 144,000 total, with per-camera cadence, frame gaps, and CPU utilization/headroom recorded. No four-camera pass is claimed from the present one-decoder evidence. | Run all four configured image nodes concurrently on target hardware for 30 minutes; analyze IDs and monotonic EOF times per feed and attach a CPU report. This check is pending four attached decoders and target hardware. | Side-camera feature owner |

## Rollout and rollback

| Stage | Mechanism | Advance condition | Rollback |
|---|---|---|---|
| Build and local bring-up | Default-on manager entry; use explicit paths for whichever image nodes are attached. Verify empty configuration before capture. | AC-1 through AC-5 pass on the current one-camera target where hardware allows; AC-6's two-camera reconnect case and AC-8 remain explicitly pending if devices are unavailable. | Set SIDECAMERAD=0 and restart the manager; native camerad remains active under its prior policy. |
| Four-camera sign-off | Keep the explicit per-position mapping and run AC-8 on target hardware with four devices. | AC-8 passes and the result is reviewed by the feature owner. | Set SIDECAMERAD=0 and restart the manager. No data migration or camera-consumer rollback is required. |

No rollout or validation step installs a package. Existing FFmpeg is the only decoder dependency.

## Risk handling

| Risk | Kind | Severity | Consequence | Mitigation | Owner |
|---|---|---|---|---|---|
| Only one decoder is present in current evidence; four-stream cadence and CPU headroom are unknown. | Technical dependency | Major | The target may not sustain 80 aggregate frames/s. | Make AC-8 a required performance sign-off and preserve the one-camera scope of AC-4. | Side-camera feature owner |
| Physical port paths identify USB topology but not camera sides. | Integration | Major | Incorrect explicit assignment would publish a valid image under the wrong service name. | Require an operator-supplied path per service; recommend video-index0; do not infer the side from enumeration; verify the chosen port during installation. | Integrator |
| Default-on process has no path configuration in some deployments. | Operational | Major | Returning normally would leave the manager's non-restarting process entry expected but dead. | Keep the process alive on bounded signal waits, publish all false, and warn once per unconfigured feed. | Side-camera feature owner |
| Device path and MJPEG are untrusted input to FFmpeg. | Security | Major | Invalid path handling or decoder failure could affect the sidecar or consume resources. | Follow security-seed-r2.md: validate a resolved image character device, pass the path as a single direct-argv argument, keep one decoder thread and fixed buffers, close/reap and retry per worker, and keep publication host-local. Record any unsupported child resource limit as residual risk. | Build/security owner |
| Cross-channel sends are not transactional. | Consistency | Minor | A consumer may observe one transport before the other for a frame. | Mark availability true only after both send calls complete; treat it as readiness, not subscriber delivery acknowledgement. | Side-camera feature owner |

## Decision Register

| Decision | Status | Chosen | Source | Rejected options / rationale | Owner | Reopen trigger |
|---|---|---|---|---|---|---|
| Separate process and native camera boundary | Resolved | Keep sidecamerad separate and leave native camerad unchanged. | Intake D-1; current scope change | Replacing or reconfiguring native capture changes the prior behavior and is not required. | Feature owner | A new approved consumer requires integration with native capture. |
| Sidecar process default | Resolved; latest scope supersedes intake D-5's old opt-in setting | Enable when SIDECAMERAD is unset under driverview; retain 0 as an opt-out. | Current scope change and requirements brief r2 | Making the user set 1 contradicts the latest instruction. | Feature owner | Reopen only through a later approved scope decision. |
| Four availability states | Resolved | Use one SideCameraAvailability JSON value with stable service-name boolean fields and transition-only writes. | Current scope change; architecture/interface r2 | Four separate Params or per-frame writes add unnecessary state and churn. | Feature owner | A downstream consumer needs richer status/error details. |
| Device position identity | Resolved | Keep explicit side-to-path assignments; recommend a by-path video-index0 symlink, accept /dev/videoN. | Requirements r2; observed host topology | USB-order guessing cannot establish which physical camera faces which side; video-index1 is metadata-only on the observed decoder. | Integrator | A later deployment provides a verified, labeled discovery contract. |
| Service ordering | Resolved | Append new services at the end for a minimal additive ordering change. | Parent source audit; current service registry | This is not needed to preserve numeric service IDs; index is unused and C++ registry uses service names. | Build owner | Repository service registry semantics change. |
| Capture mode and dependency | Resolved | Keep 1280×720 MJPEG-to-NV12 using existing FFmpeg at requested 20 fps; add no packages. | Intake D-3/D-4; current scope | Package or BGR conversion expands CPU/memory and dependency footprint. | Feature owner | The target lacks the existing FFmpeg binary or the specified V4L2 mode. |
| Resource allocation | Resolved | Three VisionIPC frames and one reusable scratch frame per configured feed; keep one FFmpeg thread per worker. | Prior approved architecture, retained in r2 | Extra application queues or conversion copies increase memory without a consumer requirement. | Feature owner | VisionIPC contract or target hardware evidence requires a revised bound. |
| Four-camera soak | Deferred performance sign-off | Require the 30-minute AC-8 check before claiming four-camera cadence/headroom. | Prior r1 plan carried forward; current hardware evidence | A one-camera smoke cannot establish concurrent throughput. | Feature owner | Complete AC-8 on target hardware. |
| USB auto-discovery, persistent assignment, and side inference | YAGNI-deferred | Keep side mapping explicit. | Intake D-14; current scope | No unique decoder serial or verified side labels are available. | Feature owner | A deployment requires unattended assignment and supplies stable identity labels. |
| Logging, encoding, model/UI use, synchronization, transforms, or persistent image storage | Deferred/non-goal | Keep publication limited to local cereal and VisionIPC interfaces. | Intake D-7–D-9/D-13 | These consumers are outside the MVP and add unrelated behavior. | Feature owner | A named downstream consumer is approved. |

## Migration / deprecation

No existing process, public API, consumer, or persisted image format is removed or replaced. The only changed operational default is that the sidecar starts under its current driverview condition without requiring SIDECAMERAD=1; setting SIDECAMERAD=0 and restarting the manager disables it. Native camerad needs no migration or rollback.

## Non-goals and deferred work

- Four-camera performance and CPU headroom are unproven until AC-8 passes.
- USB auto-discovery and inferred side labels remain out of scope.
- No native camerad, model, UI, loggerd, encoderd, network, or persistent-image integration.
- No cross-camera synchronization or camera transforms.

