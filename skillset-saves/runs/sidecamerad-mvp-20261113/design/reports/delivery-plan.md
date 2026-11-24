# Delivery Plan — opt-in sidecamerad MVP

**Revision**: 13  
**Architecture**: `reports/report_architecture.md` @ `da35a916fdf28e9ab1a5a5849f598c9783232026fb9d4e8e465d47033ee68ca5`  
**Interfaces**: `reports/report_interfaces.md` @ `7b6299040d93ab96a875639f24656988ef3ac97f0643987c146faf4855a62755`  
**Requirements**: `reports/requirements-brief.md` @ `1d0813a7fca5e7cee8d987c80842c22fd52ef8dca7dcdd4197632d3fe3040288`  
**Security seed**: `reports/security-seed.md` @ `edf6e4c50d7d125a710a14b68182d32a3d929eec3a2cae6a496d9e30bf798a64`. The build owes the seed's path validation, direct fixed-argv FFmpeg launch without a shell, least privilege and descriptor/resource limits, exact-size complete-frame checks, per-camera retry, and host-local IPC constraints.

## Objective and scope

Deliver one additive `sidecamerad` slice, enabled only by `SIDECAMERAD=1`, alongside unchanged native `camerad`. Explicit optional `SIDECAM_FRONT_LEFT`, `SIDECAM_FRONT_RIGHT`, `SIDECAM_BACK_LEFT`, and `SIDECAM_BACK_RIGHT` paths feed separate state topics and VisionIPC streams. Use the existing FFmpeg executable to decode 1280×720 MJPEG directly to NV12, with no added packages, BGR conversion, or application frame queue.

Keep each configured camera independent: a missing path omits that worker; device or FFmpeg errors close and reap that camera's child, log, and retry with bounded backoff (1, 2, 4, 8, 16, then 30 seconds maximum). Publish three VisionIPC buffers per configured stream and one reusable scratch frame per worker. At completed read, use host-monotonic `timestampEof = monotonic_ns` and nominal `timestampSof = timestampEof - 50,000,000 ns`; do not promise cross-camera synchronization.

## Milestone path

| # | Milestone | Delivery slices | Depends on | Decision gate before it | Target |
| - | --------- | --------------- | ---------- | ----------------------- | ------ |
| 1 | Opt-in side-camera feed | S1 — one additive MVP slice | Approved process, cereal, VisionIPC, and error contracts; existing FFmpeg and explicitly configured V4L2 paths | Implementation spec preserves IDs 0–6, ordinals through 149, native `camerad` lifecycle, and the approved 3-buffer-plus-scratch budget | A configured camera publishes its matching state and distinct stream at the 20 Hz target; full four-camera performance sign-off remains gated on target-hardware soak evidence |

## Ordered work and dependency map

All work below belongs to S1; these are ordered implementation dependencies, not separately shippable slices.

| Order | Work | Depends on | Why this order is required |
| ----- | ---- | ---------- | ------------------------- |
| 1 | Add the four `FrameData` arms at ordinals 150–153, four 20 Hz service declarations, stream IDs 7–10 in both enum surfaces, and the `sidecamerad` ZMQ endpoint-name allowance. | Existing cereal and VisionIPC declarations | Consumers and publisher must agree on stable topic, ordinal, endpoint, and stream mappings while all existing numeric values remain unchanged. |
| 2 | Register the separate `sidecamerad` process under existing `driverview` lifecycle, gated only by `SIDECAMERAD=1`; parse the four optional explicit paths and start one isolated worker per configured camera. | Order 1 contracts; process-manager boundary in IF-1/IF-2 | Process opt-in and partial configuration must not change native `camerad` policy or make siblings depend on one device. |
| 3 | Implement each worker's one-thread FFmpeg MJPEG-to-NV12 pipe, complete-frame reads into its reusable buffer, monotonic EOF/SOF metadata, per-position publication, and bounded close/reap/retry handling. | Order 2 worker/configuration ownership; IF-3/IF-4/IF-5 | The stream and metadata mapping, frame sizing, and worker failure behavior are fixed by the interfaces. |
| 4 | Verify no-path/disabled behavior, then run the available single-camera smoke; execute the four-camera target-hardware soak before any performance sign-off. | Orders 1–3; one known V4L2 device for smoke; four devices and target device for performance gate | A single-camera result establishes bring-up only. It cannot establish concurrent 80 fps throughput or CPU headroom. |

## Acceptance criteria

| ID | Slice | Observable condition | Check that proves it | Acceptor |
| -- | ----- | -------------------- | -------------------- | -------- |
| AC-1 | S1 — opt-in feed | With `SIDECAMERAD` unset or `0`, `sidecamerad` is absent; with `SIDECAMERAD=1`, it runs as a separate process and native `camerad` retains its existing lifecycle and policy. | On the target build, inspect manager process state and startup logs in both flag states; confirm `camerad` remains present under its pre-change policy and only `sidecamerad` follows the new flag. | side-camera feature owner |
| AC-2 | S1 — opt-in feed | For every configured position, the matching `*CameraState` topic maps to exactly its assigned distinct VisionIPC stream (front-left 7, front-right 8, back-left 9, back-right 10); unset positions publish neither frames nor state during absence/backoff. | Configure one path at a time and then a partial set; inspect emitted service names, stream IDs, and frame IDs against the fixed mapping; confirm IDs 0–6 and ordinals through 149 are unchanged. | side-camera feature owner |
| AC-3 | S1 — opt-in feed | The available single-camera smoke, when the current camera is attached, runs for five minutes and records at least 6,000 successfully decoded/published frames, a 20.0 fps mean, and no EOF-to-EOF interval over 60 ms. This is a one-camera smoke result only. | Capture frame IDs and monotonic EOF timestamps for five steady-state minutes; calculate frame count, mean cadence, maximum interval, and decode failures from the run record. | side-camera feature owner |
| AC-4 | S1 — performance gate | Before four-camera performance sign-off, all four configured cameras complete a 30-minute target-hardware soak at 20 fps each: at least 36,000 successfully published frames per camera and 144,000 total; the report records per-camera cadence, frame gaps, and CPU utilization/headroom. No four-camera success is claimed before this check passes. | Run all four explicit V4L2 paths concurrently on target hardware for 30 minutes; analyze each stream's frame IDs and monotonic EOF timestamps, and attach the CPU measurement report. | side-camera feature owner |
| AC-5 | S1 — opt-in feed | Four configured streams allocate exactly three 1,382,400-byte VisionIPC frames plus one 1,382,400-byte scratch frame per camera: 16 frames and no more than 22,118,400 bytes (21.09 MiB) of image payload, below the 27,648,000-byte requirements ceiling. Decoder, kernel/V4L2, metadata, and other RSS are excluded from this payload bound. | Inspect allocation counts and dimensions for four configured workers and verify the payload calculation: `(3 + 1) × 4 × 1,382,400 = 22,118,400` bytes. | side-camera feature owner |
| AC-6 | S1 — opt-in feed | Every published frame has a monotonically increasing frame ID, host-monotonic EOF timestamp, and nominal SOF exactly 50,000,000 ns before EOF; no cross-camera alignment is implied. | Inspect frame metadata from the single-camera smoke and verify monotonic EOF values and `timestampSof = timestampEof - 50,000,000 ns` for each frame. | side-camera feature owner |
| AC-7 | S1 — opt-in feed | An unavailable or failing camera is isolated: its FFmpeg child is closed/reaped and retried at 1, 2, 4, 8, 16, then no more than 30 second intervals; configured siblings and native `camerad` continue. | Inject an unavailable path or stop one camera/FFmpeg child; inspect per-worker logs and retry timestamps and confirm a second configured camera and `camerad` continue publishing/running. | side-camera feature owner |

## Rollout and rollback shape

| Stage | Audience | Mechanism | Rollback | Trigger to advance |
| ----- | -------- | --------- | -------- | ------------------ |
| Disabled baseline | All deployments | Leave `SIDECAMERAD` unset or set to `0`; native camera startup remains under current policy. | No action. | Process-manager check confirms the sidecar is absent and `camerad` is unchanged. |
| Single-camera bring-up | Integrator with the currently available camera and target build | Set `SIDECAMERAD=1` and only the known position's V4L2 path; run the five-minute smoke. | Unset/disable `SIDECAMERAD` and restart the manager; native `camerad` remains enabled. | AC-1 through AC-3, AC-5 through AC-7 pass. If the camera cannot be attached, record the smoke as pending and do not claim hardware validation. |
| Four-camera performance sign-off | Feature owner on target hardware with four configured cameras | Keep the same opt-in configuration and run the 30-minute concurrent soak; collect cadence, gaps, and CPU headroom. | Disable `SIDECAMERAD` and restart the manager. No consumer migration or native-camera rollback is needed. | AC-4 passes. Until then, four-camera throughput and CPU headroom remain unverified and no performance sign-off is granted. |

## Risk handling

| Risk | Kind | Severity | Consequence if it lands | Mitigation | Owner |
| ---- | ---- | -------- | ----------------------- | ---------- | ----- |
| Concurrent four-camera throughput and CPU headroom are unmeasured; single-camera prior art does not prove them. | Technical dependency | Major | The 80 fps aggregate target may not be sustainable on target hardware. | Make the 30-minute four-camera soak a required performance gate; record per-camera cadence, gaps, and CPU headroom before sign-off. | side-camera feature owner |
| Availability of four cameras and target hardware for soak is not established by the approved evidence. | Staffing | Major | Functional code may be ready while performance evidence remains pending. | Secure the target device and four V4L2 sources before scheduling the performance gate; keep the runtime opt-in until then. | side-camera feature owner |
| Existing FFmpeg executable or target V4L2 mode may differ from the prior one-camera environment. | Technical dependency | Minor | A worker may fail to decode the specified 1280×720 MJPEG mode. | Confirm existing binary and mode on target hardware during bring-up; do not install packages; isolate and report per-camera failures. | side-camera feature owner |
| Device paths and MJPEG data are untrusted inputs to an external FFmpeg child. | Technical dependency | Major | Malformed configuration or input could affect the sidecar, while decoder memory or execution time may exceed the application's fixed image-payload budget. | Follow `security-seed.md`: validate path shape and pass it only as data to a fixed direct-argv command without a shell; retain unprivileged identity, close unrelated descriptors, enforce child resource limits where supported, accept only complete 1,382,400-byte frames, cap workers/buffers and retry per camera, and report any unenforceable decoder-resource exposure for owner review. Keep publication on the host-local sidecar IPC path. | side-camera feature owner |
| A stream ID, cereal ordinal, or endpoint-name collision could break consumers or alternate ZMQ transport. | Technical dependency | Major | Existing stream values or consumers may be disrupted. | Preserve IDs 0–6 and ordinals through 149; append only approved IDs 7–10 and ordinals 150–153; verify both VisionIPC enum surfaces and endpoint allowlist. | side-camera feature owner |

## Decision Register

| Decision | Status | Chosen | Source | Rejected options (why) | Owner | Reopen trigger / latest safe point |
| -------- | ------ | ------ | ------ | ---------------------- | ----- | ---------------------------------- |
| D-1 / D-5 process boundary and rollout | resolved | Separate `sidecamerad`, enabled only by `SIDECAMERAD=1`, with optional explicit paths and partial configuration; native `camerad` remains unchanged. | user | Replacing `camerad` or requiring all cameras would change existing behavior or block the current partial setup. | side-camera feature owner | Reopen before implementation spec freeze only if the manager cannot support the approved separate opt-in process. |
| D-2 topic and stream contract | resolved | Four position-specific state topics and unique streams 7–10 at 20 Hz. | user | Sharing a stream would lose the required position mapping; changing existing IDs risks compatibility. | side-camera feature owner | Reopen before interface implementation if repository constraints contradict the approved mapping; return any boundary change to architecture. |
| D-3 / D-4 capture mode and dependency | resolved | 1280×720 MJPEG input, existing FFmpeg, direct NV12 output, one FFmpeg thread, no new package. | prior-artifact / user | Package installation or BGR conversion adds dependencies and memory/copy cost outside the approved MVP. | side-camera feature owner | Reopen during target bring-up if the existing binary or explicit V4L2 mode is unavailable; do not silently substitute another path. |
| D-6 payload allocation | resolved | Three VisionIPC buffers and one reusable scratch frame per configured camera; four-camera payload ceiling 22,118,400 bytes. | delegated-default / prior-artifact | Four VisionIPC buffers plus scratch exceeds the selected architecture's tighter allocation without an approved need. | side-camera feature owner | Reopen before implementation if the fixed allocation cannot satisfy the VisionIPC publisher contract. |
| Four-camera soak duration | resolved | Use a 30-minute concurrent soak as the minimum performance-sign-off run. | delegated-default | A single-camera run or shorter bring-up smoke cannot establish sustained concurrent throughput. | side-camera feature owner | Extend before the run if target thermal or operational policy requires a longer soak; no four-camera performance claim before the gate. |
| CPU headroom measurement | deferred | Record CPU utilization and headroom during the four-camera soak; no numeric utilization ceiling is introduced because the approved target hardware budget is unspecified. | prior-artifact | Inferring a CPU ceiling without the target-device operating budget would create an unsupported requirement. | side-camera feature owner | Set/record the applicable device budget before interpreting the soak for deployment sign-off. |
| Logging/encoding, model/UI use, synchronization, and camera transforms | deferred | Keep these integrations outside S1; independent streams use the approved host timestamps. | prior-artifact | These consumers are not required to validate capture and would expand this MVP. | side-camera feature owner | Reopen in design before an approved downstream consumer is added. |
| Replace native camera, require four devices at startup, or add an image package | rejected | Keep the additive sidecar, partial startup, and existing FFmpeg path. | user | Each option conflicts with approved scope or adds unsupported deployment requirements. | side-camera feature owner | Reopen only through a new approved scope and architecture revision. |
| Auto-discovery and persistent camera assignment | yagni-deferred | Keep explicit path-to-position configuration for this MVP. | prior-artifact | The current setup can be assigned explicitly; discovery and persistence add behavior without a current requirement. | side-camera feature owner | Reopen if a deployment requires unattended discovery or persistent assignment. |

## Migration / deprecation

Not applicable — this slice adds an opt-in process, services, and streams. It removes or replaces no existing process, API, or consumer workflow. Runtime rollback is to disable `SIDECAMERAD` and restart the manager.

## Non-goals and YAGNI deferrals

| Deferred | Why it is safe to defer | Reopen trigger | Owner |
| -------- | ----------------------- | -------------- | ----- |
| Model subscriptions, UI exposure, driving behavior, drive-log inclusion, or encoder output | S1 is complete when capture is published through position-specific state and VisionIPC streams. | An approved consumer requires one of these integrations. | side-camera feature owner |
| Cross-camera synchronization and camera-specific transforms | The approved contract promises independent streams and host timestamps only. | A consumer specifies alignment or orientation requirements. | side-camera feature owner |
| USB auto-discovery and persisted position assignment | Explicit path configuration supports the current MVP and partial setups. | Deployment requires unattended discovery or persistent mapping. | side-camera feature owner |
