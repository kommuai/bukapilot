# Architecture — sidecamerad default-on availability MVP

**Run/revision:** `sidecamerad-mvp-20261113` / r2  
**Requirements:** [requirements-brief-r2.md](requirements-brief-r2.md) · SHA-256 `bff3dc6825e257d653772d43d64a3d7f6221ed91bea0dff4c08ce50b82d7f748`  
**Prior approved design:** r1; updated only where the default-on lifecycle, availability Param, and explicit physical-port paths require it.

## Components and data flow

| Component | Responsibility and failure boundary |
|---|---|
| Process manager | Starts `sidecamerad` by default when the existing `driverview` condition is true. An explicit `SIDECAMERAD=0` may disable it; the unset/default case enables it. Native `camerad` retains its existing entry, condition, and behavior. |
| `sidecamerad` | Owns the one `SideCameraAvailability` JSON value, four optional position workers, the side-specific VisionIPC streams, and the matching cereal publishers. Camera failures stay local to one worker. |
| Device configuration | Maps each position only from its matching `SIDECAM_*` variable. Recommend `/dev/v4l/by-path/...-video-index0` for stable physical-port identity; continue accepting `/dev/videoN`. Do not assign a side from USB enumeration order. |
| Capture worker and FFmpeg | One worker and one fixed-argv FFmpeg child (one thread) per configured input. Decode 1280×720 MJPEG to NV12 into a reusable frame buffer. Complete-frame timeout, decoder exit, or disconnect marks that feed unavailable, closes/reaps the child, then retries with bounded backoff. |
| Params availability writer | Owns a four-boolean Python dictionary keyed by the exact cereal service names. A mutex protects state transitions and the synchronous `Params.put(dict)` call; it writes only when a boolean changes. The Params layer serializes the dictionary as JSON; do not pass a JSON text string to Python `Params.put()`. There is no per-frame Params write because `Params.put()` is synchronous. |
| VisionIPC and cereal | Publish the same monotonically increasing per-position frame ID to its isolated stream and its `FrameData` service. Availability becomes true only after both publication calls complete for that frame. Cross-channel delivery is not transactional. |

For each configured `SIDECAM_FRONT_LEFT`, `SIDECAM_FRONT_RIGHT`, `SIDECAM_BACK_LEFT`, or `SIDECAM_BACK_RIGHT` path, the data path is V4L2 MJPEG 1280×720 at 20 fps → existing FFmpeg → raw NV12 → the matching VisionIPC stream and cereal `FrameData` topic. At read completion, `timestampEof` is host monotonic time and `timestampSof` is nominally `timestampEof - 50,000,000 ns`; this is not an exposure timestamp or cross-camera synchronization guarantee.

## Startup, availability, and shutdown

1. Manager initialization clears keys flagged `CLEAR_ON_MANAGER_START` and writes declared default values. Declare `SideCameraAvailability` as `JSON` with the compact all-false JSON literal as its C++ key default. The Python manager reads that default as a dictionary and writes it through `Params.put(dict)`, so consumers see four unavailable feeds before sidecar capture begins.
2. At sidecar startup, initialize the in-memory dictionary and synchronously write the same all-false dictionary through Python `Params.put()`; do not pass a JSON text string. Configure signal handlers before selecting the configured-camera or idle path.
3. If no `SIDECAM_*` values are configured, log exactly one missing-feed warning for each unconfigured service, keep every availability value false, create no camera worker or VisionIPC buffers, and wait using a bounded `stop_event.wait(60)` loop that wakes only on timeout or signal until SIGINT or SIGTERM. `main()` must not return on this normal default-on path because the manager entry does not restart a normally exited process.
4. For configured paths, start only their workers and buffers. A worker marks its field true after a full 1,382,400-byte NV12 frame has been published to both VisionIPC and cereal with the same `frameId`. A worker marks the field false on capture startup/read failure, timeout, decoder exit, disconnect, or stop; repeated unavailable state causes no extra Params write. Configured missing by-path nodes stay in their worker retry loop, so the same explicit port can become available again after reconnect.
5. On an orderly SIGINT/SIGTERM shutdown, stop and join all workers, reap decoder children, then publish one final all-false JSON object. The next manager startup also resets the Param default. An uncatchable process termination cannot execute this final write and may leave stale runtime data until manager startup; it is an operational limitation, not a captured-frame state.

## Invariants and resource envelope

- Fixed position mapping: `frontLeftCameraState` ↔ stream 7 ↔ `SIDECAM_FRONT_LEFT`; front-right ↔ 8; back-left ↔ 9; back-right ↔ 10. Keep existing stream IDs 0–6, cereal ordinals, and native camera behavior unchanged. Add side services at the end of the service declarations for minimal diff/order; `SERVICE_LIST` enumeration index is unused and C++ registry lookup is by service name, so there is no numeric service-ID or ABI shift.
- Add `FrameData` event arms at ordinals 150–153 after `audioFeedback @149`; each service remains 20 Hz and non-logged. `FrameData.image` stays unset; pixels remain in VisionIPC.
- Create three VisionIPC buffers for each configured stream and one reusable scratch frame per worker. At four cameras, the image-payload allocation is `(3 shared + 1 scratch) × 4 × 1,382,400` = **22,118,400 bytes (21.09 MiB)**, excluding FFmpeg, V4L2/kernel buffers, metadata, and other RSS. Use no BGR conversion or application frame queue.
- Bound capture concurrency to at most four workers and one FFmpeg thread per worker. Keep the existing bounded retry schedule of 1, 2, 4, 8, 16, then 30 seconds maximum. No additional package, persistent image sink, remote listener, or new consumer is introduced.
- Treat device paths and MJPEG bytes as untrusted local input. Validate each configured path as an accepted V4L2 image path under `/dev` and as a character device after resolution; reject metadata-only `video-index1`. Pass the validated path as a single direct-argv argument to the fixed FFmpeg executable, without a shell or configuration-controlled options.

## Decision register

| Decision | Status and basis |
|---|---|
| Default-on sidecar under the current `driverview` lifecycle; optional explicit `SIDECAMERAD=0` opt-out | Resolved from the latest user instruction; supersedes r1 opt-in behavior. Native `camerad` stays independent and unchanged. |
| Explicit position-to-path assignment, recommending a `video-index0` by-path symlink | Resolved from user scope and observed USB topology; `/dev/videoN` remains accepted. USB order cannot identify a side. Same-port reconnect recovery is inferred from physical topology and remains a build verification item. |
| One `SideCameraAvailability` JSON Param, all-false on manager/sidecar startup, transition-only updates | Resolved for minimal state surface. True follows successful publication to both interfaces; false follows feed loss/failure or orderly shutdown. |
| Stay alive when no paths are configured | Resolved to avoid a normal exit while the default-on manager entry remains expected to run; only signal shutdown ends the idle wait. |
| Keep the r1 capture, stream, and resource bounds | Retained: fixed 1280×720 MJPEG-to-NV12, target 20 fps, streams 7–10, no packages, fixed buffers, and no image queue. |
| Do not add physical auto-discovery, side inference, synchronization, UI, or persistent image storage | Deferred/YAGNI. Each requires an explicitly approved consumer or mapping requirement. |

## Acceptance and limitations

- With `SIDECAMERAD` unset, verify manager default enablement while native `camerad` remains independently managed; verify explicit opt-out if retained.
- Start with no camera variables: confirm four one-time missing-feed warnings, a live process awaiting SIGINT/SIGTERM, and the all-false JSON value.
- With one connected image node configured by its `video-index0` by-path path, verify V4L2 1280×720 MJPEG capture at the requested 20 fps, valid cereal metadata and a nonzero VisionIPC NV12 payload, with at least one matching `frameId` across both interfaces.
- Verify availability false→true after the first complete dual publication, true→false on disconnect/failure, and false again on orderly shutdown; confirm a configured missing by-path path retries and can recover after replug. This is build validation, not evidence produced by this architecture stage.
- Only one USB decoder is attached in current evidence. It supports a one-camera smoke test; it does not establish four-camera throughput, CPU headroom, disconnect/reconnect behavior, or sustained 20 fps per camera. Those remain target-hardware acceptance work.
