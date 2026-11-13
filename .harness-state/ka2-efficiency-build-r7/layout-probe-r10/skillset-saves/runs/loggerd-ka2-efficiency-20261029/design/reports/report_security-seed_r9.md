# Security seed — KA2 rotation liveness, r9

**Stage:** design `security-seed`  
**Return boundary:** `design-to-build`  
**Architecture:** `report_architecture_r9.md`, r9, SHA-256 `a788e36c49a30aceb4e8bacaa97040db7eb20c5c0d5340f33fe878fc9a68b886`  
**Interfaces:** `report_interfaces_r9.md`, r9, SHA-256 `2ef10c8c8964203da994dd6d1b4a47db238820c3210ec365ca700f35365f007c`  
**Decision:** assessed no trust-boundary movement (`applicable: false`, `boundaries: []`). This is a determination, not a waiver.

## Assessment

The r9 correction expands the loggerd-owned `LoggerdRotationReport` diagnostic from five to exactly seven top-level fields: `route`, `epoch`, `segment`, `degraded`, `missing_streams`, `elapsed_ms`, and `known_drop_counts`. The nested object has exactly five fixed nonnegative integer counters: `stale_video_messages`, `future_video_queue_drops`, `audio_no_writer_events`, `audio_expired_events`, and `audio_capacity_drops`. It reports loggerd-observed drops since the previous report, or since route startup for the first report. The report is emitted after deterministic-KA2 runtime commits and is non-authoritative. The existing `LoggerdRotateState`, `LoggerdSegment`, and `LoggerdSegmentEpoch` remain the sole segment controls.

This additive diagnostic remains inside the current local Params boundary. It adds no public endpoint, external actor, credential, privilege transition, cereal schema, or separate IPC channel. The fields contain route and stream identifiers, elapsed time, health state, and bounded counters; they must not contain frame content, secrets, or filesystem paths. Invalid or stale-route diagnostic data must never affect rotation. Encoder-side MPP rejection, output stalls, drain failures, pending-frame drops, and retry counts remain per-output encoderd logs/counters; they are not represented as loggerd drop counts.

The audio queue's overflow and expiry rules are a bounded data-integrity control. Preserve the timestamp-ordered FIFO queue; before insertion, reject the newest complete `rawAudioData` event atomically if it would exceed the 256 KiB cap. Do not split the payload, evict older events, or mutate the existing queue on rejection; increment `audio_capacity_drops` once for that event. For an accepted queued event, the terminal causes are disjoint: whichever happens first, five-second expiry or a commit that makes the event's tagged segment unretainable, owns the one count. If both deadlines are observed in one event-loop iteration, compare monotonic deadlines and classify an exact tie as expired. This preserves one-count attribution and prevents a later no-writer count for an already-expired event.

Therefore this design introduces or moves no adversarial trust boundary. This finding is limited to the architectural change; it is not a security qualification of the existing DMA-BUF path or its platform implementation.

## Controls the build must preserve

- Keep D-28's full-resolution MPP input DMA-BUF-only: validate the fd, make one MPP DMA import attempt, and drop a rejected frame. Do not copy rejected data, restore a selector, or use pointer-based import retry. Keep qRoad's existing RGA resize path.
- Keep all segment authority in the existing three Params values. Treat the exact seven-field `LoggerdRotationReport` as local diagnostics only; validate its fixed schema and configured stream names, ignore a report for another route, and never use any report field to decide a rotation or filesystem path.
- Keep `known_drop_counts` to the five fixed keys and count only loggerd-observed terminal drops, once per defined message/event. Do not attribute MPP import rejection, encoder stalls, drains, pending IDs, or retries to loggerd. Keep those encoder losses in rate-limited per-output logs/counters.
- Keep the qRoad audio queue in timestamp-ordered FIFO order. Reject the newest whole event when it would exceed 256 KiB; do not split, evict older entries, or change the queue on rejection. Count it once as a capacity drop. For accepted events, expiry wins an exact tie against the segment-commit/no-writer terminal condition; otherwise the first terminal cause owns the single count.
- Bound future-video queues, audio buffering, pending encoder identities, retry schedules, and diagnostic output. Retire every accepted frame identity exactly once as a published packet or an explicit drop.
- Compare every queued encoded index against the authoritative segment before writing. Drop stale items and retain only valid bounded future items; never relabel a packet or audio sample into a different segment. Keep stream names from the static encoder inventory and do not derive paths from packet or diagnostic content.
- In the test fixture, set `LOG_ROOT`, `PARAMS_ROOT`, and `OPENPILOT_PREFIX` before constructing Params or starting processes. Keep the DMA-heap publisher and invalid-input/fault seams test-only and isolated. Retain DMA-BUF fds/mappings until VisionIPC and encoderd stop, use `DMA_BUF_IOCTL_SYNC` around CPU writes, and call `VisionIpcServer::send(..., sync=false)`.

## Residual risks and qualification limit

- **Info:** The synthetic DMA-heap fixture does not qualify the live V4L2 exporter, its DMA-BUF fence/synchronization semantics, or buffer reuse lifetime. Keep this limitation visible in build evidence; do not generalize synthetic success to the production camera/driver image.
- **Info:** This design also does not qualify MPP driver behavior on the live DMA-BUF path. Keep production camera/driver qualification separate from synthetic fixture results.
- **Info:** Whether KA2 RGA accepts the ordinary shared-memory fixture's mapped address remains target-specific. If it does not, use the architected test-only per-output rejection seam for full lifecycle containment with a valid DMA-BUF source, while retaining direct invalid-fd/shared-memory MPP rejection evidence.
- **Info:** A rejected or unavailable track may be absent or partial, and bounded qRoad audio can be dropped after expiry. Logs and the diagnostic report must describe loss without presenting incomplete media as complete.
- Reassess this seed if the report is exposed beyond the existing local Params readers or is used to drive control decisions; either change would alter the trust/data-flow analysis.

This is design-level evidence only. It does not grade an implementation, claim a clean scan, or qualify the live V4L2 camera path. The typed record is `artifacts/security_seed_r9.json`.
