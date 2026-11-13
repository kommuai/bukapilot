# Architecture decision — KA2 rotation liveness and encoder fault containment, r8

**Revision:** r8  
**Requirements brief:** reports/report_research_r8.md (sha256 b89553772eb8c1a829a68f54d0b8db759dd7a3976a88c3e8045446dd983cb1ab)  
**Decision scope:** D-29; loggerd/encoderd lifecycle and synthetic VisionIPC regression coverage  
**Stack:** existing C++ / KA2 RK3588 stack unchanged

## Outcome

Keep full-resolution MPP input strictly DMA-BUF-only: validate the fd, make one MPP DMA import attempt, and drop/count a rejected frame. Do not restore a copy path, environment selector, or pointer-based retry. Keep qRoad's RGA resize as its existing downscaled path.

The liveness and recovery changes below apply only while the deterministic KA2 protocol is active: RK3588 with ENCODERD_DETERMINISTIC_DISABLE unset or zero; in a loggerd test, LOGGERD_TEST_DETERMINISTIC=1 must also be set for this protocol to be active. Non-KA2 runs and RK3588 runs with ENCODERD_DETERMINISTIC_DISABLE=1 retain their current fallback behavior.

Within that protocol:

1. loggerd starts a runtime rotation at the existing segment boundary, commits early when every configured stream reports a packet for the next epoch, and otherwise commits after a **5,000 ms deadline from started**. The timer runs even when no messages arrive. The deadline is checked against a monotonic clock and the poll wake is clipped to the next deadline, with at most a 50 ms timer tick.
2. encoderd tracks each configured output's actual emitted-packet progress and recovery separately. A successful MPP frame submission is not a packet heartbeat. Error, silent-output, and drain-timeout recovery reopen or quarantine only the affected output. A VisionIPC/camera reconnect may reopen every output because its buffer/fd generation has changed.
3. A missed output is reported as loss. Its media file is naturally absent if no first keyframe arrived, or partial if output stopped after writing began. Do not create an empty placeholder file or backfill frames. Healthy tracks keep their existing codec, index, timestamp, and segment behavior.
4. Add a test-only C++ VisionIPC publisher backed by the KA2 system DMA heap. It registers external VisionBuf objects and exercises the full loggerd+encoderd lifecycle with the same four-buffer ring count as the RK camera. Keep a separate rejected shared-memory/invalid-input case and a targeted silent-output/drain-timeout case.

Keep the startup logger_rotate() call writer-free and do not emit LoggerdRotationReport from it. Place the new commit hook outside that shared helper and invoke it only for a runtime commit while the deterministic KA2 protocol above is active. The non-deterministic fallback branch, including its existing writer/queue/audio handling and timeout policy, remains unchanged.

The 5-second ceiling uses observed source limits: the test fixture documents at least 3 seconds before qcamera output because of lookahead/reordering; encoderd polls segment authority every 100 ms and the MPP drain limit is 900 ms. Five seconds covers those delays plus one second of scheduling margin. Normal successful deterministic-KA2 rotation must complete before the deadline; the deadline is for a missing/failed output and is not a new steady-state delay.

## Why this change is needed

- In system/loggerd/loggerd.cc:57-73, deterministic KA2 mode publishes started at the segment boundary and returns without the non-deterministic timeout. all_ready compares a count of next-segment packets with the full configured encoder count; handle_encoder_msg increments it only after an encoder emits such a packet.
- rotate_if_needed currently runs only while processing a received message. If all relevant output stops, even a newly added elapsed-time condition would not be evaluated unless the event loop calls it on a timer.
- system/loggerd/encoderd.cc:212-249,454-480 makes an MPP drain timeout return false; its callers set global do_exit, stopping the camera encoder thread and healthy sibling outputs.
- encoderd.cc:192-210,485-528 reopens all encoders when one output crosses ten failures or the aggregate output watchdog fires. Road and qRoad share one thread, so a road import failure can churn qRoad.
- mpp_encoder.cc:365-430 returns success after submitting an MPP frame, not after publishing a packet. The current frame_encoded aggregate watchdog therefore cannot detect an encoder that accepts frames but stops emitting. Empty payload handling also must retire or invalidate the associated pending frame so later packets cannot inherit its frame identity.
- The current KA2 rotation test is skipped because its create_buffers_with_sizes fixtures are ordinary shared-memory buffers. The 100/100 DMA-heap class smoke proves import/packet behavior but does not prove the loggerd/encoderd segment lifecycle.

## Components and ownership

| Component | Owns | Depends on | Failure behavior |
| --- | --- | --- | --- |
| loggerd deterministic-KA2 rotation coordinator | In the active deterministic KA2 protocol only: route segment ordinal, rotation epoch, start/deadline, per-stream readiness for that epoch, committed missing-stream report | Encoder message sockets; monotonic timer; existing Params authority | In that protocol, commits at all-ready or the 5-second deadline. A timeout commits a degraded segment and reports the missing set; it never waits forever. Other modes retain existing behavior. |
| encoderd deterministic-KA2 per-output supervisor | In the active deterministic KA2 protocol only: each output's consecutive input errors, last successful submission, last actual packet, pending frame IDs, retry/backoff state, and segment assignment | MPP/FFmpeg encoder instance; camera frame; loggerd segment Params | In that protocol, marks one output unavailable, counts its drops, and recovers it independently. A local failure does not set process-wide do_exit or reopen siblings. Other modes retain existing behavior. |
| MPP encoder instance | DMA-BUF validation/import cache, pending input-to-packet association, packet publication, segment number | One input VisionBuf fd and MPP | Rejects non-DMA-BUF or import failure without copying. Empty/invalid output is counted and cannot shift the identity of a later packet. On watchdog/drain failure it is closed without flushing its pending queue; pending IDs are counted once as dropped. |
| loggerd stream writer | Per-stream output writer and existing first-keyframe/audio rules | loggerd's committed segment; encoded packets and raw audio | On deterministic-KA2 runtime commits only, finalizes active writers and resets per-segment state so a silent stream cannot leave its old file open. Startup and non-deterministic fallback behavior remain unchanged. It creates no fake stream file when no valid first keyframe arrives. |
| DMA-BUF test publisher | Test-only buffer fds, mappings, NV12 contents/frame identity, VisionIPC server lifetime | /dev/dma_heap/system; C++ VisionIPC register_external_buffers | Owns/unmaps/closes allocations after encoderd and VisionIPC have stopped. No production camera or route root is used. |

## Data flow and rotation protocol

1. **Start boundary (active deterministic KA2 protocol only).** On the duration boundary, loggerd records the current monotonic start time and epoch, publishes the existing rotation state started, and begins the readiness deadline. Readiness is a set keyed by configured encoder service and tagged with the active epoch, not a cumulative counter. A packet from the next segment marks only that encoder ready for this epoch. The fallback protocol retains its existing readiness behavior.
2. **Encoder advance.** When the deterministic KA2 protocol is active, encoderd observes the existing LoggerdRotateState, LoggerdSegment, and LoggerdSegmentEpoch Params at its existing 100 ms cadence. It advances its segment target as today, but drains pending MPP outputs in fair short rounds under the existing 900 ms total drain budget. This gives every healthy sibling progress during the same bounded window. A stream still pending at that deadline is marked unavailable and its own pending IDs are retired as loss; the camera thread and other outputs continue. The failure does not abort the global segment advance. Outside this protocol, preserve existing drain/recovery behavior.
3. **Commit hook (active deterministic KA2 runtime commit only).** Keep the shared logger_rotate() helper writer-free because loggerd calls it at startup to create the initial route segment. Before a later deterministic KA2 commit advances the route directory, finalize every active VideoWriter so a stream that never emits another packet cannot leave its previous segment open. Increment segment and epoch exactly once; reset each writer, recording/audio state, and readiness entry for the new epoch. Reclassify each queued encoder packet by its mapped segment: discard/count packets older than the new segment, retain the current segment, and never flush a different segment's packet into the current writer. Tag rawAudioData by segment; after commit, hold post-commit audio in a bounded per-segment queue until the current segment's qRoad writer opens, then write those samples in order. At commit, pending audio from the previous segment is either written to its existing writer before finalization or counted/dropped if no track writer existed. Never reassign older audio to a newer segment. This hook does not run during startup or in fallback modes.
4. **Early commit (active deterministic KA2 protocol only).** If every configured stream reports a next-segment packet for the active epoch, loggerd commits immediately, preserving the existing healthy handshake. The commit hook then clears the current missing-stream status. Fallback modes keep their existing handshake.
5. **Deadline commit.** Only while the deterministic KA2 protocol is active, if readiness is incomplete at 5 seconds, loggerd commits once anyway, records the stream IDs still missing, and emits a rate-limited warning with segment, epoch, elapsed time, missing stream names, and loggerd queue-drop counts. The timer check runs after each poll/message batch even with zero received packets; the deterministic-KA2 poll timeout is capped at 50 ms and shortened to the remaining deadline. The run-time upper bound is 5 seconds plus one timer/scheduler tick. Non-KA2 and RK3588 with ENCODERD_DETERMINISTIC_DISABLE=1 retain the existing fallback behavior.
6. **Packet alignment (active deterministic KA2 protocol only).** Packets whose index maps to a segment before loggerd's committed segment are stale and are dropped; packets from a future segment remain in the existing bounded queue until commit. No stale packet is rewritten or placed in a newer segment. Each segment/epoch has a fresh readiness set, so a previous epoch's ready flag cannot satisfy the next rotation. A queue drain writes only entries matching the active segment; stale entries are dropped and future entries remain queued. Fallback modes keep their existing queue handling.
7. **Recovery.** While the deterministic KA2 protocol is active, after a local encoder is unavailable, skip its submissions until its retry time. Reopen only that encoder, set its segment number to the current camera-thread target before submitting again, and resume at the next valid keyframe. Start retry backoff at 1 second, double after repeated failures, and cap at 30 seconds. Reset backoff after one second of actual packet progress. A camera/VisionIPC reconnect may reopen all outputs immediately because imported DMA-BUF descriptors may no longer refer to the current producer generation; all reopened outputs are set to the authoritative current segment. Other modes retain existing recovery scope.

## Per-output health and drain contract

- Maintain a per-output packet counter and last-packet monotonic timestamp updated only after a valid packet is published. Do not use encode_frame() > 0 as output progress.
- Treat ten consecutive encode_frame == -1 results as a local encoder error threshold, preserving the existing threshold while changing its recovery scope to one encoder. Continue counting -2 drops separately; if they coincide with a saturated pending queue or a packet-progress stall, recover that output rather than allowing silent backlog loss.
- Detect a silent encoder when it continues accepting input but produces no valid packet for its configured output-latency window. Preserve the existing one-second watchdog for main outputs. Give qRoad a three-second startup/segment grace because the existing test records at least three seconds before qcamera output; pause the packet watchdog during an explicit started rotation barrier. Tests must prove that this grace does not let the loggerd deadline be exceeded.
- A zero-byte/invalid packet cannot leave an unbounded pending_extras entry or shift the next valid packet onto the wrong frame ID. If MPP completion maps to a submitted frame, retire that identity as one empty-packet drop. If that mapping cannot be trusted, quarantine/reopen the output and count all in-flight frame IDs as dropped exactly once.
- During segment advance, send EOS once per output and poll pending outputs in fair slices under the common 900 ms deadline. A slice that expires while packets remain is interim, not a failure; emit one warning only if the common deadline expires. A drain timeout affects only the output whose pending list remains. Disable flush-on-close only for that output, close/reopen or quarantine it, and continue advancing healthy siblings. Do not set process-wide do_exit; only the external shutdown path ends encoderd.
- On ordinary repeated failure, do not reopen every 10 frames. Enter the capped retry schedule. Camera/VisionIPC reconnect is the sole exception that deliberately reopens all outputs.

## Segment and media contract

| Case | Segment/file behavior | Index/timestamp behavior | Reporting |
| --- | --- | --- | --- |
| All inputs valid | Existing route names, segment ordinals, file set, codec/container, audio/front-camera selection, first-keyframe behavior, and sentinel placement remain unchanged. | Preserve input frame IDs and timestamps; each emitted EncodeIndex maps to exactly one output packet and the directory containing it. No duplicates, reordering, or cross-segment packets. | No degraded-rotation entry and no encoder drop/recovery count in the valid arm. |
| One output unavailable before its first keyframe in a segment | No file is created for that track in that segment. | No synthesized indices or frames. When it recovers, its first valid keyframe starts the current segment; it cannot backfill earlier IDs. | A warning identifies segment, epoch, affected stream, reason, retry count, and dropped frame count. |
| Output becomes unavailable after writing began | Existing writer closes on normal segment/process teardown, leaving that segment partial. No placeholder or fabricated completion marker is created. | Existing emitted indices stay unchanged. In-flight and rejected inputs are counted as drops once; stale packets after commit are discarded. | The same rate-limited stream/segment warning and counters identify partial media. |
| Deterministic KA2 rotation barrier expires | loggerd commits a new segment/epoch and names missing outputs; healthy writers continue. | The segment mapping remains authoritative. Late old packets are dropped; new packets are written only after their index maps to the committed segment. | Update internal Params key LoggerdRotationReport (JSON {route, epoch, segment, degraded, missing_streams}) after this runtime commit only, and issue one LOGW per degraded boundary. Startup logger_rotate() emits no report. Keep encoderd's per-output errors/drop/recovery counters in rate-limited health/error logs. No cereal schema or route format changes. |

A failed video output is allowed to be absent/partial under D-28's input-drop rule. This degraded contract applies to deterministic KA2 runtime commits only. The absence is the honest media contract; do not emit an empty file that suggests a successful stream. A deterministic runtime commit finalizes active partial files. Hold post-commit audio for up to 5 seconds or 256 KiB, whichever limit comes first, tagged to the current segment and timestamp. Flush it in order when that segment's qRoad writer is created. If no writer appears by expiry, drop/count those samples; never attach them to the next segment. The report Param is diagnostic only and does not affect the existing segment authority keys.

## Interfaces

Full contract details are in reports/report_interfaces_r8.md and the typed interface set is artifacts/interface_set_r8.json.

- **IF-1 — deterministic-KA2 loggerd/encoderd segment authority (existing Params plus diagnostic):** loggerd remains sole writer of LoggerdRotateState, LoggerdSegment, and LoggerdSegmentEpoch; encoderd remains a reader. Their existing meaning stays started / committed, current segment ordinal, and monotonic epoch. Only after a deterministic-KA2 runtime commit, loggerd writes LoggerdRotationReport with route, epoch, segment, degraded, and missing_streams. Startup logger_rotate() emits no report. Readers accept a diagnostic only when its route matches CurrentRoute. The diagnostic cannot change segment/epoch authority and is not required for encoderd operation. Non-deterministic modes retain existing behavior.
- **IF-2 — deterministic-KA2 encoder instance to encoderd supervisor (internal C++ status):** while the deterministic KA2 protocol is active, each output exposes submitted-frame count, successfully published-packet count, last packet time/frame ID, pending IDs, drop/failure count, state, and retry time. Only the owning camera thread mutates that output's state; sibling outputs are observed independently. Other modes retain their existing recovery behavior.
- **IF-3 — test-only VisionIPC DMA-BUF fixture:** a C++ test helper creates and owns four system DMA-heap buffers per camera stream, sets NV12 metadata and identity patterns, registers external VisionBuf pointers, and publishes with send(sync=false) after explicit DMA_BUF_IOCTL_SYNC around CPU writes. The existing Python binding does not expose register_external_buffers, so no production Python API is added. This fixture is compiled/used only by isolated tests.
- **IF-4 — deterministic-KA2 loggerd per-segment writer/audio lifecycle:** only a deterministic-KA2 runtime commit finalizes active video writers, resets per-segment writer/readiness/audio state, and classifies queued encoded packets by their actual mapped segment. Raw audio is tagged to the active segment, buffered for at most 5 seconds or 256 KiB, whichever comes first, and flushed in order after that segment's qRoad writer opens. Older/future audio or video is never reassigned to the current segment; expired audio is counted and reported. Startup and non-deterministic fallback behavior remain unchanged.

There is no public API, endpoint, cereal schema, manager protocol, camera selection, route naming, or user-facing surface change.

## Test and acceptance architecture

### A. Valid DMA-BUF end-to-end rotation

- Build a test-only C++ publisher for /dev/dma_heap/system; allocate exactly four buffers per camera stream, matching KA2 FRAME_BUF_COUNT, at production 1920x1200 NV12 geometry. Keep the mapped buffer/fd lifetime valid until both processes and the VisionIPC server stop. Use DMA_BUF_IOCTL_SYNC START/END around CPU writes and VisionIpcServer::send(..., sync=false) to avoid the existing ION-only sync method.
- Put logs, Params, and IPC in per-run temporary locations by setting LOG_ROOT, PARAMS_ROOT, and OPENPILOT_PREFIX before constructing Params or starting loggerd/encoderd. Seed RecordFront and RecordAudio under that isolated Params root. Use the real loggerd and encoderd processes; no direct encoder-class substitute.
- Send deterministic distinct luma/chroma patterns and exact frame IDs/timestamps on all configured camera streams at 20 FPS, plus synthetic audio. Cross at least two rotations and stop only after delayed qRoad output and process shutdown drain complete.
- Require all configured video outputs including qRoad and driver (when RecordFront is enabled) to become ready and rotate before the 5-second deadline in repeated runs. Require segment 0 -> 1 -> 2 with one epoch increment per boundary and no timeout-degraded report. Assert exact expected/emitted frame IDs once per stream across startup, rotation, and teardown; no drops, retries, stale/duplicate/out-of-order packet, or backlog saturation. Decode outputs and verify frame identity/pixels, timestamps, codec/container metadata, audio samples in order across segment files, segment directories, sentinels, index ordering, and unchanged healthy-path file set. Confirm every prior writer finalizes at each boundary and no segment's audio/video lands in an adjacent directory.
- Repetition is required because valid DMA-BUF reuse/lifetime across a four-buffer ring and qRoad startup latency are part of the acceptance, not covered by a class-level smoke.

### B. Rejected shared-memory/invalid input and healthy sibling

- Use the existing ordinary SHM VisionIPC fixture with valid, changing NV12 frame IDs and continuous road/wide/driver state/audio messages. Assert full-resolution MPP DMA-BUF validation/import rejection, one import attempt per fd per encoder generation, no copy fallback or pointer retry, and exact per-output rejection/drop counters. Verify that qRoad's RGA resize on buf->addr works on the target; require qRoad packets, decode, indexes, and audio to continue while full-resolution outputs are rejected.
- Require loggerd to advance the segment and epoch within the 5-second deadline after started, with exact missing stream names in LoggerdRotationReport; both processes remain alive. Confirm failed tracks are absent/partial according to the media contract and no fabricated empty file exists.
- If the KA2 RGA cannot consume this SHM fixture, do not weaken the assertion or declare qRoad healthy. Use a LOGGERD_TEST-only per-output MPP rejection seam with a valid DMA-BUF source for the full lifecycle isolation assertion, and keep a separate actual invalid-fd/SHM MPP test proving no-copy rejection. Record that RGA/SHM combination as an unproven limitation.
- Add a separate silent-output fault case in which a selected encoder accepts submissions but publishes no packet. Verify the actual packet watchdog detects it, retires pending IDs, recovers only that stream, and qRoad is not closed/reopened. Add a segment-drain timeout fault case and verify the same containment and bounded rotation. No test may simulate a global camera reconnect for a local encoder fault.
- Keep all file output and Params state under temporary roots. Never attach to the live camera/V4L2 exporter, write production roots, or start a production route.

### C. Regression checks

- Keep the no-copy static scan: no full-resolution copy route, ENCODER_ZERO_COPY, or pointer import retry.
- Ensure invalid input drops are counted but do not use D-13's healthy-path zero-drop rule to reject the expected D-28 failure arm. The valid DMA-BUF arm remains zero-drop under D-16.
- Assert each emitted index is assigned to its segment directory; the readiness state resets on every epoch; a stream with no packet cannot block progress; qRoad's valid output remains present and decodable.
- Add explicit legacy-mode regression coverage: run a non-KA2 legacy build/configuration and an RK3588 KA2 configuration with ENCODERD_DETERMINISTIC_DISABLE=1. For each, assert rotation timing, writer finalization, encoded-packet queue handling, and audio handling match the pre-r8 behavior; specifically, no deterministic five-second deadline, runtime commit finalization, or LoggerdRotationReport may leak into either path. Assert no deterministic-KA2 per-output watchdog/recovery policy is active and the deterministic-KA2 runtime commit hook does not run. Also assert startup logger_rotate() remains writer-free and emits no report in every mode.
- Recheck loggerd, encoderd, and the test helper process status, FD/buffer cleanup, Params values, queue/drop counters, audio sample totals, and temporary files in finally even on test failure.

## Non-functional targets and invariants

The new deadline, readiness, recovery, finalization, queue, audio, and diagnostic targets below apply only while the deterministic KA2 protocol is active. Non-KA2 and ENCODERD_DETERMINISTIC_DISABLE=1 acceptance is legacy regression coverage against the pre-r8 behavior.

| Target / invariant | Number | Evidence required |
| --- | --- | --- |
| Rotation liveness | Deterministic KA2 only: 5,000 ms after started (test upper bound includes one ≤50 ms event-loop/scheduler tick) | Missing-packet case advances loggerd segment and epoch; no service-message dependency. |
| MPP graceful drain | Deterministic KA2 only: existing 900 ms shared segment drain budget | Per-output fair polling proves a stuck encoder cannot consume every sibling's drain opportunity. |
| Segment authority poll | Deterministic KA2 only: existing 100 ms encoderd poll cadence | Start/commit transitions and epoch are observed without skipped ordinal. |
| Actual packet-stall detection | Deterministic KA2 only: main outputs 1,000 ms; qRoad 3,000 ms startup/segment grace | Watchdog is based on published packets, armed only when valid input continues, and paused during explicit rotation wait. |
| Retry load | Deterministic KA2 only: 1 s exponential start, doubles to 30 s cap | Persistent bad DMA-BUFs cannot trigger a ten-frame sibling-wide reopen loop. |
| Segment finalization | Deterministic KA2 runtime commits only: close active writers and reset segment state | A failed silent stream cannot keep an old segment file open; queued messages are segment-checked; audio remains ordered. |
| Audio buffering | Deterministic KA2 runtime commits only: maximum 5 seconds or 256 KiB while the current qRoad writer is not yet open | Delayed healthy qRoad output preserves samples; expiry with unavailable qRoad is counted and visible. |
| Healthy path | Zero unexpected drops or recoveries | Exact frame ID, output, media, and segment equivalence in DMA-BUF arm. |
| Failure containment | One failed output does not stop or reopen siblings | Per-stream rejection, silent output, and drain-timeout fault cases. |

## Alternatives rejected

- Restore full-resolution copy fallback after rejected MPP import — violates D-28 and recreates the memory/copy behavior the user removed.
- Keep waiting for every stream indefinitely — preserves the observed segment-0 stall.
- Rotate immediately at started with no readiness window — can truncate normal qRoad lookahead/reordering and healthy old-segment drains.
- Treat MPP submission success as packet output — current source shows these are separate events.
- Reopen every output after one failure or silently continue the failed track — either interrupts healthy qRoad or hides loss.
- Emit an empty output file for a failed stream — falsely implies a valid media track and changes existing writer behavior.
- Add a new cereal event schema or redesign manager/camera capture — unnecessary to implement bounded segment progress and test its current interfaces.

## Decision register

### Resolved

| ID | Decision | Chosen | Source |
| --- | --- | --- | --- |
| D-30 | Rotation deadline | Active deterministic KA2 protocol only: 5,000 ms from started; event loop checks independently of message traffic and polls no longer than 50 ms | delegated-default; source qcamera wait at test_loggerd.py:201, 900 ms MPP drain, 100 ms sync |
| D-31 | Readiness state | Active deterministic KA2 protocol only: per-stream and per-epoch; reset all readiness on every commit | codebase; current count/flag lifecycle in loggerd.cc:20,49,61,230-237 |
| D-32 | Failure recovery scope | Active deterministic KA2 protocol only: per-output threshold, packet watchdog, drain failure, and retry/backoff; all-output reopen only for producer reconnect/fd-generation invalidation | user D-29; encoderd.cc:192-210,485-550 |
| D-33 | Output heartbeat | Count valid published packets, not accepted MPP input submissions; empty packet handling must not shift pending frame identity | codebase mpp_encoder.cc:365-430,465-515; user D-29 |
| D-34 | Failed track file semantics | Preserve absent/partial stream media; no empty placeholder or invented frames; report missing track and drops | delegated-default grounded in loggerd.cc:194-208, D-28 |
| D-35 | Synthetic DMA-BUF fixture | Test-only C++ VisionIPC external buffers with the production four-buffer ring and explicit DMA-BUF CPU sync | codebase rk.h:14, visionipc_server.h:41; user D-29 |
| D-36 | Runtime diagnostics | Active deterministic KA2 only: one rate-limited boundary warning and per-output health/drop/retry logs; write the internal diagnostic Params report only after a runtime commit. No startup report or cereal schema. | delegated-default; supports operator triage without a route/API change |
| D-45 | Writer/audio finalization | Active deterministic KA2 runtime commits only: close every active VideoWriter, prune queued video by segment, and buffer current-segment audio up to 5 seconds or 256 KiB pending qRoad writer creation. Startup and fallback modes retain existing behavior. | codebase loggerd.cc:185-241,375-417 and VideoWriter::~VideoWriter; parent review |

### Deferred

| ID | Decision | Why deferral is safe | Owner | Reopen phase |
| --- | --- | --- | --- | --- |
| D-37 | Live V4L2 exporter and fence/lifetime qualification | User scope authorizes isolated synthetic tests only; DMA-heap E2E proves the code path but not production camera exporter behavior | Admiral / build owner | build/runtime acceptance before claiming live-camera qualification |
| D-38 | SHM-to-RGA target compatibility | Source uses mapped virtual-address RGA input, but only a target run proves the exact fixture/driver pairing | build test owner | build acceptance; use the scoped MPP rejection seam if incompatible |

### Rejected

| ID | Option | Rationale |
| --- | --- | --- |
| D-39 | Copy rejected full-resolution input into MPP-owned memory | Directly violates D-28. |
| D-40 | Let loggerd's deterministic barrier wait without a deadline | Produces unbounded segment 0/old-route output when one stream never publishes. |
| D-41 | Reopen all encoders for a local output error/stall/drain timeout | Couples unrelated outputs and caused qRoad churn. |

### Non-goals and YAGNI deferrals

| ID | Non-goal | Reopen trigger | Owner |
| --- | --- | --- | --- |
| D-42 | Live camera or on-road testing, production Params/log writes, or device-specific efficiency claims | User explicitly authorizes such a future run | Admiral |
| D-43 | Manager launch/gating changes, route/schema changes, broad capture or storage redesign | New requirement identifies a contract those changes must own | Admiral |
| D-44 | Generalizing results beyond the tested KA2 image/MPP/RGA/dma-heap combination | A second image/platform is included in scope | Admiral |

## Security and operational considerations

This design does not add a new public endpoint, credential, privilege transition, or message schema. It does add bounded parsing/formatting of local encoder status and one internal diagnostic Params record. Validate stream identifiers against the configured encoder set; never use untrusted packet text as a path. Rate-limit persistent failure logs and store numeric counts/stream names only. Security-builder should assess DMA-BUF fd lifetime, MPP import errors, queue bounds, malformed packet handling, and stale-segment drops in the implementation.

## Open risks

- The live RK V4L2 exporter, its DMA-BUF sync/fence behavior, and reuse lifetime remain unqualified; synthetic DMA-heap tests are not a substitute for live camera qualification.
- RGA handling of SHM virtual addresses must be verified on the idle KA2. If it fails, use the documented per-output test seam for end-to-end sibling containment while preserving a direct real SHM rejection test.
- Current loggerd writer semantics do not create a file until its first eligible packet/keyframe. Existing downstream handling of absent/partial per-stream files should be inspected during build QA; the design makes absence explicit in logs/report and does not claim a file is present.
- Runtime diagnostic on 2026-10-31: Python 3.12.3 is below required 3.13 and the project runtime scan exceeded its byte limit. `/data/kommu/.codex/hooks.json` and its referenced pre/post scripts under `/data/kommu/.agents/skills/harness/hooks/` exist; both scripts are executable. Whether this host actively registered or ran those hooks is not independently verified. This architecture stage read source and saved artifacts; no product tests or lifecycle runs were performed.
- No new runtime, dependency, or framework is proposed. Existing stack scan was incomplete, so the unchanged stack claim means r8 adds no runtime/framework/dependency; it is not a claim that the scan fully classified the whole repository.

## Architecture acceptance for downstream build

The build is ready only when it implements the rotation deadline and timer path, per-epoch readiness, per-output actual-packet health/recovery, nonfatal drain timeout, visible missing/loss report, no-copy preservation, and the three lifecycle fault/valid arms above. Passing the direct MPP class smoke alone is insufficient. The valid DMA-BUF integration arm must show qRoad and every configured healthy stream produce decodable, correctly indexed segment files before the deadline; the invalid/fault arms must demonstrate bounded progress without sibling reopen or global exit.
