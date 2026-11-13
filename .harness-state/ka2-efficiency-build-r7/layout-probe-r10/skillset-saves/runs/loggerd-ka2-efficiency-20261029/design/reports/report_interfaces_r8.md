# Internal interface contracts — loggerd rotation liveness, r8

**Architecture:** report_architecture_r8.md  
**Revision:** r8  
**Public endpoint changes:** none  
**Cereal schema changes:** none

## IF-1 — Loggerd to encoderd segment authority and rotation report

**Kind:** internal process coordination via Params  
**Authority:** loggerd writes; encoderd reads the existing coordination values.

The new readiness deadline, epoch-scoped readiness, and diagnostic report apply only while the deterministic KA2 protocol is active: RK3588 with ENCODERD_DETERMINISTIC_DISABLE unset or zero; in tests, LOGGERD_TEST_DETERMINISTIC=1 must select that protocol. Non-KA2 and ENCODERD_DETERMINISTIC_DISABLE=1 retain their existing fallback behavior.

| Key | Direction | Contract |
| --- | --- | --- |
| LoggerdRotateState | loggerd → encoderd | started opens the next-segment drain/advance phase; committed means loggerd has advanced the segment. Unknown/empty values are not treated as a new state. |
| LoggerdSegment | loggerd → encoderd | Current authoritative route segment ordinal. Increases by one per commit. |
| LoggerdSegmentEpoch | loggerd → encoderd | Monotonic rotation generation, increasing exactly once per commit. Readiness belongs to one epoch and is reset on every commit. |
| LoggerdRotationReport | loggerd → local diagnostics | Written only after a deterministic-KA2 runtime commit. JSON report: { "route": current route name, "epoch": integer, "segment": integer, "degraded": boolean, "missing_streams": [configured stream names] }. It is diagnostic only; the three authority keys above remain authoritative. An all-ready runtime commit publishes an empty missing list. Startup logger_rotate() does not emit this report. Readers must ignore a saved report if its route does not match CurrentRoute. |

**Failure taxonomy:** in the active protocol, malformed/empty authority values are ignored with a rate-limited warning and retried at the next poll; missed packets trigger the 5-second deadline and degraded commit; stale packets are discarded; future packets remain in the existing bounded queue; invalid diagnostic JSON must not affect segment authority. Fallback behavior is unchanged.

**Liveness:** in the active protocol, encoderd polls at the existing 100 ms cadence. loggerd evaluates segment-duration and rotation-deadline timers independently of service messages, with a poll timeout capped at 50 ms / remaining time to the deadline. The no-packet case therefore cannot suppress either the start or commit check. This timer behavior is not applied to legacy fallback modes.

## IF-2 — Encoder instance to its encoderd supervisor

**Kind:** internal C++ status contract  
**Owner:** one encoder instance and its containing per-camera-thread supervisor.

This status and per-output recovery contract is used only while the deterministic KA2 protocol is active. Non-KA2 and ENCODERD_DETERMINISTIC_DISABLE=1 keep existing recovery behavior.

| Field | Meaning |
| --- | --- |
| input_submitted_count | Frames accepted by the encoder API. This is not output progress. |
| packet_published_count | Valid non-empty encoded packets successfully published to messaging. Only this advances packet progress. |
| last_packet_time, last_packet_frame_id | Monotonic time and input identity of the latest valid published packet. |
| pending_frame_ids | Accepted inputs awaiting a packet or an explicit drop disposition. Bounded by the existing MPP pending limit. |
| rejected_count, empty_packet_count, overflow_drop_count | Per-output loss reason counts; every input identity is retired once. |
| state | active, unavailable, or recovering. |
| retry_at, reopen_count | Capped exponential retry timing and count. |

A -1 input error, silent packet window, or segment drain failure affects only this output. Reopen it at its current authoritative segment when retry is due. Camera/VisionIPC reconnect may reopen all outputs to invalidate old imported fds. The supervisor must not count positive frame submission as packet success.

## IF-3 — Test-only DMA-BUF VisionIPC publisher

**Kind:** test fixture  
**Caller:** isolated loggerd/encoderd integration test  
**Callee:** C++ VisionIPC server and its connected encoderd client

The fixture allocates four system DMA-heap buffers per configured camera stream; initializes VisionBuf fd, mapping, size, 1920x1200 NV12 geometry, stride, UV offset, and frame identity; passes the pointer vector to register_external_buffers; explicitly brackets CPU writes with DMA_BUF_IOCTL_SYNC START/END; then calls VisionIpcServer::send(..., sync=false) to avoid the generic ION-specific sync method. The fixture retains every mapping/fd until the VisionIPC server and clients stop, then unmaps/closes them. The helper is test-only; the existing Python binding remains unchanged.

## IF-4 — Loggerd per-segment writer and audio lifecycle

**Kind:** internal loggerd event handling  
**Owner:** loggerd rotation coordinator and stream writers.

This contract applies only to runtime commits while the deterministic KA2 protocol is active. The startup logger_rotate() call remains writer-free and does not emit LoggerdRotationReport. Non-KA2 and ENCODERD_DETERMINISTIC_DISABLE=1 fallback writer, queue, and audio handling remain unchanged.

At each such runtime commit, finalize all active video writers, reset their current-segment/recording/audio state, and reset the per-stream readiness epoch. Before a queued video message is written, compare its mapped EncodeIndex segment with the active route segment: drop/count older messages, write only matching messages, and keep future messages in the bounded queue. No queued packet may be relabeled or drained into a different segment's file.

Tag rawAudioData with the current segment. If qRoad's writer for that segment is not open yet, buffer its audio events in timestamp order, capped at five seconds or 256 KiB, whichever comes first. When the matching qRoad writer opens, flush only same-segment audio in order and then release the queue. At commit, finish same-segment audio into the old writer before finalizing; drop/count any old-segment samples for which no writer existed. On expiry, drop/count samples and include the loss in rate-limited health logs. Never carry old audio into a newer segment. The healthy DMA-BUF acceptance compares decoded audio samples and timestamps in order across the complete route.

## Interface boundary notes

- No route filenames, codecs, timestamps, frame/encode index schema, RecordFront/RecordAudio semantics, or manager protocol change.
- Deterministic-KA2 runtime segment finalization is an internal lifecycle change: old writers close exactly at commit, while delayed audio and video are kept in their own tagged segment or counted as loss. Startup and fallback behavior are unchanged.
- The SHM rejection and silent-output test seams are compiled/usable only under LOGGERD_TEST; they must not introduce a production environment switch or copy route.
- Keep stream names drawn from the static configured encoder inventory. Never derive file paths from diagnostic names or packet payloads.
