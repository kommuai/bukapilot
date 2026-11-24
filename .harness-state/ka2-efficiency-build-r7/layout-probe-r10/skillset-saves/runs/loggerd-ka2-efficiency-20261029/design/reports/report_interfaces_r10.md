# Internal interface contracts — loggerd rotation liveness, r10

**Architecture:** report_architecture_r10.md  
**Revision:** r10  
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
| LoggerdRotationReport | loggerd → local diagnostics | Written after every deterministic-KA2 runtime commit, including all-ready commits, with the exact schema below. It is diagnostic only; the three authority keys above remain authoritative. Startup logger_rotate() does not emit this report. Readers must ignore a saved report if its route does not match CurrentRoute. |


### LoggerdRotationReport exact JSON contract

The object has exactly these seven top-level keys, with no aliases or additional fields. Every key is present on every runtime report, including zero-valued drop counters and an empty `missing_streams` list on an all-ready transition:

```json
{
  "route": "<CurrentRoute>",
  "epoch": 1,
  "segment": 1,
  "degraded": false,
  "missing_streams": [],
  "elapsed_ms": 0,
  "known_drop_counts": {
    "stale_video_messages": 0,
    "future_video_queue_drops": 0,
    "audio_no_writer_events": 0,
    "audio_expired_events": 0,
    "audio_capacity_drops": 0
  }
}
```

| Field | Exact type and meaning |
| --- | --- |
| `route` | Non-empty JSON string equal to the current `CurrentRoute` value at commit. |
| `epoch` | Nonnegative JSON integer equal to `LoggerdSegmentEpoch` after the commit. It identifies the destination rotation generation, incremented once for this transition. |
| `segment` | Nonnegative JSON integer equal to `LoggerdSegment` after the commit. It identifies the destination route segment ordinal. |
| `degraded` | JSON boolean; true exactly when this transition's `missing_streams` is nonempty or any value in this report's `known_drop_counts` snapshot is greater than zero. It is a loggerd diagnostic predicate, not a complete encoder-health bit. |
| `missing_streams` | Array of unique configured encoder service-name strings that did not publish a valid packet for the destination epoch before this commit. Preserve static configured encoder order. An all-ready transition has `[]`. |
| `elapsed_ms` | Nonnegative JSON integer equal to monotonic `steady_clock` duration truncated toward zero (equivalent to floor because the duration is nonnegative) in milliseconds from the instant loggerd publishes `LoggerdRotateState=started` to the commit decision. This is the current transition's elapsed time; sub-millisecond duration reports `0`. It is not wall-clock time. |
| `known_drop_counts` | Object with exactly the five fixed keys shown below. Each value is a nonnegative JSON integer, never a boolean, fraction, or negative number. Values count loggerd-observed dropped input events/messages, not bytes or samples. |

`known_drop_counts` is a loggerd observation window, not a destination-segment total: it accumulates events since the previous `LoggerdRotationReport`; the first report covers events since route startup. Snapshot the values when the current report is created and reset the window only after emission. Drops that happen after a commit are logged when they occur and included in the next runtime report; therefore delayed audio expiry is immediately visible in a rate-limited loggerd log and counted in the next report. A final post-report loss may have no later JSON snapshot if the route stops before another commit; the drop-time log remains the operator signal.

The drop keys have exact ownership and one-count semantics:

- `stale_video_messages`: encoded video messages discarded by loggerd because their mapped `EncodeIndex` segment is older than the currently committed segment.
- `future_video_queue_drops`: encoded video messages rejected by the existing bounded future-segment queue when its capacity policy drops a message. This counter does not authorize changing the queue policy.
- `audio_no_writer_events`: accepted `rawAudioData` message events whose queued payload is discarded because no matching qRoad writer exists when a segment commit first makes the event's tagged segment unretainable. If its five-second hold expiry occurred first, classify it as expired instead.
- `audio_expired_events`: accepted `rawAudioData` message events whose queued payload is discarded because the existing five-second hold limit expires before the event's tagged segment becomes unretainable at commit or a matching writer opens.
- `audio_capacity_drops`: incoming newest `rawAudioData` message events rejected atomically because adding the whole event would exceed the 256 KiB audio-queue cap. Leave the existing timestamp-ordered queue unchanged; do not split the event or evict older entries. Increment this counter exactly once for the rejected event.

Each dropped encoded video message increments at most one video key; each audio input event increments exactly one audio key at its terminal drop reason, even if its payload contains multiple samples. Capacity rejection is terminal at admission and therefore cannot also become expired or no-writer. For an accepted queued event, whichever occurs first—five-second expiry or the segment commit that makes its tagged segment unretainable—owns its single count; if both are observed in the same event-loop iteration, compare their monotonic deadlines, with an exact tie classified as expired. Do not count one event under multiple keys or reinterpret these values as encoder-side MPP rejections, stalls, drain failures, pending-frame drops, or retry counts. Encoder-local losses remain in encoderd's per-output logs/counters; no Params IPC or cereal channel is added for them.

**Contract-test ownership:** AC-1 asserts the fake-monotonic-clock start-to-commit `elapsed_ms` value and deadline bound (5,000 ms plus one ≤50 ms wake/scheduler tick), including the no-message path and immediate all-ready path. AC-9 asserts the exact top-level and nested key sets, JSON types/ranges, post-commit route/epoch/segment, missing-stream names/order, all five map entries including zeros, known loggerd-drop values, and the `degraded` truth table; it also asserts that asynchronous audio expiry is logged immediately and appears in the next report. AC-6 and AC-8 assert encoder-only drain/MPP loss logs and counters; they assert those losses do not get invented in `known_drop_counts` absent a loggerd-observed drop. These assertions do not introduce a new IPC channel.

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
| retry_at | Next per-output retry time under the capped exponential schedule. |
| failure_retry_count | Number of retry attempts made for this output after its own error, silent-output watchdog, or drain failure. Increment once when the scheduled retry begins, even if opening the context fails. Planned segment changes and camera/VisionIPC reconnects do not increment it. |
| failure_recovery_reopen_count | Number of successful context reopens caused by this output's own failure recovery. It is per-output and excludes planned segment transitions. |
| segment_transition_reopen_count | Number of successful context close/open pairs caused by ordinary authoritative segment changes for this output. A healthy output increments exactly once per committed boundary; a quarantined output does not reopen at that boundary. |

A -1 input error, silent packet window, or segment drain failure affects only this output. Drain active healthy outputs with EOS under the shared 900 ms monotonic deadline using fair round-robin poll slices; an individual slice is interim, and only outputs still pending when the common deadline expires are marked unavailable. Retire each quarantined output's pending frame identities once, disable flush-on-close for it, and continue the segment advance. Every remaining healthy output, including qRoad, receives its single planned close/open for the new authoritative segment. That planned transition is distinct from failure recovery. In a sibling drain-fault test with one ordinary committed boundary, qRoad must increment `segment_transition_reopen_count` exactly once, keep `failure_recovery_reopen_count` and `failure_retry_count` unchanged, remain active, and publish valid packets tagged to the new segment. If no boundary overlaps a silent-output fault, qRoad's planned-transition and failure counters all remain unchanged. Reopen an unavailable output only at its own retry time, already tagged to the current authoritative segment. Camera/VisionIPC reconnect may reopen all outputs to invalidate old imported fds; this producer-generation reset is not a local failure recovery. The supervisor must not count positive frame submission as packet success.

**Per-output counter invariants:** a planned close/open at a normal segment boundary counts once in `segment_transition_reopen_count`, even when another output failed to drain; it must never also be attributed to the sibling's failure. `failure_recovery_reopen_count` and `failure_retry_count` change only on the affected output. A future implementation may use an in-place post-EOS continuation only after the deployed MPP build is pinned and its encoder continuation contract is demonstrated; r10 does not assume that `MPP_ENC_SET_IDR_FRAME` proves EOS-context reuse.

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

Tag rawAudioData with the current segment. If qRoad's writer for that segment is not open yet, buffer complete events in timestamp order, capped at five seconds or 256 KiB, whichever comes first. Before adding a new event, reject that whole newest event if it would exceed 256 KiB; preserve the existing ordered queue unchanged, do not split payloads or evict older entries, and increment `audio_capacity_drops` exactly once. When the matching qRoad writer opens, flush only same-segment audio in order and then release the queue. At commit, finish same-segment audio into the old writer before finalizing; if no writer existed and expiry had not occurred first, drop/count each queued event as no-writer. If expiry occurs first, drop/count it as expired. Log either loss immediately and include it in the next LoggerdRotationReport drop-count window. Never carry old audio into a newer segment. The healthy DMA-BUF acceptance compares decoded audio samples and timestamps in order across the complete route. AC-3 asserts the atomic capacity policy and event order; AC-9 asserts exact report counter values.

## Interface boundary notes

- No route filenames, codecs, timestamps, frame/encode index schema, RecordFront/RecordAudio semantics, or manager protocol change.
- Deterministic-KA2 runtime segment finalization is an internal lifecycle change: old writers close exactly at commit, while delayed audio and video are kept in their own tagged segment or counted as loss. Startup and fallback behavior are unchanged.
- The SHM rejection and silent-output test seams are compiled/usable only under LOGGERD_TEST; they must not introduce a production environment switch or copy route.
- Keep stream names drawn from the static configured encoder inventory. Never derive file paths from diagnostic names or packet payloads.
