---
submission_id: loggerd-ka2-efficiency-20261029-design-r9-report-contract
revision: r9
---

# Design package — KA2 rotation liveness and encoder fault containment, r9

## Outcome

This r9 package resolves the LoggerdRotationReport contract contradiction before implementation. It defines one exact seven-field report schema, one loggerd-only drop-counter window, and explicit ownership for elapsed-time, report, and encoder-local-loss assertions. The audio queue's 256 KiB overflow behavior is also fixed so its drop count is deterministic.

The production objective remains D-29: keep deterministic-KA2 recording moving when a no-copy MPP input is rejected or an encoder stops publishing packets, while preventing one encoder's failure from stopping or churning healthy siblings such as qRoad. D-28 remains mandatory: full-resolution MPP input validates and imports DMA-BUF once; rejected input is dropped and counted, with no copy fallback, pointer retry, selector, or alternate input path. qRoad keeps its existing RGA resize path.

## Requirements and decisions

The r9 requirements brief is `reports/report_research_r9.md`. It carries forward the approved r8 evidence and requirement rows without changing stakeholder goals or scope. Its historical three-second timeout suggestion is explicitly marked as an inferred candidate resolved by architecture decision D-30: the selected deterministic-KA2 bound is 5,000 ms from `started`. D-32, D-34, and D-36 resolve the recovery, absent/partial-media, and operator-reporting choices. Remaining evidence limits are the SHM-to-RGA target check, downstream handling of absent/partial files before release, and live V4L2 qualification.

## Failure mechanism and selected architecture

Loggerd's deterministic KA2 rotation waits for a next-segment encoded packet from every configured encoder before committing. A rejected DMA-BUF input emits no packet, so readiness never reaches the configured count. In addition, an encoderd drain timeout can currently request process-wide exit, while local error/stall paths can reopen healthy sibling encoders. These conditions can leave the route on the old segment indefinitely or interrupt healthy streams.

In deterministic KA2 mode only, loggerd will evaluate its monotonic rotation deadline independently of incoming service messages and commit when all outputs are ready or 5,000 ms has elapsed since `started`. Poll wake is capped at 50 ms. Readiness is tracked per stream and epoch. A timed commit records missing outputs and finalizes active writers so a silent stream cannot leave an old media file open. Startup `logger_rotate()` remains writer-free and emits no report. Non-KA2 and explicitly disabled deterministic mode preserve existing fallback behavior.

Encoderd will use actual valid published packets as output progress. A local MPP rejection, silent output, or drain timeout isolates and recovers only that output; a camera/VisionIPC reconnect may reopen every encoder because buffer generation changed. Pending input identities retire once as a packet or a counted drop. Fair drain rounds remain bounded by the existing 900 ms budget. Missing media is absent before its first keyframe or partial if writing began; no fake empty track, backfill, or cross-segment relabeling is permitted.

## Exact diagnostic contract

`LoggerdRotationReport` is a local, diagnostic-only Params value written after each deterministic-KA2 runtime commit, including all-ready commits. It does not control segment authority; `LoggerdRotateState`, `LoggerdSegment`, and `LoggerdSegmentEpoch` remain authoritative. Startup and legacy fallback modes emit no report. The report has exactly this schema:

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

The top-level key set is exactly `{route, epoch, segment, degraded, missing_streams, elapsed_ms, known_drop_counts}`. The nested drop-map key set is exactly `{stale_video_messages, future_video_queue_drops, audio_no_writer_events, audio_expired_events, audio_capacity_drops}`. All fields are always present; counters and elapsed time are nonnegative JSON integers. `epoch` and `segment` name the post-commit destination. `elapsed_ms` is monotonic time from publishing `started` to the commit decision, truncated to integer milliseconds. `missing_streams` contains unique configured service names in configured order that did not publish a valid packet for that epoch before commit. `degraded` is true exactly when `missing_streams` is nonempty or any counter in that report snapshot is nonzero.

The drop map is a loggerd-observed window since the previous report, or since route startup for the first report; it is not an encoder-loss total or a destination-segment total. It counts only the defined stale/future encoded-message and audio terminal-drop events. Encoder-side MPP rejection, packet stall, pending-frame, drain, and retry losses remain in rate-limited per-output encoderd logs/counters. No new cereal schema or IPC channel is introduced. AC-1 owns the monotonic elapsed/deadline assertion; AC-9 owns exact report shape and semantics; AC-6/AC-8 own encoder-local loss reporting and assert those losses are not invented as loggerd counts.

## Audio and media contract

At deterministic-KA2 runtime commits, loggerd finalizes active video writers and resets segment recording/audio/readiness state. Encoded packets are written only when their mapped index matches the current segment; stale packets are dropped and counted, future packets stay within the existing bounded queue. Audio events are tagged with segment and held in timestamp order for at most five seconds or 256 KiB while the matching qRoad writer is not open. When adding an entire new event would exceed 256 KiB, loggerd rejects the newest whole event atomically, preserves the existing FIFO queue unchanged, does not split payloads or evict older events, and increments `audio_capacity_drops` once.

An accepted queued audio event receives one terminal loss cause: if the five-second expiry happens before a commit makes its segment unretainable, count `audio_expired_events`; otherwise count `audio_no_writer_events`. Compare monotonic event times; expiry wins an exact tie. Capacity rejection occurs at admission and cannot also count as expiry or no-writer. When the matching qRoad writer opens, flush only same-segment events in timestamp order. Never move older audio into a later segment. AC-3 asserts the queue policy and ordering; AC-9 asserts the report values and counter windows.

The healthy path preserves route and segment names, codecs/containers, stream selection, timestamps, sentinels, index ordering, audio/front-camera behavior, and the complete file set. Each healthy input frame ID appears exactly once per configured output across startup, rotations, and shutdown. The failure path may omit a rejected output while healthy qRoad continues; no absent stream is represented as a successful empty file.

## Acceptance and test boundary

The build plan orders a test-only C++ DMA-heap VisionIPC publisher, isolated lifecycle setup, per-output containment, loggerd boundary behavior, and then end-to-end acceptance. The valid arm uses four system DMA-heap buffers per stream with external VisionBuf registration, explicit `DMA_BUF_IOCTL_SYNC`, and `VisionIpcServer::send(..., sync=false)`. `LOG_ROOT`, `PARAMS_ROOT`, and `OPENPILOT_PREFIX` are set before Params construction or process startup. The rejected-input arm retains an actual SHM/invalid-fd MPP rejection assertion; if KA2 RGA cannot consume that mapping, use the documented test-only per-output rejection seam with valid DMA-BUF input to prove full-lifecycle sibling isolation. Separate silent-packet and drain-timeout cases verify bounded progress and qRoad continuity.

AC-1 checks a missing-packet timeout with a fake monotonic clock, exact `elapsed_ms`, and immediate all-ready commit. AC-3 checks writer finalization, segment isolation, ordered audio, and exact atomic newest-event queue overflow semantics. AC-6/AC-8 check per-output drain and invalid-input containment. AC-9 checks the exact report schema, post-commit values, loggerd-only counters, missing-stream ordering, and `degraded` predicate. AC-12 proves legacy and startup behavior remain unchanged. The complete criteria and implementation touchpoints are in `reports/delivery-plan_r9.md` and `reports/implementation-spec_r9.md`.

## Security, stack, and Taste

Security-builder assessed that the additive diagnostic and test fixture introduce no new trust-boundary movement. The report remains local and non-authoritative; its stream names come from configured inventory and no paths or payloads are included. The full-resolution input path remains DMA-BUF-only. The stack remains the existing C++/KA2/MPP/RGA/VisionIPC stack with no runtime or dependency change; runtime scan byte-limit and target-version limitations remain recorded in `reports/report_stack-lock_r9.md`.

Taste rechecked both preference stores for this revision. Both are absent at revision 0 and the effective profile is empty, so no user-facing taste preference applies. There is no user-facing surface; the typed UI evidence record remains the sanctioned no-surface applicability result.

## Evidence and open limits

The prior M0 DMA-BUF fixture report is included as `evidence/r9/prior_report_dmabuf_fixture_r8.md` (SHA-256 `85905c38f551867c6b80619cf4c139aa866ebd20aa210ec23b1a29a1fa9b2a45`). It proves isolated synthetic DMA-heap VisionIPC registration, send/import, and fixture-owned resource cleanup in serial and xdist smoke runs. It does not exercise full loggerd+encoderd rotation or production MPP encoding. No r9 product implementation or lifecycle test has run.

The live V4L2 exporter, its DMA-BUF fences/synchronization, MPP driver behavior on live camera buffers, and buffer-reuse lifetime remain unqualified. Synthetic DMA-heap success must not be generalized to the production camera path. Whether KA2 RGA accepts the SHM fixture mapping is also unproven until the target acceptance case runs. These are explicit build/release evidence limits, not unresolved report-schema choices.
