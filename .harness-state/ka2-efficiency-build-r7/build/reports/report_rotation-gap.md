# KA2 loggerd rotation gap: build-phase diagnosis

## Observed result

The parent ran the focused synthetic rotation test after explicit user authorization and a KA2 target preflight. The corrected loggerd/encoderd processes crossed the five-second boundary and produced both segments, but the run failed. The raw rlog/qlog files have been preserved in `build/evidence/raw_rotation_failure/` and parsed offline.

The producer supplied 144 frame IDs, 1 through 144, to each of road, wideRoad, and driver; rlog contains all 144 camera-state messages for each input and 144 audio messages. Each of the four encoded outputs emitted 138 unique IDs, with no extras or duplicates. All observed EncodeIndex segment numbers match their qlog segment. Missing IDs are:

| Output | Expected | Emitted | Missing IDs |
|---|---:|---:|---|
| roadEncodeIdx | 144 | 138 | 1, 2, 99, 101, 102, 103 |
| driverEncodeIdx | 144 | 138 | 1, 2, 99, 101, 102, 103 |
| qRoadEncodeIdx | 144 | 138 | 1, 2, 99, 101, 102, 103 |
| wideRoadEncodeIdx | 144 | 138 | 1, 2, 99, 100, 102, 103 |

IDs 1 and 2 are startup omissions. Four more IDs are absent around the loggerd rotation. The test correction now reads the actual `*EncodeIdx` qlog union fields and compares every output set with IDs 1..last_frame_id; it will report the missing IDs directly. It has not been rerun after correction.

Raw counts, logs, SHA-256 values, the 576-row input/output ledger, and BOOTTIME timestamps are in `build/evidence/loggerd-rotation-frame-id-summary.txt`, `build/evidence/loggerd-expected-emitted-frame-ledger.csv`, and `build/evidence/loggerd-rotation-boundary-timeline.txt`.

## Source-supported mechanism

The rotation omissions align with the current deterministic handoff code:

1. In deterministic mode, `loggerd.cc::rotate_if_needed` publishes rotation state and immediately advances the log segment when the segment clock reaches its limit.
2. `encoderd.cc` reads the loggerd segment authority on a 100 ms interval. When it observes a newer segment, it sets `forced_segment_drop_pending` and calls `advance_segment_once(true)`.
3. `advance_segment_once(true)` sets each RK3588 MPP encoder's `flush_pending_on_close` to false before closing it. `MppEncoder::encoder_close` then clears `pending_extras`, explicitly discarding any encoded packets still in flight.
4. Once loggerd has advanced, `handle_encoder_msg` also discards old-segment encoder packets that arrive late. Newer-segment packets are queued until loggerd catches up.

The distinct per-stream holes around the boundary are consistent with asynchronous encoder backlog and forced catch-up. They do not prove which exact loss site removed each ID: this run did not include per-frame drop counters or process log output. The first two startup omissions are consistent with the existing `write_encode_data` first-keyframe gate, which discards packets until a keyframe starts the recording. That attribution also needs an explicit counter/path observation before it is treated as proven.

## Bounded proposal for Commander / design r4

Keep the existing copy MPP mode and default unchanged. Scope a new design slice to `system/loggerd/loggerd.cc`, `system/loggerd/encoderd.cc`, `system/loggerd/encoder/mpp_encoder.cc`, and the focused regression in `system/loggerd/tests/test_loggerd.py`.

- Change deterministic rotation from an immediate commit into a bounded drain handoff. Loggerd announces the target epoch while the old segment remains writable; each encoder stops old-segment submissions at a shared boundary, flushes its pending MPP packets, publishes a drained acknowledgment, and only then does loggerd commit the new segment. Loggerd must keep accepting the old-segment tail through the drain and not silently classify it as stale.
- Use `flush_pending_on_close=true` for a normal scheduled rollover. Reserve forced pending-packet discard for recovery/teardown, and emit a direct counter with the affected frame IDs whenever that path runs.
- Give the handshake an explicitly designed upper bound. If any encoder misses it, record a fail-closed timeout/drop result and preserve service liveness under a reviewed fallback; do not report that run as no-drop. Commander should set the bound from existing latency limits and target behavior rather than infer one from this single run.
- Add gated, per-stream counters for first-keyframe/startup drops, stale loggerd segment drops, pending MPP packets discarded on close, queue overflows, and output/index publication failures. Include the relevant frame IDs and rotation epoch so the next run proves the loss mechanism instead of inferring it from holes.
- Keep the regression's expected input window explicit. Under the user's no-drop requirement, do not silently omit IDs 1-2 from the expected set. The new design must either prove those startup inputs are encoded or make a separately approved startup contract with the loss observable and accounted for. The current build cannot claim that baseline behavior has no drops.
- Re-run the deterministic synthetic case only after the r4 design and gate approve the protocol. Require exact input/output ID-set equality for all four streams, segment/sentinel assignment, audio continuity, all counters, queue drain, and cleanup. Only after this correctness gate passes should the resource comparison resume.

This is a bounded design proposal, not an implementation or a claim that the listed paths caused every missing ID. No production runtime source was changed in this build follow-up.
