# Requirements brief — KA2 segment rotation and encoder fault containment, r9

**Revision:** r9 (intake revision 153; requirement rows carried forward unchanged from approved r8)  
**Submission ID:** `loggerd-ka2-efficiency-20261029-design-r9-report-contract`  
**Prepared for:** `design/architect`  
**Sources mined:** `intake/report_grilling.md`, `intake/intake-brief.md`, `build/reports/report_no_copy_fallbacks_r8.md`, `build/evidence/loggerd_shm_rotation_not_applicable_pytest.log`, `build/evidence/mpp_encoder_no_fallback_smoke.log`, `system/loggerd/loggerd.cc`, `system/loggerd/loggerd.h`, `system/loggerd/encoderd.cc`, `system/loggerd/encoder/mpp_encoder.cc`, `system/loggerd/tests/test_loggerd.py`, `msgq_repo/msgq/visionipc/visionipc_server.h`, `msgq_repo/msgq/visionipc/visionipc_server.cc`.


## r9 carry-forward status

This brief preserves the approved r8 requirement rows, source anchors, confidence levels, and recorded risks unchanged. The active intake remains D-29; this r9 design correction reconciles the diagnostic report schema and test ownership and pins audio-queue overflow handling, without changing stakeholder goals, non-goals, or acceptance scope. No new domain research or requirement is introduced.

R1's suggested three-second timeout was an inferred candidate for architecture review, not a user requirement. The approved architecture selected a five-second monotonic bound after reviewing qRoad startup/lookahead, the existing 900 ms MPP drain, and scheduling margin; see `report_architecture_r9.md` (SHA-256 `a788e36c49a30aceb4e8bacaa97040db7eb20c5c0d5340f33fe878fc9a68b886`). Thus the binding requirement remains finite segment progress, while the architecture owns the five-second implementation value. D-32, D-34, and D-36 resolve the recovery, failed-media, and diagnostics questions carried forward from r8. IF-1 and AC-9 own the exact seven-field report semantics and loggerd-only drop counters. M0 passed synthetic VisionIPC transport and fixture-owned lifetime cleanup only; it does not prove MPP import or full loggerd/encoderd rotation. Open items are limited to the RGA/SHM fixture choice, downstream absent/partial-file consumer review before release, and live V4L2/exporter qualification.

## Problem statement

On KA2, full-resolution MPP must receive a valid DMA-BUF fd. D-28 requires one validation and one MPP import attempt; rejected input is dropped, with no copy, environment switch, or pointer-based retry. The current KA2 loggerd rotation handshake waits for a next-segment packet from every configured encoder. If even one stream emits none, deterministic rotation can remain in `started` forever. Encoderd also has two independent whole-pipeline failure paths: an MPP drain timeout makes the camera thread set global `do_exit`, and an encoder-local error threshold or aggregate output stall reopens every encoder in that camera thread. Together these paths can turn one invalid or stalled encoder into both a stuck segment and churn or shutdown for healthy sibling output such as qRoad.

The design requirement is to treat liveness and data loss separately: preserve normal segment/media behavior when DMA-BUF inputs work; when an input is rejected, drop and count that stream's frames, keep healthy streams alive, and let the route advance on a deadline. Do not restore full-resolution copying.

## Actors and jobs

| Actor | Job to be done | Success looks like | Frequency / volume |
| --- | --- | --- | --- |
| KA2 vehicle logger | Record route segments while synthetic or camera frames and audio continue | Every healthy stream keeps its existing segment/index/file behavior; one failed output does not stall the route or stop its siblings | Continuous at configured camera FPS; normally 20 FPS |
| `encoderd` | Turn each camera frame into the configured encoded streams and align outputs to loggerd's segment authority | Each encoder's health and recovery are independent; valid outputs resume at the current segment after recovery | One call per configured encoder per input frame |
| `loggerd` | Write encode packets, logs, and segment boundaries | Segment commit occurs after all outputs report ready or after a bounded wait; missing streams are identified and no packets are written to the wrong segment | One readiness event per configured output per boundary |
| Test owner | Exercise success and failure lifecycles without live camera or production storage | Isolated tests cover DMA-BUF-backed VisionIPC rotation and rejected shared-memory input through the full loggerd/encoderd lifecycle | Bounded synthetic runs only |

## Requirements

| # | Requirement | Source | Confidence | Affects | Open assumption |
| --- | --- | --- | --- | --- | --- |
| R1 | In deterministic KA2 mode, once loggerd publishes rotation state `started`, it must commit the next segment within a finite deadline even if one or more encoder streams never publish a next-segment packet, provided camera input and loggerd's event loop continue. | User decision D-29, `intake/report_grilling.md`; observed branch in `system/loggerd/loggerd.cc:57-73` | `reported` (request); mechanism `observed` | Rotation authority, deadline contract, failure-case acceptance | Resolved by D-30 in `report_architecture_r9.md`: the deterministic-KA2 implementation target is a 5,000 ms monotonic bound from `started`, checked independently of message traffic. The original 3 s value was an inferred candidate, not the user requirement. AC-1 owns the deadline assertion; implementation remains unverified. |
| R2 | A failed encoder or drain timeout must not set process-wide exit while healthy encoder outputs in the same camera thread can continue. | D-29; observed `encoderd.cc:212-228, 454-461, 478-480` | `reported` + `observed` | Encoderd error model, segment-advance interface, containment tests | Resolved by D-32: in deterministic KA2 a local error, silent output, or drain timeout is contained to that output and cannot set process-wide exit or reopen healthy siblings; all-output reopen is reserved for producer reconnect/fd-generation invalidation. AC-5/AC-6 verify implementation. |
| R3 | Error thresholds and output-stall detection must be tracked per encoded output; a healthy sibling's packet must not reset or trigger another output's recovery decision. | D-29; observed `encoderd.cc:162-165, 192-210, 485-528` | `reported` + `observed` | Per-encoder state and recovery policy | Resolved by D-32 and the approved recovery contract: per-output recovery uses a 1 s exponential retry starting point capped at 30 s, returns at the current authoritative segment, and bases packet health on published output. AC-4/AC-5 verify implementation; no implementation claim is made here. |
| R4 | When MPP rejects a full-resolution input fd or import, the frame is dropped and counted for that output; the encoder must not copy it, attach a pointer for a second import attempt, or select a copy mode. | D-28, `intake/report_grilling.md`; observed `mpp_encoder.cc:337-363, 399-415` and `report_no_copy_fallbacks_r8.md` | `reported` + `observed` | MPP path, error classification, regression scans | No unresolved assumption. This remains binding for every failure and recovery path. |
| R5 | Failure reporting must identify the encoder/stream, boundary or segment, error/recovery event, and accumulated dropped-frame count without logging every frame at error severity. | D-29's liveness and production-readiness intent; current aggregate health/failure logs in `encoderd.cc:497-510, 531-537` | `inferred` | Operational diagnostics and supportability | Resolved by D-36: rate-limited per-output encoder health/loss logs and the local diagnostic `LoggerdRotationReport` provide operator visibility; no new IPC or cereal channel is added. IF-1 owns the exact seven-field schema and loggerd-observed counter meanings; AC-9 tests them. Encoder-only MPP/stall/drain losses remain per-output logs/counters under AC-6/AC-8. |
| R6 | With valid synthetic DMA-BUF inputs, the actual isolated loggerd+encoderd lifecycle must rotate and keep processing; every healthy configured stream must produce valid, decodable output assigned to the correct segment. | D-3/D-16 and D-29; current real-path test at `test_loggerd.py:212-365` is skipped on KA2; class-level DMA-heap smoke in `mpp_encoder_no_fallback_smoke.log` | `reported`; path-level readiness is unproven | Fixture, integration test, acceptance | M0 passed the synthetic DMA-heap VisionIPC publisher transport and fixture-owned fd/mapping lifetime checks; see `evidence/r9/prior_report_dmabuf_fixture_r8.md`. That fixture did not exercise MPP import, full loggerd/encoderd rotation, or live camera buffers. Those remain unverified acceptance work in AC-7/M3. |
| R7 | With invalid ordinary shared-memory input, full-resolution MPP must reject/drop without copying, loggerd must advance by its deadline, and a healthy qRoad sibling must continue producing correctly indexed output from the same frame stream. | D-28/D-29; observed test skips at `test_loggerd.py:192-195, 212-215`; observed downscale path uses `buf->addr` with RGA while full-resolution imports `buf->fd` (`mpp_encoder.cc:399-415`) | `reported` + `observed`; qRoad survival under SHM is `inferred` | Fault-injection/test case and output contract | Open for M3 fixture selection: prove whether KA2 RGA accepts the ordinary SHM fixture mapping while MPP rejects its fd. If it does not, use the approved test-only per-output rejection seam with valid DMA-BUF input and retain a separate actual invalid-fd/SHM MPP rejection assertion. |
| R8 | Healthy behavior retains the existing route naming, segment ordinals, sentinel placement, encode index ordering, timestamps, codecs, and audio/front-camera behavior. Each emitted encode index must name the segment directory that contains it; no stale packet may cross a committed boundary. | D-3/D-13/D-16; test assertions at `test_loggerd.py:314-365`; writer/index path at `loggerd.cc:194-241` | `reported` + `observed` | Segment/media contracts and tests | The healthy DMA-BUF case must continue to require exact per-frame coverage under D-16, including startup, rotation, and teardown. |
| R9 | In the explicit degraded-input case, loggerd and healthy output files/logs continue, while failed-stream frame loss and any absent failed-stream media for the affected segment are surfaced; do not fabricate empty media to imply successful encoding. | D-28 permits rejected-frame loss; D-29 requires liveness and failure containment; `VideoWriter` is created on first matching encoded packet at `loggerd.cc:194-208` | `inferred` | Degraded media contract, operator reporting | Resolved by D-34: a failed stream is absent before its first keyframe or partial after writing begins; no empty placeholder or invented frame is created. D-36/IF-1 define its report/log signal. The downstream consumer review of absent/partial files remains open before release. |
| R10 | All test logs, Params, IPC endpoints, and output routes are isolated under per-run temporary roots; testing uses synthetic frame/audio publishers only and stops if competing loggerd/encoderd/camera/MPP work appears. | D-6/D-29 in intake brief | `reported` | Test harness, safe runtime procedure | The isolation and preflight procedure is explicit in intake and AC-10: set temporary roots before Params/process startup, recheck process/device idleness before each target run, and never use live camera or production roots. The full M3 lifecycle has not yet run. |

## Constraints

| # | Constraint | Kind | Source | Confidence | Affects |
| --- | --- | --- | --- | --- | --- |
| C1 | Full-resolution MPP has a single mandatory DMA-BUF route and no copy fallback. qRoad retains its existing RGA resize path. | Technical / binding user decision | D-28, `intake/report_grilling.md`; `mpp_encoder.cc:337-415` | `observed` | Encoder architecture and tests |
| C2 | Current deterministic loggerd starts rotation at the segment boundary but commits only when `ready_to_rotate == max_waiting`; `ready_to_rotate` increments only on a next-segment encoded packet. This branch has no deadline. | Technical / failure mechanism | `loggerd.cc:57-73, 230-241`; `loggerd.h:125-150`; max count setup at `loggerd.cc:328-335` | `observed` | Rotation state machine and missed-stream accounting |
| C3 | KA2 MPP segment drain has a shared 900 ms deadline; on timeout `advance_segment_once` returns false, and its caller sets global `do_exit = true`. | Technical / failure mechanism | `encoderd.cc:31, 212-228, 454-461` | `observed` | Drain-failure containment |
| C4 | At 10 consecutive encode errors, and when the aggregate stream thread emits no output for 1 second, current code reopens all encoders in that camera thread. This includes road and qRoad, which share the road camera thread. | Technical / failure mechanism | `encoderd.cc:23-24, 138-164, 192-210, 485-528`; road/qRoad membership in `loggerd.h:135-139` | `observed` | Fault isolation and retry semantics |
| C5 | Test acceptance is synthetic and isolated. No live camera/V4L2 exporter or sensor path is qualified by this design. | Operational | D-6/D-29 in `intake-brief.md` | `reported` | Test scope and release claims |

## Measurable targets

| Target | Number | Source | Confidence | Why this number |
| --- | --- | --- | --- | --- |
| Deterministic rotation completion after the published `started` state while new input is flowing | D-30 design bound: 5,000 ms monotonic from `started`, with at most one ≤50 ms timer/scheduler tick | Approved `report_architecture_r9.md` D-30; based on the observed qcamera delay, 900 ms MPP drain, 100 ms sync cadence, and scheduling margin. The earlier 3 s figure was an inferred candidate only. | `reported` (design target; runtime unverified) | Gives healthy qRoad its documented ≥3 s startup/lookahead plus drain and scheduling margin; AC-1 asserts the bound. It is not a measured production guarantee. |
| Healthy DMA-BUF lifecycle loss | Zero missing, duplicate, or out-of-order frame IDs per configured output; zero stream-recovery events | D-16 / `intake-brief.md`; deterministic test assertions `test_loggerd.py:347-359` | `reported` | Exact same behavior contract for the all-healthy arm; no performance claim is made here. |
| Invalid-input lifecycle | Rejected full-resolution frames are counted; no copy path; qRoad output and route progress continue; no loggerd/encoderd global exit or sibling reopen caused only by one encoder fault | D-28/D-29; source mechanisms above | `reported` | Failure arm intentionally permits drops on the rejected output; it does not waive liveness, containment, segment assignment, or observability. |

## Evidence and failure chain

1. **Rotation barrier:** KA2's deterministic branch in `loggerd.cc:63-72` publishes `started` after the segment duration, but it returns without the non-deterministic timeout at lines 75-93. `all_ready` is a count equality against all configured encode outputs (`loggerd.cc:328-335`). In `handle_encoder_msg` only a newer segment packet marks an encoder ready (`loggerd.cc:230-241`). An encoder that rejects each frame and emits no packet can therefore hold the writer at the old segment indefinitely.
2. **Observed integration gap:** `test_rotation` and `test_ka2_deterministic_rotation_alignment` return `skip` on KA2 because `create_buffers_with_sizes` produces the fixture's ordinary shared-memory buffers while full-resolution MPP requires DMA-BUF (`test_loggerd.py:192-215, 245-249`). The historical r8 focused pytest result was **4 skipped**. The previous run remained at segment 0 with zero post-boundary frames; it is not evidence that full encoderd rotation works.
3. **Drain path can stop the process:** `encoderd.cc:212-228` spends up to one 900 ms deadline flushing each MPP output. A failed flush returns false after disabling close-time flush; both forced-segment and ordinary segment callers set global `do_exit` (`encoderd.cc:454-461, 478-480`). One output's failure can thus terminate all camera encoder threads.
4. **Recovery path churns siblings:** `reopen_all_encoders` closes/reopens every output in the current camera thread and resets counters (`encoderd.cc:192-210`). It is called after any one output reaches 10 consecutive errors and after one second with no successful output in the thread (`encoderd.cc:485-528`). `road_camera_info` contains both main road and qRoad encoders (`loggerd.h:135-139`), so a road MPP rejection can repeatedly interrupt qRoad despite qRoad having a distinct RGA resize input path (`mpp_encoder.cc:399-415`).
5. **Positive smoke is narrower:** the saved DMA-heap direct-class smoke emitted 100/100 road and qRoad packets with zero failures; road imported 24 DMA-BUF fds and qRoad imported none. It verifies the encoder class and current system DMA-heap route, not loggerd's Params-driven segment lifecycle, camera-thread isolation, or the live V4L2 exporter (`mpp_encoder_no_fallback_smoke.log`, `report_no_copy_fallbacks_r8.md`).

## Recommended test contract

### Valid DMA-BUF end-to-end arm

- Add a test-only VisionIPC publisher with a finite pool of DMA-BUF heap allocations. Map and fill each allocation with changing frame identity/pattern data, set `VisionBuf` geometry, stride, UV offset, fd, and frame-id metadata, then register those buffers through `VisionIpcServer::register_external_buffers`. Handle DMA-BUF CPU synchronization explicitly; the existing generic `VisionIpcServer::send` defaults to `buf->sync()` which is ION-specific in this checkout, so the test must use an appropriate DMA-BUF sync sequence and avoid an incompatible ION sync call.
- Run the actual loggerd and encoderd processes with isolated `LOG_ROOT`, `PARAMS_ROOT`, and `OPENPILOT_PREFIX`; use fixed deterministic segment length, 20 FPS camera messages, and synthetic audio/Params state.
- Assert segment 0 -> 1 commit within the configured deadline; encoderd remains alive and converges to loggerd's segment/epoch; exact segment directories, sentinels, index-to-directory mapping, per-stream frame IDs, audio behavior, and decoded video identity remain valid. Drain through shutdown so delayed packets are included.

### Rejected shared-memory / invalid-input arm

- Feed mapped shared-memory VisionIPC buffers (or an isolated test input explicitly known not to implement DMA-BUF sync). Assert full-resolution MPP validation/import rejection with no memcpy or pointer retry.
- Keep the road camera stream flowing so road MPP rejection and qRoad RGA processing receive the same frame cadence. Assert loggerd commits within the deadline, qRoad emits and decodes frames into correct segment/index entries, and the loggerd/encoderd processes remain alive.
- Inject one independent encoder failure or stalled output to prove only that output is reopened/marked unavailable. Assert the healthy sibling's packet cadence and segment sequence continue without sibling reopen. If ordinary SHM is rejected by RGA too on the target image, test failure isolation using a per-encoder rejection seam with a valid DMA-BUF camera source, while retaining a separate invalid-fd/no-copy assertion.
- At the deadline, verify the failed output is named in rate-limited health/error reporting and its loss count increases. Verify the failure case's missing/partial media is handled per the documented contract; do not expect fake empty media or conceal the stream loss.
- Run the isolated test repeatedly enough to cover recovery to a valid DMA-BUF generation and a subsequent rotation. Recovery must not reset sibling state, decrease or skip segment ordinals, or place stale packets in a later file.

## Non-goals

- No full-resolution copy fallback, fallback mode, or `ENCODER_ZERO_COPY` selector.
- No changes to codec/profile, dimensions, frame rate, camera/audio selection, message schema, route naming, or healthy-path file set.
- No live V4L2/camera or sensor capture, on-road recording, production log/Params writes, broad manager/capture-path redesign, or resource-efficiency claim.

## Open implementation and qualification questions

| Question | Blocks | Owner | Latest safe decision point |
| --- | --- | --- | --- |
| Does the KA2 test image's RGA accept the shared-memory fixture's mapped virtual address while full-resolution MPP rejects its fd? If not, use the approved valid-DMA-BUF per-output rejection seam and retain direct invalid-input rejection proof. | M3 invalid-input fixture choice | KA2 runtime/build-test owner | Before selecting the M3 failure fixture |
| Does the live V4L2 exporter satisfy the DMA-BUF sync/fence, MPP import, and buffer-reuse lifetime assumptions exercised by the synthetic DMA-heap fixture? | Live-camera production qualification | Runtime/build owner | Before any live-camera or general production qualification claim |
| Do downstream consumers handle absent-before-first-keyframe and partial-after-start stream files as specified by D-34? | Release readiness | Loggerd consumer/build owner | Before release review |

## Limitations

- The historical r8 focused loggerd rotation and RecordFront pytest run skipped four KA2 fixture-dependent cases because they used shared-memory inputs; the full loggerd/encoderd rotation remains unverified.
- The r8 M0 DMA-heap fixture passed serial and xdist checks for synthetic VisionIPC registration, transport, and fixture-owned fd/mapping cleanup only. It did not exercise MPP import or full loggerd/encoderd rotation; see `evidence/r9/prior_report_dmabuf_fixture_r8.md`.
- The direct MPP smoke establishes 100 road and 100 qRoad packets only for a class-level synthetic run. It does not verify rotation, per-encoder recovery, failed-drain handling, writer file semantics, or stream health reporting.
- The live V4L2 exporter, DMA-BUF fence/synchronization behavior, MPP use with live camera buffers, and buffer-reuse lifetime remain unqualified.
- Historical r8 runtime/hook note: that environment reported Python 3.12.3 below the validator's ≥3.13 requirement, and hook registration was not independently verified. The r9 design self-check uses Python 3.13.15; the whole-repository runtime scan still stops at its byte limit, so stack/version discovery remains incomplete. This brief records no claim about active hook registration.
- No r9 product implementation or full lifecycle test has run.

## Severity-ranked findings

These findings describe the pre-implementation code observed in r8; D-30, D-32, D-34, and D-36 define the r9 design response, while implementation and full lifecycle acceptance remain outstanding.

- **Critical:** Deterministic KA2 rotation has no ready-packet timeout; a single stream with no output can prevent segment advancement indefinitely.
- **Critical:** One MPP segment-drain timeout currently returns failure to a caller that sets process-wide `do_exit`, allowing one output to stop the encoder service.
- **Major:** Per-encoder failure and aggregate stall recovery reopen all outputs in a camera thread, so one road failure can churn qRoad and other siblings.
- **Major:** The full loggerd/encoderd DMA-BUF lifecycle and recovery contract are untested; existing KA2 rotation assertions are skipped, and the direct-class smoke is narrower.
- **Minor:** Current health logging aggregates encoded/recovery/drop counters at camera-thread level and does not make a single output's persistent unavailability and segment loss easy to diagnose.
