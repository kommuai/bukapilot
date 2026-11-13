# Requirements brief — KA2 segment rotation and encoder fault containment, r8

**Revision:** r8 (intake revision 153)  
**Prepared for:** `design/architect`  
**Sources mined:** `intake/report_grilling.md`, `intake/intake-brief.md`, `build/reports/report_no_copy_fallbacks_r8.md`, `build/evidence/loggerd_shm_rotation_not_applicable_pytest.log`, `build/evidence/mpp_encoder_no_fallback_smoke.log`, `system/loggerd/loggerd.cc`, `system/loggerd/loggerd.h`, `system/loggerd/encoderd.cc`, `system/loggerd/encoder/mpp_encoder.cc`, `system/loggerd/tests/test_loggerd.py`, `msgq_repo/msgq/visionipc/visionipc_server.h`, `msgq_repo/msgq/visionipc/visionipc_server.cc`.

## Problem statement

On KA2, full-resolution MPP must receive a valid DMA-BUF fd. D-28 requires one validation and one MPP import attempt; rejected input is dropped, with no copy, environment switch, or pointer-based retry. The current KA2 loggerd rotation handshake waits for a next-segment packet from every configured encoder. If even one stream emits none, deterministic rotation can remain in `started` forever. Encoderd also has two independent whole-pipeline failure paths: an MPP drain timeout makes the camera thread set global `do_exit`, and an encoder-local error threshold or aggregate output stall reopens every encoder in that camera thread. Together these paths can turn one invalid or stalled encoder into both a stuck segment and churn or shutdown for healthy sibling output such as qRoad.

The r8 design must treat liveness and data loss separately: preserve normal segment/media behavior when DMA-BUF inputs work; when an input is rejected, drop and count that stream's frames, keep healthy streams alive, and let the route advance on a deadline. Do not restore full-resolution copying.

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
| R1 | In deterministic KA2 mode, once loggerd publishes rotation state `started`, it must commit the next segment within a finite deadline even if one or more encoder streams never publish a next-segment packet, provided camera input and loggerd's event loop continue. | User decision D-29, `intake/report_grilling.md`; observed branch in `system/loggerd/loggerd.cc:57-73` | `reported` (request); mechanism `observed` | Rotation authority, deadline contract, failure-case acceptance | Recommend a hard bound of **3 seconds from `started`**. This is inferred from the existing 40-frame qRoad lookahead at 20 FPS (~2 seconds), the 900 ms MPP drain limit, and the 100 ms deterministic segment-sync cadence (`test_loggerd.py:274-275`, `encoderd.cc:29-31`). Architect should confirm the bound and ensure the graceful wait never compounds into an unbounded wait. |
| R2 | A failed encoder or drain timeout must not set process-wide exit while healthy encoder outputs in the same camera thread can continue. | D-29; observed `encoderd.cc:212-228, 454-461, 478-480` | `reported` + `observed` | Encoderd error model, segment-advance interface, containment tests | The recovery owner must specify how a failed encoder is isolated, closed/reopened, or temporarily marked unavailable without closing healthy siblings. |
| R3 | Error thresholds and output-stall detection must be tracked per encoded output; a healthy sibling's packet must not reset or trigger another output's recovery decision. | D-29; observed `encoderd.cc:162-165, 192-210, 485-528` | `reported` + `observed` | Per-encoder state and recovery policy | Retry/backoff limits and the condition for returning an unavailable stream to service are not set by intake; architect should make them explicit and bounded. |
| R4 | When MPP rejects a full-resolution input fd or import, the frame is dropped and counted for that output; the encoder must not copy it, attach a pointer for a second import attempt, or select a copy mode. | D-28, `intake/report_grilling.md`; observed `mpp_encoder.cc:337-363, 399-415` and `report_no_copy_fallbacks_r8.md` | `reported` + `observed` | MPP path, error classification, regression scans | No unresolved assumption. This remains binding for every failure and recovery path. |
| R5 | Failure reporting must identify the encoder/stream, boundary or segment, error/recovery event, and accumulated dropped-frame count without logging every frame at error severity. | D-29's liveness and production-readiness intent; current aggregate health/failure logs in `encoderd.cc:497-510, 531-537` | `inferred` | Operational diagnostics and supportability | No new cereal schema or externally visible service is requested. Architect should decide whether rate-limited logs plus existing health logs provide sufficient operator visibility. |
| R6 | With valid synthetic DMA-BUF inputs, the actual isolated loggerd+encoderd lifecycle must rotate and keep processing; every healthy configured stream must produce valid, decodable output assigned to the correct segment. | D-3/D-16 and D-29; current real-path test at `test_loggerd.py:212-365` is skipped on KA2; class-level DMA-heap smoke in `mpp_encoder_no_fallback_smoke.log` | `reported`; path-level readiness is unproven | Fixture, integration test, acceptance | A test publisher can allocate/map DMA-BUF buffers and pass `VisionBuf*` entries through the observed `VisionIpcServer::register_external_buffers` API (`visionipc_server.h:41`, `visionipc_server.cc:81-93`). The current MPP smoke is not full lifecycle proof. |
| R7 | With invalid ordinary shared-memory input, full-resolution MPP must reject/drop without copying, loggerd must advance by its deadline, and a healthy qRoad sibling must continue producing correctly indexed output from the same frame stream. | D-28/D-29; observed test skips at `test_loggerd.py:192-195, 212-215`; observed downscale path uses `buf->addr` with RGA while full-resolution imports `buf->fd` (`mpp_encoder.cc:399-415`) | `reported` + `observed`; qRoad survival under SHM is `inferred` | Fault-injection/test case and output contract | The existing test uses shared-memory VisionIPC buffers and skips before asserting lifecycle. Prove whether RGA accepts this fixture's mapped address on the test image; if not, inject a per-encoder MPP rejection while keeping a valid qRoad input and separately retain a direct invalid-fd rejection assertion. |
| R8 | Healthy behavior retains the existing route naming, segment ordinals, sentinel placement, encode index ordering, timestamps, codecs, and audio/front-camera behavior. Each emitted encode index must name the segment directory that contains it; no stale packet may cross a committed boundary. | D-3/D-13/D-16; test assertions at `test_loggerd.py:314-365`; writer/index path at `loggerd.cc:194-241` | `reported` + `observed` | Segment/media contracts and tests | The healthy DMA-BUF case must continue to require exact per-frame coverage under D-16, including startup, rotation, and teardown. |
| R9 | In the explicit degraded-input case, loggerd and healthy output files/logs continue, while failed-stream frame loss and any absent failed-stream media for the affected segment are surfaced; do not fabricate empty media to imply successful encoding. | D-28 permits rejected-frame loss; D-29 requires liveness and failure containment; `VideoWriter` is created on first matching encoded packet at `loggerd.cc:194-208` | `inferred` | Degraded media contract, operator reporting | Intake did not choose placeholder-file semantics. Recommend a missing/partial failed-stream media file as the honest contract, with no empty fake file; architect should confirm against downstream consumers and state how the missing stream is reported. |
| R10 | All test logs, Params, IPC endpoints, and output routes are isolated under per-run temporary roots; testing uses synthetic frame/audio publishers only and stops if competing loggerd/encoderd/camera/MPP work appears. | D-6/D-29 in intake brief | `reported` | Test harness, safe runtime procedure | Recheck host idleness immediately before a bounded KA2 run; do not use live V4L2/camera or production log/Params locations. |

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
| Deterministic rotation completion after the published `started` state while new input is flowing | Recommended maximum 3 seconds | Inferred from the observed 40-frame qRoad lookahead comment at 20 FPS (about 2 s), 900 ms MPP drain deadline, and 100 ms sync cadence; `test_loggerd.py:274-275`, `encoderd.cc:29-31` | `inferred` | Provides margin for supported qRoad delay while retaining a hard, reviewable upper bound. This is a proposed target for architecture approval, not a measured current guarantee. |
| Healthy DMA-BUF lifecycle loss | Zero missing, duplicate, or out-of-order frame IDs per configured output; zero stream-recovery events | D-16 / `intake-brief.md`; deterministic test assertions `test_loggerd.py:347-359` | `reported` | Exact same behavior contract for the all-healthy arm; no performance claim is made here. |
| Invalid-input lifecycle | Rejected full-resolution frames are counted; no copy path; qRoad output and route progress continue; no loggerd/encoderd global exit or sibling reopen caused only by one encoder fault | D-28/D-29; source mechanisms above | `reported` | Failure arm intentionally permits drops on the rejected output; it does not waive liveness, containment, segment assignment, or observability. |

## Evidence and failure chain

1. **Rotation barrier:** KA2's deterministic branch in `loggerd.cc:63-72` publishes `started` after the segment duration, but it returns without the non-deterministic timeout at lines 75-93. `all_ready` is a count equality against all configured encode outputs (`loggerd.cc:328-335`). In `handle_encoder_msg` only a newer segment packet marks an encoder ready (`loggerd.cc:230-241`). An encoder that rejects each frame and emits no packet can therefore hold the writer at the old segment indefinitely.
2. **Observed integration gap:** `test_rotation` and `test_ka2_deterministic_rotation_alignment` return `skip` on KA2 because `create_buffers_with_sizes` produces the fixture's ordinary shared-memory buffers while full-resolution MPP requires DMA-BUF (`test_loggerd.py:192-215, 245-249`). The saved focused pytest result is **4 skipped**. The previous run remained at segment 0 with zero post-boundary frames; it is not evidence that full encoderd rotation works.
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

## Open questions for architecture

| Question | Blocks | Owner | Latest safe decision point |
| --- | --- | --- | --- |
| Confirm the proposed 3 second hard deadline after `started`, and define which normal stream delay it covers. | Rotation contract and deterministic test bound | `design/architect` | Before delivery-plan acceptance is written |
| After one encoder misses drain or fails repeatedly, should it be retried with capped backoff or marked unavailable until a new valid input/reconnect? Define retry cap, segment rejoin point, and no-busy-loop rule. | Per-encoder state machine and recovery tests | `design/architect` | Before build implementation spec is locked |
| For a failed stream with no packet in a segment, should the media file be absent or partial? Confirm consumer expectations and define the exact loss marker in logs/health output. | File-set and operator-visible loss contract | `design/architect` with loggerd consumer evidence | Before build acceptance is written |
| Does the KA2 test image's RGA accept the shared-memory fixture's mapped virtual address while full-resolution MPP rejects its fd? | Invalid-input integration design | `design/architect`/build test owner | Before selecting the failure-test fixture |
| Does the synthetic DMA-heap VisionBuf publisher require any target-specific cache operation beyond DMA-BUF `START/END` sync, and can it register and serve those fds through the current C++ VisionIPC API? | Valid end-to-end fixture | Build test owner on idle KA2 | Before claiming full lifecycle coverage |
| Does the live camera/V4L2 exporter satisfy the same input-fd synchronization and lifetime contract as the system DMA heap? | Production camera qualification | Runtime/build owner | Before generalizing synthetic qualification to live recording |

## Limitations

- The focused loggerd rotation and RecordFront tests were skipped on KA2; full loggerd/encoderd rotation using valid DMA-BUF VisionIPC frames is unverified.
- The direct MPP smoke establishes 100 road and 100 qRoad packets only for a class-level synthetic run. It does not verify rotation, per-encoder recovery, failed-drain handling, writer file semantics, or stream health reporting.
- The live V4L2 exporter, synchronization/fence semantics, and MPP input lifetime remain unqualified.
- Runtime readiness is limited: the available Python is 3.12.3 (<3.13 required) and the project scan exceeded its byte limit. `/data/kommu/.codex/hooks.json` and its referenced pre/post hook scripts exist, and the scripts are executable; automatic host registration or execution was not independently verified. This research stage read source and saved artifacts only; it did not run product tests or a board lifecycle experiment.

## Severity-ranked findings

- **Critical:** Deterministic KA2 rotation has no ready-packet timeout; a single stream with no output can prevent segment advancement indefinitely.
- **Critical:** One MPP segment-drain timeout currently returns failure to a caller that sets process-wide `do_exit`, allowing one output to stop the encoder service.
- **Major:** Per-encoder failure and aggregate stall recovery reopen all outputs in a camera thread, so one road failure can churn qRoad and other siblings.
- **Major:** The full loggerd/encoderd DMA-BUF lifecycle and recovery contract are untested; existing KA2 rotation assertions are skipped, and the direct-class smoke is narrower.
- **Minor:** Current health logging aggregates encoded/recovery/drop counters at camera-thread level and does not make a single output's persistent unavailability and segment loss easy to diagnose.
