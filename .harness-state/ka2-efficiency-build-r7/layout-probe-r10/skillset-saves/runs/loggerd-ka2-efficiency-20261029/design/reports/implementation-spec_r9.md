---
submission_id: loggerd-ka2-efficiency-20261029-design-r9-report-contract
revision: r9
---

# Implementation Spec — KA2 rotation liveness and encoder fault containment

**Revision:** r9  
**Design package revision:** r9  
**Source contracts:** Research `reports/report_research_r8.md` SHA-256 `b89553772eb8c1a829a68f54d0b8db759dd7a3976a88c3e8045446dd983cb1ab`; architecture `reports/report_architecture_r9.md` SHA-256 `a788e36c49a30aceb4e8bacaa97040db7eb20c5c0d5340f33fe878fc9a68b886`; interfaces `reports/report_interfaces_r9.md` SHA-256 `2ef10c8c8964203da994dd6d1b4a47db238820c3210ec365ca700f35365f007c`; delivery plan `reports/delivery-plan_r9.md` SHA-256 `32515ef83487806ec57310c36a85ca21450f6ba24ac17406c04c56c87816ed88`; security seed `artifacts/security_seed_r9.json` SHA-256 `519a2f68052384083e261eba83e9b1c4e4ddf5f479ae5b197acce08fd3a94846`.
**Target stack:** Existing C++ KA2/RK3588 loggerd/encoderd, MPP, RGA and VisionIPC stack; no new runtime or dependency. The Python 3.13.15 runtime probe stopped at the repository byte limit, so the whole-project stack classification remains incomplete.

**Behavior gate:** Every new production liveness behavior—rotation deadline/readiness, rotation report, runtime writer/audio/queue finalization, packet watchdog, and per-output recovery—runs only in deterministic KA2 mode: RK3588 with `ENCODERD_DETERMINISTIC_DISABLE` unset or zero; loggerd tests must also set `LOGGERD_TEST_DETERMINISTIC=1`. Non-KA2 and RK3588 with `ENCODERD_DETERMINISTIC_DISABLE=1` retain pre-r8 behavior. The shared startup `logger_rotate()` remains writer-free and emits no report in every mode; invoke the new commit hook only after a deterministic-KA2 runtime commit.

## Slice order and dependency graph

M0 establishes the test-only DMA-BUF fixture and executable contracts. M1 and M2 may be implemented in parallel after M0 because they own separate modules and communicate only through IF-1 through IF-4. Both must land before M3 full-lifecycle integration.

| # | Slice id | Depends on | Ships behind | Acceptance owner |
|---|---|---|---|---|
| 1 | `slice.test.dmabuf-fixture` | None; creates isolated test infrastructure first | Test-only C++ helper; no runtime switch | Build/test owner |
| 2 | `slice.encoder.per-output-recovery` | `slice.test.dmabuf-fixture` for fault-seam and packet identity contracts | New recovery policy only in deterministic KA2; preserve legacy paths. D-28 remains mandatory at all times | Encoder and build/test owners |
| 3 | `slice.loggerd.rotation-finalization` | `slice.test.dmabuf-fixture`; may develop alongside slice 2 after IF contracts are frozen | Deadline/report/finalization only in deterministic KA2; early all-ready commit remains immediate there | Loggerd and build/test owners |
| 4 | `slice.integration.lifecycle-proof` | Slices 1, 2, and 3; both runtime and segment contracts must be stable | Test-only harness and fault injection only | KA2 runtime and build/test owners |

## Slices

### `slice.test.dmabuf-fixture`

**Modules owned:** New `system/loggerd/tests/visionipc_dmabuf_publisher.cc` and its focused wrapper `system/loggerd/tests/test_dmabuf_fixture.py`. This slice does not change production VisionIPC allocation or camera code.

**Depends on:** None. The four-buffer DMA-heap publisher is a prerequisite for exercising full-resolution MPP import through VisionIPC rather than testing the encoder class alone.

**Data / migration changes:** None. The fixture allocates four `/dev/dma_heap/system` buffers for each configured stream, retains the mappings and fds through both client shutdowns, explicitly brackets CPU writes with `DMA_BUF_IOCTL_SYNC`, registers `VisionBuf*` entries with `register_external_buffers`, and calls `send(..., sync=false)` to avoid VisionBuf's ION-specific default sync method. It is test-only and adds no Python VisionIPC API or production environment selector.

**Contract tests:** IF-3. `test_visionipc_dmabuf_fixture` checks that registered external buffers reach an isolated VisionIPC client with expected fd, dimensions, stride, UV offset, frame id, and changing content; it verifies that fds/mappings remain live until clients stop and are released afterward.

**Proof plan:** First compile and run the fixture smoke against the target DMA heap. Fail the smoke if allocation, explicit sync, registration, fd delivery, metadata, frame identity or cleanup fails. Keep its scope at VisionIPC transport; MPP import and route behavior are proven in M3.

**Rollback:** Remove the test-only publisher and wrapper if fixture prerequisites cannot be met. No production behavior is changed. Do not replace the fixture with shared-memory buffers for the healthy MPP lifecycle proof.

**Acceptance evidence:** `pytest -q system/loggerd/tests/test_dmabuf_fixture.py::test_visionipc_dmabuf_fixture` passes on the pinned idle KA2 image, with no fd/mapping residue. Accepted by the build/test owner.

### `slice.encoder.per-output-recovery`

**Modules owned:** `system/loggerd/encoderd.cc`, `system/loggerd/encoder/mpp_encoder.h`, `system/loggerd/encoder/mpp_encoder.cc`, `system/loggerd/tests/test_mpp_encoder.cc`, `system/loggerd/tests/test_mpp_encoder.py`, and new `system/loggerd/tests/test_encoderd_recovery.cc` plus its test wrapper. No loggerd writer/rotation files belong to this slice.

**Depends on:** `slice.test.dmabuf-fixture` defines valid input and test-only fault-injection contracts. M1/M2 share no code files; merge both against the frozen Params segment contract before M3.

**Data / migration changes:** No database, file-format, cereal, route, or public-interface migration. In active deterministic KA2 only, IF-2 adds internal per-output status: submitted inputs, valid published-packet progress/time/frame id, bounded pending identities, drop reasons, active/unavailable/recovering state, and retry time. Non-KA2 and RK3588 disable=1 recovery behavior remains pre-r8. The Params authority keys remain owned by loggerd. Do not add a production env toggle.

**Contract tests:** IF-2 and D-28. In deterministic KA2 tests, `test_mpp_packet_progress_watchdog` proves that `encode_frame() > 0` alone is not an output heartbeat, zero-length/invalid packets retire their matching pending identity once (or quarantine the uncertain queue), and pending IDs never shift onto a later packet. `test_encoder_error_reopen_isolated` proves ten consecutive `-1` errors affect only that encoder, a persistent invalid fd cannot create a tight reopen loop, and retry backoff is 1, 2, 4, … seconds capped at 30 seconds. `test_segment_drain_timeout_isolated` proves one failed drain does not set process-wide `do_exit` or reopen qRoad. AC-12 verifies those deterministic watchdog/recovery policies do not leak into legacy modes. Preserve all-output reopen only for camera/VisionIPC reconnect where the producer fd generation is invalidated.

**Proof plan:** Introduce deterministic unit fault injection before changing recovery. In active deterministic KA2 only, add output progress accounting at the MPP publish point and a per-output supervisor for error streak, pending loss, watchdog, backoff, and segment assignment. Change `advance_segment_once` to give each output fair opportunities under the shared 900 ms drain budget; mark and reopen/quarantine only the output whose drain remains pending. Count every dropped frame identity once, including pending identities retired on forced close. Preserve the existing recovery path in non-KA2 and RK3588 disable=1 modes, proven by AC-12. Keep full-resolution import as one DMA-BUF validation followed by one MPP DMA import attempt in every mode. Rejected frames return an error and are dropped; no full-resolution copy path, pointer retry, selector, or copy state is allowed. qRoad's existing RGA resize remains its normal input path.

**Rollback:** Revert this slice together with `slice.loggerd.rotation-finalization` if the shared boundary contract fails. Do not roll back to a copy fallback or add a selector. Before release, stop promotion and use only a separately known-good approved build if available; otherwise hold and escalate rather than shipping the known stuck no-copy baseline.

**Acceptance evidence:** `pytest -q system/loggerd/tests/test_mpp_encoder.py system/loggerd/tests/test_encoderd_recovery.py` proves the focused MPP and recovery contracts. The M3 process-level fault arms remain required before this slice is release-ready. Accepted by the encoder owner and build/test owner.

### `slice.loggerd.rotation-finalization`

**Modules owned:** `system/loggerd/loggerd.cc` and new `system/loggerd/tests/test_loggerd_rotation.cc` with its Python test wrapper. No encoderd/MPP supervisor code belongs to this slice.

**Depends on:** `slice.test.dmabuf-fixture` freezes IF-1/IF-4 input contracts; development may proceed parallel to encoder slice 2, but M1 drain behavior must be integrated before M3. Do not change the existing non-deterministic fallback branch or segment/media schema.

**Data / migration changes:** No persisted migration, cereal change, public interface change, or new IPC channel. Continue to publish the existing `LoggerdRotateState`, `LoggerdSegment`, and `LoggerdSegmentEpoch`. Only deterministic-KA2 runtime commits write the non-authoritative `LoggerdRotationReport`, with exactly seven top-level keys: `route` (current route string), `epoch` and `segment` (post-commit nonnegative integers), `degraded` (boolean), `missing_streams` (unique configured names in order), `elapsed_ms` (nonnegative integer milliseconds from monotonic `started` publication to the commit decision), and `known_drop_counts`. The nested map has exactly five always-present nonnegative integer keys: `stale_video_messages`, `future_video_queue_drops`, `audio_no_writer_events`, `audio_expired_events`, and `audio_capacity_drops`. Its loggerd-observed counters cover events/messages since the prior report (since route startup for the first report), not destination-segment totals. `stale_video_messages` counts encoded messages loggerd drops because their mapped segment is older than the committed segment; `future_video_queue_drops` counts encoded messages rejected by the existing bounded future queue; `audio_no_writer_events` counts accepted rawAudioData events discarded when a commit makes their tagged segment unretainable before expiry; `audio_expired_events` counts accepted events whose five-second hold expires first; `audio_capacity_drops` counts newest whole events rejected at admission because adding the event would exceed 256 KiB. Video counters count one per dropped encoded message; audio counters count one per dropped rawAudioData message event, not payload bytes or samples, with mutually exclusive terminal reasons. MPP rejections, encoder stalls, drain failures, pending-frame retirement, and retry counts remain encoder-local and never enter this loggerd map. `degraded` is true exactly when a stream is missing or a reported loggerd counter is nonzero. Loggerd alone writes the report; it never gates encoderd segment authority. Startup `logger_rotate()` remains writer-free and emits no report.

**Contract tests:** IF-1 and IF-4, with `LOGGERD_TEST_DETERMINISTIC=1` explicit in deterministic-KA2 tests. AC-1 owns monotonic `started`-to-commit `elapsed_ms`, the 5,000 ms deadline plus one ≤50 ms wake/scheduler tick, no-message liveness, and immediate all-ready behavior. `test_ka2_rotation_readiness_resets_each_epoch` verifies readiness is counted once per configured stream and active epoch. AC-3's `test_loggerd_commit_finalizes_writers_and_bounds_audio` checks runtime writer closure, segment resets, `EncodeIndex` mapping, and audio order/cap behavior. At the 256 KiB boundary it asserts that an incoming newest whole event that would overflow is dropped atomically, the existing FIFO contents and timestamp order are preserved, no payload is split or older event evicted, and only `audio_capacity_drops` increments once. It also verifies that an accepted event receives exactly one terminal classification by monotonic expiry-versus-commit order, with expiry winning an exact tie. AC-9's `test_loggerd_rotation_report_contract` checks the exact report schema and loggerd-observed counters. AC-12 proves legacy behavior and startup remain pre-r8.

**Proof plan:** Add deterministic control tests against a fake monotonic clock and packet set before code. In active deterministic KA2 only, start the deadline when `started` publishes; evaluate on message batches and zero-message poll wakes with a ≤50 ms poll cap. Commit once all streams are ready or once at the deadline with exact missing names. On deterministic-KA2 runtime commits only, finalize existing writers, reset recording/audio/readiness state, and classify queued video by its actual `EncodeIndex` segment. Tag raw audio by segment and timestamp; preserve the existing ordered queue. If admitting an entire newest `rawAudioData` event would exceed 256 KiB, reject that whole event atomically, preserve the FIFO prefix unchanged, do not split the payload or evict older events, and count exactly one capacity drop. For accepted events, count one terminal cause only: whichever occurs first, five-second expiry or commit making the segment unretainable; compare monotonic times and classify an exact tie as expired. Never move older audio/video into a later segment. Keep non-KA2 and RK3588 disable=1 behavior pre-r8, and keep startup `logger_rotate()` writer-free/no-report in every mode. A missing track may be absent or partial; never create an empty placeholder.

**Rollback:** Revert only with encoder slice 2 so the two sides cannot disagree about segment catch-up. No Params backfill is needed because `LoggerdRotationReport` is diagnostic only. Reverting does not authorize re-enabling full-resolution copy; if the liveness fix cannot pass, hold release and escalate.

**Acceptance evidence:** `pytest -q system/loggerd/tests/test_loggerd_rotation.py` proves deterministic-KA2 timeouts without packets, epoch reset, runtime writer finalization, segment queue mapping, and audio order/bounds. `test_loggerd_legacy_modes_preserve_rotation_behavior` (AC-12) proves legacy branches and startup behavior. Accepted by the loggerd owner and build/test owner.

### `slice.integration.lifecycle-proof`

**Modules owned:** New `system/loggerd/tests/test_loggerd_liveness.py`, integration assertions, and test-only fault configuration. The C++ DMA-BUF publisher remains owned by slice 1; production code remains owned by slices 2 and 3.

**Depends on:** Fixture, encoder supervisor, and loggerd timer/writer lifecycle must each pass their focused contracts. The integration suite is last because a mixed implementation would not prove the agreed IF-1–IF-4 protocol.

**Data / migration changes:** None to production data. Before creating any `Params` object or starting any process, set unique temporary `LOG_ROOT`, `PARAMS_ROOT`, and `OPENPILOT_PREFIX`; assert the resolved paths are under the test temp directory. Seed only isolated `RecordFront` and `RecordAudio`. The fixture owns its dma-heap fds/mappings and closes them after VisionIPC, encoderd, and loggerd stop.

**Contract tests:** Every deterministic-KA2 process test sets `LOGGERD_TEST_DETERMINISTIC=1` explicitly and leaves `ENCODERD_DETERMINISTIC_DISABLE` unset or zero. `test_ka2_valid_dmabuf_rotation_end_to_end` (AC-7) runs real loggerd and encoderd with four DMA-BUF buffers per configured camera stream, continuous synthetic audio and two rotations; it requires qRoad and all configured healthy streams to be ready before every 5 s deadline and verifies exact frame identities/pixels, timestamps, indexes, segments, sentinels, codecs/container metadata, audio decode, expected file set, no drops/recoveries, and clean shutdown. `test_ka2_rejected_shm_rotation_preserves_qroad` (AC-8) proves full-resolution MPP rejects ordinary SHM after one validation/import path without copying, loggerd advances, both processes remain alive, and qRoad/media/audio remain healthy. If RGA cannot consume SHM on the target, retain a direct actual invalid-fd/SHM MPP rejection test and use the specified LOGGERD_TEST-only per-output rejection seam with valid DMA-BUF for the lifecycle sibling-isolation arm; record the SHM/RGA limitation. AC-9 asserts the exact seven-field `LoggerdRotationReport` and five-key nested map, JSON types/ranges (integer fields reject booleans/fractions), zero-key presence, counter window/reset, post-commit values, ordered missing names, and degraded truth table. It also verifies delayed audio expiry is logged immediately and appears in the next report. AC-6 and AC-8 separately assert encoder-local drain/MPP loss logs and counters and prove they do not enter `known_drop_counts` or a new IPC channel. AC-10 asserts process/device preflight and all process/fd/mapping/temp-file/Params cleanup in `finally`. AC-11 verifies D-28 no-copy source invariants and unchanged healthy media contracts. AC-12 runs `test_loggerd_legacy_modes_preserve_rotation_behavior` in isolated subprocesses for a non-KA2 legacy configuration and RK3588 with `ENCODERD_DETERMINISTIC_DISABLE=1`; assert pre-r8 rotation timing, writer finalization, packet queues, and audio behavior, no deterministic deadline/runtime finalization/report/per-output recovery, and startup `logger_rotate()` writer-free/no-report.

**Proof plan:** Use the fixture and test seams created in earlier slices; launch no manager-owned route and no live camera/sensor. Fail the healthy case for any missing qRoad, timeout, retry, duplicate/lost frame, changed content, wrong segment/index, audio mismatch, altered healthy file set, or residue. Fail the degraded case if the failed output blocks rotation, changes segment authority, kills/reopens qRoad, writes a placeholder or cross-segment packet, or hides its drop. Repeat on the pinned KA2 image for DMA-buffer-ring reuse. Recheck device/process users immediately before every board run; if another camera/MPP task is active, do not run or stop it.

**Rollback:** Test-only changes can be removed without changing runtime. If any integration acceptance fails, keep the implementation unpromoted and revert the coupled loggerd/encoderd implementation as one unit; no environment switch or copy fallback is an approved rollback. If an older approved build is unavailable or known unsafe, hold and escalate instead of deploying the no-copy base with unbounded rotation.

**Acceptance evidence:** `pytest -q system/loggerd/tests/test_loggerd_liveness.py` on an idle, pinned KA2 board with isolated roots, retained process/preflight/cleanup logs, and decoded media assertions; run AC-12's non-KA2 and disable=1 compatibility configurations in their required isolated build/process environments. Accepted jointly by the KA2 runtime owner and build/test owner. AC-1 through AC-12 must close before release readiness: valid full loggerd+encoderd DMA-BUF rotation, invalid input, silent-output and drain-failure cases all pass with qRoad present and no healthy sibling churn. A direct MPP class smoke alone is insufficient.

## Acceptance-to-touchpoint map

| Planner criterion | Implementation/test touchpoint | Slice |
|---|---|---|
| AC-1 rotation deadline | `loggerd.cc`; `test_loggerd_rotation.py::test_ka2_rotation_deadline_without_packets` | M2 |
| AC-2 epoch readiness | `loggerd.cc`; `test_loggerd_rotation.py::test_ka2_rotation_readiness_resets_each_epoch` | M2 |
| AC-3 writer/audio/queue finalization | `loggerd.cc`; `test_loggerd_rotation.py::test_loggerd_commit_finalizes_writers_and_bounds_audio` | M2 |
| AC-4 actual packet progress and pending IDs | `mpp_encoder.{h,cc}`, `test_mpp_encoder.cc`, `test_encoderd_recovery.cc::test_mpp_packet_progress_watchdog` | M1 |
| AC-5 local error/backoff recovery | `encoderd.cc`, `test_encoderd_recovery.cc::test_encoder_error_reopen_isolated` | M1 |
| AC-6 nonfatal drain timeout | `encoderd.cc`, `test_encoderd_recovery.cc::test_segment_drain_timeout_isolated` | M1 |
| AC-7 valid DMA-BUF full lifecycle | `visionipc_dmabuf_publisher.cc`, `test_loggerd_liveness.py::test_ka2_valid_dmabuf_rotation_end_to_end` | M0, M3 |
| AC-8 invalid input and qRoad | `mpp_encoder.cc`, test-only rejection seam, `test_loggerd_liveness.py::test_ka2_rejected_shm_rotation_preserves_qroad` | M1, M3 |
| AC-9 loggerd report diagnostics | `loggerd.cc`, `test_loggerd_rotation.py::test_loggerd_rotation_report_contract`, `test_loggerd_liveness.py` assertions | M2, M3; AC-6/AC-8 retain encoder-local logs/counters in M1/M3 |
| AC-10 isolation/cleanup | `visionipc_dmabuf_publisher.cc`, `test_loggerd_liveness.py` fixture setup/teardown | M0, M3 |
| AC-11 no-copy/regression contract | `mpp_encoder.{h,cc}`, static scan, focused suite and diff review | M1, M4 review |
| AC-12 legacy-mode/startup compatibility | `loggerd.cc`, `encoderd.cc`, `test_loggerd_liveness.py::test_loggerd_legacy_modes_preserve_rotation_behavior`; assert pre-r8 rotation/writer/queue/audio behavior for non-KA2 and RK3588 `ENCODERD_DETERMINISTIC_DISABLE=1`, no deterministic deadline/finalization/report/per-output recovery, and writer-free/no-report startup; explicit `LOGGERD_TEST_DETERMINISTIC=1` on KA2 tests | M1–M4 |

## Operational constraints

| Constraint | Slices affected | Owner | Removal criterion |
|---|---|---|---|
| Never enable full-resolution copy; exactly one DMA-BUF validation and one MPP import attempt | M1, M3 | Encoder owner | Permanent D-28 rule; remove only with a new explicit user decision and architecture revision |
| No production selector/fallback or test seam in production build | M0–M3 | Encoder/build owner | Test-only seams compile only under test build; no removal from runtime because none exists |
| `LoggerdRotationReport` is exactly seven top-level fields with five fixed loggerd-observed drop keys; it is diagnostic only and existing segment authority Params are unchanged | M2, M3 | Loggerd owner | Keep diagnostic non-authoritative; no consumer migration planned |
| New deadline, report, runtime finalization, queue/audio handling, packet watchdog, and per-output recovery run only in deterministic KA2; non-KA2 and RK3588 disable=1 retain pre-r8 behavior; startup `logger_rotate()` is always writer-free/no-report | M1–M4 | Encoder/loggerd owners | AC-12 passes both legacy configurations and startup assertions |
| Deterministic-KA2 rotation deadline starts at `started`, monotonic 5 s, timer poll ≤50 ms | M2, M3 | Loggerd owner | No deadline-removal criterion; active only in deterministic KA2 |
| Deterministic-KA2 qRoad startup grace is 3 s; packet watchdog pauses during explicit rotation; MPP drain total is 900 ms and fairly shared | M1–M3 | Encoder owner | Keep until target measurements justify a new design revision; do not tune silently |
| Deterministic-KA2 qRoad audio pre-writer queue is bounded to 5 s or 256 KiB; overflowing newest events are atomically rejected, preserving FIFO prefix; accepted events have one expiry/no-writer terminal cause | M2, M3 | Loggerd owner | Expiry and capacity rejection are permanent failure behavior in deterministic KA2; queue remains bounded |
| Set isolated `LOG_ROOT`, `PARAMS_ROOT`, `OPENPILOT_PREFIX` before any Params/service startup | M0, M3 | Test owner | Keep for every synthetic lifecycle test |
| Recheck board process/device users before each KA2 test; never stop unrelated manager/uploader/deleter | M3 | KA2 runtime owner | Keep for every target run |
| No live camera/V4L2, sensor, on-road run, production routes, merge, or deployment in this design | M0–M4 | Admiral / runtime owner | Requires separate scope/approval and qualification design |

## Non-goals

- No full-resolution copy fallback, mode flag, pointer import retry, or alternate MPP import route.
- No camera/manager, route naming, cereal schema, codec, stream-selection, public API, or broad storage redesign.
- No claim that DMA-heap synthetic inputs prove the live V4L2 exporter, fence ordering, or camera buffer-reuse lifetime.
- No CPU/RSS/latency efficiency claim or benchmark; this design repairs liveness and failure containment only.
- No test against production logs/Params or live cameras/sensors.

## Blocking decisions and open risks

| Decision / risk | Owner | Blocks slices | Latest safe decision point |
|---|---|---|---|
| Confirm RGA can consume the current SHM fixture's mapped address while full-resolution MPP rejects its fd; otherwise use the documented test seam and preserve a direct invalid-fd test | KA2 runtime/build-test owner | M3 invalid-input arm | Before selecting the target failure fixture |
| Live V4L2 DMA-BUF sync/fence/reuse qualification remains outside synthetic tests; no general production-camera claim can be made from this package | KA2 runtime owner / Admiral | Any live-camera release claim | Before any live-camera/on-road run |
| Incomplete whole-repository runtime detection (Python 3.13.15 scan exceeded byte cap); architecture adds no stack dependency, but version detection remains incomplete | Commander | Stack-lock evidence only; no code slice if no dependency is added | Recheck during stack-lock/package validation |
| Existing downstream handling of absent/partial stream files must be inspected before release | Loggerd/build-review owner | M3 release readiness | Before merge/release review |

No migration is required. There is no production feature flag and no safe copy-path rollback. If a slice requires a new public or Params contract, changes file naming/index behavior, or needs a production selector, return to architect/commander instead of introducing it in build.

## Handoff

This specification returns to `design/commander` for stack-lock and `design-to-build` assembly. The build team receives it only after design gate approval. It does not authorize implementation, target testing, commit, merge, or deployment by itself.
