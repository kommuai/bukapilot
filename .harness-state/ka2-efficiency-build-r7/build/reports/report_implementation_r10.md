# Implementation evidence — KA2 rotation liveness r10

**Run:** `loggerd-ka2-efficiency-20261029`  
**Submission:** `loggerd-ka2-efficiency-20261029-build-r10`  
**Owner:** build/bob-the-builder  
**Boundary:** build-to-review

## Approval and scope

Before editing, I verified the approved package hashes:

- Phase verdict `design/verdict_design-to-build-r10.json`: `f25b3447111888de96608b9a47af5aae3bc6b00500f6923a125219e832b01f5b`.
- Cross-stage verdict `design/verdict_design-to-build-r10.cross-stage.json`: `bc2d854ba4675408eb4c5329ee1252b4645f78075629aa4fe70d5f952042d75d`.
- Active `build/reports/build-change-list-r10.md`: `d6d92f05a6c840b5cdb02cd02954f8c7269fa67a1892ee6cd0204c1019581b4c`.

All product edits are within the approved r10 paths. No change-list amendment was needed. The prior `common/params_pyx.so` and `selfdrive/appbridged/appbridged.py` changes were not touched. I did not edit or run the test suite, start camera/manager-owned processes, or write production Params/log roots.

## Implemented slices

**M1 — encoderd and MPP.** Deterministic KA2 progress now follows valid published-packet count deltas, including the aggregate health update; positive input submission is not counted as progress. MPP records submitted inputs, valid packets, packet timestamps/frame IDs, pending identities, and per-output dispositions. Empty output retires the oldest FIFO identity once; forced close retires outstanding identities once. Segment advance uses one shared 900 ms deadline and gives active outputs repeated 10 ms drain slices. Only outputs still pending at expiry are quarantined and locally retried; healthy outputs continue through their planned close/open transition. Planned transition, failure retry, and failure-recovery reopen counters stay distinct. Non-KA2/disabled recovery paths remain separate.

Authority reads use full-string nonnegative decimal parsing for segment and epoch, plus exact `started`/`committed` recognition, only in deterministic mode. Invalid or empty active-mode values are retained as-is and logged with `LOGW_100`; affinity parsing and legacy parsing remain unchanged.

Full-resolution MPP still validates DMA-BUF synchronization and makes one `mpp_buffer_import` attempt per fd, with no copy fallback, pointer retry, or selector. qRoad continues through the existing RGA resize branch.

**M2 — loggerd.** Deterministic KA2 rotation is timer driven, starts the 5,000 ms readiness deadline at `started`, evaluates readiness without requiring service messages, and carries the single commit-decision timestamp through elapsed reporting and audio expiry/finalization. Runtime commits retain the exact seven-field report and five fixed loggerd-observed drop counters; startup remains writer-free and emits no report. Queued audio stays timestamp ordered, expires after five seconds, and is capped at 256 KiB. Capacity is checked before allocating the queued payload copy; matching-writer events write directly. Commit and expiry share the decision timestamp, so expiry wins an exact tie and events expiring after the decision are classified by the commit first.

## Test-only seam contract for test-builder

The controllable loggerd monotonic clock is compiled only with `LOGGERD_TEST_BUILD` and only selects its fake source when the process has `LOGGERD_TEST=1` exactly. It stays disabled by default, including in `LOGGERD_TEST=1` lifecycle subprocesses. The test-build C++ API is:

```cpp
void loggerd_test_enable_monotonic_clock();
void loggerd_test_advance_monotonic_ms(int64_t delta_ms);
int64_t loggerd_test_monotonic_ms();
```

The direct `test_loggerd_rotation.cc` test should set `LOGGERD_TEST=1`, call `loggerd_test_enable_monotonic_clock()` before loggerd's first clock-dependent operation, then advance with positive deltas. Advancing is ignored until explicitly enabled. The same clock drives segment duration, rotation deadline, report elapsed time, audio admission/expiry/finalization, and poll-deadline calculation. Full E2E loggerd subprocess tests do not call the enable API and therefore use real `steady_clock`, including when they set a short `LOGGERD_SEGMENT_LENGTH`. There is no production environment, Params, or IPC clock selector.

The MPP test seam is `MppEncoder::inject_test_drain_timeout(const VisionIpcBufExtra &)`, declared and defined only under `LOGGERD_TEST_BUILD`. Encoderd calls it immediately before ordinary deterministic boundary drain. The one-shot selector requires `LOGGERD_TEST=1`, `LOGGERD_TEST_MPP_FAULT_STREAM=<publish_name>`, and `LOGGERD_TEST_MPP_FAULT_MODE=drain_timeout`; it leaves one pending test identity so the output reaches the shared 900 ms boundary, not the 1 s watchdog. The existing `reject` and `silent` modes remain compile-guarded and runtime-gated. No test source or target definition was added here.

For lifecycle counter assertions, test builds also log a snapshot immediately after each per-output retry-start, successful failure-reopen, and successful planned segment-transition counter increment. The helper and all call sites compile only with `LOGGERD_TEST_BUILD` and return unless `LOGGERD_TEST=1` exactly. Each snapshot is one stable line:

```text
LOGGERD_TEST_MPP_COUNTERS stream=<publish_name> cur_seg=<n> segment_transition_reopen_count=<n> failure_recovery_reopen_count=<n> failure_retry_count=<n>
```

Production binaries contain no counter logging. Test-builder can assert two qRoad planned-transition increments and zero qRoad failure-recovery/retry deltas across two ordinary boundaries, including the sibling drain-fault arms.

## Checks

The complete command output, including earlier compile errors that were corrected, is recorded in [impl-r10-checks.log](../evidence/impl-r10-checks.log), SHA-256 `05c71f29c18f6199fdcde8b15db08342792b764e23862af055a393423d3e9790`.

- `scons -Q system/loggerd/loggerd system/loggerd/encoderd` — final RK3588 production build, exit 0.
- `scons -Q --ccflags=-DLOGGERD_TEST_BUILD system/loggerd/loggerd system/loggerd/encoderd` — final test-seam build, exit 0.
- The same two build commands were rerun after making the test clock opt-in; both exited 0.
- The same two build commands passed after adding the guarded encoder counter snapshots; both exited 0.
- `git diff --check -- system/loggerd/loggerd.cc system/loggerd/encoderd.cc system/loggerd/encoder/mpp_encoder.cc system/loggerd/encoder/mpp_encoder.h system/loggerd/encoder/mpp_encoder_input.h` — exit 0.
- D-28 source invariant scan — one `mpp_buffer_import` call, no full-resolution selector/fallback/pointer-retry markers, exit 0.
- Added-line placeholder/unowned-marker scan — no matches (ripgrep exit 1, expected).

No behavioral or integration tests were run in this implementation pass; those belong to test-builder and the synthetic board-backed lifecycle run. These compile checks did not start product processes.

## Changed scoped artifact set

The current gate-facing implementation evidence is the aggregate diff `evidence/impl-r10-aggregate.diff`, SHA-256 `d73d9c5c97511e2a2d6e9a596a08abb201e06b31831f9d9c77ecad48b1144ef5`. It contains the exact current workspace changes for the 16 in-scope files below and excludes the unrelated `common/params_pyx.so` and `selfdrive/appbridged/appbridged.py` changes. The prior `evidence/impl-r10.diff` (SHA-256 `ce57a1045d9554d11355579c69833032e42e59f1dbb7401099dbca64da14c542`) is retained as lineage evidence, but is not the gate-facing diff: it predates five test-builder-owned lifecycle/recovery files and its fixture-file hashes no longer describe the current source snapshot.

All 16 paths are first-party; no generated, vendored, or third-party path is included. The five test-builder lifecycle/recovery files are identified by owner below. Their current hashes, along with the other changed source hashes, match the latest test probe `artifacts/tests-r10-probe.json` (SHA-256 `dc3aa851d5ba4a2ef3e2080547fe82f01411c7059207d0009b7d26146d2a01f5`). The unchanged `system/loggerd/loggerd.h` is also hashed as a test/runtime input, but is not a changed path.

| Path | Status | Classification | Slice | SHA-256 | Scope / owner |
|---|---|---|---|---|---|
| `system/loggerd/SConscript` | modified | First-party | M0 | `4ef88831a84c24cde07114ffd4b43f163b4e43ea02260caba400d98c2127173b` | First-party; production build integration for the test-only DMA-BUF publisher target. |
| `system/loggerd/encoder/mpp_encoder.cc` | modified | First-party | M1 | `24537361363ace4fd11485fca4284938f22e9b73fdb13ff84f100dd8368d5477` | First-party; packet and pending-identity tracking, guarded test seam, and the approved no-copy import path. |
| `system/loggerd/encoder/mpp_encoder.h` | modified | First-party | M1 | `253c02a7c10624d0333f3360cb13281d6edb1ede10efe91e705c03ac284e8f43` | First-party; encoder status interface and guarded test seam. |
| `system/loggerd/encoder/mpp_encoder_input.h` | modified | First-party | M0/M1 | `965e011ff0676341996a13974f7c86f87a26bc72441765eefc68f115ee068ce3` | First-party; DMA-BUF input validation contract retained with the no-copy path. |
| `system/loggerd/encoderd.cc` | modified | First-party | M1 | `9e2ef373325dc6eb7deec19cf65deb0a23b3fd3e19edbf27009c93fc135bbd10` | First-party; per-output progress, bounded recovery, drain fairness, and strict authority parsing. |
| `system/loggerd/loggerd.cc` | modified | First-party | M2 | `3a2e621adc50f44f727a12b0d8deb7410a4c502948ae2297069aa40b4ea8f5a9` | First-party; deterministic-KA2 rotation, reports, finalization, and bounded audio queue. |
| `system/loggerd/tests/test_dmabuf_fixture.py` | added | First-party | M0 | `cfc0baf39d00be3871f7e8359083adc86266e29333e3b2dc09cd349e175a7b9c` | First-party; synthetic DMA-heap VisionIPC fixture; earlier test-fixture surface, refreshed by test-builder. |
| `system/loggerd/tests/test_encoderd_recovery.cc` | added | First-party | M1/M3 | `9c7317eedf4f446507702fb8f9b29ccfe4e161a7d0115c46105e1677f60c080f` | First-party; added by build/test-builder for encoder recovery and drain contracts. |
| `system/loggerd/tests/test_encoderd_recovery.py` | added | First-party | M1/M3 | `aa5cddc9ace90969362375873e7d9cfeb225a80ec3cd4ce5bf716ba950f3ffd6` | First-party; added by build/test-builder for encoder recovery and drain contracts. |
| `system/loggerd/tests/test_loggerd.py` | modified | First-party | M3 | `0e6404662d1dad2ff6ba1fe8ff13d41e55f3f30494fe833d41d996de44a013bb` | First-party; test-owner lifecycle wrapper changes, authored by build/test-builder. |
| `system/loggerd/tests/test_loggerd_liveness.py` | added | First-party | M3 | `c024ff8fb62baba3a5601dab6cba5198115a5d18397c3f783b29fc5e98480b61` | First-party; added by build/test-builder for isolated lifecycle and cleanup assertions. |
| `system/loggerd/tests/test_loggerd_rotation.cc` | added | First-party | M2 | `bf185f8d3b9335182d347e010af35b1a20913b15bdeb8ad35a23b19edf1e7962` | First-party; added by build/test-builder for deterministic rotation/report/audio contracts. |
| `system/loggerd/tests/test_loggerd_rotation.py` | added | First-party | M2 | `d40aa6644fc2ccefe952c7310c242c8a950348a2f980b9f3740db458fc50b85f` | First-party; added by build/test-builder for deterministic rotation/report/audio contracts. |
| `system/loggerd/tests/test_mpp_encoder.cc` | modified | First-party | M1 | `4ba25c832faa0c5385d8cd1a5c02bac626099bcaf03635067b2501146004460d` | First-party; test-owner MPP validation and no-copy coverage, authored by build/test-builder. |
| `system/loggerd/tests/test_mpp_encoder.py` | modified | First-party | M1 | `533f179729d841dfe0386d34ebc92a282f0395e867a4f057e4ce623a6b7964b4` | First-party; test-owner MPP test wrapper changes, authored by build/test-builder. |
| `system/loggerd/tests/visionipc_dmabuf_publisher.cc` | added | First-party | M0 | `0994305f86b1c7c1481d3276ee2dd5b5fd81d033847db822a198568941fefad9` | First-party; synthetic four-buffer DMA-heap VisionIPC fixture publisher; refreshed by test-builder. |

No placeholder or unowned follow-up marker was added in the production changes; the final added-line scan found none. The aggregate contains no product or test source edits beyond this listed set.

## Residual risks

- The synthetic four-slot DMA-heap VisionIPC lifecycle passed with the production-built loggerd/encoderd binaries, covering 104 frames, segments 0–2, all six files per segment, and qRoad video/audio decode. The later memfd SHM-to-RGA probe and full SHM loggerd lifecycle also passed: three full-resolution streams were rejected by MPP while qRoad RGA output retained valid video and audio.
- AC-12 remains unproven for a genuine non-KA2 legacy runtime target. The `-URK3588` loggerd guard helper only checks the deterministic-mode guard and is partial; the TICI override aborted before test code because no TICI device-map entry is available on this KA2 host.
- Live V4L2 DMA-BUF export, synchronization fences, camera-buffer reuse, and deployed production MPP behavior remain unqualified.
- The deployed MPP build and post-EOS in-place continuation remain unpinned/unqualified; r10 keeps planned EOS/drain/close/open transitions.
- The test clock and drain seam compile, and their behavioral acceptance is documented in the test-builder evidence.
