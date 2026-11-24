# Intake grilling: KA2 capture-path robustness architecture review

## Resolved decisions

| ID | Decision | Chosen | Source |
| --- | --- | --- | --- |
| D-1 | Requested code surface | Analyze all files under `system/loggerd`, with runtime optimization scoped to the KA2/RK3588 loggerd and MPP encoderd path plus directly required manager/test integration. | user |
| D-2 | Launch behavior to document | Follow the manager-managed path: loggerd is enabled while started and logging is enabled; encoderd is on-road; `--stream` encoderd is used for non-car mode. Explain the direct executable invocation only as the manager's concrete command and working directory. | codebase |
| D-3 | Regression contract | Add deterministic behavioral coverage for the real loggerd+encoderd path using isolated log/Params/IPC namespaces where supported; assert segment contents, boundaries, encode ordering/indices, and configured audio/front-camera behavior. Preserve existing behavior. | user |
| D-4 | Optimization acceptance | Implement only candidates that preserve those observable behaviors and can be checked against a pinned baseline; keep risky candidates such as enabling MPP zero-copy disabled unless KA2 runtime evidence validates them. | delegated-default |
| D-5 | Analysis deliverable | Inventory every file in `system/loggerd`; prioritize runtime CPU/RSS candidates and identify build/launch simplifications without deleting files absent repository-wide proof they are unused. | user |

| D-17 | Response to unavailable S0 target evidence | Revise the design to pursue only an efficiency scope whose behavior and safety can be validated without the unavailable target evidence. Preserve recording behavior, zero-drop, and quantitative-proof requirements; if no such scope is supportable, return an explicit no-go rather than waiving unknowns or claiming an unmeasured gain. | user |

## Binding follow-up user requirements

| ID | Decision | Chosen | Source |
| --- | --- | --- | --- |
| D-13 | Hard optimization acceptance | No behavioral changes and no stalls or drops are acceptable. Reject a candidate if the controlled baseline or candidate workload records any stall/drop, or if output, frame/index ordering, segment, audio/front-camera, or recovery behavior differs. During the stable post-sync interval, require exact expected-versus-emitted frame IDs/counts and zero drops/recovery events; record per-stream input/output inter-arrival, end-to-end encode latency, and backlog. Capture startup synchronization and first-keyframe filtering separately and require their event sequence/outcomes to match the copy baseline exactly; report any baseline loss. If the hard rule is applied to the entire process lifetime and baseline records a startup loss, no candidate qualifies. Fail if existing watchdog stall thresholds are reached (1 s output silence, 3 s VisionIPC input silence), backlog grows or saturates, or the candidate has a statistically supported p95/p99 latency regression. Compare `rlog.zst`/`qlog.zst` event contents exactly after canonicalizing only source-proven run-varying values: `Event.logMonoTime` (monotonic since boot), `EncodeData.unixTimestampNanos` (UTC from `timespec_get`), `InitData.wallTimeNanos` (UTC from `nanos_since_epoch`), and `df -h` numeric disk-usage fields captured in `InitData.commands`. Validate clock domains, per-run ranges, per-producer/stream ordering, and source semantics; do not impose global cross-service timestamp ordering. Preserve deterministic input `timestampSof`/`timestampEof` exactly. For `df -h`, preserve command, schema, filesystem/device and mount rows, and validate numeric usage fields. Compare sentinels, indexes, other metadata, and segment/file boundaries exactly. Require the timestamp-derived route directory to match its format and the run's UTC window; compare segment ordinals and file sets exactly. Seed identical isolated Params inputs for both arms. Audio samples must decode identically. Camera media must decode to the same ordered frame identities and pixels with matching timestamps, segments, codec/container metadata, and decodability. Record encoded media hashes and byte-size deltas. If repeated baseline runs prove compressed media is nondeterministic, hash differences alone do not fail when decoded content and metadata match; any decoded difference fails. | user hard rule; Admiral interpretation grounded in source, 2026-10-29 |
| D-14 | Optimization objective | Reduce CPU and memory use. Also make one behavior-preserving LOC-minimization pass over the scoped KA2 loggerd/encoderd runtime path (shared runtime code and MPP path) plus affected regression tests; report per-file before/after counts separately. Exclude unrelated utilities and unsupported platform-specific implementations. Do not force a shortening when it harms clarity, increases risk, changes behavior, or regresses measured CPU/memory. Stop when no further safe reduction is identified. LOC reduction is not a runtime-efficiency proof. | user request; Admiral operationalization, 2026-10-29 |
| D-15 | Quantitative proof | Compare a pinned current-copy baseline with each candidate on identical KA2 binaries/workloads using interleaved paired samples, warmups, medians/tails, confidence intervals, CPU, process RSS, and available DMA/device-memory metrics. Claim gains only when the confidence interval excludes no change and zero-stall/zero-drop and output-equivalence gates pass. | user, 2026-10-29 |
| D-16 | Whole-process zero-loss refinement | Every submitted synthetic input frame ID must appear exactly once in each configured output stream from the first frame through teardown. Startup sync, first-keyframe filtering, segment rollover, recovery, and shutdown do not waive missing frames. A baseline that loses any submitted frame fails the hard rule and cannot qualify a candidate; identify and correct the loss mechanism before performance comparison. | user hard rule, clarified 2026-10-30 |

## Deferred decisions

| ID | Decision | Why deferral is safe | Owner | Reopen phase |
| --- | --- | --- | --- | --- |
| D-6 | Bounded testing and performance comparison on this host | The user authorized bounded synthetic correctness tests while this production KA2 is idle. Defer full performance comparison until whole-process no-loss and candidate correctness gates pass; recheck that no competing workload has started and honor the isolation and sample/output ceilings. | user | build/performance verification |

## Rejected options

| ID | Option | Rationale |
| --- | --- | --- |
| D-7 | Reduce bitrate, frame rate, camera set, segment contents, or audio to save resources | These change observable recording behavior and are outside the requested no-regression contract. |
| D-8 | Turn on MPP zero-copy by default before KA2 validation | Existing source documents zero-length HEVC output on some KA2 MPP builds; CPU savings do not justify an unverified behavior regression. |
| D-9 | Delete non-KA2 loggerd implementations solely because this device is KA2 | The current request is to analyze the full directory, and these files may be shared by other build targets; deletion needs whole-repository reachability evidence and separate scope. |

## Non-goals and YAGNI deferrals

| ID | Non-goal | Reopen trigger | Owner |
| --- | --- | --- | --- |
| D-10 | Change log format, route/segment naming, uploader/deleter policy, or recording retention | Reopen if a required efficiency target cannot be reached while preserving current behavior and the user explicitly accepts a behavior change. | user |
| D-11 | Optimize unrelated loggerd utilities without evidence they contribute materially to KA2 CPU or memory use | Reopen if profiling attributes meaningful resource cost to a utility in the KA2 operating path. | build lead |
| D-12 | Run cameras, I2C probes, or real on-road recording as part of this work | Reopen only with a confirmed non-production KA2 target and a bounded test plan. | user |

## User update — Intake revision 50 (2026-10-30)

The user clarified that the KA2 is production hardware but is currently idle, and explicitly authorized running the tests on it. This updates D-6: bounded synthetic loggerd/encoderd/MPP tests for this task may run while the device is idle, using isolated temporary log, Params, and IPC roots. Recheck for competing camera, loggerd, encoderd, sensor, or MPP work before each run and stop if any starts. This does not authorize on-road capture, live camera/sensor input, I2C probes, or writes to production log/Params roots. Performance comparisons remain subject to the existing correctness gates, interleaved-pair protocol, and time/output ceilings.

## User update — Intake revision 108 (2026-10-30)

| ID | Decision | Chosen | Source |
| --- | --- | --- | --- |
| D-18 | Current objective | Replace the blocked efficiency-only objective with a whole-path KA2 robustness architecture review and a concrete rearchitecture recommendation. Inspect for flaws across camera/VisionIPC buffers, encoderd/MPP, encoded-event transport, loggerd queues and rotation, output writing, and process startup/shutdown/recovery. | user |
| D-19 | Rearchitecture authority | Rearchitecture may be proposed throughout the path. This turn requests an evidence-backed recommendation and regression-proof plan; it does not request product-source edits or deployment. Isolated/synthetic experiments are allowed where the target and test preconditions are safe. | user |
| D-20 | Retained correctness contract | Keep the no-regression recording contract: every accepted input frame is accounted for exactly once in each applicable output, with valid media and metadata through startup, rotation, recovery, and teardown. Keep format, stream selection, timing, and segment behavior stable unless a change is explicitly justified and its compatibility proof is specified. | user direction, consistent with D-3/D-13/D-16 |
| D-21 | Unknown target contracts | Do not treat missing exact-target MPP identity/API, EOS/IDR behavior, VisionIPC retention/lifetime, packet bounds, or RSS/DMA headroom as facts. A recommendation may name evidence and qualification gates needed to resolve them. | retained from R6; not waived |

D-18 supersedes D-17 only as to the current objective: the task is now robustness architecture, not a claim of measured efficiency improvement. D-13, D-15, D-16 and the prohibition on waiving unknown behavior contracts remain binding wherever applicable; no performance gain is claimed. The old D-17 intake record is preserved at `intake/artifacts/report_grilling_d17_before_d18.md`.


## User update — Intake revision 110 (2026-10-30)

| ID | Decision | Chosen | Source |
| --- | --- | --- | --- |
| D-22 | Current objective | Resume the KA2 encoder-efficiency objective. Focus the work on measurable CPU and memory reduction in the encoder path; do not pursue the broad capture-path reliability rearchitecture from D-18 as the main deliverable. | user, 2026-10-30 |
| D-23 | Best-effort semantics | The user accepts the existing best-effort recording approach. D-13 and D-16 are superseded for this revision only as strict zero-drop gates. Continue to record and report input/output IDs, loss counters, media decodability, timestamps, and segment behavior; do not claim lossless behavior. Preserve normal encoding settings and avoid introducing material regressions under the matched workload. | user clarification, 2026-10-30 |
| D-24 | Optimization evidence | Make a practical attempt and report measured results where the authorized idle KA2 synthetic path permits. Exact deployed MPP contracts remain unknown; this uncertainty no longer blocks a bounded experiment, but must be reported and no generic MPP guarantee may be inferred. Keep tests isolated from production logs/Params/IPC and do not use live camera/sensor input or on-road recording. | user clarification plus prior D-6 authorization |
| D-25 | Scope and change authority | Prepare a scoped design/build attempt for the KA2 encoder path. Avoid a general camera/VisionIPC/Manager rearchitecture and avoid unrelated LOC reduction. A candidate may change internal copy/queue behavior only when its best-effort tradeoff is measured and disclosed; preserve codec, resolution, frame rate, camera/audio selection, and route/file compatibility unless the user separately changes them. | user, 2026-10-30 |

D-22 supersedes D-18 as the current objective. D-23 supersedes the zero-drop portions of D-13/D-16 for this revision; it does not waive measurement, output-quality, format, or compatibility reporting. D-24 reopens a bounded efficiency experiment despite the prior S0 no-go, while preserving the target/API caveats. D-25 supersedes D-19's no-source-edit scope: implementation may proceed through the existing design/build gates for a concrete KA2 encoder-efficiency candidate. The whole-path robustness report remains historical context, not the active implementation architecture.


## User update — Intake revision 149 (2026-10-31)

| ID | Decision | Chosen | Source |
| --- | --- | --- | --- |
| D-26 | Default input path | Enable MPP DMA-BUF zero-copy by default for eligible full-resolution encoder streams. Keep `ENCODER_ZERO_COPY=0` as the explicit copy-path rollback. Downscaled streams such as qRoad continue using their RGA resize path. | user, 2026-10-31, after reviewing the measured qRoad tail-latency report |
| D-27 | Refactor scope | Recursively inspect and simplify the KA2 MPP encoder implementation, its immediate encoderd lifecycle seam, and affected regression coverage. Reduce duplicated and unnecessary code substantially while preserving encoded output contracts, buffer lifetime/fallback, frame accounting, segment rotation/drain, and the explicit copy override. Keep unrelated loggerd, camera/VisionIPC, storage, Manager, and other-platform code out of scope. | user, 2026-10-31 |

D-26 supersedes D-8 and the r7 copy-default requirement for the current revision. It changes the default only for eligible full-resolution MPP inputs; it does not enable zero-copy for downscaled streams. The previously measured qRoad p95 penalty remains a known performance tradeoff (+0.29 ms in the matched 10-launch set); this decision authorizes the default change with that evidence disclosed. It does not qualify the target image generically or waive output validity, packet, segment, or lifetime checks. D-27 supersedes D-25 only for the listed encoder implementation/refactor scope; public interfaces, codec settings, stream selection, timestamps, and recording formats remain fixed.

## User update — Intake revision 152 (2026-10-31)

| ID | Decision | Chosen | Source |
|---|---|---|---|
| D-28 | Full-resolution MPP input policy | Remove every full-resolution copy route, the `ENCODER_ZERO_COPY=0` switch, and the pointer-attached retry. A frame must pass DMA-BUF validation and one MPP import attempt; otherwise return an encode error and let existing encoder failure handling account for it. Keep qRoad's RGA resize as its normal downscaled path. | user, 2026-10-31 |

D-28 supersedes D-26's copy rollback and D-27's fallback-preservation clauses. It does not change codecs, output formats, qRoad resizing, segment draining, or encoderd's existing recovery policy. The user accepts that unsupported input fds or failed MPP imports are dropped rather than copied.

## User update — Intake revision 153 (2026-10-31)

| ID | Decision | Chosen | Source |
|---|---|---|---|
| D-29 | Production liveness after MPP input rejection | Investigate and fix the segment-rotation stall exposed by the no-copy DMA-BUF policy. The fix may extend into loggerd/encoderd lifecycle handling and regression fixtures. Keep full-resolution MPP copy fallback removed; ensure a failed encoder cannot hold segment rotation indefinitely and that healthy streams continue through the failure. Validate with isolated synthetic DMA-BUF and invalid-input tests. | user, 2026-10-31 |

D-29 broadens D-27's implementation scope from the encoder seam to the directly coupled segment-rotation lifecycle because the previous synthetic integration run exposed that dependency. D-28 remains in force: rejected or unimportable full-resolution inputs are dropped, not copied. No live camera/sensor capture, production Params/log writes, commit, or deployment is authorized by this request.

