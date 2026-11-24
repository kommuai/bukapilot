# Intake brief: loggerd and encoderd KA2 efficiency

## User goal
Analyze all of `system/loggerd`; document how loggerd and encoderd are launched and what they do; add behavioral regression coverage; and identify and validate KA2-specific ways to reduce CPU and memory without behavior changes. The user requires no stalls or drops and requests fewer code lines where the same behavior and runtime efficiency are retained, plus quantitative proof of any claimed gain.

## Tier and route
- Preamble tier: 2 — repository-wide analysis of a multi-file runtime surface, regression-test authoring, and possible measured implementation changes.
- Route: Admiral intake -> design/commander -> design gate -> build/test/performance evidence -> review as required by gates.
- Platform: KA2 / RK3588, confirmed by the user; implementation path in `system/loggerd/SConscript` includes `mpp_encoder.cc`, `rockchip_mpp`, and `rga`.

## Scope
- Inventory all files under `system/loggerd` and describe runtime vs build/test/maintenance responsibilities.
- Trace manager gating and exact executable commands for `loggerd`, regular `encoderd`, and `encoderd --stream`.
- Analyze the KA2 MPP encoding, loggerd message queues, segment rotation, file writing, memory-pressure behavior, and directly related tests.
- Add a behavioral regression test around the real managed loggerd+encoderd path, using isolated paths/namespaces and synthetic input if the test harness supports it.
- Measure resource impact against the current revision before and after any accepted optimization, only on a confirmed safe target/workload.

## Behavior invariants
- Manager run predicates and process shutdown ordering remain the same.
- Keep current route/segment file set, log schema, frame/encode indexes and order, rotation synchronization, stream behavior, RecordFront, and RecordAudio semantics.
- Preserve recovery behavior for encoder stalls/reconnects, malformed packets, and bounded queues.
- Reject any optimization that introduces a stall or drop, or changes any observable output or recovery behavior. Controlled baseline and candidate workloads must record zero stalls and drops to qualify for acceptance.
- In the stable post-sync interval, compare exact expected and emitted frame IDs/counts, require zero drop/recovery events, record per-stream input/output inter-arrival, encode latency, and backlog, and fail if the existing 1 s output or 3 s VisionIPC stall watchdog is reached, backlog grows/saturates, or p95/p99 latency statistically regresses.
- Capture startup synchronization and first-keyframe filtering separately. D-16 applies the zero-loss rule to the entire process lifetime: every submitted input frame ID must appear exactly once per configured stream, and any startup loss disqualifies the baseline and candidate.
- Require exact `rlog.zst`/`qlog.zst` event contents after canonicalizing only source-proven run-varying values: `Event.logMonoTime` (monotonic since boot), `EncodeData.unixTimestampNanos` (UTC from `timespec_get`), `InitData.wallTimeNanos` (UTC from `nanos_since_epoch`), and `df -h` numeric disk-usage fields captured in InitData. Validate clock domains, per-run ranges, per-producer/stream ordering, and source semantics; do not impose global cross-service timestamp ordering. Compare deterministic input `timestampSof`/`timestampEof`, sentinels, indexes, and other metadata exactly. For `df -h`, preserve the command, schema, filesystem/device and mount rows, and validate numeric usage fields. Require timestamp-derived route names to match their format and each run's UTC window; compare segment ordinals and file sets exactly. Seed identical isolated Params inputs for both arms. Audio samples must decode identically. Camera media must decode to identical ordered frame identities/pixels and preserve frame timestamps, segments, codec/container metadata, and decodability. Record media hashes and byte sizes; if repeated baselines prove encoded output nondeterministic, hash-only variance is not a failure when decoded content and metadata match.
- Do not enable optional MPP zero-copy by default without runtime proof that encoded HEVC remains non-empty and behaviorally equivalent on the deployed build.
- A code-size reduction alone is not a runtime efficiency gain; report line-count changes separately from measured CPU and memory changes.
- Make one behavior-preserving LOC-minimization pass over the scoped KA2 loggerd/encoderd runtime path (shared runtime code and MPP path) plus affected regression tests, with per-file before/after counts. Exclude unrelated utilities and unsupported platform-specific implementations. Keep a shortening only when it preserves clarity, behavior, and measured CPU/memory; do not force one when it increases risk or regresses any of those. Stop when no further safe reduction is identified.

## Acceptance
- Explain launch entry point, conditions, command line, working directory, and producer/consumer behavior.
- New test fails on a targeted behavioral regression and passes on the unmodified baseline.
- Run focused behavioral tests where safe; state all unavailable checks explicitly.
- For performance claims, pin baseline/workload/environment, capture CPU and peak RSS with repeatable samples and variance, and report only supported quantitative results.
- Quantitative gains require interleaved paired samples, warmups, median and tail percentiles, confidence intervals, and separate process-RSS and available DMA/device-memory accounting. Report a gain only when the confidence interval excludes no change and all behavior/stall/drop gates pass.
- No unrelated source edits, dependency installs, commits, device camera runs, I2C access, or writes to production log/Params locations.

## Known constraints
- Existing tests already include synthetic VisionIPC input, but current `test_encoder.py` is marked TICI and imports `parameterized`; the broad loggerd unit/integration suite can be environment-limited.
- `LOG_ROOT`, `PARAMS_ROOT`, and `OPENPILOT_PREFIX` provide isolation hooks, subject to verification of test behavior.
- The host is production KA2 hardware and currently idle, per the user. Preflight observed the `/KA2` marker, Linux aarch64, `/dev/mpp_service`, and no running loggerd/encoderd/camerad process or open MPP device handle. The user explicitly authorized bounded synthetic tests on this idle unit; keep logs, Params, and IPC under temporary isolated roots and stop if competing work appears.
- Harness hooks are missing; runtime readiness reports Python 3.12.3 below >=3.13. Deterministic validators are available; the run can continue with these limitations recorded. MCP inventory is blank/undiscovered and not needed for this local task.

## D-6 update
- The user authorized running bounded tests on the currently idle production KA2. This permits only the synthetic, isolated workloads in the approved plan; no on-road capture, real camera/sensor input, I2C, or production log/Params writes. Recheck the device is idle before each run and stop if competing work appears. Resource measurements remain gated on zero-drop behavior, candidate correctness, and the approved sample/output ceilings.

## Current request — Intake revision 153 (2026-10-31)

The user asks to investigate and fix the segment-rotation stall caused by full-resolution MPP input rejection, making the path production ready. D-28's no-copy policy remains mandatory. Scope now includes directly coupled loggerd/encoderd rotation and recovery behavior plus isolated synthetic test coverage. Acceptance requires rotation to progress within a bounded interval when an encoder emits no packets, healthy streams to continue, and no copy path to be reintroduced. Use synthetic DMA-BUF-backed inputs for the healthy end-to-end case and invalid shared-memory input for the failure case; do not use live cameras/sensors or production log/Params paths.
