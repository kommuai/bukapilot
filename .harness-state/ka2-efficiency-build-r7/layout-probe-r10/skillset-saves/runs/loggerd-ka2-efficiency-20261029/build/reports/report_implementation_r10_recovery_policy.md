# Implementation follow-up: encoderd retry policy extraction

**Run:** `loggerd-ka2-efficiency-20261029`  
**Owner:** build/bob-the-builder  
**Boundary:** build-to-review handback  
**Starting save revision:** 243  
**Source status:** `system/loggerd/encoderd.cc` modified; SHA-256 `a30d52209fbfb79dc8cf3efe1a1bc8c7eb9e76a2dde3db422f0c22684537396e`  
**Scope:** the existing encoderd per-output recovery/watchdog/backoff/quarantine policy only. No test source, manifest, completeness report, or unrelated source was edited.

## Change and behavior preservation

Extracted the due-retry attempt from the `encoder_thread` lambda into the internal RK3588 helper `retry_output_policy`. The production loop now calls that same helper. It accepts the captured due-check timestamp and a clock callback, plus the current segment, allowing deterministic tests to drive the production path without waiting for a retry interval.

The helper preserves the prior order and conditions. The loop still captures one shared `steady_clock::now()` before scanning outputs. A retry begins only when the output is unavailable and due. Retry counters and the test snapshot precede `encoder_open`; `set_segment_num(current_segment)` follows `encoder_open` and precedes the open-state check; the flush setting and successful/failed branches remain in the same order. On success, the recovery counter, test snapshot, qRoad grace timestamp, packet/submission snapshots, health reset, failure-counter resets, and log occur in their prior order. On failed open, it uses the existing per-output scheduler with `context_already_closed=true`, avoiding a second close and retaining the same fresh retry timestamp and backoff. The production caller still gates retries on deterministic mode and keeps them isolated per output.

The only adjacent build-only change wraps the two MPP drain timing constants in `#if defined(RK3588)`. The non-RK compile identified them as unused under `-Werror`; this guard does not change runtime policy. No production selector, fallback, or tuning change was added.

## Internal test seam

`test_encoderd_recovery.cc` includes `encoderd.cc`, so its test translation unit can call the static helper under `RK3588`:

```cpp
template <typename NowFn>
static bool retry_output_policy(
    EncoderOutputRecovery &health, MppEncoder &mpp,
    EncoderOutputRecovery::TimePoint due_now, NowFn &&now,
    int current_segment, bool qroad,
    const char *thread_name, const char *publish_name,
    int &encode_failure_count, int &encode_failure_burst_count);
```

A test can set an output to `Unavailable` with `retry_at <= due_now`, pass controlled values for `due_now` and `now`, then feed a valid DMA-heap frame and verify the published packet uses `current_segment`. The helper returns `false` without side effects when not due, and `true` after a retry attempt, including a failed open. The direct helper is internal, used by production, and does not use a test environment selector. `EncoderOutputRecovery`'s existing controlled-time methods remain available for exact watchdog, retry-backoff, progress-reset, pending-drain, and quarantine assertions.

No behavioral tests were edited or run in this implementation task. Test-builder owns the deterministic assertions and suite rerun. The existing r220/r229 test logs and r231 runtime report/log were preserved. Their probes predate this `encoderd.cc` change and must be refreshed before build submission.

## Compile and formatting checks

Executed results are captured in `evidence/impl-r10-recovery-policy-checks.log`, SHA-256 `480ab422737d188643782a96deb89c62cb569b6951c568cd191c90a96ee50510`.

- `git diff --check -- system/loggerd/encoderd.cc` — exit 0 on the frozen source.
- `scons -Q system/loggerd/encoderd.o` — exit 0, RK3588 production translation unit.
- `scons -Q --ccflags=-DLOGGERD_TEST_BUILD system/loggerd/encoderd.o` — exit 0, RK3588 test-seam translation unit; no test process ran.
- `scons -Q --ccflags=-URK3588 system/loggerd/encoderd.o` — exit 0, HardwarePC/non-RK software guard, with no `__TICI__`; compile-only, not a genuine non-KA2 runtime test.
- `scons -Q --ccflags='-URK3588 -D__TICI__' system/loggerd/encoderd` — exit 0 after guarding the MPP-only constants. This was compile-only; no TICI binary ran and it is not AC-12 evidence.
- The first TICI guard compile exited 2 with `-Wunused-const-variable` for those two MPP-only constants. The `RK3588` guards fixed the compile failure; the successful repeat is in the log.
- `git diff --check` and a path-set check confirm this bounded diff names only `system/loggerd/encoderd.cc`.

No board test, behavioral suite, product runtime smoke, or live camera test was run here. Runtime health must rebuild the RK3588 production binary and rerun its smoke because `encoderd.cc` is an input.

## Bounded implementation artifact

The source-only diff is `evidence/impl-r10-recovery-policy.diff`, SHA-256 `58d33f5a35ab83c658e6fe07afb48db774d52f757be38ae7837b995248a3a441`; it names only `system/loggerd/encoderd.cc`. The final 16-path aggregate change set is intentionally deferred until test-builder finishes its source edits, as directed by build-management. The prior aggregate remains preserved.

The synthetic SHM-to-RGA probe and full SHM loggerd lifecycle passed in the prior test-builder report, but those tests must rerun against this source. AC-12 still lacks a genuine non-KA2 runtime target. Live V4L2 DMA-BUF export, synchronization fences, camera-buffer reuse, and deployed MPP behavior remain unqualified. No placeholder or unowned follow-up marker was added.
