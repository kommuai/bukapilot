---
submission_id: loggerd-ka2-efficiency-20261029-build-copy-lifecycle-r1
revision: r1
status: implementation-and-focused-test-evidence
---

# Copy-mode no-drop correction: implementation evidence

## Outcome

The resumed worktree candidate builds and passes one bounded synthetic KA2 exact-frame regression. The run submitted IDs 1..148 on each camera stream and emitted each ID exactly once in all four encoded outputs. This is a single correctness experiment, not a performance claim or a release approval.

## Scoped implementation

The candidate worktree covers the startup and rotation loss path in `system/loggerd/loggerd.cc`, `system/loggerd/encoderd.cc`, the RK3588 MPP encoder, and the focused test. This resumed implementation pass added explicit failure bounds to the candidate's pending-packet drains:

- `MppEncoder::flush_pending_packets(timeout_ms)` polls only until its deadline. `encoder_close()` makes one 500 ms attempt; if packets remain, it logs their frame-ID range and follows the existing close cleanup path, which records the pending discard.
- A normal segment transition shares one 900 ms deadline across its encoders. If a drain fails, encoderd does not commit that segment; it exits the process and uses the explicit pending-discard cleanup path. The exact-ID test must fail if that path loses a submitted frame.
- The test's exact input/output ID-set assertion and first-keyframe requirement remain intact. No non-keyframe is accepted as the recording start.

`git diff --check` passed. The changed loggerd and encoderd binaries were rebuilt with `scons -Q system/loggerd/loggerd system/loggerd/encoderd` (exit 0).

## Runtime evidence

Command:

```text
timeout --signal=INT --kill-after=5s 60s pytest system/loggerd/tests/test_loggerd.py::TestLoggerd::test_ka2_deterministic_rotation_alignment -q --basetemp=skillset-saves/runs/loggerd-ka2-efficiency-20261029/build/evidence/loggerd-copy-runtime-r1-data
```

The exact test exited 0; pytest reported `1 passed in 14.31s` (9.33 s test call). The hard timeout was not reached. The run used temporary `LOG_ROOT` and `PARAMS_ROOT` beneath the test's run-local temporary directory, synthetic VisionIPC camera frames, and synthetic audio. It generated 143,092 bytes, below the 768 MiB cap.

The parsed rlog/qlog evidence shows:

| Stream | Submitted | Emitted | Missing / duplicates | Output segment counts |
|---|---:|---:|---|---|
| road | 148 (IDs 1..148) | 148 | none / none | 100 / 48 |
| wideRoad | 148 (IDs 1..148) | 148 | none / none | 100 / 48 |
| driver | 148 (IDs 1..148) | 148 | none / none | 100 / 48 |
| qRoad | — | 148 (IDs 1..148) | none / none | 100 / 48 |

The run also recorded 148 raw audio events. The first output ID had flags value 8 in all four streams, and IDs 1 and 2 were present. The test retained its segment/sentinel checks. Raw segment files are preserved under `build/evidence/loggerd-copy-runtime-r1-data/`.

Fresh preflight at 2026-10-30 06:24 UTC found the aarch64 KA2 marker and MPP device, no existing loggerd/encoderd/camerad process or MPP holder, 2.03 GB free on `/data`, 25.5% memory used, and a maximum thermal reading of 48.1 C. Postflight again found no loggerd, encoderd, camerad, or MPP holder.

## Evidence files

- `build/evidence/loggerd-copy-build.log`
- `build/evidence/loggerd-copy-diff-check.log`
- `build/evidence/loggerd-copy-preflight-r1.log`
- `build/evidence/loggerd-copy-runtime-regression.log`
- `build/evidence/loggerd-copy-runtime-frame-summary.json`
- `build/evidence/loggerd-copy-runtime-r1-data/` (raw rlog/qlog and media output)

## Current artifact hashes

These hashes describe the worktree after the focused run; the source files were already modified when the run resumed.

| Path | SHA-256 |
|---|---|
| `system/loggerd/encoder/mpp_encoder.cc` | `652fd1843f876872a2d4aa252c6498df2c709c9175fe7b94df3110cfdf18a405` |
| `system/loggerd/encoder/mpp_encoder.h` | `cd5369b19e5113c917a2b4ffd3244aa56112eee31aa0051a7ef64b9134c7cc47` |
| `system/loggerd/encoderd.cc` | `37e72f8678e0585cc26dc7b15301254166a8d7279c0a1a983ec56791f0a2d72f` |
| `system/loggerd/loggerd.cc` | `19e7eed8658f2d5a8141dc770bfc17ee9d11c3754323c43b12aaf643ad83e7e0` |
| `system/loggerd/tests/test_loggerd.py` | `1ee228dc484f4af8724e09ca8a73f3d0790279d4d20a86c9c7edcb09016501b9` |
| `system/loggerd/loggerd` (built target) | `432820cfec6730c7021e655f0bea94b474cadd4c2460ae0818c2e6ba8b0e8048` |
| `system/loggerd/encoderd` (built target) | `d0629b769acacc73011ba56b9f564822f46a45ebbf34156f4560f2850c1b422c` |

## Open risks and limits

- This was one synthetic run. It does not establish repeated-run reliability or full D-13 decoded-media, audio-sample, metadata, and expected per-ID segment equivalence.
- The test stops encoderd before loggerd. It does not prove the real manager's concurrent SIGINT order or the five-second managed shutdown drain; the design package already records that Major gap.
- The run exercises the deployed MPP path successfully for this workload, but the synchronous `encode_put_frame(EOS)` call itself is not preempted by the polling deadline. Its broader behavior and worst-case latency remain unproven.
- No CPU/RSS/DMA measurement or efficiency benchmark was run.
- Design r5 remains UNVERIFIED, the cross-stage verdict remains ESCALATE, and external save-path eligibility remains unresolved. This implementation/test result does not approve either gate and must not advance to performance work or `build-to-review`.
- The unrelated existing `selfdrive/appbridged/appbridged.py` edit and generated `common/params_pyx.so` change were preserved and are outside this report's scope.

## Next action

Return these implementation and focused-test results to Admiral. Resolve the open design and cross-stage decisions before any performance work or build-to-review submission.

## Additional target and artifact evidence

The loaded MPP object was `/usr/lib/aarch64-linux-gnu/librockchip_mpp.so.0` (SHA-256 `ea762b7da71b63a73cdf14d053d84b091654aee5e5c4f26ceb1449b8fb8d92f9`). Its exported `get_mpp_version()` returned `unknown mpp version for missing VCS info`. The exact synthetic run completed EOS drains successfully, but no version-matched source or package provenance is available to establish broader EOS or timeout semantics. The synchronous `encode_put_frame(EOS)` call itself remains outside the polling deadline.

The per-file hashes for both segment qlog/rlog and media outputs are listed in `build/evidence/loggerd-copy-runtime-r1-raw-hashes.txt`.
