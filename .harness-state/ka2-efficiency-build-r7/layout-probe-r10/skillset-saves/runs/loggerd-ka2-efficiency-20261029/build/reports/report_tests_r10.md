# Test report: KA2 loggerd rotation liveness r10

## Latest result

**PASS on the KA2 board.** The full grouped serial suite completed with **14 passed, 0 skipped in 301.56 seconds**. The executed runner output is `evidence/tests-r10-ac6-full-r267.log` (SHA-256 `1a331173847b4e933a3be4d38f4e0a4e18e027514705de8d88e9a77a89cec328`). The run includes the AC-6 all-output-reopen assertion and the non-RK3588/HardwarePC software-branch lifecycle. Fresh postflight at 2026-10-31 13:36:13 UTC found no `camerad`, `encoderd`, or `loggerd` process and no users of the enumerated camera, media, subdevice, MPP, or RGA nodes. The focused drain-timeout selector also passed: **1 passed in 29.08 seconds**.

This is a KA2/RK3588 aarch64 hardware run. The HardwarePC test used a dedicated non-RK3588 software configuration on that actual KA2 host; it does not claim a non-KA2 silicon run.

## Executed command

The focused AC-6 drain-timeout case ran first and passed with `1 passed in 29.08 seconds`:

```sh
TMPDIR=/data/openpilot/.harness-state/ka2-efficiency-build-r7/tmp-ac6-focused-r266 \
  /usr/local/venv/bin/python3 -m pytest -s -v -n0 -x \
  --basetemp=/data/openpilot/.harness-state/ka2-efficiency-build-r7/ac6-focused-r266 \
  'system/loggerd/tests/test_loggerd_liveness.py::test_ka2_output_fault_preserves_qroad[drain_timeout]'
```

The full serial rerun then used:

```sh
TMPDIR=/data/openpilot/.harness-state/ka2-efficiency-build-r7/tmp-ac6-full-r267 \
  /usr/local/venv/bin/python3 -m pytest -s -v -n0 -x \
  --basetemp=/data/openpilot/.harness-state/ka2-efficiency-build-r7/full-suite-ac6-r267 \
  system/loggerd/tests/test_mpp_encoder.py \
  system/loggerd/tests/test_loggerd_rotation.py \
  system/loggerd/tests/test_encoderd_recovery.py \
  system/loggerd/tests/test_dmabuf_fixture.py \
  system/loggerd/tests/test_loggerd_liveness.py \
  system/loggerd/tests/test_loggerd.py::TestLoggerd::test_rotation \
  system/loggerd/tests/test_loggerd.py::TestLoggerd::test_ka2_deterministic_rotation_alignment
```

The run was explicitly serial (`-n0`). Pytest basetemp and `TMPDIR` were both placed under `/data`, not the nearly full `/tmp` tmpfs. The lifecycle wrappers built isolated `LOGGERD_TEST_BUILD` binaries and synthetic DMA-heap or memfd fixtures; they used isolated log, Params, and process roots.

## Coverage and results

- **AC-1 through AC-3, loggerd rotation and audio:** fake-clock contract tests cover the rotation deadline, per-epoch readiness, startup without a report, report schema, writer/audio/queue finalization, queue capacity, audio expiry, and the expiry/commit tie. These are deterministic helper tests; lifecycle subprocesses use real time.
- **AC-4, packet progress and pending identities:** deterministic qRoad watchdog assertions check its three-second startup grace, no expiration through the one-second stall boundary, rotation-barrier suppression, and exact expiry. The MPP-backed retry test keeps frame ID 41 attached while pending, quarantines and retires it once, proves close does not retire it again, then publishes frame IDs 42 and 43 on authoritative segment 7 after retry. The old ID is not shifted or reassigned.
- **AC-5, local errors and retry policy:** assertions verify nine errors remain below the local threshold, the tenth reaches it without affecting a healthy sibling, and the retry schedule is 1/2/4/8/16/30/30 seconds. Each retry is rejected one nanosecond before due and accepted at its deadline. Successful reopen waits for a full second of valid packet progress before clearing backoff. The direct invalid-FD test also verifies rejection before MPP submission and without a pending identity. The deterministic helper exercises the policy; this does not emulate a live V4L2 exporter failing continuously.
- **AC-6, shared drain deadline and sibling isolation:** the test-only pending road identity remains bound through the shared 900 ms drain deadline and is retired exactly once when only road is quarantined. qRoad remains active, completes one planned segment reopen, and records zero failure-recovery reopens and retries. To distinguish the all-output fallback from qRoad's per-output counters, a `LOGGERD_TEST_BUILD`-only, `LOGGERD_TEST=1`-guarded counter increments at the actual `reopen_all_encoders()` helper after the first successfully encoded frame in each encoder thread. At thread shutdown the drain-timeout lifecycle emits and asserts exactly one zero-count record for each expected thread: road, driver, and wide-road. Initial startup construction is outside the counted interval.
- **DMA-BUF validation, qRoad, and synthetic lifecycle:** direct invalid-FD and memfd-backed full-resolution inputs are rejected before MPP submission. A real memfd RGA probe accepts the qRoad resize path, and the SHM lifecycle publishes qRoad video/audio while correctly reporting the full-resolution streams as missing. The valid four-slot DMA-heap lifecycle completed segments 0–2 with all expected files, matching frame identities and decoded pixels, qRoad video/audio, healthy reports, and zero known drops. Reject, silent, and drain-timeout fault lifecycles retain qRoad media and audio and report no unrelated loggerd drops.
- **AC-12 legacy software configurations:** RK3588 with `ENCODERD_DETERMINISTIC_DISABLE=1` passed two legacy rotations without deterministic reports, watchdog/retry logs, or counter snapshots. The non-RK3588/HardwarePC software-branch lifecycle also passed on the KA2 host. Dedicated loggerd/encoderd test binaries were built with `-URK3588`, without `__TICI__`, without changing `/KA2` or `OPENPILOT_DEVICE`, and with the required FFmpeg implementation linked. The emitted route reports `DeviceType.pc`; both daemons rotate through segments 0–2, all expected writers and frame indexes are present, full-resolution video pixels and qRoad video/audio decode, and legacy mode emits no deterministic report, deadline, watchdog, retry, or MPP recovery counters. A separate non-RK3588 loggerd guard helper verifies no startup/runtime report and immediate all-ready rotation. This verifies the non-RK3588 software configuration on KA2 silicon, not an independent PC or other non-KA2 device. The earlier TICI identity-override attempt remains preserved as a failed configuration probe; it aborted at actual-board detection before test code and is not treated as evidence.
- **Existing rotation tests:** `TestLoggerd::test_rotation` and `TestLoggerd::test_ka2_deterministic_rotation_alignment` passed in the same full serial run.

## Evidence and hashes

| Evidence | Path | SHA-256 |
|---|---|---|
| Focused drain-timeout test r266 | `evidence/tests-r10-ac6-focused-r266.log` | `41d6d2d24ed85b9c8243529fef21bdea6b2f356ae8882b721e7454415ef050d8` |
| Focused preflight r266 | `evidence/tests-r10-ac6-focused-preflight-r266.log` | `bfab475467ab9085f9a6ade360961a60f40feabc48a7de5c56558580c8694b51` |
| Focused postflight r266 | `evidence/tests-r10-ac6-focused-postflight-r266.log` | `37d9305b12df30221a680ea9f41115bd1afa3c411a78c13263d72579159e777d` |
| Full grouped serial suite r267 | `evidence/tests-r10-ac6-full-r267.log` | `1a331173847b4e933a3be4d38f4e0a4e18e027514705de8d88e9a77a89cec328` |
| Full-suite preflight r267 | `evidence/tests-r10-ac6-full-preflight-r267.log` | `0d81284ff9d934b998b1b04a6aa6e2c21da1cbcab62f0786903919539f3b870c` |
| Full-suite postflight r267 | `evidence/tests-r10-ac6-full-postflight-r267.log` | `65c843b7eaf7e3aef749e79df8740845ecacc55855f23a95af51c862f7e52fde` |
| Prior full grouped suite r260, retained | `evidence/tests-r10-full-suite-r260.log` | `dec86fd51f86550a55a872ed600fe94ea7030659b55bff9e3be0189b760a7614` |
| Prior full-suite preflight r260, retained | `evidence/tests-r10-full-preflight-r260.log` | `986a39aeac6c120ab798fdcab799dd8cf9337ef44c176e93e7b19f86939b7cc8` |
| Prior full-suite postflight r260, retained | `evidence/tests-r10-full-postflight-r260.log` | `bd874ae7ef779cf15da864e0b726994a8871b5038960835c657559621dc8f9bf` |
| Earlier full suite r229, retained | `evidence/tests-r10-run-r229.log` | `3192bac0c9edfff73d584fb81047b585939d0a0a2c74f89756be06603d983c37` |
| Earlier TICI mismatch and gap-closure evidence, retained | `evidence/tests-r10-gap-closure.log` | `952c12929edc7fb54f8cfb9f86e64a22e3ade001aa633bb547f30bec1dfe42c9` |
| Original full suite r220, retained | `evidence/tests-r10-run-r220.log` | `fee537cafe812163ed24a2467d0a138f144cfda90c6f06634b6f1f112792a775` |

Tested source hashes are bound individually in `artifacts/tests-r10-probe.json`. Notable final inputs include `system/loggerd/encoderd.cc` `fde3d8cb1adfd5308279df4c3c37989ebed7a63127a1f51ed1cbdb48e7eab8d9`, `system/loggerd/encoder/ffmpeg_encoder.cc` `7124dee947f6eb409e5fc0e720182f077554d470871dc5c400b6430abc1ea8de`, `system/loggerd/encoder/ffmpeg_encoder.h` `ca58013144a178daf80b1e483dcfd56a231d3af72129aec5a7052e8df269f8d8`, `system/loggerd/tests/test_encoderd_recovery.cc` `38872a2c9d5ffd9a8a4d5fa4cf09941c3e768b8001cd8414bc50416feaaf88fe`, and `system/loggerd/tests/test_loggerd_liveness.py` `f4911703b8d9a6e500336106b855a1182aabd135078c1c8ca58aadb3875efea9`. The test-only `encoderd.cc` instrumentation changed an input bound by the earlier runtime probe; health-check must refresh that runtime evidence before build submission.

## Remaining qualification limits

The DMA-BUF lifecycle source is synthetic. It does not qualify live V4L2 DMA-BUF export, camera fences or buffer reuse under capture, or post-EOS behavior on the deployed production MPP build. The HardwarePC configuration ran on the KA2 board and does not qualify other silicon or physical camera hardware. The suite also does not run a long-duration live capture with a persistently failing V4L2 descriptor; retry/backoff behavior is covered through deterministic policy assertions and isolated encoder fault seams.
