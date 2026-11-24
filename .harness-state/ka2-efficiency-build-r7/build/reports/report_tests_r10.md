# Test report: KA2 loggerd rotation liveness r10

## Latest result

**PASS on KA2.** The updated serial grouped suite completed with **13 passed in 258.53 seconds and no skips**. Its runner output is `evidence/tests-r10-run-r229.log` (SHA-256 `3192bac0c9edfff73d584fb81047b585939d0a0a2c74f89756be06603d983c37`). This rerun includes the SHM-to-RGA lifecycle test and the compile-time loggerd guard helper. The earlier revision-220 suite log remains preserved with its original 11-pass result and was not overwritten.

The suite ran on the KA2/RK3588 aarch64 board using test-build binaries and isolated temporary `LOG_ROOT`, `PARAMS_ROOT`, and `OPENPILOT_PREFIX` values. Tests seeded only isolated `RecordFront` and `RecordAudio` Params. No manager-owned process was stopped or modified.

## Executed command

```sh
python3 -m pytest -s -v -n0 -x \
  system/loggerd/tests/test_mpp_encoder.py \
  system/loggerd/tests/test_loggerd_rotation.py \
  system/loggerd/tests/test_encoderd_recovery.py \
  system/loggerd/tests/test_dmabuf_fixture.py \
  system/loggerd/tests/test_loggerd_liveness.py \
  system/loggerd/tests/test_loggerd.py::TestLoggerd::test_rotation \
  system/loggerd/tests/test_loggerd.py::TestLoggerd::test_ka2_deterministic_rotation_alignment
```

The full suite was run serially (`-n0`). Its test wrappers used `LOGGERD_TEST_BUILD` for the isolated test binaries and fixture tools.

## Test clock and fault configuration

- **Clock/report/audio contract helper:** `LOGGERD_TEST=1`, `LOGGERD_TEST_DETERMINISTIC=1`, `LOGGERD_SEGMENT_LENGTH=5`, `ENCODERD_DETERMINISTIC_DISABLE=0`. The in-process C++ helper explicitly calls `loggerd_test_enable_monotonic_clock()`. It advances the test clock by 5,000 ms to start rotation and another 5,000 ms to reach the unchanged readiness deadline. Loggerd and encoderd lifecycle subprocesses do not enable the fake clock.
- **Valid/failure DMA-heap E2E:** `LOGGERD_TEST=1`, `LOGGERD_TEST_DETERMINISTIC=1`, `LOGGERD_SEGMENT_LENGTH=2`, `ENCODERD_DETERMINISTIC_DISABLE=0`, with the test-build fault selectors unset for valid mode. Reject, silent, and drain-timeout modes set `LOGGERD_TEST_MPP_FAULT_STREAM=roadEncodeData` and the respective `LOGGERD_TEST_MPP_FAULT_MODE` value. The subprocesses use real steady time.
- **SHM E2E:** same deterministic loggerd configuration and 2-second segment length, with a memfd-backed SHM VisionIPC fixture. The full-resolution MPP inputs are rejected by the actual DMA-BUF validation path; qRoad runs its RGA resize/encode path.
- **Legacy KA2 E2E:** `LOGGERD_TEST=1`, `LOGGERD_TEST_DETERMINISTIC=0`, `LOGGERD_SEGMENT_LENGTH=2`, `ENCODERD_DETERMINISTIC_DISABLE=1`, with MPP fault selectors unset.
- **Recovery helper:** `LOGGERD_TEST=1`, `LOGGERD_TEST_DETERMINISTIC=1`, `LOGGERD_SEGMENT_LENGTH=5`, `ENCODERD_DETERMINISTIC_DISABLE=0`; the `invalid_fd`, `shm`, `reject`, `silent`, and `drain_timeout` arms use `LOGGERD_TEST_MPP_FAULT_STREAM=roadEncodeData` and the corresponding mode. Parser and counter-contract checks do not require MPP context.

## Results and gap qualification

- **Loggerd clock, report, and audio contracts:** fake-clock helper coverage passed for startup without a report, the no-message 5,000 ms deadline, exact report schema, all-ready rotation, audio ordering, byte-preserving queue-cap rejection, expiry, and the expiry/commit tie.
- **DMA-BUF validation and encoder recovery:** direct invalid-FD and real `memfd_create` SHM NV12 cases passed. The SHM case observed one DMA-BUF validation rejection, zero MPP input submissions, and no pending frame identity. Strict authority parsing and planned/recovery/retry counter contracts passed. Reject, silent, and drain-timeout fault seams passed; the drain-timeout expectation allows road recovery after the first boundary retry.
- **Synthetic DMA-heap lifecycle:** the fixture provided four 1920×1200 DMA-BUF slots per road, driver, and wide stream. The valid loggerd/encoderd run completed segments 0–2 with all expected files, frame/index/decoded-pixel assertions, qRoad video and audio decode, healthy reports, and zero known drops. Persistent reject and silent road-output cases retained qRoad media and audio, reported only the road output missing, and kept all known drops at zero. The drain-timeout arm observed the timeout and valid qRoad media; later road recovery is allowed. qRoad counter snapshots showed planned reopen counts 1 and 2 at segments 1 and 2 with zero recovery and retry increments.
- **SHM-to-RGA, now exercised:** the isolated memfd RGA probe returned `status=1 detail=Run successfully`; passing the same SHM allocation to the qRoad MPP path submitted and published 12 packets with zero rejects, empty packets, or encode errors. The full loggerd/encoderd SHM lifecycle test then ran 1920×1200 memfd buffers with four fixture slots per stream through segments 0–2. The real full-resolution road, wide-road, and driver MPP paths rejected their non-DMA-BUF inputs; qRoad used the actual RGA path and continued producing decodable video and audio. Rotation reports listed exactly those three full-resolution streams as missing, with all known loggerd drop counters zero. No full-resolution HEVC files or fabricated full-resolution indices were created. The RGA API and synthetic full SHM lifecycle are therefore proven for this memfd fixture.
- **AC-12 legacy behavior:** the KA2/RK3588 `ENCODERD_DETERMINISTIC_DISABLE=1` lifecycle passed two legacy rotations without deterministic reports/counters and preserved expected media. The test-only loggerd helper also passed when built with `-URK3588 -DLOGGERD_TEST_BUILD`, reporting `deterministic_test_mode=1`, `protocol_active=0`, no startup/runtime report, and immediate all-ready rotation. That guard-only helper is not a non-KA2 service runtime test. A TICI override attempt aborted before test code at `HardwareTici::get_device_type()` because this host is actual KA2 and lacks a TICI device-map entry; it is preserved as an environment/config mismatch. There is no genuine non-KA2 target available here, so **AC-12 remains unproven** for production loggerd/encoderd runtime behavior, writer finalization, packet/audio queues, and absence of the deterministic deadline/watchdog/recovery.
- **Existing loggerd rotation wrappers:** both `TestLoggerd::test_rotation` and `TestLoggerd::test_ka2_deterministic_rotation_alignment` passed in the full grouped rerun.

## Evidence and hashes

| Evidence | Path | SHA-256 |
|---|---|---|
| Updated full grouped suite | `evidence/tests-r10-run-r229.log` | `3192bac0c9edfff73d584fb81047b585939d0a0a2c74f89756be06603d983c37` |
| Gap-qualification probes, TICI mismatch attempt, and postflight | `evidence/tests-r10-gap-closure.log` | `952c12929edc7fb54f8cfb9f86e64a22e3ade001aa633bb547f30bec1dfe42c9` |
| Preserved original full suite | `evidence/tests-r10-run-r220.log` | `fee537cafe812163ed24a2467d0a138f144cfda90c6f06634b6f1f112792a775` |
| Original revision-220 preflight | `evidence/tests-r10-full-preflight-r220.log` | `546b3e911fa4e8de2dea0b05c254453e7edc10c2859f163505926d99d8f99dc1` |
| Original revision-220 postflight | `evidence/tests-r10-full-postflight-r220.log` | `546b3e911fa4e8de2dea0b05c254453e7edc10c2859f163505926d99d8f99dc1` |
| Earlier valid-lifecycle diagnostic after frame-id fix | `evidence/tests-r10-valid-diagnostic2.log` | `aa821ebfcb847a350ae7e07556180e800a9fc2894be7efe242ff8489f1eab39d` |

Current test-source hashes:

| Test source | SHA-256 |
|---|---|
| `system/loggerd/tests/test_loggerd_rotation.cc` | `bf185f8d3b9335182d347e010af35b1a20913b15bdeb8ad35a23b19edf1e7962` |
| `system/loggerd/tests/test_loggerd_rotation.py` | `d40aa6644fc2ccefe952c7310c242c8a950348a2f980b9f3740db458fc50b85f` |
| `system/loggerd/tests/test_encoderd_recovery.cc` | `9c7317eedf4f446507702fb8f9b29ccfe4e161a7d0115c46105e1677f60c080f` |
| `system/loggerd/tests/test_encoderd_recovery.py` | `aa5cddc9ace90969362375873e7d9cfeb225a80ec3cd4ce5bf716ba950f3ffd6` |
| `system/loggerd/tests/test_dmabuf_fixture.py` | `cfc0baf39d00be3871f7e8359083adc86266e29333e3b2dc09cd349e175a7b9c` |
| `system/loggerd/tests/test_loggerd_liveness.py` | `c024ff8fb62baba3a5601dab6cba5198115a5d18397c3f783b29fc5e98480b61` |
| `system/loggerd/tests/visionipc_dmabuf_publisher.cc` | `0994305f86b1c7c1481d3276ee2dd5b5fd81d033847db822a198568941fefad9` |

The final board postflight at 2026-10-31T10:10:47Z found no camerad, encoderd, or loggerd process and no users of the enumerated `/dev/video*`, `/dev/media*`, `/dev/v4l-subdev*`, `/dev/mpp_service`, or `/dev/rga` devices. Test children and temporary lifecycle resources had been cleaned.

## Remaining limits

The SHM result covers the synthetic memfd-backed VisionIPC source and KA2 RGA implementation only. It does not qualify a live camera/V4L2 DMA-BUF exporter, its fences or buffer reuse under live capture, or post-EOS behavior on the deployed production MPP build. AC-12 non-KA2 runtime behavior remains unproven until a genuine non-RK3588 target can run the isolated legacy service lifecycle.
