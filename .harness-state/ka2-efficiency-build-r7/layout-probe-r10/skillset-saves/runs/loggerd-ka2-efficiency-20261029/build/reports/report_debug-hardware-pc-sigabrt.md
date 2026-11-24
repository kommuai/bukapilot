# Debug report: HardwarePC loggerd SIGABRT on KA2 host

## Disposition

The failure is reproduced and the software branch selection is supported by the build configuration, but the abort site is not isolated. Available logs and static source inspection cannot distinguish a loggerd defect from an interaction specific to running the HardwarePC software configuration on KA2 hardware. Do not treat this run as AC-12 evidence. Escalate the unknown mechanism to the build `investigate` stage before changing product code or claiming the non-RK3588 branch passes.

## Reproduction

- Run: `loggerd-ka2-efficiency-20261029`, build revision 246; the failed execution was preserved as the r245 test log.
- Host: the actual KA2 aarch64 board (`/KA2` remained visible). The test did not override `OPENPILOT_DEVICE` or hardware identity.
- Build: dedicated test-only `loggerd_non_rk3588` and `encoderd_non_rk3588` targets, compiled with `-URK3588`, without `__TICI__`, with isolated `.non_rk3588.o` objects. The target compilation had previously been verified with `-URK3588` after inherited `-DRK3588`; the r245 run itself reported both targets up to date.
- Fixture: synthetic DMA-heap publisher and the test's isolated Params, log root, and messaging prefix.
- Command: `python3 -m pytest -s -v -n0 -x system/loggerd/tests/test_loggerd_liveness.py::test_non_rk3588_hardware_pc_legacy_lifecycle_on_ka2_host`
- Result: loggerd exited with `-6` (SIGABRT); the test failed in its process-liveness check. The encoderd and publisher were stopped by teardown, and postflight found no remaining daemon or device users.

## Observed timeline

1. The synthetic publisher started, then the test launched the dedicated loggerd and encoderd binaries.
2. The test observed the expected readers and `LoggerdSegment=0`.
3. The test sends a frame and audio batch before checking that the child processes remain alive. Therefore the evidence does **not** establish that loggerd died before the first frame was sent.
4. Encoderd output records initialization of the road, driver, and wide-road encoders. At cleanup, segment 0 contained multi-megabyte `fcamera.hevc`, `dcamera.hevc`, and `ecamera.hevc` files. This is consistent with loggerd reaching the main-stream video-writer path, although it does not identify the abort site.
5. The parameter snapshot remained at segment 0, epoch 1, state `committed`, with no rotation report. `qcamera.ts` was absent. The compressed `rlog.zst` and `qlog.zst` files existed but were empty at cleanup; they do not provide a stack or useful event trace.
6. Captured loggerd stdout/stderr was empty. The test process had `ulimit -c 0`; no core or backtrace was produced. The captured failure was reported about 15.8 seconds into the test.

## Evidence and hashes

| Artifact | SHA-256 |
| --- | --- |
| `build/evidence/tests-r10-gap-legacy-pc-lifecycle-r245.log` | `94e7c671fd1e7f103e31fa8f9e04b036f371600528167969a8c2b828741015cb` |
| `build/evidence/tests-r10-gap-legacy-pc-postflight-r245.log` | `2b40816d1127f4c2acf0cdffca2374f4527e8c1d9fca76a95e2b02693106cc4d` |
| `build/evidence/tests-r10-gap-helper-r245.log` | `21d512b8a2e1b629796d19454f9d436133df767512bae2add4329758d3ad230b` |
| `system/loggerd/loggerd.cc` | `3a2e621adc50f44f727a12b0d8deb7410a4c502948ae2297069aa40b4ea8f5a9` |
| `system/loggerd/tests/test_loggerd_liveness.py` | `72c69e1e98cf2fdc841c51161e3d5dc0210b28035466e1798c4c4b4848d29342` |
| `system/loggerd/SConscript` | `9423e0b617e2b99b50ee97e592c65d89925a6bbad0af6f8944eb78fb056c2373` |
| `system/hardware/hw.h` | `892e05d0fc7aea3c78c1e961babc3607769fd3fb1c6e33d998618b4a9680ac7f` |
| `system/hardware/pc/hardware.h` | `ded4068f4f4d42f53bc682f188dfbbc056fd7ae40a7a400fd0b021d4385f4a1f` |
| `system/loggerd/logger.cc` | `a886b4069cff65bc7d9caa83c029fbf3cf9098ab61462990c316da13e3015f6f` |
| `system/loggerd/video_writer.cc` | `a308bb3aff24e823e8ca359d253064c9aa7f38fdb0cc0fc46e8ca2b0ee576cd3` |

The passing helper suite covers the encoder recovery helper cases only; it does not reproduce or clear this loggerd lifecycle failure.

## Confirmed source behavior

- `system/hardware/hw.h` selects `HardwarePC` when `__TICI__` and `RK3588` are both undefined. `HardwarePC::PC()` returns true and its init data identifies the device type as PC. This supports the intended non-RK3588 software-branch build; the physical host remains KA2.
- The dedicated SCons targets use an isolated object suffix and explicitly include the FFmpeg encoder source while excluding the MPP encoder source. The test does not mutate the platform-identification files or production targets.
- In `loggerd.cc`, the PC branch skips the affinity call and disables the KA2 deterministic rotation protocol. In `loggerd.h`, it selects `BIG_BOX_LOSSLESS` for main-stream output. Those choices lead to a distinct PC video-writing path, but none alone proves the observed abort.
- The fixture sets isolated `LOG_ROOT` and `PARAMS_ROOT`, and loggerd reached creation of segment 0. That makes a missing initial route directory an unlikely explanation, but does not rule out later writer, packet, or library failures.

## Cause assessment

No isolated cause is supported by the available evidence. Empty child output plus disabled core dumps leaves the SIGABRT instruction and stack unknown. Static inspection finds several possible abort sites on the exercised path, including loggerd assertions, `LoggerState`/`ZstdFileWriter` assertions, and `VideoWriter`/FFmpeg assertions. The available observations cannot select among them.

The failure is not established to be a fixture mismatch: the synthetic publisher started, the test sent frames, loggerd created main-stream video files, and the isolated logger/Params roots were in use. It is also not established as a general HardwarePC defect: this is a HardwarePC software build running on KA2 host libraries, drivers, and kernel, not a genuine PC target. The proper classification is an unresolved failure in this mixed hardware/software qualification configuration.

One test configuration detail is not causal evidence: `legacy_non_rk` calls `_make_params_and_environment` without `legacy_mode=True`, so the environment advertises the deterministic test flag even though the non-RK3588 compile removes the KA2 deterministic code path. Static inspection indicates those flags do not activate that compile-time-disabled path, and this does not explain SIGABRT.

## Bounded next probe

Have `investigate` own the next evidence-gathering pass. After an immediate board/process/device preflight, run the same isolated fixture and same binaries with loggerd under a debugger that catches `SIGABRT` and records a native backtrace with symbols (the existing binary is unstripped and has debug info), or enable a per-test core dump with a known writable destination and analyze it offline. Preserve the current failed run unchanged. Do not change the production branch or weaken the assertion before the abort site is known. If the backtrace identifies an assertion or writer/library call, route the smallest repair to its owning build specialist and rerun this exact lifecycle to establish before/after behavior.

## Instrumentation and remaining acceptance

No source, test, SCons, or manifest files were edited during this debug pass. No instrumentation was added, so the instrumentation ledger is empty. No before/after repair test was possible because no cause or candidate repair could be supported without a backtrace. AC-12 remains unproven; this run must not be counted as a passing non-KA2 software lifecycle, and it does not qualify non-KA2 silicon, live V4L2 DMA-BUF export/fences/reuse, or deployed MPP behavior.
