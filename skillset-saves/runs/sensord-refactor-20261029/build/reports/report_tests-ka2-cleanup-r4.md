# Test Report — sensord KA2 cleanup r4

## Outcome

**The required primary acceptance suite did not pass, so this stage does not provide a passing `tests` probe for `build-to-review`.** The run is recorded without warning filters or dependency changes.

## Runner and commands

The r4 handoff names the primary runner and command (runner discovery rung 1). The environment reports Python 3.12.3 and pytest 9.0.2. Manager output was redirected with `LOG_ROOT` to a temporary directory that was disposed of after the run.

Primary command:

```sh
LOG_ROOT=<temporary directory> python3 -m pytest -n 0 -q \
  system/manager/test/test_manager.py \
  system/sensord/tests/test_polling_pacing.py \
  system/sensord/tests/test_fixed_calculations.py \
  system/sensord/tests/test_gyro_buffer.py \
  system/sensord/tests/test_event_construction.py
```

Result: **2 failed, 10 passed, 1 skipped**. The two failures were `TestManager.test_manager_prepare` and `TestManager.test_set_params_with_default_value`. Both fail while importing modeld because NumPy 2.5 promotes the existing `DeprecationWarning` in `common/transformations/orientation.py:28` (setting an ndarray shape in place) to an error. No warning filter was applied. `TestManager.test_clean_exit` is explicitly skipped by the existing test annotation.

The four retained sensord test modules were also run alone as a supplemental diagnostic:

```sh
python3 -m pytest -n 0 -q \
  system/sensord/tests/test_polling_pacing.py \
  system/sensord/tests/test_fixed_calculations.py \
  system/sensord/tests/test_gyro_buffer.py \
  system/sensord/tests/test_event_construction.py
```

Result: **8 passed**. This is supplemental evidence and does not replace the failed primary command or establish runtime behavior.

Because `system/loggerd/tests/test_encoder.py` remains changed in the aggregate cleanup, the prescribed additional command was attempted:

```sh
LD_LIBRARY_PATH=/usr/local/lib pytest system/loggerd/tests/test_encoder.py
```

Collection failed before tests ran with `ModuleNotFoundError: No module named 'parameterized'`. No package was installed.

## Evidence files

- Primary runner log: `build/evidence/tests-ka2-cleanup-r4.log` — SHA-256 `dc1889a66837f6f6dec3d2d664085aa5c9a2a32e84d30e965ed93efda968bb10`
- Supplemental sensord-only log: `build/evidence/tests-sensord-retained-r4.log` — SHA-256 `69187a52d150f94efa8a0b4dbbb7626038190d5c191895226512ddcc9484d284`
- Loggerd collection log: `build/evidence/loggerd-test-ka2-cleanup-r4.log` — SHA-256 `d434caf170fa2ea9fc6277386ead41e75d00fd942394c17112f2f616e88a152d`

Input hashes for the primary test surface:

- `system/manager/process_config.py`: `bd66a0051d0310afa828a47a78a5b7bf0800c7b2f55c0c3b34f69ca3cca172e9`
- `system/manager/test/test_manager.py`: `b1d6d2713062bf65322fbaef013683f0a8df372bc3d0e2254020de2d8181fcb6`
- `system/sensord/sensord.py`: `d513926a25141a05fe1ed0330a71ce03cf544fd4db4bf57b3d3f06fe5097a06f`
- `system/sensord/tests/test_polling_pacing.py`: `5fb7f96f663141e1549c5e102fd03fbc2a239ff179121166c231e61089f88323`
- `system/sensord/tests/test_fixed_calculations.py`: `ced678f349156c9c198509ba9d6a0c4240a64f2b0a6a50466b7c92185a87f045`
- `system/sensord/tests/test_gyro_buffer.py`: `12881a874d39cd0e63555af0d735878ffdd967947cc278aefaf0008dc4ecd63b`
- `system/sensord/tests/test_event_construction.py`: `7b72579a563af65a943c7c4c2ecd79a0c824dff48a81f3e91e4604efda812fb6`

## Coverage and limits

The retained offline tests cover polling pacing, fixed calculations, gyro buffering and KA2 event construction; the broad manager suite also ran and exercised process preparation before the unrelated NumPy warning stopped two tests. The test-only KA2 manager gate and `test_sensord.py` platform guard test were removed as required by D-20, with no replacement platform assertions.

The Bob implementation report's static check confirms the supported KA2 path retains realtime setup before construction, the ordered accel/gyro/temp/magnetometer sensors, bus 3, and the manager's default `enabled=True` with `only_onroad`. These tests did not start sensord, access I2C or a board, establish sensor identity/cadence/shutdown, or prove runtime health.

## Open blocker

**Major:** The primary required command has two environment-triggered failures, and the loggerd test cannot collect because `parameterized` is absent. The test key is unproven; do not submit a passing typed test probe. The build gate remains unsubmitted until the prescribed test evidence is available. Runtime health is a separate required key and remains unproven without an authorized KA2 target.
