# Test report — KA2-only sensord cleanup

**Result:** The scoped sensord suite and the new manager-selection contract pass. The exact combined acceptance command is not clean because two existing manager tests fail while importing modeld under the local NumPy 2.5 warning policy. The required loggerd test could not collect because the project Python environment lacks `parameterized`. Those gaps remain explicit; they are not reported as passes or skips.

## Executed checks

| Check | Command/result | Evidence |
| --- | --- | --- |
| Direct-entry and manager contracts | `PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -n 0 -q system/manager/test/test_manager.py::test_sensord_enabled_only_for_ka2_and_stays_onroad system/sensord/tests/test_sensord.py::test_non_ka2_fails_before_setup_sensor_access_threads_or_publication` — exit 0, 2 passed | `evidence/contracts-ka2-cleanup.log` |
| Scoped offline proof | `LOG_ROOT=/tmp/sensord-offline-test-Z9GJkG PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -n 0 -q system/sensord/tests system/manager/test/test_manager.py::test_sensord_enabled_only_for_ka2_and_stays_onroad` — exit 0, 10 passed | `evidence/sensord-offline-ka2-cleanup.log` |
| Approved combined offline command | `PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -n 0 -q system/manager/test/test_manager.py system/sensord/tests/test_polling_pacing.py system/sensord/tests/test_fixed_calculations.py system/sensord/tests/test_gyro_buffer.py system/sensord/tests/test_event_construction.py system/sensord/tests/test_sensord.py` — exit 1, 12 passed, 1 skipped, 2 failed | `evidence/tests-ka2-cleanup.log` |
| Loggerd integration command | `LD_LIBRARY_PATH=/usr/local/lib LOG_ROOT=/tmp/sensord-loggerd-test-KZR6BC PYTHONDONTWRITEBYTECODE=1 python3 -m pytest system/loggerd/tests/test_encoder.py` — exit 1 during collection, `ModuleNotFoundError: No module named 'parameterized'`, zero tests collected | `evidence/loggerd-test-ka2-cleanup.log` |
| Scoped reference audit | Approved `rg` expression over `system/sensord`, `system/manager`, `selfdrive/test`, `system/loggerd/tests`, and `Jenkinsfile` — no matches (`rg` exit 1 means none) | `evidence/test-reference-audit-ka2-cleanup.log` |
| Whitespace check | `git diff --check` — exit 0 | `evidence/diff-check-ka2-cleanup.log` |

The two combined-suite failures are `TestManager.test_manager_prepare` and `TestManager.test_set_params_with_default_value`. Both abort while importing `openpilot/common/transformations/orientation.py:28`: NumPy 2.5 raises `DeprecationWarning: Setting the shape on a NumPy array has been deprecated`, and the repository's pytest configuration promotes warnings to errors. The new manager-selection contract and all sensord tests pass in the scoped offline proof. A diagnostic rerun with only that exact warning filtered produced 14 passed, 1 skipped; it is recorded in `evidence/tests-ka2-cleanup-warning-filter.log` and does not replace the failing approved command.

## Coverage and limits

The passing focused checks cover KA2 true/false manager selection while retaining the on-road predicate, non-KA2 direct-entry rejection before realtime setup, the four retained KA2 constructors, thread creation and publication, plus all offline sensord polling, fixed-calculation, gyro-buffer, and event-construction tests. No coverage instrumentation or coverage percentage was collected. No hardware, I2C device, or KA2 target was accessed. Physical startup, cadence, shutdown, and target CPU/RSS remain unverified. The loggerd rotation test is also unverified because collection failed before its TICI skip could be evaluated.

The test runner used the project Python 3.12.3 environment with pytest 9.0.2. The isolated Python 3.13.15 harness interpreter has no pytest installed. No dependency was installed or environment changed.

## Test-run side effects

The exact combined manager suite and the warning-filter diagnostic invoked the existing `manager.main()` preparation tests. Those tests generated boot logs in `/data/media/0/realdata/boot/`; they were left untouched:

- `2026-10-29--12-29-59---boot.zst`
- `2026-10-29--12-30-01---boot.zst`
- `2026-10-29--12-30-57---boot.zst`
- `2026-10-29--12-31-00---boot.zst`

No manager process was started by the focused checks. The loggerd test failed at collection, before starting a process.

## Input digests

These digests bind the focused test output to the implementation and test sources:

| Path | SHA-256 |
| --- | --- |
| `system/sensord/sensord.py` | `e446938074d8873701d0c8797b845d5150c3b8326ac29853e1fc05847a10b176` |
| `system/manager/process_config.py` | `a8c13f20eaa9d52325cdeda916198d99ad72a66ca1b49af50b436abb51b7d816` |
| `system/manager/test/test_manager.py` | `df0378237be8b4a2672f1ac02b42deec2ce5d64d6f749edd2e25c7db0baf7296` |
| `system/sensord/tests/test_sensord.py` | `03cac3cc204c1f8dc98c1b8a71612a015b6a44da0fa510ca800354e027b36d9c` |
| `system/sensord/tests/test_fixed_calculations.py` | `ced678f349156c9c198509ba9d6a0c4240a64f2b0a6a50466b7c92185a87f045` |
| `system/sensord/tests/test_event_construction.py` | `7b72579a563af65a943c7c4c2ecd79a0c824dff48a81f3e91e4604efda812fb6` |
| `system/sensord/tests/test_gyro_buffer.py` | `12881a874d39cd0e63555af0d735878ffdd967947cc278aefaf0008dc4ecd63b` |
| `system/sensord/tests/test_polling_pacing.py` | `5fb7f96f663141e1549c5e102fd03fbc2a239ff179121166c231e61089f88323` |
| `system/loggerd/tests/test_encoder.py` | `b7b161ba6d5175a204be97ea97f99133cb1965130d26185d1b30a893566c4fe6` |
