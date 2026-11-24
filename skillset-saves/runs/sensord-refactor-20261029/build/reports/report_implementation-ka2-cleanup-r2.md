---
submission_id: sensord-refactor-20261029_handoff-3_attempt-1
revision: r3
---

# Final implementation evidence — KA2-only sensord cleanup

## Outcome

This report refreshes the implementation artifact against the final build tree after test-builder completed the approved test changes. It makes no source changes. The baseline diff contains exactly the design allowlist: 9 modified files and 4 deleted files.

The sensord runtime now starts only on KA2, uses bus 3, and polls ICM42670 accelerometer/gyroscope/temperature plus LIS2MDL magnetometer. The first action in direct `sensord.main()` rejects non-KA2 before realtime setup or any constructor/thread/publication. The manager keeps its `only_onroad` predicate and enables the process only on KA2. TICI CPU and loggerd process expectations were removed; the dedicated LSM+MMC hardware job was replaced by the offline sensord suite in `HW + Unit Tests` with `diffPaths: ["system/sensord/"]`.

## Final diff and authorship

Unified diff: `build/artifacts/implementation-ka2-cleanup-r2.diff`  
SHA-256: `e1aec7107d84768103d5f8402b6384c2e234ec81cd2faa87b34a9eba96a6ca5a`  
File count: 13 total — 9 modified, 4 deleted. The earlier r73 diff artifact remains unchanged.

Bob-authored production/config/build paths:

| Path | Status | Final SHA-256 |
| --- | --- | --- |
| `Jenkinsfile` | modified | `1fc1d75dde2b1c0d111d52094b8ad3c89e1fcbeaa3772129d0588d44801f2cd8` |
| `selfdrive/test/test_onroad.py` | modified | `73d78c9af44aa9a45f50314aac9840b15ece9e746989419649c711e04ece7edd` |
| `system/loggerd/tests/test_encoder.py` | modified | `b7b161ba6d5175a204be97ea97f99133cb1965130d26185d1b30a893566c4fe6` |
| `system/manager/process_config.py` | modified | `a8c13f20eaa9d52325cdeda916198d99ad72a66ca1b49af50b436abb51b7d816` |
| `system/sensord/sensord.py` | modified | `e446938074d8873701d0c8797b845d5150c3b8326ac29853e1fc05847a10b176` |
| `system/sensord/sensors/lsm6ds3_accel.py` | deleted; hash of HEAD content | `2653028763b65af3b7489e646e4aec8190aae23f4a8d88e9e5587d5394b3221a` |
| `system/sensord/sensors/lsm6ds3_gyro.py` | deleted; hash of HEAD content | `830fd3bc58e40d5cafc23e44d1671df8c20e4b9fd30359cc3b2ebb54330c14b6` |
| `system/sensord/sensors/lsm6ds3_temp.py` | deleted; hash of HEAD content | `b27b9cddb0b762bd86dd4e02ee96ab1c7bcee2f35eef6a0a7db180216c3ccb36` |
| `system/sensord/sensors/mmc5603nj_magn.py` | deleted; hash of HEAD content | `ab1d1561405762aa189ced0e1ce54e36d79925c0f8eaf7bf532b508b9855b85c` |

Test-builder-authored test paths:

| Path | Status | Final SHA-256 |
| --- | --- | --- |
| `system/manager/test/test_manager.py` | modified | `df0378237be8b4a2672f1ac02b42deec2ce5d64d6f749edd2e25c7db0baf7296` |
| `system/sensord/tests/test_sensord.py` | modified | `03cac3cc204c1f8dc98c1b8a71612a015b6a44da0fa510ca800354e027b36d9c` |
| `system/sensord/tests/test_fixed_calculations.py` | modified | `ced678f349156c9c198509ba9d6a0c4240a64f2b0a6a50466b7c92185a87f045` |
| `system/sensord/tests/test_event_construction.py` | modified | `7b72579a563af65a943c7c4c2ecd79a0c824dff48a81f3e91e4604efda812fb6` |

The final allowlist diff has no outstanding implementation markers or follow-up notes. No unrelated dirty worktree files are included.

## Checks and test evidence

Prior implementation checks remain recorded: AST parsing, scoped diff check, Jenkins selection assertions, and runtime reference scan passed. Final `git diff --check` and the scoped reference audit passed as recorded in the test report.

The final test report is `build/reports/report_tests-ka2-cleanup.md` (SHA-256 `18619d4dc1fd188885d13b6e0835e721274b30b0452362a5cd280775d67cb4b4`). It records:

- The focused offline sensord suite plus manager-selection test passed: 10 passed.
- The approved combined offline command did **not** pass: 12 passed, 1 skipped, 2 failed because NumPy 2.5 raises a deprecation warning during existing modeld imports, and the repository treats warnings as errors.
- The loggerd integration test did **not** collect: the local Python environment lacks `parameterized`.

The passing focused suite does not replace those gaps. This report does not claim the combined suite or loggerd integration passed.

## Open risks

- Runtime health remains unverified; no authorized KA2 target or board/sensor identity was supplied, so no target startup or I2C probe was run. The required runtime evidence has no fallback and remains a build-to-review blocker.
- Loggerd rotation remains unverified because its test failed during collection.
- The full combined command remains red due to the local NumPy warning policy in existing manager tests.
- No KA2 physical cadence, shutdown, CPU, or RSS result is claimed.

## Return

Evidence refresh only; no code edit, commit, external boot-log operation, or hardware access. The evidence log is `build/evidence/implementation-evidence-refresh-r2.log`.
