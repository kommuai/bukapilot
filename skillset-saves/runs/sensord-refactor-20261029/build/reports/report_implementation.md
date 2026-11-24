# Sensord efficiency implementation report — r5

**Run:** `sensord-refactor-20261029`  
**Approved design:** `sensord-refactor-20261029-design-r2`  
**Candidate:** `source-r5-remove-no-win-scale-hoists`  
**Outcome:** implementation and focused offline evidence are complete; hardware runtime remains unverified.

## Final change set

| Slice | Files | Result |
| --- | --- | --- |
| Polling pacing | `system/sensord/sensord.py` | Invalid sensor data no longer skips the existing `Ratekeeper.keep_time()` call. Valid data still publishes, and error paths still reach pacing. This fixes a busy loop. |
| Fixed ICM calculations | `icm42670_accel.py`, `icm42670_gyro.py` | Hoisted fixed scale, angle sine/cosine, and degree conversion values. Existing arithmetic order and output fixture parity are preserved. |
| Focused regression coverage | Four files under `system/sensord/tests/` | Added polling, numeric formula, tuple-list calibration, and serialized event parity checks. |

Three scale-hoisting edits in LSM6DS3 and MMC5603 were tried, then removed because they offered no line-count reduction and did not justify expanding the final change set. Direct event construction in the temperature and LIS2MDL drivers remains unchanged; the shared event helper experiment was removed. Gyro calibration retains the approved tuple-list representation and its approximately 47 KB transient Python-object storage tradeoff, rather than the slower compact-array variant.

No sensor selection, register configuration, schema, service frequency, interrupt behavior, bus ownership, dependency, or startup flow was changed. No hardware or I2C device was accessed.

## Size and review artifact

The same nine production files total **864 lines at baseline** and **862 lines in the candidate**, a reduction of **2 lines (0.23%)**. This count includes blank and comment lines and excludes tests. The complete final patch covers 7 paths: 3 production files and 4 tests. The main Git index tree remained unchanged while producing the patch.

- Full patch: `build/artifacts/implementation-r5.diff`
- SHA-256: `c4c6ddb89e41fc89ef4eefbc6d3be2fe3ef5446220dcdee985b733bb7c0ab843`
- Source changes in benchmarked production files: 3
- Focused test files: 4

### Changed source hashes

| Path | SHA-256 |
| --- | --- |
| `system/sensord/sensord.py` | `45b0fa7635a541cf2024b1ceda36774fe5faea6fbc1aa5abb9df6e05cf4af35e` |
| `system/sensord/sensors/icm42670_accel.py` | `66046d50a36f3ef7b88577497aa6fb517068c9eb1f1973d040c2df1ea1e95d3d` |
| `system/sensord/sensors/icm42670_gyro.py` | `bcf82f511496ce18a9bf97aafac98ac4da7c6775e48f726b179b21ebb799158e` |

### Focused test hashes

| Path | SHA-256 |
| --- | --- |
| `system/sensord/tests/test_polling_pacing.py` | `5fb7f96f663141e1549c5e102fd03fbc2a239ff179121166c231e61089f88323` |
| `system/sensord/tests/test_fixed_calculations.py` | `c24eb1b7907a38f0fb7fd09475839cd3dfdd76b85d2b36e8caa33c9db7b45a76` |
| `system/sensord/tests/test_gyro_buffer.py` | `12881a874d39cd0e63555af0d735878ffdd967947cc278aefaf0008dc4ecd63b` |
| `system/sensord/tests/test_event_construction.py` | `ae2fd9e4a8a87cd0365877465f055c1f3dee58c8a4a1c664029277c30080c906` |

## Validation

- Focused offline suite: **12 passed** in 0.14 seconds using Python 3.12.3; see `build/evidence/tests-sensord-efficiency-r5.log`.
- R6 fixture parity: all 8 sensor event fixtures passed exact serialized-field comparison; timestamps were excluded because they come from dynamic clocks.
- Syntax: all 9 production source files parsed successfully.
- `git diff --check -- system/sensord`: passed. Full patch scope, reverse-apply check, source syntax, and unchanged main index are recorded in `build/evidence/implementation-verification-r5.log`.
- R6 paired benchmark: per-event event-time differences stayed within the approved ±5% practical threshold; invalid-data polling CPU fell about 99%.

The benchmark's paired-loader correction and limits are recorded in `build/reports/report_benchmark-simplification-r6.md`.

## Remaining verification

No daemon startup or hardware runtime smoke was performed. The saved runtime report states that no target and device-access authorization were supplied; the required runtime evidence is therefore still missing. The build cannot cross `build-to-review` until that target-specific check is completed.
