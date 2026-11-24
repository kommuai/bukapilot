# Sensord focused regression tests — r5

**Run:** `sensord-refactor-20261029`  
**Candidate:** `source-r5-remove-no-win-scale-hoists`  
**Outcome:** **12 focused offline tests passed** using project Python 3.12.3.

## Command and result

```text
python3 -m pytest -n 0 -q \
  system/sensord/tests/test_polling_pacing.py \
  system/sensord/tests/test_fixed_calculations.py \
  system/sensord/tests/test_gyro_buffer.py \
  system/sensord/tests/test_event_construction.py

12 passed in 0.14s
```

The full command output and observed timestamp are in `build/evidence/tests-sensord-efficiency-r5.log`.

## Coverage

| Acceptance | Evidence |
| --- | --- |
| Polling pacing | Invalid iterations call `keep_time()` without publishing; valid readings still publish; read errors are logged and paced. |
| Fixed calculations | ICM42670 and LSM6DS3 formulas, axis mapping, gyro conversion, and MMC SET/RESET values are checked against baseline expressions. |
| Gyro calibration | Tuple sample order, signed-short endpoints, per-axis means and population standard deviations, classification, and selected bias are checked against baseline tuple behavior. |
| Event serialization | Temperature and magnetometer payload fields are checked against baseline-shaped cereal events. |

The test suite was run after the final LSM6DS3/MMC no-win edits were removed. R6 independently reran all eight event parity fixtures while measuring. No hardware was needed for these checks.

## Limits

The hardware-marked `system/sensord/tests/test_sensord.py` suite was not run. It starts the daemon and reads physical sensors. The daemon startup/readiness smoke remains unavailable without an identified and authorized target; offline tests do not replace it.
