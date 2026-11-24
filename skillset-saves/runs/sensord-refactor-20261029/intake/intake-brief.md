# Admiral intake brief: KA2-only sensord cleanup

## Request and endpoint
Make `system/sensord` support the confirmed KA2 deployment, remove all non-KA2 sensord implementation and now-unused platform-specific files, and simplify its build/test integration. The user confirms this deployed device is KA2 and asks to remove platform checks. Preserve the previously approved sensord efficiency improvements and KA2 runtime behavior. Route this scope change back through design, then build and review; do not merge, deploy, or access hardware.

## Scope
Keep KA2 ICM42670 accelerometer, gyro and temperature polling and LIS2MDL magnetometer polling. Remove the TICI/tizi LSM6DS3 interrupt-driven accelerometer/gyro path, LSM temperature polling, optional MMC5603 path and their drivers/tests. Remove the `KA2` guard from direct `sensord.main()` startup and make the manager registration unconditionally enabled, retaining its existing `only_onroad` predicate. On the confirmed KA2 target, this preserves the current startup order and behavior because `KA2` is true and the manager's enabled default is true. Remove tests that only assert non-KA2 rejection or KA2 manager gating, including the now-unused guard-only `system/sensord/tests/test_sensord.py`. Remove stale TICI test process-budget/start/stop references and the Tizi-only sensord hardware Jenkins stage; run the remaining mocked KA2 unit tests in the existing generic HW + Unit Tests stage on sensord changes. Keep the generic manager/SCons build definitions and historical sensor-source enum values needed to read existing logs.

## Constraints and non-goals
Preserve KA2 I2C bus and sensor configuration, publication rates, data mapping, event metadata/timestamps, warmup validity, calibration algorithm, error handling and shutdown. Preserve the prior verified optimizations. Do not change shared log schema, unrelated worktree modifications, or generic build behavior. Do not access a device or claim target performance without a confirmed KA2 target and baseline.

## Acceptance
- The sensord runtime contains only KA2 sensor initialization and polling, with no platform checks, TICI/tizi IRQ handling, or non-KA2 driver imports.
- Non-KA2-only drivers and test cases are removed; the obsolete TICI integration and guard-only test file are removed.
- The manager keeps `only_onroad` and uses the default enabled state; direct `sensord.main()` begins with its prior KA2 startup order.
- TICI test, loggerd test and Jenkins references no longer expect TICI/tizi sensord.
- Existing shared build files have no obsolete sensord target or dependency; no sensord-specific Tizi CI build remains.
- Focused offline sensord and manager tests pass, remaining non-KA2 source references are only historical schema values or unrelated release history, and `git diff --check` passes.
- KA2 device startup and target CPU/RSS measurements remain explicitly unverified because no target is confirmed.

## Intake evidence
The previous approved design and build cover efficiency changes while preserving TICI/tizi support, then r3 narrowed runtime support to KA2 with explicit platform checks. Repository inspection found four non-KA2 sensor drivers, non-KA2 unit and TICI hardware tests, a Tizi-specific Jenkins sensord stage, and TICI process/test references. `system/manager/build.py` and root SCons contain no sensord-specific build target. This request supersedes the manager-gating and direct-entry guard decisions because the user confirms the deployed device is KA2 and asks to remove KA2 checks. The change reopens design before more source edits. Current readiness reports Python 3.12.3 below the harness minimum 3.13 and missing Codex hooks. No hardware access or rig authorization is provided; target startup and performance remain unverified.
