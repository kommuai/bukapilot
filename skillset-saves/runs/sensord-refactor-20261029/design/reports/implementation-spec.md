---
submission_id: sensord-refactor-20261029-design-r4
revision: r4
---

# Implementation Spec — KA2-only `system/sensord` cleanup

**Design revision:** r4  
**Return target:** `design/commander` for package assembly and design gate; this is not a direct build handoff.  
**Approved inputs:** requirements `6f50367f5749062cb8c7928d9b279626b04dff4f6fbf8d6265980e8ae3ff7037`; architecture `a1a64320fd8a6b8995d08c5ba86d8c7a83f2f00630bdda1da0cf1f6981475b16`; IF-1 `481ff241761b6b35d83837e74bbee78c4893b6800f03ba217b322fd9ac8b1763`; plan `4937e75129c4920786e4cb73ed0f17c927e33c598a8c930da3c6e5885c6f17ee`; security seed `26b6e0e98da110dbb4cc31b6373c9bcef7af852c0c93a877a4a4cf85288ee642`.

The confirmed deployment target is KA2. D-20 removes the r3 `KA2` guard from `sensord.main()` and the explicit `enabled=KA2` manager override, using the process default enabled state while retaining `only_onroad`. Non-KA2 runtime behavior is outside this design contract and is not guarded. No hardware or I2C access is authorized or claimed.

## Runtime and stack context

This slice adds no runtime, framework, dependency, schema migration, or sensord-specific build target. The prior project scan hit its byte limit and did not establish a registry stack slug. The project declares Python `>=3.11,<3.13`; the available host was Python 3.12.3, below the harness minimum 3.13. The scan and host limitation remain environmental context, not a reason to alter the product stack.

## Slice order

| # | Slice ID | Depends on | Plan milestone | Ships behind | Acceptance owner |
| - | -------- | ---------- | ------------- | ------------ | ---------------- |
| 1 | `slice.sensord.ka2-platform-cleanup` | r4 architecture/IF-1 and prior accepted efficiency baseline | M1 | One atomic build/review patch; no feature flag | `build/build-management` |

There is one slice because the runtime removal, tests, process references, and Jenkins selection must agree in one change. Build begins only after the r4 design package is approved.

## Slice: `slice.sensord.ka2-platform-cleanup`

**Modules owned — exact allowlist:**

- `system/sensord/sensord.py` — remove the r3 `KA2` startup guard; keep realtime setup before the four KA2 sensor constructors, bus 3, current initialization order, polling and shutdown. Remove TICI/tizi sensor composition and IRQ handling.
- `system/manager/process_config.py` — remove `enabled=KA2` from the `sensord` registration so it uses the process default enabled state; retain `only_onroad`. Leave other process registrations and shared build definitions unchanged.
- `system/manager/test/test_manager.py` — remove only `test_sensord_enabled_only_for_ka2_and_stays_onroad`, which exists solely to assert platform gating. Add no replacement test; leave the rest of manager coverage unchanged.
- `system/sensord/tests/test_sensord.py` — delete the r3 guard-only test file. It carries no KA2 behavior coverage and must not be replaced with a non-KA2 invocation test.
- `system/sensord/tests/test_fixed_calculations.py` — remove only LSM6DS3/MMC5603-specific cases and imports; keep KA2 calculations.
- `system/sensord/tests/test_event_construction.py` — remove only non-KA2 cases and imports; keep KA2 event parity.
- `system/sensord/sensors/lsm6ds3_accel.py` — delete.
- `system/sensord/sensors/lsm6ds3_gyro.py` — delete.
- `system/sensord/sensors/lsm6ds3_temp.py` — delete.
- `system/sensord/sensors/mmc5603nj_magn.py` — delete.
- `selfdrive/test/test_onroad.py` — remove the obsolete TICI sensord CPU-budget entry.
- `system/loggerd/tests/test_encoder.py` — remove sensord start/stop assumptions from TICI/tizi rotation coverage; preserve loggerd rotation assertions.
- `Jenkinsfile` — remove the dedicated `LSM + MMC` / `tizi-lsmc` hardware stage; run the remaining mocked sensord tests in existing `HW + Unit Tests` with `diffPaths: ["system/sensord/"]`.

No sibling slice owns these paths. No other paths may change. In particular, leave `cereal/log.capnp`, `selfdrive/test/test_ka2_onroad.py`, `system/manager/build.py`, root `SConstruct`, common build logic, and unrelated worktree changes untouched. Keep historical sensor-source enum names and ordinals unchanged.

**Depends on:** r4 architecture and IF-1 fix the KA2-only runtime contract and D-20 process policy. The previous accepted polling pacing, fixed calculations, ordered tuple-list gyro calibration, and direct event construction remain the baseline; this slice does not reopen them.

**Data and migration:** none. No persisted data, schema, backfill, or migration changes. Preserve historical `SensorSource` values `lsm6ds3 @5`, `lsm6ds3trc @10`, and `mmc5603nj @11` so old logs remain decodable.

**Contract tests:** IF-1 is the process-lifecycle contract. Verify the manager registration uses its default enabled state with `only_onroad` unchanged, and verify `sensord.main()` has no platform guard and keeps realtime setup before the existing KA2 construction sequence. D-20 intentionally removes the guard-only and manager-gating-only tests; do not add replacement platform tests. Preserve mocked KA2 event, calibration, polling, error, and shutdown coverage.

**Proof plan:** D-20 makes no behavior change on the confirmed KA2 target. The two r3 platform assertions encode decisions that D-20 supersedes, so delete them and do not replace them. For the supported KA2 path, run the retained mocked suite and review startup/configuration order against IF-1. The smallest green change is removing the guard and manager override, then deleting only the obsolete guard/gating tests and non-KA2 cases, followed by the planned driver, reference, and CI cleanup. If a KA2 mocked contract fails, return to the owning architecture or plan owner instead of changing the accepted KA2 behavior locally.

**Primary acceptance command** (instruction for build; not run during design authoring):

```sh
python3 -m pytest -n 0 -q \
  system/manager/test/test_manager.py \
  system/sensord/tests/test_polling_pacing.py \
  system/sensord/tests/test_fixed_calculations.py \
  system/sensord/tests/test_gyro_buffer.py \
  system/sensord/tests/test_event_construction.py
```

Additional plan acceptance instructions: if `system/loggerd/tests/test_encoder.py` changes, run `LD_LIBRARY_PATH=/usr/local/lib pytest system/loggerd/tests/test_encoder.py`; inspect Jenkins selection to confirm `system/sensord/` changes run the mocked suite; run the scoped reference audit and classify allowed historical schema or unrelated release-history matches; run `git diff --check`. These are proposed checks, not results. No tests or hardware checks were run for this spec.

```sh
rg -n -i '(lsm6ds3|mmc5603|tizi-lsmc|LSM \+ MMC|sensord.{0,30}(TICI|tizi)|(TICI|tizi).{0,30}sensord)' \
  system/sensord system/manager selfdrive/test system/loggerd/tests Jenkinsfile
```

**Acceptance evidence:**

| Plan AC | Evidence required | Acceptor |
| ------- | ---------------- | -------- |
| AC-1 | Source/configuration review confirms default-enabled `sensord`, unchanged `only_onroad`, no `KA2` guard, and existing KA2 startup order. No replacement platform test. | `build/build-management` |
| AC-2 | The primary offline command passes; review confirms only the four KA2 drivers remain on bus 3 and the accepted KA2 event/calibration/pacing/error/shutdown behavior remains. No target-performance claim. | `build/build-management` |
| AC-3 | Four LSM/MMC drivers, non-KA2 test cases, and `test_sensord.py` are removed; manager gating-only test is removed with no replacement; historical enums remain unchanged and `cereal/log.capnp` has no diff. | `build/build-management` |
| AC-4 | TICI CPU budget and TICI/tizi loggerd sensord start/stop assumptions are absent; loggerd rotation checks and KA2 onroad coverage remain. The loggerd test passes if changed. | `build/build-management` |
| AC-5 | Dedicated Tizi sensord hardware stage is removed; mocked suite runs in existing `HW + Unit Tests` only on sensord path changes; shared build files/schema remain unchanged; no target or dependency is added. | `build/build-management` |
| AC-6 | Focused offline checks, scoped reference audit, and `git diff --check` pass; no device is accessed. | `build/build-management` |

**Rollback:** Revert this single cleanup slice to the post-efficiency, pre-r3 baseline, restoring the former TICI/tizi paths, process/test references, and dedicated hardware stage while preserving the accepted efficiency changes. Do not revert unrelated worktree changes. No data restoration or compensating migration is required.

## Operational constraints

| Constraint | Applies to | Owner | Removal criterion |
| ---------- | ---------- | ----- | ----------------- |
| No migration, feature flag, dependency, generic build change, or sensord-specific target. | `slice.sensord.ka2-platform-cleanup` | `build/build-management` | None; permanent scope boundary. |
| Keep the generic-stage sensord suite mocked and offline. | `slice.sensord.ka2-platform-cleanup` | CI and test maintainers | Permanent property; verify the command and path filter at AC-5. |
| D-20 removes platform rejection. Non-KA2 managed or direct invocation may reach fixed bus 3 and is not claimed safe. | `slice.sensord.ka2-platform-cleanup` | Admiral / platform owner | Reopen before any future non-KA2 deployment is considered. |
| No physical KA2 identity/startup/cadence/shutdown or CPU/RSS result is established. | Later target validation only | KA2 target owner with `build/build-management` | Requires an identified authorized rig, sensor identity, baseline, workload, measurement window, and threshold. No device access in this slice. |
| Host Python is 3.12.3, below harness minimum 3.13; the project scan was incomplete. | Harness lifecycle checks | Admiral/operator | Run any required lifecycle check in a supported environment; do not change product environment in this slice. |

## Non-goals and blocking decisions

- No non-KA2 support or off-target startup safety guarantee; no replacement guard.
- No change to the KA2 sensor set, bus/register configuration, event fields, rates, mapping, timestamps, warmup, calibration, polling/error/shutdown behavior, or accepted efficiency edits.
- No schema change, generic manager/SCons change, dependency, or new hardware build target.
- No device testing, hardware readiness signoff, or target startup/CPU/RSS measurement.
- No merge, release, or fleet rollout; this spec ends at the design gate and subsequent build/review boundary.

| Decision | Severity | Owner | Blocks | Latest safe decision point |
| -------- | -------- | ----- | ------ | -------------------------- |
| Identify an authorized KA2 board and sensor, then agree on baseline, workload, measurement window, and threshold. | Major | KA2 target owner with `build/build-management` | Hardware validation and target startup/cadence/CPU/RSS claims only; not this offline slice. | Before any device validation or target-performance claim. |
| Use a supported harness runtime if local lifecycle checks require Python 3.13. | Minor | Admiral/operator | Harness lifecycle evidence only. | Before submitting a gate that depends on the unavailable check. |

**Handoff:** Return this r4 spec and its hash to `design/commander`. Commander owns stack-lock and the `design-to-build` submission. Build receives it only through the approved design package.
