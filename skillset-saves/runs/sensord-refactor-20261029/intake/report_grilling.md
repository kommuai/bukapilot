# Sensord refactor intake

Request: simplify `system/sensord` and its build/test integration to support KA2 only, while preserving KA2 behavior and the previous verified efficiency changes.

## Resolved decisions

| ID | Decision | Chosen | Source |
| --- | --- | --- | --- |
| D-1 | Previous behavior boundary | The prior approved revision preserved KA2 and TICI/tizi behavior; D-12 supersedes that platform scope for this revision. | prior-artifact |
| D-2 | Previous efficiency slice | Keep the validated polling pacing, fixed calculations, calibration representation, and direct event construction changes. | prior-artifact |
| D-8 | Meaning of code efficiency | Prefer fewer source lines when behavior is identical; keep an abstraction only when it reduces total lines without a material per-path slowdown above the existing 5% threshold. | user |
| D-11 | Gyro calibration storage | Keep the ordered tuple list; accept its approximately 47 KB temporary storage rather than use the slower compact array. | user |
| D-12 | Supported sensord platform scope | Support KA2 only: retain ICM42670 accelerometer, gyro and temperature plus LIS2MDL magnetometer; remove TICI/tizi LSM6DS3, MMC5603 and interrupt-path implementation. | user |
| D-13 | Manager process enablement | Start sensord only on KA2. | user |
| D-14 | Shared log schema compatibility | Retain historical LSM/MMC sensor-source enum values so existing logs remain readable. | codebase |
| D-15 | Build and CI boundary | Remove the obsolete Tizi sensord hardware build/test stage; keep the shared manager and SCons build definitions, which have no sensord-specific build target. | codebase |
| D-16 | Tests | Keep KA2-relevant offline tests, remove non-KA2-only test cases, and replace the TICI hardware integration suite with an offline KA2-only guard test. | user |
| D-18 | Direct process invocation | Fail fast at the start of `sensord.main()` when `KA2` is false, before realtime setup or constructing sensors, so a manual invocation cannot access the wrong I2C bus. | delegated-default |
| D-19 | Offline sensord CI coverage | Remove the Tizi LSM/MMC hardware stage and run the mocked KA2 sensord suite, including the guard test, in the existing generic HW + Unit Tests stage when sensord paths change. | delegated-default |
| D-20 | Platform checks for the confirmed target | Treat the deployed target as definitively KA2: remove the `KA2` startup guard and make the managed sensord process unconditionally enabled, while retaining `only_onroad`. On KA2, keep the current startup order, four drivers, bus 3, polling, services and shutdown behavior. Remove tests that only assert non-KA2 rejection or manager gating; remove the now-unused sensord guard-test file. | user |

## Deferred decisions

| ID | Decision | Why deferral is safe | Owner | Reopen phase |
| --- | --- | --- | --- | --- |
| D-3 | KA2 target startup and CPU/RSS measurements | No KA2 target, known sensor identity, or before/after target baseline is available; offline checks cannot establish hardware readiness or target performance. | build/build-management | build |

## Rejected options

| ID | Option | Rationale |
| --- | --- | --- |
| D-4 | Merge sensor polling/IRQ threads or share bus handles | Changes scheduling, concurrency or bus ownership and is unnecessary for platform cleanup. |
| D-5 | Replace gyro calibration statistics with an online approximation | Could change floating-point accumulation and stationary classification near thresholds. |
| D-9 | Keep shared event helper where it adds LOC or slows a driver path | Prior paired measurements found slower event paths; direct builders are shorter and faster. |
| D-10 | Build Cap’n Proto events by passing nested dictionaries to `new_message(**fields)` | Prior local measurements showed substantial regressions for temperature and magnetometer event construction. |

## Non-goals and YAGNI deferrals

| ID | Non-goal | Reopen trigger | Owner |
| --- | --- | --- | --- |
| D-6 | Change KA2 sensor selection, I2C register configuration, schema, polling cadence, event fields or calibration behavior. | A KA2 hardware revision or approved interface change requires different sensor behavior. | design/commander |
| D-7 | Claim KA2 hardware startup, target CPU/RSS improvement, or hardware integration validation without a target and baseline. | A KA2 rig and agreed measurement window become available. | build/build-management |
| D-17 | Remove historical LSM/MMC source identifiers from the shared log schema. | A separately approved schema migration explicitly drops decoding support for old logs. | design/commander |

## Superseded decisions

| ID | Superseded by | Reason |
| --- | --- | --- |
| D-13 | D-20 | The user confirmed this deployment is KA2 and asked to remove the manager's platform check; retain the on-road predicate. |
| D-16 guard-test portion | D-20 | Non-KA2 rejection is outside the confirmed target contract, so the guard-only test file is no longer needed. |
| D-18 | D-20 | Direct-entry platform rejection is unnecessary on the confirmed KA2 deployment; preserve the former KA2 startup order. |
