---
submission_id: sensord-refactor-20261029-design-r4
revision: r4
---

# Interface contracts — KA2-only `system/sensord` cleanup

**Boundary:** `design-to-build`  
**Requirements:** `reports/requirements-brief.md` · SHA-256 `6f50367f5749062cb8c7928d9b279626b04dff4f6fbf8d6265980e8ae3ff7037`  
**Confirmed intake:** `../intake/report_grilling.md` · SHA-256 `a51313a34983497a27a5c9ddc5cdc0c9f82d725f6c8cdf382ab2dae2c75edd4e`; `../intake/intake-brief.md` · SHA-256 `c2e026d40ff71df11fd3d0277704255c51df061bbf3ffc5c7b81187513b9dc75`

## Contract result

`change: changed`. The existing KA2 cereal sensor-event contracts are unchanged. The runtime scope is now KA2-only, and D-20 removes the r3 platform checks: manager registration uses its unconditional enabled default while retaining `only_onroad`; `sensord.main()` has no `KA2` guard. On the confirmed KA2 deployment, realtime setup and the existing four-sensor construction sequence are unchanged. This report defines no support contract for non-KA2 startup.

There is no HTTP/API endpoint, webhook, or new user-facing endpoint in scope. `IF-1` is an internal process-lifecycle contract. The existing `PubMaster` service messages and `SensorEventData` payloads remain the externally consumed interfaces.

## IF-1 — KA2 on-road managed startup and direct process entry

| Contract field | Definition |
| --- | --- |
| ID / kind / name | `IF-1` · `process-lifecycle` · KA2-only on-road startup with the existing direct process entry |
| Provider | `system/manager/process_config.py` process registration and `system/sensord/sensord.py:main()` |
| Callers | Manager selection/launcher for managed start; local process entry for direct invocation on the supported deployment. |
| Deployment assumption | The target is KA2, as confirmed by D-20. This is the only platform whose runtime behavior this contract specifies. |
| Managed registration | Register `sensord` with the process type's default enabled state (unconditionally enabled); retain the existing `only_onroad` predicate. Do not add a `KA2` enablement expression or another platform gate. |
| Managed off-road result | The existing `only_onroad` predicate keeps the process stopped while off-road. This predicate is independent of platform selection. |
| Direct KA2 entry | Call `sensord.main()`; no `KA2` input/check or unsupported-platform exception is introduced. The first action remains existing realtime setup, followed by construction of ICM42670 accelerometer, gyro, and temperature plus LIS2MDL magnetometer on bus 3 in the existing order. |
| Successful KA2 result | Retain reset/init/error handling, worker start, the existing four published services, per-iteration pacing, worker signaling/join, and sensor shutdown. No service names or payload fields change. |
| Non-KA2 result | Unspecified and outside this design's confirmed deployment scope. Neither manager registration nor `main()` rejects non-KA2 platforms. Direct or managed off-target execution is not tested or guaranteed. |
| Observability | Existing daemon and sensor error logging remains. No new metrics, log service, or event is added. |
| Compatibility | No command-line arguments, process name, manager name, `only_onroad` predicate, service names, or cereal payload fields are changed for KA2. Historical source enum identifiers remain unchanged. |

### Failure taxonomy for IF-1

| Failure | Response | Message or data effect |
| --- | --- | --- |
| Manager is off-road | The existing `only_onroad` predicate prevents managed start. | No child process is created while off-road. |
| Sensor object construction on the confirmed KA2 target fails | Existing constructor exception propagates out of `main()`. | Startup fails; no synthetic service event is introduced. |
| Sensor reset fails | Existing exception is logged; initialization continues. | Other sensors remain eligible to initialize. |
| Sensor initialization fails | Existing exception is logged; no polling worker starts for that sensor; initialization of other sensors continues. | That service may be absent; no replacement event is introduced. |
| Poll/read/event/publish operation fails | Existing exception is logged for that service; its polling loop continues and reaches `Ratekeeper.keep_time()`. | Failed iteration emits no successful event; later iterations continue. |
| Shutdown fails | Existing exception is logged and shutdown of remaining sensors continues. | Previously published data is unaffected. |
| A non-KA2 process reaches this entry | No platform-specific response is defined by this contract. | Non-KA2 behavior is outside scope; no guard or safety claim is made. |

## Existing KA2 cereal event contracts — unchanged

The process publishes the existing service names through `messaging.PubMaster`, using a top-level message with `valid=True`. Each inner `SensorEventData` retains the current driver-local builder, source, timestamp, metadata, union, payload mapping, status, and field defaults.

| Service / cadence | Driver and source | Inner event contract | Timestamp and mapping |
| --- | --- | --- | --- |
| `accelerometer` · 104 Hz | ICM42670 · `icm42670` | `version=1`, `sensor=1`, `type=1`; `acceleration.v` vector and `status=1`. | Supplied `ts`, or `time.monotonic_ns()` when omitted. Preserve raw scaling, signs, axis mapping, and X/Z rotation. |
| `gyroscope` · 104 Hz | ICM42670 · `icm42670` | `version=1`, `sensor=5`, `type=16`; `gyroUncalibrated.v` vector and `status=1`. | Supplied `ts`, or `time.monotonic_ns()` when omitted. Preserve calibrated counts, mapping, X/Z rotation, and selected bias. |
| `temperatureSensor` · 2 Hz | ICM42670 · `icm42670` | `version=1`, `type=4`; `sensor` remains at its schema default `0`; scalar `temperature`. | Supplied `ts`, or `time.monotonic_ns()` when omitted. Preserve `raw / 128.0 + 25.0`. |
| `magnetometer` · 25 Hz | LIS2MDL · `lis2mdl` | `version=1`, `sensor=3`, `type=14`; `magneticUncalibrated.v` vector and `status=1`. | Supplied `ts`, or `time.monotonic_ns()` when omitted. Preserve scale `0.15 µT/LSB` and mapping `x=-word1`, `y=word0`, `z=-word2`. |

Keep message and event field presence, source, status, values, mapping, and configured service rates in `cereal/services.py`. Do not normalize optional fields through a shared builder. Do not change `cereal/log.capnp`; preserve historical `SensorSource` values `lsm6ds3 @5`, `lsm6ds3trc @10`, and `mmc5603nj @11` for archived logs.

### Polling, validity, calibration, and shutdown contract

- Poll one worker per successfully initialized sensor, paced to the service frequency: accel and gyro 104 Hz, magnetometer 25 Hz, temperature 2 Hz.
- Publish only after the existing `Sensor.is_data_valid()` window exceeds 0.5 seconds. Each successful service message remains valid.
- Keep read, validity gate, event construction, send, and error logging order. `Ratekeeper.keep_time()` runs once per loop, including invalid-data iterations and caught read/publish failures.
- Keep gyro calibration at 300 ordered XYZ integer tuples with 5 ms sample sleeps, `fmean`, population `pstdev`, current stationary thresholds, and selected-bias behavior. Retain the accepted tuple-list memory tradeoff.
- On exit, set the worker event, join live workers, then invoke sensor `shutdown()`. Preserve existing reset/init/poll/shutdown logging and continue behavior.
- Remove TICI/tizi IRQ behavior: no GPIO 84, IRQ affinity, IRQ timestamp conversion, interrupt worker, or LSM/MMC event source remains in runtime.

## Contract verification and build placement

1. Delete the now-unused guard-only `system/sensord/tests/test_sensord.py`. It contains no remaining KA2 behavior coverage after the r3 TICI integration was replaced by a `KA2=False` guard assertion. Do not replace it with a non-KA2 invocation test.
2. Remove the manager test in `system/manager/test/test_manager.py` whose sole purpose is to assert `enabled` follows `KA2`; do not add a replacement gating test. The process registration's unconditional enabled default and unchanged `only_onroad` predicate are reviewed from configuration.
3. Retain the mocked KA2 sensord suite. Remove only non-KA2-only cases from `test_fixed_calculations.py` and `test_event_construction.py`; preserve KA2 pacing, calibration, event parity, error, and cleanup coverage.
4. Remove the dedicated `LSM + MMC` / `tizi-lsmc` hardware sensord stage. Run the remaining mocked sensord suite in existing `HW + Unit Tests` conditionally on `system/sensord/` path changes; tests must not read I2C or require attached sensors.
5. Remove stale TICI sensord process budget/start/stop expectations from the onroad and loggerd tests while preserving loggerd rotation assertions and existing KA2 process mapping/coverage.
6. Keep `system/manager/build.py`, root `SConstruct`, common build logic, and `cereal/log.capnp` unchanged. Add no sensord-specific target or dependency.

The retained offline tests establish code-level KA2 behavior only. They do not establish physical chip identity, target startup, live cadence, physical accuracy, or target CPU/RSS. No hardware or I2C access is part of this contract.

## Typed `interface_set` evidence

```yaml
change: changed
items:
  - id: IF-1
    kind: process-lifecycle
    name: KA2 on-road managed startup and direct process entry
artifacts:
  - reports/report_interfaces.md
```

`IF-1` is named in this report and the architecture report. The cereal event inventory is unchanged; `change: changed` covers the KA2-only runtime/process lifecycle boundary. This report does not define a platform-rejection interface.

## UI applicability

```yaml
applicable: false
reason: no user-facing surface - design system not engaged
scope: design-to-build
decided_by: architect
```

This daemon and its machine consumers introduce no screen, visual component, or end-user interaction.
