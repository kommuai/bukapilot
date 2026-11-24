# Sensord refactor intake

Request: investigate and refactor `system/sensord` for CPU and memory efficiency and substantially less implementation code, without losing behavior.

## Resolved decisions

| ID | Decision | Chosen | Source |
| --- | --- | --- | --- |
| D-1 | Behavior boundary | Preserve KA2 and TICI/tizi sensor selection, service rates, event fields and timestamps, calibration behavior, error handling, and shutdown. | user |
| D-2 | Refactor boundary | Keep the sensor topology and register operations; focus on polling-loop pacing, fixed per-sample calculations, calibration sample storage, and repeated event construction where equivalence can be tested. | delegated-default |

## Deferred decisions

| ID | Decision | Why deferral is safe | Owner | Reopen phase |
| --- | --- | --- | --- | --- |
| D-3 | Quantified CPU and RSS improvement on target hardware | `/dev/i2c-*` character-device nodes are present, but their board/sensor identity and functional access are unverified; no KA2/TICI rig or before/after baseline is confirmed, so a local result cannot establish target behavior or performance. | build/build-management | build |

## Rejected options

| ID | Option | Rationale |
| --- | --- | --- |
| D-4 | Merge sensor polling/IRQ threads or share bus handles | This changes scheduling, concurrency, or bus ownership and is not necessary to remove the identified overhead. |
| D-5 | Replace the gyro calibration statistics with an online approximation | Changed floating-point accumulation could alter the stationary classification near its thresholds. |

## Non-goals and YAGNI deferrals

| ID | Non-goal | Reopen trigger | Owner |
| --- | --- | --- | --- |
| D-6 | Change sensor choice, I2C register configuration, service schema, or publication cadence | A hardware revision or approved interface change requires different sensor behavior. | design/commander |
| D-7 | Claim a measured target speedup or complete hardware integration validation in this environment | A KA2/TICI rig with an agreed baseline and CPU/RSS measurement window becomes available. | build/build-management |
