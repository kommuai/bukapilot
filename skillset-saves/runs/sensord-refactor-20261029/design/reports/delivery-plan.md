---
submission_id: sensord-refactor-20261029-design-r4
revision: r4
---

# Delivery Plan — KA2-only `system/sensord` cleanup

**Revision:** r4  
**Requirements:** `reports/requirements-brief.md` @ `6f50367f5749062cb8c7928d9b279626b04dff4f6fbf8d6265980e8ae3ff7037`  
**Architecture:** `reports/report_architecture.md` @ `a1a64320fd8a6b8995d08c5ba86d8c7a83f2f00630bdda1da0cf1f6981475b16`  
**Interfaces:** IF-1 in `reports/report_interfaces.md` @ `481ff241761b6b35d83837e74bbee78c4893b6800f03ba217b322fd9ac8b1763`  
**Security seed:** `reports/report_security-seed.md` @ `26b6e0e98da110dbb4cc31b6373c9bcef7af852c0c93a877a4a4cf85288ee642`; assessed no new trust boundary.  
**Intake:** D-20 is confirmed in `../intake/report_grilling.md` @ `a51313a34983497a27a5c9ddc5cdc0c9f82d725f6c8cdf382ab2dae2c75edd4e` and `../intake/intake-brief.md` @ `c2e026d40ff71df11fd3d0277704255c51df061bbf3ffc5c7b81187513b9dc75`.

## Milestone path

| # | Milestone | Delivery slice | Depends on | Decision gate before it | Target |
| - | --------- | -------------- | ---------- | ----------------------- | ------ |
| M1 | Keep the existing KA2 sensord path while removing the obsolete non-KA2 runtime and integration | `slice.sensord.ka2-platform-cleanup` | R4 architecture and IF-1; preserve the approved polling pacing, fixed calculations, ordered tuple-list gyro calibration, and direct event builders | AC-1 through AC-6 pass; build/review confirms the single slice matches this plan. It then stops at `build-to-review`; merge, release, and device rollout are outside this plan. | One build/review slice; no date supplied |

The cleanup is one atomic slice because runtime composition, its mocked tests, stale TICI/tizi references, and Jenkins selection must agree. Within the slice, first make only the D-20 process-entry changes: remove the `KA2` guard from `sensord.main()` and remove the `enabled=KA2` override from the manager registration, while retaining `only_onroad` and the existing KA2 startup order. Then remove the obsolete non-KA2 drivers and cases, the guard-only `test_sensord.py`, and the manager test that only asserts platform gating. Keep the remaining mocked KA2 suite, then remove stale TICI/tizi process references and move sensord CI coverage into the existing generic `HW + Unit Tests` stage. The CI suite stays offline and conditional on `system/sensord/` changes.

The confirmed deployment target is KA2. On KA2, `main()` continues to configure realtime scheduling before constructing the four sensors on bus 3 and preserves the existing polling, service, error, thread-join, and shutdown behavior. No guard or manager platform-selection test is added. Non-KA2 use is outside this design contract and is not guarded. Keep `system/manager/build.py`, root `SConstruct`, common build logic, and `cereal/log.capnp` unchanged; add no sensord-specific target or dependency.

## Acceptance criteria

| ID | Slice | Observable condition | Check that proves it | Acceptor |
| -- | ----- | -------------------- | ------------------- | -------- |
| AC-1 | `slice.sensord.ka2-platform-cleanup` | The manager registration uses the process type's default enabled state and retains `only_onroad`; `sensord.main()` has no `KA2` guard and the KA2 path begins with the existing realtime setup and four-sensor construction order. | Review `system/manager/process_config.py` and `system/sensord/sensord.py` against IF-1 and the pre-cleanup KA2 path. Remove the manager gating-only assertion; do not replace it with a platform test. | `build/build-management` |
| AC-2 | `slice.sensord.ka2-platform-cleanup` | Only the four KA2 drivers remain in the runtime on bus 3. Existing service names/rates, event metadata/mapping/timestamps, warmup, polling and error pacing, calibration, shutdown, and all accepted efficiency changes are preserved. | Run the retained mocked KA2 suite: `python3 -m pytest -n 0 -q system/manager/test/test_manager.py system/sensord/tests/test_polling_pacing.py system/sensord/tests/test_fixed_calculations.py system/sensord/tests/test_gyro_buffer.py system/sensord/tests/test_event_construction.py`. Review lifecycle and bus/register configuration; do not claim target performance. | `build/build-management` |
| AC-3 | `slice.sensord.ka2-platform-cleanup` | The four LSM6DS3/MMC5603 drivers and their non-KA2-only test cases are removed. The guard-only `system/sensord/tests/test_sensord.py` and manager-gating-only assertion are removed, with no replacement platform tests. Historical `SensorSource` identifiers `lsm6ds3 @5`, `lsm6ds3trc @10`, and `mmc5603nj @11` remain unchanged. | Verify the four driver paths and `test_sensord.py` are absent, inspect the `test_manager.py` test removal, audit runtime imports and tests, and confirm `cereal/log.capnp` has no diff. | `build/build-management` |
| AC-4 | `slice.sensord.ka2-platform-cleanup` | The TICI sensord CPU budget is removed; TICI/tizi loggerd rotation tests no longer start or stop sensord; loggerd rotation assertions and KA2 onroad process coverage remain. | Audit the onroad, loggerd, and KA2 process tests; inspect the loggerd test diff. If `system/loggerd/tests/test_encoder.py` changes, run `LD_LIBRARY_PATH=/usr/local/lib pytest system/loggerd/tests/test_encoder.py`. | `build/build-management` |
| AC-5 | `slice.sensord.ka2-platform-cleanup` | The dedicated `LSM + MMC` / `tizi-lsmc` sensord hardware stage is removed. The mocked KA2 sensord suite runs in the existing `HW + Unit Tests` stage only when `system/sensord/` changes. Shared manager/SCons build definitions and schema have no cleanup edits; no new target or dependency is added. | Inspect the Jenkins stage, command, and `diffPaths: ["system/sensord/"]`; confirm the selected suite uses mocks and performs no I2C access. Review the scoped diff for unchanged generic build definitions and schema. | `build/build-management` |
| AC-6 | `slice.sensord.ka2-platform-cleanup` | Focused offline KA2 and manager suites pass, the scoped reference audit has no stale non-KA2 runtime/test/process/CI assumptions beyond preserved schema identifiers or unrelated release history, and `git diff --check` passes. No hardware is accessed. | Run AC-2's offline command, run `rg -n -i '(lsm6ds3|mmc5603|tizi-lsmc|LSM \+ MMC|sensord.{0,30}(TICI|tizi)|(TICI|tizi).{0,30}sensord)' system/sensord system/manager selfdrive/test system/loggerd/tests Jenkinsfile`, classify permitted schema/release-history hits, and run `git diff --check`. Do not run `test_sensord.py` because D-20 deletes it. | `build/build-management` |

The source/configuration review in AC-1 is intentional: the former guard and platform-gating-only tests are out of scope under D-20. General manager tests remain in AC-2, but no test asserts a non-KA2 rejection or platform gate. These checks establish source and mocked behavior only; they do not establish physical sensor identity, target startup, live cadence, accuracy, CPU, or RSS.

## Rollout and rollback shape

| Stage | Audience | Mechanism | Rollback | Trigger to advance |
| ----- | -------- | --------- | -------- | ------------------ |
| Build/review only | Build and review owners | Submit the one scoped patch and its offline evidence to `build-to-review`; no merge, release, or device rollout is authorized here. Keep the cleanup delta separable from the approved efficiency baseline. | Revert this atomic cleanup slice to the post-efficiency, pre-r3 baseline, restoring the prior platform paths while preserving the accepted efficiency work. Do not revert unrelated worktree changes. | AC-1 through AC-6 pass and the build-to-review gate accepts the scoped package. |
| Target validation and performance | KA2 target owner with build management | Deferred; no hardware or I2C operation in this revision. A later target run requires an identified KA2 board and sensor, an authorized rig, baseline, workload, measurement window, and threshold. | Define target-specific rollback before any separately approved hardware rollout. | A new decision after the target and measurement prerequisites are recorded. |

## Risk handling

| Risk | Kind | Severity | Consequence if it lands | Mitigation | Owner |
| ---- | ---- | -------- | ----------------------- | ---------- | ----- |
| Non-KA2 manager or direct invocation is not rejected and may reach the fixed bus-3 path. | technical dependency | Info | Off-target use is not protected by this design. | The confirmed deployment is KA2; make no off-target safety claim. Any future non-KA2 deployment requires a new design decision before use. | Admiral / platform owner |
| Removing drivers leaves stale imports, tests, TICI budget/loggerd expectations, or CI references. | technical dependency | Major | The cleanup can fail collection or retain obsolete hardware assumptions. | Apply the exact allowlist, run the focused tests and scoped reference audit, and retain the loggerd rotation assertions. | `build/build-management` |
| Offline checks pass but physical sensor identity, KA2 startup/cadence/shutdown, CPU, and RSS are unknown. | external | Major | Source-level success could be mistaken for hardware readiness or a target-performance result. | Make no target claim and perform no device access. Defer target validation until an authorized rig and before/after baseline are recorded. | KA2 target owner with `build/build-management` |
| The generic `HW + Unit Tests` stage risks making sensord tests depend on hardware. | rollout | Major | A mocked-suite regression could silently read I2C or require an attached sensor. | Keep the suite offline and confirm the Jenkins command/path condition; zero hardware access is part of AC-5/AC-6. | CI and test maintainers |
| Local harness Python is reported as 3.12.3, below the harness minimum 3.13, with hooks missing. | technical dependency | Minor | A lifecycle validator or hook-enforced check may be unavailable locally. | Preserve exact command results; have Admiral/operator run any required unsupported lifecycle check in a supported harness. Do not change the environment in this slice. | Admiral/operator |
| No release date or staffing capacity was supplied. | staffing | Info | Calendar estimates would be unsupported. | Plan one bounded build/review slice without a date; resequence only if implementation or review capacity is unavailable. | `design/commander` / `build/build-management` |

## Decision Register

| Decision | Status | Chosen | Source | Rejected options (why) | Owner | Reopen trigger / latest safe point |
| -------- | ------ | ------ | ------ | ---------------------- | ----- | ---------------------------------- |
| D-12: sensord runtime scope | resolved | Keep only ICM42670 accel/gyro/temp and LIS2MDL magnetometer; remove TICI/tizi LSM6DS3, MMC5603, and interrupt paths. | user | Retaining non-KA2 runtime contradicts the confirmed scope. | `design/commander` | A new platform-scope request before implementation. |
| D-2: accepted efficiency baseline | resolved | Preserve the approved polling pacing, fixed calculations, ordered tuple-list gyro calibration, and direct event construction. | prior-artifact | Reverting accepted work or adding a further optimization broadens the cleanup. | `build/build-management` | A scoped parity failure requires a new design amendment. |
| D-20: confirmed target and process entry | resolved | Treat KA2 as the confirmed deployment; remove `KA2` checks from `sensord.main()` and managed enablement, use the process default enabled state, retain `only_onroad`, preserve KA2 startup order, and remove guard/gating-only tests. | user | Keeping a platform guard or platform-specific gating test contradicts the confirmed target and requested simplification. | `build/build-management` | A new deployment target or process behavior requirement before future implementation. |
| D-13: manager platform gating | superseded by D-20 | Do not pass `enabled=KA2`; use the default enabled state and preserve `only_onroad`. | D-20 user | `enabled=KA2` adds an unnecessary platform branch for the confirmed deployment. | `build/build-management` | A new process-lifecycle requirement. |
| D-18: direct-entry platform guard | superseded by D-20 | Do not reject non-KA2 inside `main()`; keep the existing KA2 entry sequence. | D-20 user | A `KA2` guard adds a check the user removed from the confirmed-target design. | `build/build-management` | A new explicit platform-support requirement. |
| D-16: test surface | partly superseded by D-20 | Retain mocked KA2 coverage and remove non-KA2 cases; delete the guard-only `test_sensord.py` and the manager-gating-only assertion, with no replacement platform tests. | D-16 user; D-20 user | Preserving tests that assert removed platform checks would constrain the new design incorrectly. | `build/build-management` | A new approved test scope. |
| D-14/D-17: historical log source identifiers | resolved | Preserve `lsm6ds3 @5`, `lsm6ds3trc @10`, and `mmc5603nj @11` in `cereal/log.capnp`. | codebase / prior-artifact | Removing or renumbering identifiers requires a separate schema migration and archive plan. | `design/commander` | A separately approved schema migration with decode evidence. |
| D-15/D-19: CI and generic build boundary | resolved | Remove the Tizi sensord hardware stage and run mocked sensord tests in existing `HW + Unit Tests` on `system/sensord/` changes; leave generic manager/SCons definitions unchanged. | codebase / delegated-default | Keeping the dedicated hardware stage makes offline coverage depend on obsolete non-KA2 hardware. | `build/build-management` | A proven path-filter miss or independent need for a generic build target. |
| D-3/D-7: target validation | deferred | Do not access hardware or claim startup/cadence/CPU/RSS results; no numeric target or board/sensor identity is established. | prior-artifact; intake | Inferring hardware readiness from platform designation or mocked tests is unsupported. | KA2 target owner with `build/build-management` | Before any target validation or performance claim, when rig, baseline, window, and threshold are agreed. |
| D-6: topology and interface expansion | YAGNI-deferred | Do not change KA2 sensors, bus/register setup, services, event schema, threads, calibration, or dependencies beyond removing non-KA2 paths. | user | These changes are unnecessary to complete the scoped cleanup. | `design/commander` | An approved hardware or interface change. |
| D-4/D-5: worker/bus merge and approximate calibration | rejected | Keep separate workers and bus handles; keep exact ordered-sample statistics. | prior-artifact | They add concurrency/numerical risk without serving the cleanup. | `design/commander` | A separately measured and approved design. |

## Migration / deprecation

**Replacement readiness:** The existing KA2 path is the replacement for the removed non-KA2 runtime: ICM42670 accel/gyro/temp and LIS2MDL magnetometer on bus 3, with current service and event contracts. No KA2 sensor behavior change is intended. The removal of platform guards and gating tests follows D-20 and must be reviewed as source/configuration; the cleanup does not claim off-target behavior is safe.

**Consumer discovery:** Repository evidence names `system/manager/process_config.py`, `sensord.main()`, the TICI sensord CPU-budget entry in `selfdrive/test/test_onroad.py`, loggerd rotation expectations in `system/loggerd/tests/test_encoder.py`, Tizi Jenkins test selection, and the LSM/MMC drivers and test cases. The scoped audit must confirm these references are removed. No runtime telemetry or out-of-repository consumer inventory is available.

**Posture:** This is a repository cleanup plan, not a fleet migration, merge, release, or deployment authorization. If an out-of-repository non-KA2 consumer is discovered before merge, pause and route a new scope decision through Admiral rather than adding compatibility code here.

**Migration path:** Retain mocked KA2 tests in the existing unit stage and remove obsolete platform-specific test assumptions in the same slice. No non-KA2 replacement daemon is designed. Generic manager/SCons build behavior and old-log decoding remain intact.

**Removal criteria:** AC-1 through AC-6 pass; the reference audit finds no stale TICI/tizi runtime, test, process, or CI expectations beyond explicitly allowed schema identifiers or unrelated release history; the historical enum names/ordinals are unchanged.

**Fallback:** If the atomic slice cannot pass its offline checks, revert the cleanup delta to the post-efficiency pre-r3 baseline without touching unrelated worktree changes. If target or platform assumptions change, stop and request an updated design rather than expanding this slice.

## Non-goals and YAGNI deferrals

| Deferred | Why safe to defer | Reopen trigger | Owner |
| -------- | ----------------- | -------------- | ----- |
| Physical KA2 board/sensor identity, startup, live cadence, and shutdown validation | D-20 confirms the deployment platform but does not identify an accessible board or authorize probing. Mocked/source checks cannot establish physical behavior. | A target owner provides an identified, authorized rig. | KA2 target owner with `build/build-management` |
| Target CPU/RSS comparison or speedup claim | No target baseline, workload, window, or threshold is established; synthetic results are not target evidence. | Agree rig, workload, baseline, measurement window, and threshold. | KA2 target owner with `build/build-management` |
| Remove historical LSM/MMC enum identifiers | A schema migration could impair old-log decoding and is outside the cleanup. | Separate schema proposal includes archive compatibility evidence. | `design/commander` and log owner |
| Change generic manager/SCons build behavior, add a sensord target, or add a dependency | Not required to remove the hardware-specific stage or retain offline tests. | Independent build requirement is evidenced and approved. | `design/commander` |
| Support non-KA2 startup or introduce new platform safety checks | D-20 confirms KA2 and removes platform checks; off-target behavior remains outside the design contract. | A future deployment target requires new intake and architecture approval. | Admiral / platform owner |
| Change KA2 sensor/event/thread/bus behavior or add another optimization | This slice preserves the current KA2 contract and approved efficiency baseline. | A separate behavior/performance requirement and parity evidence. | `design/commander` |

**Handoff:** `design/engineer` turns this milestone and acceptance table into the implementation spec. Commander packages the r4 evidence for `design-to-build`; build receives the scope only after gate approval.
