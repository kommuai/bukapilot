---
submission_id: sensord-refactor-20261029-design-r4
revision: r4
---

# Design package — KA2-only sensord cleanup

**Boundary:** design-to-build  
**Run:** sensord-refactor-20261029  
**Submission:** sensord-refactor-20261029-design-r4  
**Revision:** r4

## Outcome and scope

Make `system/sensord` KA2-only for the confirmed KA2 deployment, remove the obsolete TICI/tizi sensor runtime and hardware-specific CI, and retain the previously accepted efficiency improvements. Preserve the KA2 ICM42670 accelerometer, gyro, and temperature sensors and LIS2MDL magnetometer on I2C bus 3, with their existing startup order, polling, services, event data, calibration, error behavior, pacing, and shutdown.

Apply user decision D-20 to process entry. Remove the `KA2` guard from `sensord.main()` and remove the explicit `enabled=KA2` manager override so `sensord` uses the process type's unconditional enabled default. Keep the existing `only_onroad` predicate. Delete the r3 guard-only `system/sensord/tests/test_sensord.py` and the manager test whose only purpose was to assert platform gating. Do not add replacement platform tests. On the confirmed KA2 deployment, the first action remains realtime setup, followed by the existing four sensor constructors in their current order.

D-20 supersedes D-13 and D-18, and the guard-test portion of D-16. Non-KA2 runtime behavior is outside this design contract and is not guarded; an off-target manager or direct invocation may reach the fixed bus-3 path. No non-KA2 safety or startup claim is made.

Keep historical `SensorSource` identifiers and ordinals `lsm6ds3 @5`, `lsm6ds3trc @10`, and `mmc5603nj @11` for archived-log decoding. Keep `system/manager/build.py`, root `SConstruct`, other generic build definitions, and `cereal/log.capnp` unchanged. Add no runtime, framework, dependency, sensord-specific build target, or schema migration. Keep all unrelated worktree modifications untouched.

## Design decisions and evidence

- **Requirements:** the r4 brief carries D-20, removes stale guard/gating requirements, and retains the KA2 event, timing, calibration, and operational limits.
- **Architecture:** one KA2-only runtime composition with the four existing drivers on bus 3; no TICI/tizi branches, LSM/MMC runtime imports, IRQ handling, manager platform filter, or main-entry guard.
- **Interface IF-1:** managed KA2 on-road startup uses the unconditional enabled default and the existing `only_onroad` predicate. Direct KA2 entry uses the existing `sensord.main()` path. Existing KA2 cereal service contracts are unchanged; no non-KA2 entry behavior is specified.
- **Security:** security-builder assessed that r4 moves or adds no trust boundary. It records the unguarded non-KA2 fixed-bus path as an explicit out-of-target risk, not as a new guard requirement.
- **Plan:** one atomic cleanup slice with AC-1 through AC-6. Source/configuration review covers the removed platform checks; the retained mocked suite proves KA2 behavior. The deleted guard/gating tests have no replacements.
- **Implementation spec:** exact file allowlist and acceptance commands are in `reports/implementation-spec.md`; those commands are instructions for build and have not been run during design authoring.
- **Stack:** r4 adds no runtime, framework, dependency, or generic build change. The prior project scan was incomplete and establishes no repository-wide stack slug; the unchanged stack decision is in `reports/report_stack-lock.md`.
- **Taste:** the immutable no-profile snapshot remains unchanged at global/project revision 0. The r4 read-only revalidation found both stores absent and the effective profile empty.

The accepted efficiency baseline remains in scope: polling uses `Ratekeeper` on every loop iteration, ICM fixed calculations and driver-local event construction remain, and gyro calibration keeps 300 ordered XYZ tuple samples with the current `fmean`/`pstdev` behavior. Prior local synthetic evidence in `build/reports/report_benchmark-simplification-r6.md` (SHA-256 `b62b7257428861caafe0772d22914c817dabbff83275af67417e821a98e4896b`) recorded about 99.04% lower CPU in the invalid-polling window, 1.98% faster ICM accelerometer events, 3.35% faster ICM gyro events, no measurable calibration change, and two fewer lines in the compared production files. Those measurements used mocked I/O and are not target-device results; the KA2 event-only estimate does not include bus reads, publishing, or full daemon cost.

## Delivery and acceptance

The one atomic slice removes non-KA2 runtime drivers and test cases; removes the r3 `KA2` guard and explicit manager enablement argument; deletes the guard-only sensord test file and manager platform-gating-only assertion; removes stale TICI CPU-budget/loggerd expectations and the Tizi LSM/MMC hardware stage; and runs the remaining mocked sensord suite in the existing `HW + Unit Tests` stage when `system/sensord/` changes.

The plan and spec define six acceptance criteria:

1. Review manager configuration for the default enabled state and unchanged `only_onroad`; review `sensord.main()` for no platform guard and the existing KA2 startup order.
2. Run the retained mocked KA2 sensord and generic manager coverage and review retained sensor/event/calibration/pacing/error/shutdown behavior. Do not claim target performance.
3. Remove the four non-KA2 drivers, their test cases, the obsolete guard-only `test_sensord.py`, and the manager gating-only test, with no replacement platform tests; retain all historical source enum ordinals.
4. Remove the stale TICI sensord CPU budget and TICI/tizi loggerd sensord start/stop assumptions while retaining loggerd rotation checks and KA2 process coverage.
5. Remove the dedicated Tizi sensord hardware stage and keep the mocked suite in the generic unit stage with the `system/sensord/` path condition. Leave generic build definitions and schema unchanged.
6. Run the scoped source-reference audit and `git diff --check`; no device or I2C access.

No implementation, unit-test, or hardware command was run in this design step. The exact allowlist, proposed test command, audit command, rollback, and acceptance owners are in `reports/implementation-spec.md` and `reports/delivery-plan.md`.

## Artifact map

| Artifact | Purpose | SHA-256 |
| --- | --- | --- |
| `../intake/report_grilling.md` | Admiral decision register including D-20 and supersession | `a51313a34983497a27a5c9ddc5cdc0c9f82d725f6c8cdf382ab2dae2c75edd4e` |
| `../intake/intake-brief.md` | Confirmed r4 scope and constraints | `c2e026d40ff71df11fd3d0277704255c51df061bbf3ffc5c7b81187513b9dc75` |
| `reports/requirements-brief.md` | Sourced, confidence-tiered r4 requirements | `6f50367f5749062cb8c7928d9b279626b04dff4f6fbf8d6265980e8ae3ff7037` |
| `reports/report_taste-snapshot-r4.md` | Immediate r4 profile revalidation | `26038f53ef41b626c89c0a6fe80840a422bfd7c76a22cc3f3e8469e6016bdb61` |
| `artifacts/taste_snapshot.json` | Immutable no-profile snapshot; reused unchanged | `4b29408d876a909bc0a1eec35f78abf384a72276b6f6972da35e2f99a6244a8f` |
| `reports/report_architecture.md` | KA2-only components, lifecycle, invariants, risks | `a1a64320fd8a6b8995d08c5ba86d8c7a83f2f00630bdda1da0cf1f6981475b16` |
| `reports/report_interfaces.md` | IF-1 process lifecycle and unchanged KA2 event contracts | `481ff241761b6b35d83837e74bbee78c4893b6800f03ba217b322fd9ac8b1763` |
| `reports/report_security-seed.md` | Security assessment for architecture r4 | `26b6e0e98da110dbb4cc31b6373c9bcef7af852c0c93a877a4a4cf85288ee642` |
| `reports/delivery-plan.md` | One slice and AC-1 through AC-6 | `4937e75129c4920786e4cb73ed0f17c927e33c598a8c930da3c6e5885c6f17ee` |
| `reports/implementation-spec.md` | Exact allowlist, proof plan, rollback, limits | `04d2f77dc68609be00d2f96b0f332643c67c4b69e890b941642da3bf791ed420` |
| `reports/report_stack-lock.md` | Typed no-new-runtime/framework decision | `709304c891b3f30a8bac419aa6b80dce9b73632a7aa1ed70228ddf6df8a9062f` |
| `packages/r3-prior-design-records-complete.zip` | Archived r3 manifest, phase/cross-stage gate records, handoff, and superseded Taste reports | `22f432adb5f06b115e166b7a0d44d393773b00c562a0a503ab2b6c50300cedc3` |

All evidence-key artifacts and hashes are bound in `manifest.json`; this package report is also hashed in that manifest.

## Open risks and limits

- D-20 removes manager and direct-entry platform checks. An off-target non-KA2 process may reach bus 3; this is explicitly out of scope and not claimed safe. A future non-KA2 deployment requires a new intake/design decision.
- No physical KA2 board, sensor identity, or authorized target rig is established. Startup, sensor identity, live cadence, physical accuracy, shutdown, and target CPU/RSS remain unverified. D-20 confirms the platform target but does not authorize hardware probing.
- No representative archived log fixture was supplied. Preserving historical schema identifiers is a source-level compatibility condition; end-to-end old-log decoding remains optional follow-up.
- The project runtime scan exceeded its byte limit, and the local Python 3.12.3 is below the harness minimum of 3.13. These are recorded environment limits; this design does not change the product environment or infer a repository-wide runtime slug.

## Phase handoff

This r4 package submits the user-confirmed KA2-only design amendment to `gatekeeper-design` at `design-to-build`. Build may begin only after the r4 phase verdict and Admiral cross-stage approval. This package does not authorize merge, release, device validation, or fleet deployment.
