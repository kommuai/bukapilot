# Completeness cross-check — KA2-only sensord cleanup

## Outcome

**Incomplete; not ready for `build-to-review`.** The approved r3 implementation is in the final tree and the sensord-focused offline suite passes, but the required runtime startup evidence is unavailable because no KA2 target or authorized rig was named. The exact combined acceptance command also has two environment failures, and the loggerd integration test did not collect. No build manifest was assembled, and this report is a completeness assessment, not a gate verdict.

## Baseline and read-only posture

- Approved design: r3, submission `sensord-refactor-20261029-design-r3`; design manifest SHA-256 `c930ce04da797cf135f37bd3f68890d158b0bea183299d0057e8e0d08445930e`.
- Cross-stage approval: `design/verdict_design-to-build.cross-stage.json`, APPROVED; SHA-256 `de5048cd6e5d09bc5b49a39462017f20932c614d4effced47e6181ba33c17483`.
- Intake decisions: `intake/report_grilling.md`, SHA-256 `053fa85dafb522f02486f3544ace79e35665ebfc837a8b0ffeb9e99b71a2e763`.
- Approved implementation slice: `slice.sensord.ka2-platform-cleanup`; its exact allowlist is in `design/reports/implementation-spec.md`.
- No `build/manifest.json` exists. The prior r5 reports are not treated as evidence for this r3 slice.
- Run state says the hooks are unregistered. The hook did not enforce this pass's read-only boundary; I followed it directly and changed no source, tests, manifests, or other package files. This report is the only file written by this pass.

The final implementation diff is `build/artifacts/implementation-ka2-cleanup-r2.diff`, SHA-256 `e1aec7107d84768103d5f8402b6384c2e234ec81cd2faa87b34a9eba96a6ca5a`. Its hash matches the current scoped `git diff` exactly and it names the 13 approved paths (9 modified, 4 deleted). The diff excludes unrelated pre-existing worktree changes.

## Approved-decision traceability matrix

The matrix covers every resolved, deferred, rejected, and non-goal decision in the approved intake log. A decision row is proven by its listed implementation or approved deferral and focused evidence; the security column points to the shared typed applicability record. The package-level gate keys are assessed separately below.

| Decision | Changed artifact / binding | Test evidence | Security evidence | Status | Reason |
| --- | --- | --- | --- | --- | --- |
| D-1 — prior behavior boundary; superseded by D-12 | `system/sensord/sensord.py`; final diff `e1aec710…` | Sensord-focused suite, 10 passed | Typed waiver `276d2b13…` | proven | r3 explicitly supersedes non-KA2 support and retains the KA2 path. |
| D-2 — retain prior KA2 efficiency changes | `system/sensord/sensord.py`; KA2 sensor drivers remain unchanged | Polling, fixed-calculation, gyro-buffer, event and guard tests are in the 10-pass scoped run | Typed waiver `276d2b13…` | proven | The accepted KA2 implementation is retained; no target-performance claim is made. |
| D-3 — defer KA2 target startup and CPU/RSS measurements | Approved deferred-decision row in intake; `report_runtime-health-gap.md` | No hardware test was run | Typed waiver `276d2b13…` | proven as an approved deferral | Deferral is recorded with owner and reopen conditions. The separate runtime evidence key remains unproven. |
| D-4 — reject merging polling/IRQ threads or sharing bus handles | `system/sensord/sensord.py` | Sensord-focused suite, 10 passed | Typed waiver `276d2b13…` | proven | No KA2 thread or bus-ownership redesign was introduced. |
| D-5 — reject approximate online gyro calibration | `system/sensord/sensors/icm42670_gyro.py` is unchanged | `test_gyro_buffer.py` passed in the scoped run | Typed waiver `276d2b13…` | proven | The accepted calibration implementation remains unchanged. |
| D-6 — non-goal: change KA2 topology, registers, rates, payloads, timestamps, warmup, calibration, polling, errors, or shutdown | Final diff and retained KA2 driver files | Sensord-focused suite, 10 passed | Typed waiver `276d2b13…` | proven | Only platform composition and stale non-KA2 paths changed. |
| D-7 — non-goal: claim hardware startup or target performance without a target/baseline | Approved deferral and runtime gap report | No target test or benchmark claimed | Typed waiver `276d2b13…` | proven as an explicit deferral | The report does not claim physical startup, cadence, CPU, or RSS results. |
| D-8 — prefer fewer lines when behavior is equivalent; retain the 5% performance threshold | Final scoped diff `e1aec710…` | Sensord-focused suite, 10 passed | Typed waiver `276d2b13…` | proven within the approved scope | Non-KA2 paths were removed. No new target-performance conclusion is drawn. |
| D-9 — reject shared event helpers where they add lines or slow a driver | KA2 event builders unchanged | `test_event_construction.py` passed in the scoped run | Typed waiver `276d2b13…` | proven | Direct KA2 event construction remains. |
| D-10 — reject nested-dictionary `new_message` construction | KA2 event builders unchanged | `test_event_construction.py` passed in the scoped run | Typed waiver `276d2b13…` | proven | The slower nested-dictionary approach was not introduced. |
| D-11 — retain ordered tuple-list gyro calibration storage | `system/sensord/sensors/icm42670_gyro.py` is unchanged | `test_gyro_buffer.py` passed in the scoped run | Typed waiver `276d2b13…` | proven | The accepted representation remains in place. |
| D-12 — support KA2 only; remove LSM6DS3/MMC and IRQ runtime | `sensord.py`; four driver deletions in final diff | Guard and sensord-focused tests passed; scoped reference audit has no hits | Typed waiver `276d2b13…` | proven | Runtime and test references to the removed platform paths are absent in the audited scope. |
| D-13 — enable the manager process only on KA2 | `system/manager/process_config.py` | Manager-selection contract passed for both KA2 values; on-road predicate retained | Typed waiver `276d2b13…` | proven | No manager process was started by this contract test. |
| D-14 — retain historical sensor-source schema identifiers | `cereal/log.capnp` is unchanged and absent from the final diff | Schema is unchanged; no migration was attempted | Typed waiver `276d2b13…` | proven | Historical enum values were not removed or renumbered. |
| D-15 — remove dedicated Tizi stage; leave generic manager/SCons build alone | `Jenkinsfile`; generic build files unchanged | Jenkins selection assertions passed | Typed waiver `276d2b13…` | proven | The offline suite is selected by `system/sensord/` changes in `HW + Unit Tests`. |
| D-16 — keep KA2 offline tests and replace TICI hardware suite with a guard test | Sensord and fixed/event test files in final diff | Sensord-focused suite, 10 passed | Typed waiver `276d2b13…` | proven | Non-KA2-only test cases were removed; the guard test is mocked and offline. |
| D-17 — non-goal: remove historical LSM/MMC schema identifiers | `cereal/log.capnp` unchanged | No schema change | Typed waiver `276d2b13…` | proven | Historical identifiers remain available for archived-log decoding. |
| D-18 — direct non-KA2 invocation fails before realtime/I2C/thread/publication | `system/sensord/sensord.py` | Focused manager/guard contracts: 2 passed | Typed waiver `276d2b13…` | proven | Guard assertion verifies no recorded side-effect path is entered. |
| D-19 — run mocked sensord suite in generic HW + Unit stage on sensord path changes | `Jenkinsfile` | CI selection assertions passed; scoped sensord suite, 10 passed | Typed waiver `276d2b13…` | proven | No new sensord build target or dependency was added. |

**Decision-row count:** 19 proven, 0 proven solely by waiver, 0 unproven. The shared security record is an additional required build key, not a substitute for the direct implementation/test evidence in these rows.

## Acceptance criteria

| Criterion | Observed evidence | Status | Gap / owner |
| --- | --- | --- | --- |
| AC-1 — manager selection and direct-entry guard | Both focused contracts passed. The exact combined command failed in two existing manager preparation tests during modeld import under local NumPy 2.5. | unproven for the prescribed combined command | `unverifiable`: environment warning policy; test-builder/operator to supply a supported environment or a clean required run. |
| AC-2 — retained KA2 polling, fixed calculations, gyro calibration, event construction, and guard | Scoped sensord suite passed 10 tests; final diff keeps the four KA2 drivers on bus 3. | proven | No target performance result is claimed. |
| AC-3 — delete four legacy drivers, remove their runtime/tests, preserve schema | Four driver paths are deleted; scoped reference audit has no matches; `cereal/log.capnp` is unchanged. | proven | None in the audited scope. |
| AC-4 — remove TICI CPU budget and loggerd sensord lifecycle; retain rotation checks | Static source diff confirms the edits; the focused loggerd command failed during collection because `parameterized` is absent. | unproven | `unverifiable`: loggerd test-builder/CI environment must provide the declared test dependency and rerun. |
| AC-5 — remove Tizi job and select mocked suite on `system/sensord/` changes | Jenkins selection assertion passed; no generic build file changed. | proven | Actual Jenkins execution was not run. |
| AC-6 — offline suite, reference audit, diff check, no device access | Scoped suite passed; audit and diff check passed. The prescribed combined command had 2 unrelated environment failures. | unproven for the prescribed combined command | `unverifiable`: test-builder/operator to provide a clean required run; no device/I2C testing is in scope. |

**Acceptance-row count:** 3 proven, 0 proven by waiver, 3 unproven.

## Build-to-review evidence readiness

| Required key | Available evidence | Status in a gate package | Owner / reason |
| --- | --- | --- | --- |
| `approved_design_revision` | Approved cross-stage verdict exists and is hash-verified. | unproven until the typed `revision_ref` is assembled in a manifest | build-management — `missing-artifact` (no manifest). |
| `implementation` | Final-tree diff is hash-verified and matches the current 13-path scoped diff; Bob's report lists file hashes. | unproven until the typed `change_set` is assembled in a manifest | build-management / bob-the-builder — `missing-artifact` (no manifest record). |
| `tests` | Scoped pass logs exist (10 passed and 2 focused contracts passed); the prescribed combined command failed (12 passed, 1 skipped, 2 failed), and loggerd collection failed on missing `parameterized`. | unproven | test-builder — `unverifiable`; no clean package-level pass probe is available. |
| `runtime` | Health-check gap report exists; it ran no probe. | unproven; required and unwaivable | health-check and KA2 target owner — `missing-artifact`; no authorized target/rig or board identity. |
| `traceability` | This report maps all approved decisions and acceptance criteria. | unproven until build-management serializes the required typed `trace_map` in a manifest | build-management — `missing-artifact` (no manifest record). |
| `security_evidence` | `build/artifacts/security-applicability-ka2-cleanup.json`, SHA-256 `276d2b13a05a45fd8ff4146e49c92cdfd8dafaad73817d3cace412632d58f9c5`; record is `{applicable:false, reason:"no trust-boundary change - security-builder not engaged", scope:"build-to-review", decided_by:"build-management"}`. | **proven by sanctioned typed waiver** | No additional security checkpoint is required for this scoped change. |

**Gate-key count:** 0 directly proven, 1 proven by waiver, 5 unproven. The absent manifest is why the available design approval, implementation diff, test reports, and traceability matrix are not yet assembled as a gate package. Runtime remains the hard blocker even if those records are later assembled.

## Evidence reviewed

- Gatekeeper-build package-shape check: `build/evidence/build-package-shape-final.log` SHA-256 `8f3ea8978ac1c9b2564e828aac1ea0411eb6c6e54a795ef9ed24be30d7805ad6`; exit 0, `NEEDS_JUDGMENT`, zero failures, and two conditional findings. The check reports r3 revision coherence and no blocked phrases. The security conditional is covered by the registered typed applicability record. The other conditional is the missing gatekeeper-build verdict lineage; no build submission or manifest exists, so no verdict lineage is available. This is package-shape evidence only and does not change the incomplete conclusion above.
- Implementation: `build/reports/report_implementation-ka2-cleanup-r2.md` SHA-256 `bd86ceb9a7fe6bb97ed8e260d7d4c11d19ed8de32f8d3205d56e656184b14353`; final diff `build/artifacts/implementation-ka2-cleanup-r2.diff` SHA-256 `e1aec7107d84768103d5f8402b6384c2e234ec81cd2faa87b34a9eba96a6ca5a`.
- Tests: `build/reports/report_tests-ka2-cleanup.md` SHA-256 `18619d4dc1fd188885d13b6e0835e721274b30b0452362a5cd280775d67cb4b4`; focused sensord log SHA-256 `7f0f58a9db2309e673be4c59abfdc9f49cf925f1353322541be84bf73bccd7a3`; combined command log SHA-256 `4bea80272b72ec7d25052f371a17594e4f9b1cdc03a0e83d9f33c017fa1b68ee`; loggerd collection log SHA-256 `f91f12552af043e9a946e33c70eca031c00b92c446a325fb4f9425c2ba429bae`.
- Runtime: `build/reports/report_runtime-health-gap.md` SHA-256 `8c09633c752f450bd52df6c4a2763b4a1291b9079566e0ff74957afb5494f36d`.
- Scoped reference audit: `build/evidence/test-reference-audit-ka2-cleanup.log` SHA-256 `e3e5883f24aa839237e20c3e2d244b2d872989a8110439303e2f5f151aa9a7f9`; no matches.
- Whitespace check: `build/evidence/diff-check-ka2-cleanup.log` SHA-256 `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`; exit 0.

The combined test command's two failures were `TestManager.test_manager_prepare` and `TestManager.test_set_params_with_default_value`; both encountered a NumPy 2.5 deprecation warning promoted to error during modeld import. A diagnostic rerun filtering that exact warning passed 14 tests with 1 skipped, but it does not replace the failed approved command. The loggerd test did not reach its TICI skip because collection stopped at `ModuleNotFoundError: parameterized`.

The manager preparation tests generated four boot logs under `/data/media/0/realdata/boot/` outside the workspace. They were left untouched and are listed in the test-builder report. No sensor device or I2C access occurred.

## Gaps and next actions

1. **Major, runtime — `missing-artifact`:** KA2 target owner with build-management must name the board/sensor identity, authorized rig, baseline, workload, measurement window, and threshold. Health-check can then produce the required startup/runtime probe. Until then, `build-to-review` cannot pass.
2. **Major, tests — `unverifiable`:** test-builder/operator must obtain a clean run of the approved combined command in a supported environment; the current broad run and loggerd test are not clean evidence.
3. **Major, package assembly — `missing-artifact`:** build-management must create the schema-2 manifest with typed `revision_ref`, current `change_set`, passing probe records, the typed security waiver, and the `trace_map`. The manifest must not claim runtime or test success absent evidence.

**Completeness conclusion:** the scoped implementation and its sensord-focused tests are present and verified. The full build package remains incomplete; no `build-to-review` verdict is requested or implied by this report.
