# Build completeness report — `system/sensord` efficiency refactor (r2)

**Run:** `sensord-refactor-20261029`  
**Approved design:** `sensord-refactor-20261029-design-r1` (`r1`)  
**Build submission:** `sensord-refactor-20261029-build-r1`  
**Outcome:** **INCOMPLETE — do not submit to `gatekeeper-build`**

This is a read-only recheck of the prior report. The previously missing implementation diff is now present, hashed, allowlist-verified, and accepted by `git apply --stat`; the main Git index remains unchanged. Offline implementation and test evidence remains consistent. Runtime is still unverified, and no final `build/manifest.json` is assembled. The package therefore remains incomplete.

## Baseline and checks

- Approved design manifest: `design/manifest.json`, SHA-256 `4493faf7d756f49974e448973610952be6c5ca370c6cda60cbbff1f8a80acc34`.
- Valid `design-to-build` cross-stage verdict: `design/verdict_design-to-build.cross-stage.json`, SHA-256 `3c0ff4fcbb9a425f6301d2e394a35af28243378f4581e13343ac79455db62b2a`.
- Approved delivery plan with D-1 through D-8 and AC-1 through AC-4: `design/reports/delivery-plan.md`, SHA-256 `22ebe47a1bb82b6d707b0bb57a1349ab4ed814bd0d5d2ffaf8cd2c0ec57b6f14`.
- Updated implementation report hash verified: `13e72731d4bea2cd736950427b8828b9676cf0c368124ba0fec675e5c9a1b8de`.
- Complete implementation diff hash verified: `e012f6491f9e32c698f9694dbdfe3cdbad3a0a0a66306f807a27f89a129c70e0`.
- Verification log hash verified: `2df9caf80cfa9ad6b4c16bcdbda9cc8b9a190688187b94f47c671729cec0360b`. The log records an exact 14-path allowlist match, no unrelated paths, and an unchanged main index tree with no staged paths.
- Independently parsed the diff headers: exactly 14 paths, matching the approved 10 production files and four test files. `git apply --stat` exited successfully and reported 551 insertions / 53 deletions across those files. This pass did not apply the diff.
- Existing offline pytest result is unchanged: 12 passed under Python 3.12.3 / pytest 9.0.2. Test log SHA-256 `edb171c27745cc52faeedb8a5ddd12d388028982fb0e217b563cbe59a8ea759f`.
- Runtime report remains `UNVERIFIED`: no target or authorization; no startup or device probe ran.
- The prior r1 completeness report remains unchanged at SHA-256 `a14a60b6b37261f4ab0b091a163626416e7b524d3e241e0d6f2f603159795670`.
- The current run checkpoint is revision 34. Diff, verification log, updated implementation report, test report/log, runtime report, and r1 completeness report hashes are registered in the run state. Their on-disk hashes matched.
- `build/manifest.json` does not exist. No final manifest or build-to-review submission was made. The read-only run record remained active; this recheck wrote only this report.

## Artifact index

The diff and verification record below close the missing-diff issue from r1. Source/test hashes listed here were checked against the updated implementation report and the current worktree.

| ID | Path | SHA-256 |
| --- | --- | --- |
| A1 | `system/sensord/sensord.py` | `45b0fa7635a541cf2024b1ceda36774fe5faea6fbc1aa5abb9df6e05cf4af35e` |
| A2 | `system/sensord/sensors/icm42670_accel.py` | `920f5000dae46ffdf33b2b758fd7b55f0484f1f3d626433592823ad1b98a8b6a` |
| A3 | `system/sensord/sensors/lsm6ds3_accel.py` | `2ba05e9e360e241403a5934d5de9059e350ba4e5ea151423ef6e9719fddc97ab` |
| A4 | `system/sensord/sensors/lsm6ds3_gyro.py` | `08be99246e9a0b1a0beab4ec0ac31810ea74a3d10e97c416ffd13723dfb7ed6f` |
| A5 | `system/sensord/sensors/mmc5603nj_magn.py` | `b983013d00e9df832729cd632ae67a76013c8b5fb56e5cdfa4516a6c593892e2` |
| A6 | `system/sensord/sensors/icm42670_gyro.py` | `431be11360602a6240804d1b4c56cbac194a69d8e9b23d530756953638b45f12` |
| A7 | `system/sensord/sensors/sensor_event.py` | `20cf1e74a0fefade68c3a39771e3a45fb975cf25fc7b4601265a009e38ba79d3` |
| A8 | `system/sensord/sensors/icm42670_temp.py` | `d997fd3ac8fcde85b62f0562278256f803dcc8f894a462096879b854178f9a88` |
| A9 | `system/sensord/sensors/lsm6ds3_temp.py` | `9bc8a9120c8985bfb357308c770dd03d4fb9d18e26a2adab4b4e2a9371f1e495` |
| A10 | `system/sensord/sensors/lis2mdl_magn.py` | `133ee6c43f2892a35c1087cdf6e90201e7f4e1891d7b56de2adebd8808c773df` |
| T1 | `system/sensord/tests/test_polling_pacing.py` | `5fb7f96f663141e1549c5e102fd03fbc2a239ff179121166c231e61089f88323` |
| T2 | `system/sensord/tests/test_fixed_calculations.py` | `c24eb1b7907a38f0fb7fd09475839cd3dfdd76b85d2b36e8caa33c9db7b45a76` |
| T3 | `system/sensord/tests/test_gyro_buffer.py` | `3d8ec5f6cdc4b56faf2836d9e3dc9c01f82391aab4a2ad3b1c8e53041fa6353c` |
| T4 | `system/sensord/tests/test_event_construction.py` | `ae2fd9e4a8a87cd0365877465f055c1f3dee58c8a4a1c664029277c30080c906` |
| DIF | `build/artifacts/implementation.diff` | `e012f6491f9e32c698f9694dbdfe3cdbad3a0a0a66306f807a27f89a129c70e0` |
| DIFV | `build/evidence/implementation-diff-verification.log` | `2df9caf80cfa9ad6b4c16bcdbda9cc8b9a190688187b94f47c671729cec0360b` |
| I1 | `build/reports/report_implementation.md` | `13e72731d4bea2cd736950427b8828b9676cf0c368124ba0fec675e5c9a1b8de` |
| TLOG | `build/evidence/tests-sensord-offline.log` | `edb171c27745cc52faeedb8a5ddd12d388028982fb0e217b563cbe59a8ea759f` |
| TR | `build/reports/report_tests.md` | `dccc0089bcec9eafb69df4aaf2b91908196769b961eb3b79d3907db49f3d1914` |
| RR | `build/reports/report_runtime.md` | `f9d4a9ee0b398983ae69b4f7d039e1bf0acd8c1445da702de07200a9e843f2a8` |

## Approved decision matrix

The status is about whether the decision is supported by current implementation and verification evidence. Package-level binding is judged separately below.

| Decision | Changed artifacts | Test / verification evidence | Security evidence | Status | Reason if unproven |
| --- | --- | --- | --- | --- | --- |
| D-1 preserve rates, messages, timestamps, warmup, calibration, errors, IRQ, shutdown | A1–A10; DIF | T1–T4; TLOG; DIFV. Offline checks cover pacing, fixed numeric output, calibration, and event contracts. | Approved security seed says no trust-boundary movement; build waiver remains unbound. | **unproven** | `missing-artifact`: real KA2/TICI behavior, including device/IRQ/shutdown integration, lacks an authorized runtime target and smoke. Owner: `build/health-check`. |
| D-2 bound work to the four approved efficiency slices | A1–A10; DIF | T1–T4; TLOG; exact 14-path allowlist check DIFV. | No trust-boundary movement per approved seed. | **proven** | — |
| D-3 defer hardware validation and avoid a quantified CPU/RSS claim | No hardware change; RR; approved deferral in delivery plan. | RR states no probe; no measured target-performance claim was made. | No trust-boundary movement per approved seed. | **proven** | — |
| D-4 retain worker, IRQ, and bus topology | A1–A10; DIF contains only approved source/test paths; no topology edits. | DIFV allowlist; scoped diff review. | No trust-boundary movement per approved seed. | **proven** | — |
| D-5 preserve `fmean`, population `pstdev`, sample order, thresholds, classification, and bias | A6; DIF | T3; TLOG verifies signed-short endpoints, ordered axis values, statistics, and selected bias. | No trust-boundary movement per approved seed. | **proven** | — |
| D-6 make no schema, service, sensor, register, bus, runtime, framework, or dependency change | A1–A10; DIF has no paths outside approved `system/sensord` source/tests. | DIFV allowlist; T1–T4; AC-4 scope check. | No trust-boundary movement per approved seed. | **proven** | — |
| D-7 stop before rollout and make no measured speedup claim | No rollout or hardware change; RR. | RR confirms no device access or runtime probe; no CPU/RSS claim exists. | No trust-boundary movement per approved seed. | **proven** | — |
| D-8 route event construction only where defaults and fields remain exact | A7–A9; DIF | T4; TLOG checks ICM42670 temp, LSM6DS3 temp, and LIS2MDL payloads, defaults, timestamp behavior, and serialized values. | No trust-boundary movement per approved seed. | **proven** | — |

## Acceptance criteria

| Acceptance | Result | Evidence | Limit |
| --- | --- | --- | --- |
| AC-1 polling pacing and invalid warmup silence | **Proven offline** | T1; TLOG; A1 | No real sensor or daemon startup exercised |
| AC-2 fixed calculation and calibration parity | **Proven offline** | T2, T3; TLOG; A2–A6 | Deterministic fixtures; no performance measurement |
| AC-3 event metadata, payload, status, and defaults | **Proven offline** | T4; TLOG; A7–A9 | Covers the three approved helper drivers |
| AC-4 approved scope and diff check | **Proven for submitted source/test diff** | DIF, DIFV, I1; 14-path exact allowlist; no unrelated path; index unchanged | Does not prove target runtime behavior |

## Build-to-review package evidence matrix

| Required key / artifact | Observed evidence | Status | Reason | Owner / next action |
| --- | --- | --- | --- | --- |
| `approved_design_revision` | Approved r1 verdict is available and hash-verified, but no manifest binds it. | **unproven** | `unbound-claim` | `build/build-management`: put the approved revision reference in the final manifest. |
| `implementation` | DIF and its verification evidence are now complete; file hashes are checked. There is no final manifest `change_set` record. | **unproven** | `unbound-claim` | `build/build-management`: bind DIF and all changed-file hashes in the final manifest. **The prior missing-diff gap is closed.** |
| `tests` | TLOG is hash-verified; TR carries the passing typed probe and source inputs, but no manifest references them. | **unproven** | `unbound-claim` | `build/build-management`: bind the test probe and log hash in the final manifest. |
| `runtime` | RR states no probe/log exists; no target or authorization was named. | **unproven** | `missing-artifact` | `build/health-check`: requires an authorized safe KA2/TICI target before startup/dependency smoke. No fallback exists. |
| `traceability` | This report supplies the D/AC evidence matrix; no final manifest `trace_map` is assembled. | **unproven** | `unbound-claim` | `build/build-management`: carry the trace map into the final manifest after package assembly. |
| `security_evidence` | Approved seed says no trust boundary moved. The sanctioned typed waiver is not in a manifest. | **unproven** | `missing-artifact` | `build/build-management`: include the typed applicability record with reason `no trust-boundary change - security-builder not engaged`. |
| `build/manifest.json` | Absent; no package has been assembled or submitted. | **unproven** | `missing-artifact` | `build/build-management`: assemble only when runtime evidence exists. |

## Remaining gaps

1. **Runtime — hard blocker.** The handoff identifies no safe target or device-access authorization. The approved plan deferred hardware checks. No sensor process was started and no `/dev/i2c-*` node was accessed. A target owner must identify and authorize a safe KA2/TICI rig; then `build/health-check` can run the startup/runtime smoke and produce the required evidence.
2. **Final package binding — build-management.** The implementation diff gap is closed, but no manifest exists. Build-management still must bind the approved design revision, verified diff/change set, test probe/log, runtime probe/log, trace map, and typed security waiver after runtime evidence is available.

**Completeness count:** decision rows: 7 proven, 1 unproven; offline AC rows: 4 proven for scoped offline checks; final build-to-review keys: 0 fully bound because no manifest exists, 6 unproven. The package remains incomplete and this report is not a gate verdict.
