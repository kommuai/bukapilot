# Completeness cross-check — KA2-only sensord cleanup r4

## Outcome

**Incomplete; build gate NOT SUBMITTED.** The r4 implementation matches the saved scoped diff. The retained sensord-only suite passes, but the required primary suite has two environment-triggered failures, the loggerd test cannot collect, and runtime evidence is absent. No build manifest or gate verdict was created.

## Baseline and scope

- Approved design: r4, submission `sensord-refactor-20261029-design-r4`, cross-stage APPROVED, verdict ID `758dae8eff09975000a76ad0f5850194eb9ec97afbd33a4ad8a0ca927908ae0d`.
- Intake decision register: `intake/report_grilling.md`, SHA-256 `a51313a34983497a27a5c9ddc5cdc0c9f82d725f6c8cdf382ab2dae2c75edd4e`. D-20 supersedes D-13, D-18 and the guard-test portion of D-16.
- Approved spec: `design/reports/implementation-spec.md`, SHA-256 `04d2f77dc68609be00d2f96b0f332643c67c4b69e890b941642da3bf791ed420`.
- Implementation diff: `build/artifacts/implementation-ka2-cleanup-r4.diff`, SHA-256 `6c5736af393cea0f0cbaeb5bd9d73538dfb1afdfeeb4fc2e912903e84600d9d4`. A fresh diff of the approved allowlist matched byte-for-byte: 12 net paths, 17 insertions and 848 deletions. `system/manager/test/test_manager.py` has no net diff because the r3 gating test was removed back to baseline.
- The final code retains realtime setup, sensor order, bus 3, polling and shutdown on KA2. The manager uses default `enabled=True` while retaining `only_onroad`. The four non-KA2 drivers and obsolete guard-only test file are absent; no replacement platform test was added.
- The unrelated worktree dirt named in the handoff is outside the saved implementation diff and was left untouched. This cross-check changed no source, test or manifest files; it wrote only this report and its audit log.

## Acceptance matrix

| Criterion | Evidence | Status | Gap / owner |
| --- | --- | --- | --- |
| AC-1 — default-enabled manager, `only_onroad`, no platform guard, same KA2 startup order | `build/evidence/implementation-r4-ka2-equivalence.log` (SHA-256 `ffb38911f469068e1e6593002d3c5d199fedcac4912f359e8694725d196ff504`); static source assertions pass | proven | Source-level equivalence only; no device access. |
| AC-2 — primary offline suite passes and retained KA2 behavior remains | `tests-ka2-cleanup-r4.log`: 2 failed, 10 passed, 1 skipped; `tests-sensord-retained-r4.log`: 8 passed; test report SHA-256 `645f1485b31478052b08fdc0f6558ae8494e6a55198cf995e6819305e2519272` | unproven | `missing-artifact`: no passing primary result. Failures occur importing modeld when NumPy 2.5 promotes an existing deprecation warning to error. Owner: test-builder/operator. |
| AC-3 — obsolete drivers/tests removed; historical schema preserved | Saved diff; `completeness-r4-audit.log` has zero scoped stale-reference matches; `cereal/log.capnp` is unchanged | proven | None. |
| AC-4 — TICI CPU budget/lifecycle assumptions removed, loggerd rotation coverage retained, loggerd test passes | Static diff/audit confirm removal and retained rotation assertions; `loggerd-test-ka2-cleanup-r4.log` reports missing `parameterized` | unproven | `unverifiable`: test could not collect. Owner: test-builder/operator; rerun in an approved environment with the declared dependency. |
| AC-5 — Tizi job removed; mocked suite selected for sensord path changes; generic build unchanged | `build/evidence/implementation-ci-audit-ka2-cleanup.log` (SHA-256 `b4a5f953b272d93e720ff743bbf6787bb0b6d0dd5195aa8d19236f1583470bf2`); selection assertions pass; `SConstruct` and `system/manager/build.py` unchanged | proven | Jenkins itself was not run here. |
| AC-6 — focused offline checks, scoped audit and diff check pass; no device access | Sensord-only log: 8 passed; audit: 10 checks passed; `git diff --check` exit 0; no hardware/I2C probe | proven | This does not replace AC-2's required suite or AC-4's loggerd test. |

## Build-to-review keys

| Key | Status | Evidence / reason |
| --- | --- | --- |
| `approved_design_revision` | unproven | Approved verdict exists; no typed `revision_ref` is assembled because no manifest exists. |
| `implementation` | unproven | Diff and implementation report are hash-verified; no typed `change_set` manifest record exists. |
| `tests` | unproven | Primary run failed 2/10/1; supplemental sensord suite passed 8; loggerd collection failed. No passing required test probe. |
| `runtime` | unproven | `build/reports/report_runtime-r4.md` (SHA-256 `3d494283060721db164b9671f1ea5b7e61d2bd4e2e592a860989edf175a28610`) records no authorized target, no probe and no runtime log. Runtime is unwaivable. |
| `traceability` | unproven | This report records AC coverage; no typed `trace_map` is assembled without a manifest. |
| `security_evidence` | proven by sanctioned waiver | Typed applicability record `build/artifacts/security-applicability-ka2-cleanup.json`, SHA-256 `276d2b13a05a45fd8ff4146e49c92cdfd8dafaad73817d3cace412632d58f9c5`; sanctioned no-trust-boundary-change reason. |

Count: 0 directly proven, 1 proven by waiver, 5 unproven. The package does not meet the build-to-review completeness threshold.

## Open gaps and next actions

1. **Major — tests:** rerun the exact primary command in a compatible environment. Current Python 3.12.3/pytest 9.0.2 run failed on the NumPy 2.5 warning; isolated Python 3.13.15 has no pytest or NumPy. No dependency was installed. Also rerun loggerd tests once `parameterized` is available. Owner: test-builder/operator.
2. **Major — runtime:** the KA2 owner must identify and authorize a board/sensor and supply baseline, workload, measurement window and threshold. No hardware/I2C access is authorized in this slice. Owner: KA2 target owner with build-management.
3. **Major — package assembly:** after tests and runtime evidence exist, assemble the schema-2 manifest and run both required self-checks before requesting the build gate. Owner: build-management.

D-20 leaves non-KA2 invocation unguarded and outside the design contract; no off-target safety claim is made. Keep the run open. **No build-to-review submission or verdict was made.**
