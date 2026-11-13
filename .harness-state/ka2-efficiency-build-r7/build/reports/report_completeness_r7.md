# Completeness report — r7 MPP copy fallback

**Run:** `loggerd-ka2-efficiency-20261029`  
**Cross-check revision:** 129  
**Candidate base:** `103d166829b3be04d735005ee83c24bd119764e2` in detached worktree `/tmp/ka2-efficiency-r7`  
**Disposition:** incomplete; do not submit to `build-to-review`.

## Approved baseline

The approved r7 design boundary is the cross-stage verdict `design/verdict_design-to-build-r7.cross-stage.json`, SHA-256 `19ef9bfb4cf096dd71df3048ca8c4772633930593cd292e0e719b0e013e00f95`, verdict id `e62e1310cdca4c16e9ef1d30596d12059ea3a8e0d219cf965d269584ced38bb5`. The implementation spec is `design/reports/implementation-spec_r7.md`, SHA-256 `a7f06c00f8c671aa8c6a980c75e1abe1ae6b62f11df8db5bb66651033bfcb347`; the acceptance table is in `design/reports/r7/delivery-plan.md`, SHA-256 `c18d0370ac40cdb4f1844b23c2b7dbc38591bdb89a76f8356ebb4666d95c4d34`. Approved user decisions D-22 through D-25 are in `intake/report_grilling.md`, SHA-256 `f4de6dc6f661c758f6327c2de1995f702b046b063c9b2572a3a05cfa4aa369bc`.

The r7 security seed (`design/artifacts/r7/security_seed.json`, SHA-256 `1329311510eeaa61a6eb8a954dc368af3f06c3e2b5b44205d4c6e799e32013b4`) identifies no new or moved trust boundary and retains DMA lifetime, output integrity, and resource risks. It does not itself provide the build-level `security_evidence` applicability record.

## Evidence inspected

- G0 preflight: `build/evidence/g0-preflight.md`, SHA-256 `4f2d556b23c4bb2549885ed2c217cfc15eb2e09ac760036f01c3f9a507bb1547`. It proves the pinned detached baseline and records target identity, idle observations, initial resource counters, and MPP-library ambiguity. It found `CmaFree=0` and `nr_free_cma=0`, with userspace MPP readiness unproven; it marks the hardware A/B unavailable.
- Implementation report: `build/reports/report_implementation.md`, SHA-256 `db9757c4178f1b17cd2e4558e3bf630a9eccf6eed8a98f0a9066fb66175d43f8`.
- Candidate diff: `build/evidence/impl.diff`, SHA-256 `cdf2a3721a82e686de9f6ee6ee26cef923cd2f2d42c4763368eeb9c86ad39396`.
- Focused test report/log/probe: `build/reports/report_tests_r7.md` SHA-256 `a1ca76d29679e38f5e7d87946dbdb2c66afe89bccef9ccb2676793bf455918f2`; `build/evidence/tests-pytest.log` SHA-256 `5475017575d3243b8b6bf21300c9407ed97553550e9f764538146db7eac5108e`; `build/artifacts/tests-probe.json` SHA-256 `93f995f5e4ed2f8b63ab023f737992d1babb4b9d1dba0b3e3dfe79e4624421b5`. The command passed one pytest item and exercises the shared pure helper with fake callbacks.
- Runtime health report: `build/reports/report_runtime_health_r7.md`, SHA-256 `f1370d385a2240844ac8e4b1ea7830fcac210d3c4ac3e466331231ca539e168d`. It records runtime unavailable, with no startup smoke, runtime log, or typed runtime probe.
- The final build logs and read-only binary inspection are registered under `build/evidence/`. The candidate and merged-root overlay build, ELF identities, and loader resolution are in `report_implementation.md` and `impl-binary-inspection.log`; neither binary was launched.

At this cross-check, the run's registered evidence hashes match the files above. I also rehashed all five candidate source/test files against the implementation report and the typed test probe; each current file matches. The candidate worktree remains detached at the pinned base with only the five expected changed paths. The dirty `/data/openpilot` checkout and its supplementary merged-root overlay are not treated as the candidate.

A scoped read-only guard was active for this pass and allowed writes only under this run's save path. No code, tests, build outputs, or package files were changed here; this report is the only output of the cross-check.

## Acceptance traceability

The design has five acceptance criteria plus a package-level runtime requirement. A row is proven only when the full accepted condition has revision-matched evidence; partial evidence is described in the row but does not upgrade its status.

| decision_id | changed_artifact | test_evidence | security_evidence | status | reason |
| --- | --- | --- | --- | --- | --- |
| AC-1 | `system/loggerd/encoder/mpp_encoder.cc` (SHA-256 `c98b128bd2d7a14177ac3ed34194001e7277c117d98ed5f8c743ef1ab3340b48`); candidate build and G0 records are present. | `build/evidence/tests-pytest.log` SHA-256 `5475017575d3243b8b6bf21300c9407ed97553550e9f764538146db7eac5108e`, selector `test_mpp_encoder_private_input_helper`; it does not exercise a hardware arm or its Params/IPC/log roots. | `design/artifacts/r7/security_seed.json` SHA-256 `1329311510eeaa61a6eb8a954dc368af3f06c3e2b5b44205d4c6e799e32013b4`; build-level applicability record is absent. | unproven | `missing-artifact` — G0 is complete, but the accepted target-idle/isolated-arm proof cannot be completed while the authorized hardware A/B is unavailable. Owner: build-management / KA2 test lead. |
| AC-2 | `system/loggerd/encoder/mpp_encoder.cc` SHA-256 `c98b128bd2d7a14177ac3ed34194001e7277c117d98ed5f8c743ef1ab3340b48`; `system/loggerd/encoder/mpp_encoder.h` `c6893a6df9ae79c55e90897d96a1ad72bc3fc648759a123114f2ec21140265a7`; `system/loggerd/encoder/mpp_encoder_input.h` `7c05efe9159fdd0dcb9e30a5540941b2c22fc7d83413b4f58011a3d2d511953f`; `system/loggerd/tests/test_mpp_encoder.cc` `68d6ad08fa4a8b44df103fca82b701c97837a2560861f13b00fb97a227544a12`; `system/loggerd/tests/test_mpp_encoder.py` `155ce3a252319a4baec661549f4a55472b820c744ff182a4819952d1c8280b15`. | `build/evidence/tests-pytest.log` SHA-256 `5475017575d3243b8b6bf21300c9407ed97553550e9f764538146db7eac5108e`, selector `test_mpp_encoder_private_input_helper`, and typed probe `build/artifacts/tests-probe.json` SHA-256 `93f995f5e4ed2f8b63ab023f737992d1babb4b9d1dba0b3e3dfe79e4624421b5`. It proves callback order, same-frame copy-submit, failure no-submit, import success, and latch through the shared reset helper. The test report explicitly says it does not call actual `encoder_open()`/`encoder_close()` or MPP. | `design/artifacts/r7/security_seed.json` SHA-256 `1329311510eeaa61a6eb8a954dc368af3f06c3e2b5b44205d4c6e799e32013b4`; build-level applicability record is absent. | unproven | `missing-artifact` — production MPP encoding, actual recovery/open-close behavior, and imported-buffer lifetime are not exercised. Owner: encoder implementation lead. |
| AC-3 | `system/loggerd/encoder/mpp_encoder.cc` SHA-256 `c98b128bd2d7a14177ac3ed34194001e7277c117d98ed5f8c743ef1ab3340b48` and `system/loggerd/encoder/mpp_encoder.h` `c6893a6df9ae79c55e90897d96a1ad72bc3fc648759a123114f2ec21140265a7` are present, but no route, media, or paired-arm output artifact exists. | `build/evidence/tests-pytest.log` SHA-256 `5475017575d3243b8b6bf21300c9407ed97553550e9f764538146db7eac5108e`, selector `test_mpp_encoder_private_input_helper`, is not output/compatibility evidence; there is no copy-vs-import run, decode, qRoad control, frame/loss ledger, timestamp, segment, audio, or route/file comparison. | `design/artifacts/r7/security_seed.json` SHA-256 `1329311510eeaa61a6eb8a954dc368af3f06c3e2b5b44205d4c6e799e32013b4`; build-level applicability record is absent. | unproven | `missing-artifact` — the hardware A/B was not run. Owner: KA2 test lead after a fresh G0 readiness pass. |
| AC-4 | `system/loggerd/encoder/mpp_encoder.cc` SHA-256 `c98b128bd2d7a14177ac3ed34194001e7277c117d98ed5f8c743ef1ab3340b48` is present; no resource-measurement artifact exists. | `build/evidence/tests-pytest.log` SHA-256 `5475017575d3243b8b6bf21300c9407ed97553550e9f764538146db7eac5108e`, selector `test_mpp_encoder_private_input_helper`, is not resource evidence. No paired CPU/RSS/DMA/CMA samples, ceilings, arm order, sample counts, tails, or confidence interval were produced. Source-derived copy-traffic arithmetic is not a measurement. | `design/artifacts/r7/security_seed.json` SHA-256 `1329311510eeaa61a6eb8a954dc368af3f06c3e2b5b44205d4c6e799e32013b4`; build-level applicability record is absent. | unproven | `missing-artifact` — performance comparison is unavailable on this board state. Owner: performance lead after a fresh G0 readiness pass. |
| AC-5 | `system/loggerd/encoder/mpp_encoder.cc` SHA-256 `c98b128bd2d7a14177ac3ed34194001e7277c117d98ed5f8c743ef1ab3340b48` keeps copy as default; the implementation diff stays within the approved encoder/helper/test scope. | `build/evidence/tests-pytest.log` SHA-256 `5475017575d3243b8b6bf21300c9407ed97553550e9f764538146db7eac5108e`, selector `test_mpp_encoder_private_input_helper`, is not G3 evidence. No G3 image-specific qualification/no-go/exploratory result or rollback-arm run exists. No live-camera/on-road test was performed. | `design/artifacts/r7/security_seed.json` SHA-256 `1329311510eeaa61a6eb8a954dc368af3f06c3e2b5b44205d4c6e799e32013b4`; build-level applicability record is absent. | unproven | `missing-artifact` — the safety constraints and source-level default are evidenced, but final hash-bound disposition and `ENCODER_ZERO_COPY=0` arm evidence are absent. Owner: Design/Commander with build and test leads. |
| runtime | Candidate binary SHA-256 `655d3449384899c05f33ac98955a96671e1aae9fe1883b91946f98d416a44798`; `ldd -r` is loader inspection only. | No startup smoke log or typed runtime probe exists. The health report is an unavailable disposition, not a runtime pass. | Not applicable to this package-level row. | unproven | `missing-artifact` — runtime has no sanctioned fallback. Owner: health-check after an authorized safe runtime target exists. |

**Rows:** 0 proven, 0 proven by waiver, 6 unproven. The runtime row alone prevents the build gate from advancing.

## Approved decision register cross-check

| Decision | Status | Evidence / gap |
| --- | --- | --- |
| D-22 — pursue measurable encoder CPU/memory efficiency in a narrow path | unproven | The scoped fallback change is present, but no paired efficiency measurement exists; no gain is claimed. |
| D-23 — retain best-effort semantics and report loss/decode/metadata; no lossless claim | unproven | No candidate-vs-copy output/loss/decode comparison exists. The implementation report makes no zero-loss claim. |
| D-24 — use only bounded isolated synthetic testing; no live input or production roots | proven | G0 records that CMA was unavailable and the A/B was not run; the only behavior test is the isolated hardware-free helper harness. This proves scope compliance, not A/B results. |
| D-25 — preserve codec/resolution/FPS/camera/audio and route/file compatibility | unproven | The source diff avoids those interfaces, but media, timestamps, segment, audio, and route compatibility have not been exercised. |

The plan's additional decisions are also visible: copy remains the default and production selection is deferred (source-level check passes); broader loggerd/writer/route/LOC work is absent from the five-file candidate diff. Those scope checks do not prove AC-3/AC-4 or permit production enablement.

## Changed-file trace

Every changed candidate file below exists at the recorded SHA-256 and appears in the registered five-file `impl.diff`. The typed test probe repeats all five input hashes. Thus the *file-to-slice mapping* is proven from saved artifacts; the gate-facing `implementation.change_set` and `traceability` records are still absent because no manifest has been assembled.

| Candidate path | SHA-256 | Trace | File binding |
| --- | --- | --- | --- |
| `system/loggerd/encoder/mpp_encoder.cc` | `c98b128bd2d7a14177ac3ed34194001e7277c117d98ed5f8c743ef1ab3340b48` | AC-1, AC-2, AC-5 | proven |
| `system/loggerd/encoder/mpp_encoder.h` | `c6893a6df9ae79c55e90897d96a1ad72bc3fc648759a123114f2ec21140265a7` | AC-2 | proven |
| `system/loggerd/encoder/mpp_encoder_input.h` | `7c05efe9159fdd0dcb9e30a5540941b2c22fc7d83413b4f58011a3d2d511953f` | AC-2 | proven |
| `system/loggerd/tests/test_mpp_encoder.cc` | `68d6ad08fa4a8b44df103fca82b701c97837a2560861f13b00fb97a227544a12` | AC-2 | proven |
| `system/loggerd/tests/test_mpp_encoder.py` | `155ce3a252319a4baec661549f4a55472b820c744ff182a4819952d1c8280b15` | AC-2 | proven |

The merged dirty-root overlay is supplementary compile evidence only. Its source hashes and the unrelated dirty-root paths are listed in `build/evidence/impl-merged-root-hashes.txt`; none are added to this candidate trace.

## Package-key binding and unresolved gaps

There is **no `build/manifest.json`**. The gate-facing keys are not assembled or bound, even where underlying evidence exists:

| Build key | Status | Reason |
| --- | --- | --- |
| `approved_design_revision` | unproven | Approved cross-stage verdict is present, but no build manifest binds it. |
| `implementation` | unproven | Diff/report/build records exist, but the manifest `change_set` is absent. |
| `tests` | unproven as a package key | The typed test probe and hashed log exist, but the manifest does not reference them. |
| `runtime` | unproven | No executed smoke log or probe; no fallback exists. |
| `security_evidence` | unproven as a package key | Approved seed supports the sanctioned no-boundary applicability decision, but the build-level typed applicability record is absent. Build-management owns authoring it; this does not require security-builder review. |
| `traceability` | unproven | This report supplies the matrix input, but the manifest trace map has not been authored. |

The package is not complete: missing runtime evidence is a **Major** build-to-review blocker. Other unresolved rows are also blockers to the approved G2/G3 acceptance, but they do not change the runtime finding. The gate's self-check and package-shape check were not run because no submission package exists; this report is not a gate verdict.

## Gaps and owners

1. **Runtime startup/readiness smoke — Major.** No safe target is currently authorized under the no-encoder boundary, and this board has zero free CMA with userspace MPP readiness unproven. `build/health-check` must rerun against a named safe target after the environment/readiness condition changes. No test suite or `ldd` output substitutes for this.
2. **G1 integration/recovery proof.** The helper behavior test passes, but actual MPP encode and encoder open/close recovery behavior remain untested. Encoder implementation lead owns the next authorized deterministic/integration seam.
3. **G2 output and performance comparison.** After fresh idle/CMA/MPP preflight on a suitable target, KA2 test/performance leads own fixed ceilings, paired same-binary arms, output/loss/decode/qRoad evidence, and CPU/RSS/DMA measures. Until then no gain or zero-loss claim is supported.
4. **G3 disposition and package assembly.** After evidence exists, Design/Commander and build-management must record image-specific disposition, copy-default/rollback status, build-level security applicability record, `trace_map`, and manifest. Do not submit the current evidence set to `build-to-review`.
