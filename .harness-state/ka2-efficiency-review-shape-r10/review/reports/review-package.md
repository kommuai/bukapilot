---
revision: 281
submission_id: loggerd-ka2-efficiency-20261029-review-r10
---

# Review package — KA2 loggerd DMA-BUF rotation and encoder recovery r10

Outcome: review-to-delivery recommendation APPROVED for the bounded source revision 266 change set, with one Major qualification risk deferred and one Minor maintainability finding recorded.

Findings: one deferred Major (live camera DMA-BUF lifetime qualification), one Minor (duplicated deterministic-mode activation rule), and one Info (stale test-report sentence). Correctness review found no concrete defect. The three specialist packets and the conditional-lens decisions are in the execution manifest.

## Scope and lineage

The review covered exactly the 18 first-party files under system/loggerd/ listed by the approved build manifest at source revision 266. The build-to-review package was approved with no blockers. The review handoff began at run revision 274; this review package is revision 281. No reviewed source or test file changed during review.

Lineage: approved r10 design and its cross-stage verdict -> approved build source revision 266 -> build-to-review APPROVED -> review reports against the same r266 hashes -> review package revision 281.

The review used the approved build manifest, aggregate diff, intake decision log, implementation report, completeness report, test report and test probe, runtime report and runtime probe, plus all three specialist review packets. The code-chief probe log independently checked the 18 current changed-source hashes, the 19 test-probe input hashes, the 12 runtime-probe input hashes, the upstream report hashes, the full-suite summary and AC-6 assertions, and the production-smoke summary. No tests were rerun.

## Lens results

- Correctness: no concrete correctness defects found. The report keeps the live-hardware qualification limits separate from defects.
- Merge readiness: recommends proceeding within the approved bounded scope. It raises CR-01 as a deferred pre-release Major because the synthetic DMA-heap path does not establish live V4L2 producer-buffer reuse ordering against asynchronous MPP reads. It also records CR-02, an Info note about stale wording in the test report.
- Maintainability: records QR-1, a Minor. loggerd and encoderd encode the same deterministic-mode activation rule independently; a future one-sided edit could cause protocol drift.

## Findings and follow-up

### CR-01 — Major, deferred

The production full-resolution path imports camera DMA-BUFs into MPP, whose encode_put_frame interface is asynchronous. The passing four-slot synthetic DMA-heap fixture proves that the fixture can rotate and reuse its buffers; it does not establish that live V4L2 capture will wait until MPP has finished reading each imported buffer. A producer overwrite while MPP still reads could corrupt a frame. No such corruption was observed in the reviewed evidence.

Owner: loggerd/MPP integration owner.

Reopen trigger: before any live-camera or release-readiness claim, run encoderd with real V4L2-exported buffers and the deployed MPP build through sustained capture and segment rotations. Prove buffer-generation/fence or completion ordering while MPP has inputs in flight. If producer reuse can precede MPP completion, add and verify an explicit ownership/synchronization contract before release.

This Major is deferred because the reviewed request and approved package scope establish the synthetic DMA-BUF rotation lifecycle, while the live V4L2 integration qualification is a separate pre-release condition. This approval must not be presented as live-camera release qualification.

### QR-1 — Minor

The loggerd and encoderd deterministic-mode activation conditions currently match but are maintained in two places. Consolidate them or add a parity contract when that mode rule next changes. This is non-blocking.

### CR-02 — Info

The test report still contains a sentence saying runtime evidence needs refreshing. The r269 production smoke and typed runtime probe are current and pass against the reviewed source hashes. The wording does not invalidate the runtime evidence; no test rerun is needed.

## Conditional lens decisions

The execution manifest records every conditional stage and its false condition:

- Security review did not run because the approved build package records no changed trust boundary.
- Adversarial review did not run because the change introduces no public or untrusted input surface; it remains in trusted local camera-to-encoder IPC. Invalid DMA-BUF inputs were examined by the correctness lens.
- Frontend review did not run because no visible behavior changed.
- Design QA did not run because no user-visible surface changed; rendered verification is not applicable.
- Developer-experience review did not run because there is no change to onboarding, CLI, SDK, public integration, or developer workflow. The new fixture code is internal test infrastructure.

## Verification and residual risk

The grouped KA2 suite passed 14 tests with zero skips in 301.56 seconds. It includes the explicit AC-6 assertion that all three encoder threads recorded zero calls to the all-output reopen helper during the drain-timeout case.

The production-binary smoke sent 106 frames per camera stream through segments 0–2, completed two healthy rotations with zero known drops, decoded qRoad video and audio in every segment, and passed clean postflight. The package-shape validator found zero structural failures on a project-contained mirror verified byte-for-byte against the canonical review package; three conditional items remain for the gate to resolve from the execution manifest. The shared boundary validator passed against the canonical manifest with zero failures or warnings.

Remaining unqualified behavior includes live V4L2 DMA-BUF export, capture fences and buffer reuse, persistent live descriptor failures, deployed MPP post-EOS behavior, non-KA2 silicon, and physical PC hardware. The non-RK3588/HardwarePC software branch was exercised on KA2 hardware only. Downstream handling of absent or partial failed-stream files also remains a release follow-up.

No commit or deployment was performed. Next action: submit this package to review/gatekeeper-code.
