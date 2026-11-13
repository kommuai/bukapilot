---
submission_id: loggerd-ka2-efficiency-20261029_handoff-7_attempt-1_2026-10-31T06-56-41Z
revision: r10
boundary: design-to-build
package_path: skillset-saves/runs/loggerd-ka2-efficiency-20261029/design/manifest.json
package_sha256: 138f8b5075d6adf7741fb25bf570e225d52b10c32bb9a3fb200b67baa6c43ea1
submission_status: VERDICT_RECORDED
verdict: APPROVED
phase_verdict: APPROVED
phase_verdict_path: skillset-saves/runs/loggerd-ka2-efficiency-20261029/design/verdict_design-to-build-r10.json
phase_verdict_sha256: f25b3447111888de96608b9a47af5aae3bc6b00500f6923a125219e832b01f5b
phase_verdict_id: 89719e849ac65c6e4aed2f974e8c4154ff0bd90d6cb29c13ef02c330a3c0f6a3
phase_judgment_path: skillset-saves/runs/loggerd-ka2-efficiency-20261029/design/verdict_design-to-build-r10.judgment.json
phase_judgment_sha256: 2d50b0b0f47b5f16c1430a125ca15642baeeedbfa4d9cac703c40e697e36516b
prior_approved_cross_stage_verdict: APPROVED
prior_approved_cross_stage_verdict_path: skillset-saves/runs/loggerd-ka2-efficiency-20261029/design/verdict_design-to-build-r9.cross-stage.json
prior_approved_cross_stage_verdict_sha256: ff3541635c6edff9e9539d20567516f436b20c6cfed610865e1b1053c694a7bc
prior_approved_cross_stage_verdict_id: ffb8a3c4beadb40f90d0a087eaa9982b066448b2196d085279c2a5aa3f8d8011
cross_stage_verdict: APPROVED
cross_stage_status: VERDICT_RECORDED
cross_stage_verdict_path: skillset-saves/runs/loggerd-ka2-efficiency-20261029/design/verdict_design-to-build-r10.cross-stage.json
cross_stage_verdict_sha256: bc2d854ba4675408eb4c5329ee1252b4645f78075629aa4fe70d5f952042d75d
cross_stage_verdict_id: 89719e849ac65c6e4aed2f974e8c4154ff0bd90d6cb29c13ef02c330a3c0f6a3
cross_stage_judgment_path: skillset-saves/runs/loggerd-ka2-efficiency-20261029/design/verdict_design-to-build-r10.cross-stage.judgment.json
cross_stage_judgment_sha256: 3523ab37e7bf36b6275e27f05eeff7e4561c8c27679d7696c68f759775f96f67
---

# Admiral handoff: KA2 rotation liveness r10

## Submitted design and phase approval

This DESIGN → BUILD handoff submits the exact r10 design package at `skillset-saves/runs/loggerd-ka2-efficiency-20261029/design/manifest.json`, SHA-256 `138f8b5075d6adf7741fb25bf570e225d52b10c32bb9a3fb200b67baa6c43ea1`. Its submission is `loggerd-ka2-efficiency-20261029-design-r10-mpp-segment-recovery-contract`, revision `r10`.

Gatekeeper-design approved this exact manifest. The phase verdict is `skillset-saves/runs/loggerd-ka2-efficiency-20261029/design/verdict_design-to-build-r10.json`, SHA-256 `f25b3447111888de96608b9a47af5aae3bc6b00500f6923a125219e832b01f5b`, verdict ID `89719e849ac65c6e4aed2f974e8c4154ff0bd90d6cb29c13ef02c330a3c0f6a3`. Its judgment is `skillset-saves/runs/loggerd-ka2-efficiency-20261029/design/verdict_design-to-build-r10.judgment.json`, SHA-256 `2d50b0b0f47b5f16c1430a125ca15642baeeedbfa4d9cac703c40e697e36516b`.

The prior r9 cross-stage approval is retained as history at `skillset-saves/runs/loggerd-ka2-efficiency-20261029/design/verdict_design-to-build-r9.cross-stage.json`, SHA-256 `ff3541635c6edff9e9539d20567516f436b20c6cfed610865e1b1053c694a7bc`, verdict ID `ffb8a3c4beadb40f90d0a087eaa9982b066448b2196d085279c2a5aa3f8d8011`. The prior approved r9 handoff and delivery summary are preserved in `packages/delivery-r9-approved-before-r10.tar.gz`, SHA-256 `ff4feac59beb19b8a79708522d514d21d9fa99ddd4be99ee2159eb9ef868a28b`. The r9 design package archive is `design/packages/design-r9-approved-before-r10.tar.gz`, SHA-256 `06ca8b15925e51cbd447dea9e8a029e3dd00085a2015e3a6f6eac60c83ff5810`.

Gatekeeper-admiral approved the exact r10 package. Its cross-stage verdict is `skillset-saves/runs/loggerd-ka2-efficiency-20261029/design/verdict_design-to-build-r10.cross-stage.json`, SHA-256 `bc2d854ba4675408eb4c5329ee1252b4645f78075629aa4fe70d5f952042d75d`, verdict ID `89719e849ac65c6e4aed2f974e8c4154ff0bd90d6cb29c13ef02c330a3c0f6a3`. The judgment is `skillset-saves/runs/loggerd-ka2-efficiency-20261029/design/verdict_design-to-build-r10.cross-stage.judgment.json`, SHA-256 `3523ab37e7bf36b6275e27f05eeff7e4561c8c27679d7696c68f759775f96f67`. The exact pending handoff and summary reviewed by that gate are preserved in `packages/delivery-r10-pending-before-cross-stage.tar.gz`, SHA-256 `6072cc607b36196de2f8cc70ba3abea1f0af14237bb13d1e7c41af77c7eb3e26`.

## r10 contract for the next stage

- Preserve D-28: full-resolution MPP validates the DMA-BUF fd and makes one import attempt. Rejected input is dropped; there is no full-resolution copy fallback, pointer retry, or selector. qRoad keeps its existing RGA resize path.
- For ordinary deterministic-KA2 segment changes, retain the established EOS, fair drain, close, and open transition for each healthy output. A single shared 900 ms monotonic deadline bounds draining; per-output poll slices are interim, not independent deadlines.
- At the common deadline, quarantine only outputs still pending, retire each affected pending frame identity once, and continue segment advancement without setting process-wide `do_exit`. A drain fault must not trigger extra qRoad failure recovery.
- qRoad receives exactly one planned `segment_transition_reopen_count` for the ordinary boundary, no sibling-triggered `failure_recovery_reopen_count` or `failure_retry_count` delta, remains active, and emits valid packets tagged to the new segment. Retries and failure-recovery reopens belong only to the failed output.
- Do not assume force-IDR enables an MPP context to resume after EOS. The deployed MPP build is not pinned and safe post-EOS continuation is unproven. Camera/VisionIPC reconnect remains the all-output reset exception because producer fd generations change. Non-KA2 and explicitly disabled deterministic mode preserve prior behavior.
- Keep loggerd's 5,000 ms deterministic-KA2 rotation deadline and per-epoch stream readiness; finalize active writers at deterministic runtime commits and keep startup `logger_rotate()` writer-free. Preserve the exact seven-field `LoggerdRotationReport` and its five loggerd-observed drop counters; encoder MPP rejection, stall, drain, pending-frame retirement, and retry losses remain per-output diagnostics.
- Use the isolated synthetic DMA-heap VisionIPC publisher for valid and rejected input lifecycle coverage. Keep `LOG_ROOT`, `PARAMS_ROOT`, and `OPENPILOT_PREFIX` isolated, and use `VisionIpcServer::send(..., sync=false)` for external DMA-heap buffers.

## Qualification limits

The M0 fixture demonstrates isolated DMA-heap VisionIPC transport and fixture-owned buffer lifetime. It does not prove live-camera integration, the MPP loggerd/encoderd lifecycle, or V4L2 exporter behavior. The deployed MPP build, post-EOS encoder-context continuation, RK3588 live V4L2 DMA-BUF export, synchronization/fence handling, MPP import against live camera buffers, and camera buffer lifetime/reuse remain unqualified. SHM-to-RGA compatibility and downstream behavior for absent or partial stream files also remain open. No live-camera qualification is claimed.

## Gate state and next action

Both the design phase gate and cross-stage gate are APPROVED for this exact r10 package. Build-management may now consume `design/reports/implementation-spec_r10.md` and `design/reports/delivery-plan_r10.md`, carrying the qualification limits above. No product implementation or product tests were included in this design-contract correction.
