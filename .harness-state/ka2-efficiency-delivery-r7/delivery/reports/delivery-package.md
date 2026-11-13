---
submission_id: loggerd-ka2-efficiency-20261029_handoff-7_attempt-1_2026-10-31T06-56-41Z
revision: r10
boundary: design-to-build
---

# Delivery package: KA2 rotation liveness r10

**Status:** Design phase and cross-stage approvals are recorded. Build work has not started in this design-contract correction.  
**Boundary:** DESIGN → BUILD.  
**Submission:** `loggerd-ka2-efficiency-20261029_handoff-7_attempt-1_2026-10-31T06-56-41Z`.

## Submitted source of truth

- Design manifest: `skillset-saves/runs/loggerd-ka2-efficiency-20261029/design/manifest.json`
- Manifest SHA-256: `138f8b5075d6adf7741fb25bf570e225d52b10c32bb9a3fb200b67baa6c43ea1`
- Design submission and revision: `loggerd-ka2-efficiency-20261029-design-r10-mpp-segment-recovery-contract`, `r10`
- Phase approval: `skillset-saves/runs/loggerd-ka2-efficiency-20261029/design/verdict_design-to-build-r10.json`, SHA-256 `f25b3447111888de96608b9a47af5aae3bc6b00500f6923a125219e832b01f5b`, verdict ID `89719e849ac65c6e4aed2f974e8c4154ff0bd90d6cb29c13ef02c330a3c0f6a3`
- Phase judgment: `skillset-saves/runs/loggerd-ka2-efficiency-20261029/design/verdict_design-to-build-r10.judgment.json`, SHA-256 `2d50b0b0f47b5f16c1430a125ca15642baeeedbfa4d9cac703c40e697e36516b`
- Prior approved r9 cross-stage record: `skillset-saves/runs/loggerd-ka2-efficiency-20261029/design/verdict_design-to-build-r9.cross-stage.json`, SHA-256 `ff3541635c6edff9e9539d20567516f436b20c6cfed610865e1b1053c694a7bc`
- Preserved r9 delivery archive: `packages/delivery-r9-approved-before-r10.tar.gz`, SHA-256 `ff4feac59beb19b8a79708522d514d21d9fa99ddd4be99ee2159eb9ef868a28b`

Gatekeeper-admiral approved the r10 crossing. Its verdict is `skillset-saves/runs/loggerd-ka2-efficiency-20261029/design/verdict_design-to-build-r10.cross-stage.json`, SHA-256 `bc2d854ba4675408eb4c5329ee1252b4645f78075629aa4fe70d5f952042d75d`, ID `89719e849ac65c6e4aed2f974e8c4154ff0bd90d6cb29c13ef02c330a3c0f6a3`; the judgment SHA-256 is `3523ab37e7bf36b6275e27f05eeff7e4561c8c27679d7696c68f759775f96f67`. The prior approved r9 record remains historical and is not the r10 approval. The exact pending handoff and summary that were reviewed are preserved in `packages/delivery-r10-pending-before-cross-stage.tar.gz`, SHA-256 `6072cc607b36196de2f8cc70ba3abea1f0af14237bb13d1e7c41af77c7eb3e26`.

## Approved r10 design scope

The approved design addresses deterministic-KA2 segment liveness and encoder fault containment while preserving D-28's mandatory no-copy behavior:

- Full-resolution MPP uses one validated DMA-BUF import attempt. A rejected input is dropped with no copy fallback, pointer retry, or selector. qRoad retains its existing RGA resize path.
- Ordinary deterministic-KA2 segment transitions retain EOS/drain/close/open for each healthy encoder. A shared 900 ms monotonic drain deadline quarantines only outputs still pending, retires their pending identities once, and lets segment advance continue without process-wide exit.
- qRoad receives one planned reopen at the ordinary boundary, no failure-recovery reopen/retry caused by a sibling, stays active, and emits new-segment packets. Only the failed output follows its bounded retry schedule. The design does not rely on IDR-only continuation after EOS because the deployed MPP build is unpinned.
- Loggerd keeps the 5,000 ms deterministic rotation deadline and per-epoch readiness. Runtime commits finalize active writers; startup rotation remains writer-free. The exact diagnostic report stays seven-field, with five loggerd-observed drop counters; encoder losses remain encoder-local.
- Valid and rejected input lifecycle tests use the synthetic DMA-heap VisionIPC fixture with isolated process roots, separate fault cases, and asynchronous send for externally backed buffers. No full-resolution copy is reintroduced.
- AC-12 preserves non-KA2 and explicitly disabled deterministic-mode behavior.

Detailed acceptance rows AC-1 through AC-12 are in `skillset-saves/runs/loggerd-ka2-efficiency-20261029/design/reports/delivery-plan_r10.md`; ownership, interfaces, and build slices are in `implementation-spec_r10.md` and `report_interfaces_r10.md`.

## Qualification limit

Live RK3588 V4L2 DMA-BUF export, fence/synchronization behavior, live-buffer MPP import, camera buffer lifetime/reuse, and the deployed MPP post-EOS continuation contract remain unqualified. The M0 fixture proves synthetic DMA-heap VisionIPC transport and fixture-owned lifetime only; it does not prove MPP import or full process lifecycle. SHM-to-RGA compatibility and downstream handling of absent or partial stream files also remain open. No live-camera qualification is claimed.

## Next action

The r10 DESIGN → BUILD phase and cross-stage gates are both APPROVED. Build-management may now consume the exact r10 manifest and implementation spec. The live V4L2, fences, deployed MPP, and camera-buffer qualification limits remain open. No product source or tests were changed as part of this design-contract correction.
