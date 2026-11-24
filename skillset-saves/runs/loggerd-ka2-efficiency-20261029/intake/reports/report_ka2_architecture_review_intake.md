# Intake: KA2 capture-path robustness architecture review

**Run:** `loggerd-ka2-efficiency-20261029`  
**Date:** 2026-10-30  
**Decision:** D-18 — user explicitly broadens the objective from encoder efficiency to end-to-end KA2 path robustness and permits rearchitecture.

## Scope

Review the complete KA2 video recording path: camera/VisionIPC buffer ownership and delivery; `encoderd` scheduling and the RK3588 MPP encoder; encoded event metadata and transport; `loggerd` queues, synchronization, segment rollover, and file writing; plus process startup, shutdown, and recovery. Trace relevant shared abstractions and non-KA2 encoder paths wherever a proposed change could affect them.

## Goal

Return a source-backed, severity-ranked assessment of architectural flaws and a concrete target architecture that makes frame accounting, playable output, rotation, and recovery robust while bounding memory, waits, and failure behavior. Include a regression proof plan and staged migration/rollback path. Do not claim an efficiency gain unless separately measured.

## Constraints and decisions

- Preserve observable recording behavior: accepted frames are accounted for exactly once in every applicable output, and startup, rotation, restart, and shutdown do not silently lose frames. Preserve stream selection, formats, timing, segment semantics, and metadata/schema unless a compatibility change is explicitly justified and proven.
- Treat the current worktree candidate as unapproved evidence to inspect, not as an accepted architecture or a baseline to overwrite. Preserve unrelated dirty changes.
- The prior R6 stop identified unknown exact-target MPP identity/API and EOS/IDR behavior, VisionIPC retention/lifetime, encoded-message bounds, and RSS/DMA headroom. Keep those facts marked unknown until direct evidence is available; do not build a proposal on guessed vendor behavior.
- This request authorizes a review and isolated/synthetic experiments for the recommendation. It does not ask for production implementation or deployment. Do not change product source as part of this assessment. Device tests must remain isolated from production logs/params and obey the target preflight and known MPP compatibility limits.
- Cover behavior under overload, delayed or missing consumers, hardware errors, partial writes, stale segment authority, process restart, and concurrent shutdown. Distinguish proven defects from risks that need runtime evidence.

## Deliverable

One end-to-end recommendation with current data-flow map, concrete findings anchored to source/evidence, proposed ownership and handshake/queue/error contracts, regression matrix, validation gates, rollout/rollback sequence, and explicit unresolved evidence. The intended endpoint is a reviewable recommendation; no build handoff is implied.

