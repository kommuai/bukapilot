# Session-memory intake checkpoint: sidecamerad MVP

## Run and boundary

- Run: `sidecamerad-mvp-20261113`
- Prior run revision: 1; active lock owner: `admiral`; session pin: held
- Active boundary: normalized intake complete; no design package or gate submission exists yet
- Checkpoint outcome: mandatory session-memory engagement recorded before the first stage delegation
- Persistence: active; probe result `ok`
- Preamble tier: 2 — multi-step additive feature work with a specialist handoff
- Context tier: 2; artifact mode for the planned design handoff: reference

## Confirmed scope

- Add an opt-in `sidecamerad` process while native `camerad` continues running.
- Publish `frontLeftCameraState`, `frontRightCameraState`, `backLeftCameraState`, and `backRightCameraState`, each with a matching VisionIPC stream.
- Target the previously exercised 1280x720 MJPEG capture mode at 20 fps.
- Enable the process with `SIDECAMERAD=1`; configure each camera with an explicit V4L2 device path and allow a partial configured device set.
- Install no packages. Use an existing FFmpeg executable to decode MJPEG directly to NV12.
- Bound VisionIPC buffers, reuse one frame buffer per capture worker, and avoid BGR conversion and application-level frame queues.

## Deferred work and boundaries

- Do not replace or modify native `camerad`, the webcam override, driving behavior, or startup behavior.
- Do not integrate frames into model inputs, the UI, loggerd drive logs, or encoderd output in this MVP.
- Defer cross-camera synchronization and camera-specific transforms until a downstream consumer requires them.
- Do not auto-discover cameras or persist camera-to-corner assignment.
- No active blocker is recorded. Device-specific runtime validation will depend on configured V4L2 paths and available hardware.

## Verified artifacts and lineage

- Intake decisions: `skillset-saves/runs/sidecamerad-mvp-20261113/intake/reports/report_grilling.md` — registered at revision 1 with SHA-256 `84a7e874f86ba50a6e4c5b3e3938e4a534099e518c4dea8c9ba8a8f4f2d97a0b`.
- This checkpoint report is the supplemental evidence for the next run revision.
- No phase artifact has passed a gate yet.

## Stage handoff record

- Planned next phase: `design`
- Planned stage owner: `design/commander`
- Expected package: design package registered in `design/manifest.json` with supporting design artifacts
- Return boundary: `design-to-build`
- Handoff status: pending; no design delegation has been issued in this checkpoint
- The next delegation must carry the confirmed scope, deferred branches, intake artifact path and digest, Tier 2 rationale, persistence status, and return boundary. Re-check execution mode at that boundary.

## Session-memory engagement

- `session-memory` was engaged for the mandatory intake checkpoint and completed this save.
- Run-state engagement list will append `session-memory` to the existing `admiral` entry.

## Single next action

Admiral prepares and issues the `design/commander` delegation for the `design-to-build` boundary.
