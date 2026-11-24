# Resume checkpoint — SideCamera build

- Run: `sidecamerad-mvp-20261113`; prior saved revision 68; stale-lock recovery published revision 69.
- Recovery evidence: the only held run was `sidecamerad-mvp-20261113`; no interrupted publish journal was present. The recovery event records the stale lock hash and heartbeat.
- Approved boundary: design r2 remains approved at `design-to-build`; phase state is `BUILD_ACTIVE`.
- Product scope: default-enable `sidecamerad` while keeping native `camerad`, publish four named camera feeds over cereal and VisionIPC, and track availability in Params.
- Current local source HEAD: `6d92d63a2`. The newer recovery report says there were no product-source edits in the r2.2 refresh; the refreshed Params header and generated binding hashes are now `c2925ebd8f9f26cc04951afb0e3c67ac7cb20a1c370b79aa93a25a4aaef2a031` and `3108e29407f31b9bd86a86ccafc82a4f98a225884412987a24be5c357dbfb586`.

## Evidence reconciled from after revision 68

- `build/reports/report_implementation-recovery-r2.2.md` and its typed change-set record capture the recovered working tree; the record says it did not update the shared manifest or run state.
- `build/reports/report_tests-r2.2.md` and `build/evidence/tests-pytest-r2.2.log` report 8/8 focused tests passing in 5.85 seconds against the refreshed Params binding.
- `build/reports/report_runtime_health-r2.1-post-merge.md` and its typed probe record report an 8-second single-camera functional smoke: 117 matching cereal/VisionIPC frame IDs, clean shutdown and no TCP listeners. It makes no cadence or multi-camera claim.
- The previous five-minute AC-4 measurement remains failed (mean 19.9994 fps, maximum EOF gap 123 ms, 743 of 5,999 intervals over 60 ms). AC-6 physical reconnect/sibling continuity and AC-8 four-camera 30-minute soak remain unproven. Only one attached USB decoder is visible at `/dev/video60` here.
- The prior `build/manifest.json` and build package still describe r2.1 and pre-recovery Params hashes. The new evidence must be reconciled into one coherent package; the completeness cross-check and both build-to-review self-checks need a fresh run before gate submission.

## Readiness and persistence

- Readiness probe: active run and deterministic validators are available; Python 3.12.3 is below the declared >=3.13 target and Codex hooks are missing/unverified. These are recorded as degraded host readiness, not silently treated as passes.
- Save location: the run state records user approval for the in-project save tree and preserves the older canonical `/data/media` copy at `skillset-saves.external` (revision 34). The approved local run remains the source of truth for this resume; the older copy is not merged or changed.
- No product-source files were changed during resume/recovery.

## Next action

Build-management reconciles the r2.2 implementation, test and runtime evidence with current source hashes; reruns completeness and both build-to-review self-checks; then returns a coherent package and gate recommendation. Preserve the AC-4, AC-6 and AC-8 gaps if target hardware cannot close them.
