# Checkpoint — AC-4 five-minute health-check result

Run: `sidecamerad-mvp-20261113` · Admiral run revision before checkpoint: 135 · Build source/input revision: 68.

The single approved current-source acceptance capture completed on `/dev/v4l/by-path/platform-xhci-hcd.11.auto-usb-0:1:1.0-video-index0` (`/dev/video60`). Source inputs matched the approved package hashes; no product source or production argv changed. It captured 6,050 selected matched cereal/VisionIPC frame IDs over 302.471744064 seconds. Mean EOF interval rate was 19.998562241636 fps; maximum EOF gap was 90.981562 ms; 716 of 6,049 intervals exceeded 60 ms. Frame IDs were contiguous, with zero timestamp/validity mismatches and no source hash drift.

Runtime publication and cleanup passed: `sidecamerad` exited 0, no FFmpeg child or TCP listeners remained, and the isolated Params root was removed. The startup Params sample was taken before launch and is unverified; pre-shutdown front-left=true and post-shutdown all-false were observed. No payload was saved.

AC-4 therefore remains failed on both mean cadence and max-gap requirements. Do not submit this failed package to `gatekeeper-build` and do not apply a speculative source fix. Admiral should route the unresolved timing mechanism back through investigation. AC-6 physical reconnect/sibling continuity and AC-8 four-camera soak/resource headroom remain deferred; the Minor FFmpeg resource-cap issue remains open.

## Evidence

- Runtime report: `build/reports/report_runtime_health-ac4-r2.3.md` — SHA-256 `289cce08c02060e5432db798da8ffbad45454f2d57c1bcfae1b8a78c9f4f58bf`.
- Smoke log: `build/evidence/runtime-smoke-ac4-r2.3.log` — SHA-256 `74722d2c6d4ca799f92529c2b2ce256cf7c6531ecaccbf2d8071d3139c7df19b`.
- Per-frame intervals: `build/artifacts/runtime_cadence_intervals-r2.3.json` — SHA-256 `626f0881c22814e055d724ce8683c5206d5c3090dd507e582327362202f53696`.
- Typed probe: `build/artifacts/runtime_probe-ac4-r2.3.json` — SHA-256 `780c9e20f0f7a0d7eb8369a16beac521d6d4390f588bdd6d041819a2aad877d3`.

Next: reconcile the build package and completeness report with this failed AC-4 evidence, run package self-checks for the handback, and return to Admiral/investigation. No further capture is authorized in this handoff.
