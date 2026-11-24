# Checkpoint — route unresolved AC-4 mechanism to investigation

## Current boundary

Build-management returned the r2.3 evidence refresh at Admiral run revision 141. Build package/source input revision remains 68. The package was not submitted to `gatekeeper-build`: the prior build-to-review verdict is REVISE and AC-4 remains an unresolved Major. The canonical self-check passed with zero failures/warnings; the package-shape check has zero failures and only its conditional notice because no gate verdict is being submitted.

## Verified AC-4 evidence

One authorized five-minute run used the assigned front-left UVC device with unchanged source hashes. It selected 6,050 matching cereal/VisionIPC frames over 302.471744064 seconds. Mean EOF interval rate was 19.998562241636 fps (below 20.0), maximum EOF gap was 90.981562 ms, and 716 of 6,049 intervals exceeded 60 ms. Frame IDs were contiguous, with zero timestamp/validity mismatches. Frame publication and cleanup passed: sidecamerad exited 0, the FFmpeg child was reaped, listeners were absent, and the isolated Params root was removed. Startup Params all-false remains unverified; front-left was true before shutdown and all-false after.

The current capture reproduces cadence failure but does not identify its mechanism. Prior timing correlations and the r7 GDB stacks remain non-causal; the mux thread was in `write()` and the main thread varied between condition waits and `sws_scale_frame()`. The GDB sampling report is diagnostic only and does not satisfy AC-4.

## Package and lineage

The build package, manifest, completeness report, runtime outputs, and GDB diagnostic are hash-registered and current at the paths below. Completeness is 6/9: AC-4, AC-6, and AC-8 remain unproven. AC-6/AC-8 are hardware deferrals; the FFmpeg resource-cap issue remains an open Minor.

- `build/manifest.json` — SHA-256 `24c6293dc4dd12366b6fda838ebac7170f94b01e9441b426ad72378f7f3e6248`
- `build/reports/build-package.md` — SHA-256 `98866844cd5700acfc779ce9edd12a241fdc68edf10e9380a76fca278ea7ff41`
- `build/reports/report_completeness.md` — SHA-256 `a7d68fb1985a65b53b7b96a44dd1b8118bec6276115b656baa30a0275b1d3bef`
- `build/reports/report_runtime_health-ac4-r2.3.md` — SHA-256 `2118c1a3a04ab3ca82d8fa1ef38fdd82855537520c528f5a0801d5aa1c3ae8d2`
- `build/evidence/runtime-smoke-ac4-r2.3.log` — SHA-256 `74722d2c6d4ca799f92529c2bce256cf7c6531ecaccbf2d8071d3139c7df19b`
- `build/artifacts/runtime_cadence_intervals-r2.3.json` — SHA-256 `626f0881c22814e055d724ce8683c5206d5c3090dd507e582327362202f53696`
- `build/artifacts/runtime_probe-ac4-r2.3.json` — SHA-256 `e018216e41f7f94f0684e45b21f84ca441cfacf2a954a51888ce1def8610cb35`
- `build/reports/report_gdb-sampling-r7.md`, `build/evidence/gdb-sampling-r7.log`, `build/artifacts/ffmpeg-gdb-sampling-r7.json` — hashes registered in run state
- Prior approved investigation r7: `investigation/reports/investigation-package.md`, `investigation/manifest.json`, and `investigation/verdict_investigation-review.cross-stage.json`

## Next action

Admiral routes a new investigation revision (r8) to settle the unknown AC-4 timing mechanism and return one bounded fix path to BUILD. Use the just-completed r2.3 failure plus the hash-valid r1–r7 evidence. Do not edit product source or start another camera capture/trace in this handoff; if the existing evidence cannot settle the mechanism, state the missing observation and residual uncertainty for Admiral to authorize separately. Return through `investigation-review` before BUILD resumes.
