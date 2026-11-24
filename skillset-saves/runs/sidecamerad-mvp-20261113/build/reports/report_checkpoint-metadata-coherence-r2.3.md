# Checkpoint — package metadata lineage correction

Run `sidecamerad-mvp-20261113`; Admiral run revision before checkpoint 138. Package/source input revision remains 68.

The health-check specialist updated only the current runtime report/probe metadata. Package-level submission ID and revision now match the active build package (`sidecamerad-mvp-20261113_build-r2.3-ac4-handoff-c68`, revision 68); original health-check submission/revision remain explicitly preserved. Typed probe retains `input_revision: 68` and all observation fields. No capture rerun, source edit, test run, or evidence-value change occurred.

The build package and manifest were rebound to the revised runtime report/probe. AC-4 remains failed with the same measurements: 6,050 matched frames over 302.471744064s, mean 19.998562241636 fps, max EOF gap 90.981562ms, 716/6,049 >60ms. Functional publication/cleanup pass, startup Params remains unverified, source hashes unchanged. AC-6/AC-8 hardware gaps and Minor FFmpeg resource-cap residual remain open.

The previous gatekeeper-build shape result identified the historical r2.2 health report as retaining its prior submission ID, in addition to the current health report. Next, health-check will preserve that prior identity in explicit provenance metadata while aligning its package-level frontmatter; then the completeness pass and both validators will be rerun. Do not submit to gatekeeper-build; Admiral will route the unresolved mechanism to investigation.

## Current hashes

- `build/manifest.json`: `cc30540863d089cdd130b0d3aa42c1dcb813027c7beca4385664cc4bb4da6b0b`.
- `build/reports/build-package.md`: `0bd982d33c68cb4ab36abf5a9d5621e6eae2a179b553e4546cc8936c2287abf8`.
- `build/reports/report_completeness.md`: `aafb7233af394a2596d4956b6fd9aebfd3688bc3fca3901b751ff9065cf9663f`.
- `build/reports/report_runtime_health-ac4-r2.3.md`: `2118c1a3a04ab3ca82d8fa1ef38fdd82855537520c528f5a0801d5aa1c3ae8d2`.
- `build/artifacts/runtime_probe-ac4-r2.3.json`: `e018216e41f7f94f0684e45b21f84ca441cfacf2a954a51888ce1def8610cb35`.
- Capture log `build/evidence/runtime-smoke-ac4-r2.3.log`: `74722d2c6d4ca799f92529c2b2ce256cf7c6531ecaccbf2d8071d3139c7df19b`.
- Per-frame intervals `build/artifacts/runtime_cadence_intervals-r2.3.json`: `626f0881c22814e055d724ce8683c5206d5c3090dd507e582327362202f53696`.
