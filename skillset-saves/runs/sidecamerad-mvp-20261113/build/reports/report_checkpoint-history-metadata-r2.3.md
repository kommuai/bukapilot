# Checkpoint — historical runtime report lineage

Run `sidecamerad-mvp-20261113`; Admiral run revision before checkpoint 139. Package/source input revision remains 68.

The historical r2.2 cadence report now carries current package-level submission/revision metadata and an explicit `source_submission_id` retaining its original c68 provenance. Its body is byte-identical to the prior report. The current r2.3 health report/probe already use package lineage fields while retaining health-check-origin metadata and `input_revision: 68`. No capture, source edit, or test run occurred.

The current package and manifest hashes still bind the r2.3 runtime report/probe. The prior gatekeeper-build shape check had reported mixed IDs/revisions; this metadata-only normalization addresses the remaining historical report ID. The shape check must be rerun after the completeness refresh.

AC-4 remains failed: 6,050 matched IDs over 302.471744064s, mean 19.998562241636 fps, max EOF gap 90.981562ms, 716/6,049 >60ms. Functional publication/cleanup pass; startup Params unverified. AC-6/AC-8 hardware deferrals and Minor FFmpeg resource cap remain open.

Next: rerun read-only cross-check against the coherent package metadata, bind its new report hash, and run both package validators. Do not submit to gatekeeper-build or repeat capture; Admiral routes the mechanism to investigation.

## Relevant hashes

- Historical r2.2 runtime report: `build/reports/report_runtime_health-ac4-r2.2-round2.md` — `07c2d43997f77a8e0cbc723d44c4e5c58d8a5a55ab84434388bb687d79fdbd86`.
- Current r2.3 runtime report: `build/reports/report_runtime_health-ac4-r2.3.md` — `2118c1a3a04ab3ca82d8fa1ef38fdd82855537520c528f5a0801d5aa1c3ae8d2`.
- Current typed probe: `build/artifacts/runtime_probe-ac4-r2.3.json` — `e018216e41f7f94f0684e45b21f84ca441cfacf2a954a51888ce1def8610cb35`.
- Build package: `build/reports/build-package.md` — `0bd982d33c68cb4ab36abf5a9d5621e6eae2a179b553e4546cc8936c2287abf8`.
- Manifest: `build/manifest.json` — `cc30540863d089cdd130b0d3aa42c1dcb813027c7beca4385664cc4bb4da6b0b`.
