# Checkpoint — package self-check findings

Run `sidecamerad-mvp-20261113`; Admiral run revision before checkpoint 137. Package/source input revision remains 68.

The final r2.3 runtime and cross-check evidence are bound in the package. AC-4 remains failed: 6,050 matched IDs over 302.471744064s, mean 19.998562241636 fps, max gap 90.981562ms, 716/6,049 >60ms; IDs/timestamps match and source hashes are unchanged. Functional publication/cleanup pass is separate from the raw interval artifact's unobserved-startup functional flags. AC-6/AC-8 are unverified, and the Minor FFmpeg resource cap remains open.

Local validators run with Python 3.13.15. Canonical `build-to-review` validator passed with zero failures and zero warnings (mechanical-only; no verdict written). Gatekeeper-build package-shape check found two Major failures: `MIXED_REVISIONS` (revision 68 versus `build-ac4-r2.3`) and `MIXED_SUBMISSION_IDS` (`sidecamerad-mvp-20261113_build-r2.2-c68`, current package handoff ID, and `sidecamerad-mvp-20261113_health-check-ac4-r2.3`). It also reports the expected conditional informational notice because no gate verdict is being submitted.

Next: reconcile report frontmatter lineage while preserving original component submission identifiers as explicit provenance, re-run the read-only completeness cross-check after health-report metadata is coherent, rebind manifest hashes, then rerun both local validators. No gatekeeper-build submission, product source edit, test run, or new capture is authorized; Admiral will route unresolved AC-4 to investigation.

## Current hashes

- `build/manifest.json`: `b2b5c8823c8851e4519eb20672301529e7163c6883242dcaceacd85b9df4f1a8`
- `build/reports/build-package.md`: `4e2b4aa4f8ce584db9a12bc76d0b3bca56b6df7e166f77928cf82e5d2b840990`
- `build/reports/report_completeness.md`: `aafb7233af394a2596d4956b6fd9aebfd3688bc3fca3901b751ff9065cf9663f`
- `build/reports/report_runtime_health-ac4-r2.3.md`: `289cce08c02060e5432db798da8ffbad45454f2d57c1bcfae1b8a78c9f4f58bf`
- `build/evidence/runtime-smoke-ac4-r2.3.log`: `74722d2c6d4ca799f92529c2b2ce256cf7c6531ecaccbf2d8071d3139c7df19b`
- `build/artifacts/runtime_cadence_intervals-r2.3.json`: `626f0881c22814e055d724ce8683c5206d5c3090dd507e582327362202f53696`
- `build/artifacts/runtime_probe-ac4-r2.3.json`: `780c9e20f0f7a0d7eb8369a16beac521d6d4390f588bdd6d041819a2aad877d3`
