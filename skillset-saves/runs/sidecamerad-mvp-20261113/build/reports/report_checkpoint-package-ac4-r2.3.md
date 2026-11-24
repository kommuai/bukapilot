# Checkpoint — r2.3 build package reconciliation

Run `sidecamerad-mvp-20261113`; Admiral run revision before checkpoint 136. Build/source input revision remains 68. No product source or production argv changed.

The r2.3 health-check files are finalized and registered: 6,050 matched frames over 302.471744064s; mean EOF interval rate 19.998562241636 fps; max gap 90.981562ms; 716/6,049 gaps >60ms; zero ID gaps/timestamp mismatches; source hashes unchanged. Functional publication and cleanup pass, while strict AC-4 fails. The startup Params sample is unverified; pre-shutdown front-left=true and post-shutdown all-false were observed. The raw interval artifact still has functional/Params flags false because startup state was unobserved; this is explicitly distinguished from the typed functional publication/cleanup pass.

`build/manifest.json` has been rebound to the current typed runtime evidence and r7 GDB diagnostic; `reports/build-package.md` now records AC-4 as failed and names the unresolved mechanism. AC-6 and AC-8 remain hardware-deferred, and the Minor FFmpeg resource-cap issue remains open. The completeness report still needs an independent refresh against this package revision.

Next: delegate the read-only completeness cross-check, then refresh final manifest/report hashes and run local package self-checks for handback. Do not submit this failed package to gatekeeper-build, edit source, or repeat capture; return the unresolved timing mechanism to Admiral for investigation.

## Package hashes

- `build/manifest.json`: SHA-256 `c40a3cee82cbbdd1a946c823a3625c43a3f7aa845526de9ca337ad9d6fbd51ea`.
- `build/reports/build-package.md`: SHA-256 `47b1645d3badf1a0452fb293e477566b6cf0fd99935c2d5a3e42886965ab1c47`.
- `build/reports/report_runtime_health-ac4-r2.3.md`: SHA-256 `289cce08c02060e5432db798da8ffbad45454f2d57c1bcfae1b8a78c9f4f58bf`.
- `build/evidence/runtime-smoke-ac4-r2.3.log`: SHA-256 `74722d2c6d4ca799f92529c2b2ce256cf7c6531ecaccbf2d8071d3139c7df19b`.
- `build/artifacts/runtime_cadence_intervals-r2.3.json`: SHA-256 `626f0881c22814e055d724ce8683c5206d5c3090dd507e582327362202f53696`.
- `build/artifacts/runtime_probe-ac4-r2.3.json`: SHA-256 `780c9e20f0f7a0d7eb8369a16beac521d6d4390f588bdd6d041819a2aad877d3`.
