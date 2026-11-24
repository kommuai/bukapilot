# Checkpoint — finalized build handback, r2.3

Run `sidecamerad-mvp-20261113`; Admiral run revision before checkpoint 140. Build/source package revision and runtime input revision remain 68, distinct from the Admiral run revision.

## Result

The single authorized current-source five-minute acceptance run completed with no source drift. It captured 6,050 selected matched cereal/VisionIPC IDs over 302.471744064 seconds. Mean EOF interval rate was 19.998562241636 fps; maximum EOF gap was 90.981562 ms; 716 of 6,049 intervals exceeded 60 ms. IDs were contiguous with zero timestamp/validity mismatches. Runtime publication and cleanup passed; startup Params all-false remains unverified because the sample preceded initialization. The raw interval artifact's false functional/Params flags are preserved and distinguished from the typed publication/cleanup pass.

AC-4 remains failed on mean cadence and maximum gap, with unresolved timing mechanism. No speculative source fix was made, no test was rerun because implementation/test hashes were unchanged, and no additional capture was performed. AC-6 physical reconnect/sibling continuity and AC-8 four-camera soak/headroom remain unverified hardware deferrals. Minor `SEC-FFMPEG-RESOURCE-001` remains open. GDB stacks are callsite observations only; they do not establish the mechanism.

## Package and validation

Package metadata is coherent at build revision 68. The current r2.3 health report/probe carry package submission ID `sidecamerad-mvp-20261113_build-r2.3-ac4-handoff-c68` and separate original health-check origin metadata; the typed probe retains `input_revision: 68`. The historical r2.2 report carries current package-level lineage and preserves its original ID as `source_submission_id`. The refreshed completeness report records 6/9 rows proven; AC-4, AC-6, and AC-8 remain unproven.

Both local validators were run with Python 3.13.15 against the final manifest/package hashes:

- Canonical `build-to-review` self-check: **PASS**, `pass=true`, zero failures, zero warnings, mechanical-only; package revision 68, submission `sidecamerad-mvp-20261113_build-r2.3-ac4-handoff-c68`, fingerprint `794e18f6d7321b90731d7382ef23bbba9ff1279b807fa5eba48fb72438f02410`. No verdict output was written.
- Gatekeeper-build package-shape check: **NEEDS_JUDGMENT**, zero failures, one expected informational `ARTIFACT_CONDITIONAL` because this failed package is not being submitted and therefore has no new gate verdict. Revision and submission lineage are coherent.

Do not submit to gatekeeper-build. Admiral should route the unresolved AC-4 timing mechanism to investigation. No new capture is authorized until a bounded fix path returns; preserve AC-6/AC-8 deferrals.

## Final package hashes

- `build/manifest.json`: SHA-256 `24c6293dc4dd12366b6fda838ebac7170f94b01e9441b426ad72378f7f3e6248`.
- `build/reports/build-package.md`: SHA-256 `98866844cd5700acfc779ce9edd12a241fdc68edf10e9380a76fca278ea7ff41`.
- `build/reports/report_completeness.md`: SHA-256 `a7d68fb1985a65b53b7b96a44dd1b8118bec6276115b656baa30a0275b1d3bef`.
- `build/reports/report_runtime_health-ac4-r2.3.md`: SHA-256 `2118c1a3a04ab3ca82d8fa1ef38fdd82855537520c528f5a0801d5aa1c3ae8d2`.
- `build/evidence/runtime-smoke-ac4-r2.3.log`: SHA-256 `74722d2c6d4ca799f92529c2b2ce256cf7c6531ecaccbf2d8071d3139c7df19b`.
- `build/artifacts/runtime_cadence_intervals-r2.3.json`: SHA-256 `626f0881c22814e055d724ce8683c5206d5c3090dd507e582327362202f53696`.
- `build/artifacts/runtime_probe-ac4-r2.3.json`: SHA-256 `e018216e41f7f94f0684e45b21f84ca441cfacf2a954a51888ce1def8610cb35`.
- `build/reports/report_gdb-sampling-r7.md`: SHA-256 `d7412919734b279918b23d0ef265f44a1117a2cbabe1904ace84c834565d0fb8`.
- `build/evidence/gdb-sampling-r7.log`: SHA-256 `adb417c4888e65b16e13cb100370a4cb961307469bee7599696580bdf2ce24a9`.
- `build/artifacts/ffmpeg-gdb-sampling-r7.json`: SHA-256 `c059e3000c236a6f05cb336fe9afbdecdadc393228f4b77c7f05ec5e029e4782`.
