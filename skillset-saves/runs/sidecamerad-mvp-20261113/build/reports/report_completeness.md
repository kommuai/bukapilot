---
submission_id: sidecamerad-mvp-20261113_build-r2.3-ac4-handoff-c68
revision: "68"
admiral_run_revision: "140"
run_id: sidecamerad-mvp-20261113
---

# Build completeness cross-check — SideCamera r2.3 evidence refresh

**Boundary:** `build-to-review` · **Build/source input revision:** 68 · **Admiral run revision at handoff:** 140 (separate lineage)  
**Submission:** `sidecamerad-mvp-20261113_build-r2.3-ac4-handoff-c68` · **Review owner:** `cross-check-build-confirm`

## Result

**Completeness threshold not met:** 6 of 9 rows are proven, 0 are proven by waiver, and 3 are unproven. All rows must be proven to meet the completeness threshold. This report records evidence completeness and gaps; it makes no gate decision.

The approved baseline is design r2, with valid `APPROVED` `design-to-build` verdict. Package source/input revision 68 remains unchanged. The r2.3 acceptance run used the current source hashes; the registered r2.3 runtime report and typed probe match the current package submission and revision while preserving the distinct health-check origin metadata. All registered artifact hashes, all implementation/test/runtime input hashes, and all 10 changed-file paths in the recovery diff matched their package records before this report replacement. No source, test, manifest, build-package, or security evidence changed during this cross-check.

## Evidence lineage normalization

The current r2.3 runtime report front matter identifies package submission `sidecamerad-mvp-20261113_build-r2.3-ac4-handoff-c68`, package revision `68`, and health-check origin `sidecamerad-mvp-20261113_health-check-ac4-r2.3` / `build-ac4-r2.3`. The typed probe carries the same package identity and origin fields; its `input_revision` is 68. These match the manifest and do not use Admiral run revision 140 as the package revision. The earlier r2.2 report has the current package submission/revision while retaining `source_submission_id: sidecamerad-mvp-20261113_build-r2.2-c68`; its on-disk SHA-256 is `07c2d43997f77a8e0cbc723d44c4e5c58d8a5a55ab84434388bb687d79fdbd86`. That historical report is not registered in the current manifest and is used here only to verify preserved origin lineage, not to satisfy any current evidence cell.

The current-source five-minute capture produced 6,050 selected matching cereal/VisionIPC frame IDs across 302.471744064 seconds. Mean EOF interval rate was 19.998562241636 fps (required >=20.0); maximum EOF gap was 90.981562 ms; 716 of 6,049 intervals exceeded 60 ms (required zero). IDs were contiguous, with zero timestamp/validity mismatches. Thus AC-4 cadence **fails** even though live publication and cleanup passed.

The raw per-frame interval artifact has `functional_pass=false` and `params_transition_pass=false` because the startup Params sample was unobserved. The typed probe and runtime report classify frame publication/cleanup as functional pass; they explicitly mark startup all-false unverified and record front-left=true before shutdown and all-false after shutdown. This is a controller observation gap, not a product failure claim. The diagnostic does not close AC-4.

## Traceability matrix

The artifact IDs resolve to exact registered paths and SHA-256 values in the inventories below. `T1` is the r2.2 focused pytest log; all listed selectors are `PASSED`. `S1` is the registered security evidence record. Runtime is package-level and separate from AC-4 cadence and AC-8 soak acceptance.

| Decision | Changed artifact(s) | Test evidence | Security evidence | Status | Reason |
|---|---|---|---|---|---|
| AC-1 — default-on manager lifecycle | A1–A3 | T1: `test_manager_enables_sidecamerad_by_default_and_keeps_camerad` | S1: `SEC-SIDEIPC-001`, verified | proven | — |
| AC-2 — empty configuration | A1, A2, A4 | T1: `test_no_paths_acknowledges_each_feed_and_waits_for_signal`; `test_main_removes_inherited_zmq_before_ipc_and_pubmaster`; `test_params_default_and_json_dict_round_trip`. No-path runtime stayed alive 62.041 s and exited cleanly. | S1: `SEC-SIDEIPC-001`, verified | proven | — |
| AC-3 — availability state | A1, A2, A4–A9 | T1: `test_params_default_and_json_dict_round_trip`; `test_availability_writes_only_state_transitions_and_recovers_write_failure`; `test_worker_clears_availability_after_capture_failure`. Earlier current-binding smoke observed front-left live and false after shutdown; latest startup sample is unverified. | S1: `SEC-SIDEIPC-001`, verified | proven | — |
| AC-4 — one-camera capture and cadence | A1, A2, A6–A10 | T1: `test_main_removes_inherited_zmq_before_ipc_and_pubmaster`; `test_side_camera_service_stream_and_schema_mapping_is_stable`. Current runtime evidence R2.3 includes matched IDs, per-frame EOF intervals, and the failing measurement above. | S1: `SEC-SIDEIPC-001`, verified | unproven | `missing-artifact` — no acceptance-passing five-minute capture exists; current R2.3 evidence fails mean and maximum-gap limits. Mechanism remains unresolved. |
| AC-5 — explicit map and interface compatibility | A1, A2, A6–A10 | T1: `test_explicit_device_paths_resolve_only_to_video_character_nodes`; `test_side_camera_service_stream_and_schema_mapping_is_stable`. Mapping and path validation cover all declared services; physical runtime was front-left only. | S1: `SEC-SIDEIPC-001`, verified | proven | — |
| AC-6 — worker isolation and reconnect | A1, A2 | T1: `test_worker_clears_availability_after_capture_failure`; `test_availability_writes_only_state_transitions_and_recovers_write_failure`. These are mocked checks; the host had only one USB decoder. | S1: `SEC-SIDEIPC-001`, verified | unproven | `missing-artifact` — physical loss/reconnect, child reap/backoff, and sibling/native-camera continuity were not exercised. |
| AC-7 — application resource envelope | A1, A2, A5, A8 | T1: `test_side_camera_service_stream_and_schema_mapping_is_stable`. Source inspection confirms 4 × (3 VisionIPC buffers + 1 reusable scratch frame) × 1,382,400 bytes = 22,118,400 bytes (21.09 MiB) application image payload. | S1: `SEC-FFMPEG-RESOURCE-001`, open Minor; FFmpeg internal CPU/memory remains unbounded on this host. | proven | — |
| AC-8 — four-camera performance sign-off | A1, A2, A8 | T1 covers interface shape only. R2.3 is a single-camera run; no four-camera 30-minute soak or CPU-headroom evidence exists. | S1: `SEC-FFMPEG-RESOURCE-001`, open Minor; its reopen trigger applies before four-camera sign-off or production. | unproven | `missing-artifact` — no four-camera 30-minute run with >=36,000 published frames per feed, per-camera cadence/gap analysis, and CPU headroom. |
| runtime — package startup and dependency reachability | A1 | T1: `test_no_paths_acknowledges_each_feed_and_waits_for_signal`; `test_main_removes_inherited_zmq_before_ipc_and_pubmaster`. R2.3 is a current-source startup/publication/cleanup pass; Params startup all-false is unverified. | S1: `SEC-SIDEIPC-001`, verified | proven | Sidecamerad started; local Params/cereal/VisionIPC were reachable; dual publication, no-path liveness, clean shutdown, and no TCP listeners were observed. Package-level runtime pass does not close AC-4 or AC-8. |

## Changed-artifact inventory

All ten paths in `evidence/implementation-recovery-r2.2.diff` match the manifest's `implementation.changed_files` set and current file hashes.

| ID | Repository path | SHA-256 | Classification |
|---|---|---|---|
| A1 | `system/sidecamerad.py` | `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b` | added source |
| A2 | `system/manager/test/test_sidecamerad.py` | `cf9fb49f514ac90458d47d825716a30b296b8cb25880167695c6a86a6660a059` | added test |
| A3 | `system/manager/process_config.py` | `b7fdfda2878cae1fa6b354ed512bc33ad66f1abe1ed25c90ed980b547a8aa02d` | modified source |
| A4 | `common/params_keys.h` | `c2925ebd8f9f26cc04951afb0e3c67ac7cb20a1c370b79aa93a25a4aaef2a031` | modified Params declaration |
| A5 | `common/params_pyx.so` | `3108e29407f31b9bd86a86ccafc82a4f98a225884412987a24be5c357dbfb586` | generated Params binding |
| A6 | `cereal/log.capnp` | `d5c0372d714e65461942874c91c522b205ce208261b382283839ed03b9731f64` | modified schema |
| A7 | `cereal/services.py` | `cacd4b749d9a0a3e19e3c466784dcf34925f263fdca3c26af72578c47d740a6b` | modified source |
| A8 | `msgq_repo/msgq/visionipc/visionbuf.h` | `c971a1afa601f5bb8f560de6fb1dd78c2121cee61ea41c169f00a4154566e870` | modified VisionIPC source |
| A9 | `msgq_repo/msgq/visionipc/visionipc_pyx.pyx` | `36247e353901eb80e52735c91f466151b3026e77152dde9ad9e3bcd682c66f32` | modified VisionIPC source |
| A10 | `msgq_repo/msgq/visionipc/visionipc_server.cc` | `36b383bfcaf10eaa69f6844a579c86204b4a07464a889208fe059a6c1202d198` | modified VisionIPC source |

The generated `.so` is build output, not hand-authored source. The three `msgq_repo` paths are project-owned submodule changes within approved r2 scope.

## Evidence bindings and integrity

| ID | Package path | SHA-256 / result |
|---|---|---|
| D1 | `../design/manifest.json` | `b090c5d4952b97d66f9577f0778a8ef5c2581d2399c97bd745eb49d4dd24c837` |
| D2 | `../design/verdict_design-to-build.cross-stage.json` | `c918ff4c9f44d6fea1f450f4dfef7498eef03f3fb922186d1ff3a337cd442038` — APPROVED, valid |
| D3 | `../design/reports/delivery-plan-r2.md` | `0ef16ba626a8fb8cd117b710375d63d5d46e677cae4925abe2af919eb22eeffc` |
| D4 | `../design/reports/implementation-spec-r2.md` | `447269713326cd2a99224014d2c145a5b49d9b13dd51eb145fac7193dd00126b` |
| I1 | `artifacts/implementation-recovery-r2.2.json` | `4da275a592a8901f3da0d168ff006fb35904e5e755fbe2faa28492f9f6b48a35` |
| I2 | `evidence/implementation-recovery-r2.2.diff` | `caa56a37320eb173be0d108ed995e9a3b543e750b5d3be821e8091d18f2b36ed` |
| T1 | `evidence/tests-pytest-r2.2.log` | `6483593c0ab82eaf9e22a0e2ce423a35bf206b7325f94aa8ba37772aa3fff519` — 8/8 passed, inputs unchanged |
| TP | `artifacts/tests_probe-r2.2.json` | `9ce5fe481793099cb044aa4fd08825b1f2b33e64d8704f971b8bc2dbffef11fe` |
| R1 | `evidence/runtime-smoke-r2.1-post-merge.log` | `f8a67011b9499ad47ab0b4b90f867ef9790eeeee92395c9c50abeda2cd9351e8` — prior 8-second single-camera functional smoke |
| RP1 | `artifacts/runtime_probe-r2.1-post-merge.json` | `2da84e766bcebabbcdd3b45d26025eb321f2cd4da6a87f3742d7c9b5ffcb40d7` — input revision 68; 9 source hashes match |
| R2 | `reports/report_runtime_health-ac4-r2.3.md` | `2118c1a3a04ab3ca82d8fa1ef38fdd82855537520c528f5a0801d5aa1c3ae8d2` — current-source runtime functional pass / AC-4 fail |
| RL2 | `evidence/runtime-smoke-ac4-r2.3.log` | `74722d2c6d4ca799f92529c2b2ce256cf7c6531ecaccbf2d8071d3139c7df19b` |
| RI2 | `artifacts/runtime_cadence_intervals-r2.3.json` | `626f0881c22814e055d724ce8683c5206d5c3090dd507e582327362202f53696` — 6,050 matched per-frame records |
| RP2 | `artifacts/runtime_probe-ac4-r2.3.json` | `e018216e41f7f94f0684e45b21f84ca441cfacf2a954a51888ce1def8610cb35` — package revision 68 and health-check origin recorded; source hashes match; strict cadence failed |
| G1 | `reports/report_gdb-sampling-r7.md` | `d7412919734b279918b23d0ef265f44a1117a2cbabe1904ace84c834565d0fb8` — bounded diagnostic, not acceptance evidence |
| G2 | `evidence/gdb-sampling-r7.log` | `adb417c4888e65b16e13cb100370a4cb961307469bee7599696580bdf2ce24a9` |
| G3 | `artifacts/ffmpeg-gdb-sampling-r7.json` | `c059e3000c236a6f05cb336fe9afbdecdadc393228f4b77c7f05ec5e029e4782` |
| G4 | `reports/report_gdb-sampling-r7-attempt1.md` | `4302a653ae5b14a9a7a6afd194b2e489465c1067033c087ba15c1872880b3dda` — failed controller attempt preserved |
| G5 | `evidence/gdb-sampling-r7-attempt1.log` | `825c970caea2cb44ff0ead6a57db25a73b08884067b00dd343b28e41b59f770f` |
| G6 | `artifacts/ffmpeg-gdb-sampling-r7-attempt1.json` | `449a90ffacefd409befce3f8758b81c3aa522fdab2e0c941eecfc878ae6b5e12` |
| S1 | `artifacts/security_evidence.json` | `218259045da268b160ec7be24f60be302abbcb8558a049a2ea77dec846830688` |
| SCAN | `evidence/security-scan.json` | `077684993c9cc2bc57ec7619773b1278f9cde319f6d1e06a99faddb49a16d4a5` — scanner `not-run`, not a clean scan |

Hash audit: before this report replacement, all 59 registered package artifacts existed and matched their manifest digests. When this replacement report was written, the manifest still carried the previous `reports/report_completeness.md` digest `aafb7233af394a2596d4956b6fd9aebfd3688bc3fca3901b751ff9065cf9663f`; build-management must rebind the new report before final package self-check. Excluding the replaced completeness report, the other 58 registered artifacts match. The 10 implementation changed-file hashes match the recovery record and diff; the test, implementation, and runtime input hashes match current files. Runtime input revision is 68. The current runtime probe recorded no source hash drift. No revision mismatch was found.

The r7 GDB diagnostic captured 600/600 frames with three snapshots and no source drift. The rawvideo mux thread was observed in `write()` at all snapshots; the main thread was in condition waits twice and `sws_scale_frame()` once. These are callsite observations only; GDB pauses perturb timing, do not establish causality, and do not satisfy AC-4.

## Open gaps and owners

| Gap | Severity / owner | Evidence required to close |
|---|---|---|
| AC-4 cadence | Major · Admiral → `investigate` (unknown mechanism) | Settle the timing mechanism and return a bounded fix path. After an authorized fix, run one current-source five-minute capture meeting >=20.0 fps mean with every EOF gap <=60 ms; preserve matched IDs and per-frame EOF intervals. The present r2.3 run misses both cadence thresholds. |
| AC-6 physical reconnect and sibling continuity | Major · `build/health-check` with feature owner/integrator | Controlled loss/replug; false→true after complete dual publication; child close/reap and bounded retry; sibling and native `camerad` continuity. Only one USB decoder is attached. |
| AC-8 four-camera performance sign-off | Major · `build/health-check` with feature owner | Four attached cameras for 30 minutes, >=36,000 published frames per feed / 144,000 total, per-feed cadence/gap analysis, and CPU headroom. Only one USB decoder is attached. |
| `SEC-FFMPEG-RESOURCE-001` | Minor · build/security owner | Before AC-8 or production, establish a target-supported child CPU/memory limit or record explicit residual-risk acceptance. |
| Dependency scan coverage | Info · build/security owner | Run an advisory scan when a scanner is available; current status remains `not-run`, not a pass. |

The completeness threshold is not met; AC-4 is a known failed Major, not a hidden or waived criterion. The build owner should return this package to Admiral for investigation routing. This report makes no gate decision.
