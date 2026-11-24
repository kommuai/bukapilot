---
submission_id: sidecamerad-mvp-20261113_build-r2.3-ac4-handoff-c68
revision: "68"
run_id: sidecamerad-mvp-20261113
---

# Build Package — sidecamerad MVP

**Run:** `sidecamerad-mvp-20261113` · **Approved design:** r2  
**Build/source input revision:** 68 · **Package update:** r2.3 AC-4 evidence refresh  
**Admiral run checkpoint:** 136 (a separate lineage) · **Boundary:** `build-to-review` · **Owner:** `build-management`

## Outcome

This package update adds one authorized current-source, five-minute one-camera acceptance run. Product source, production FFmpeg argv, and dependency state did not change. The current source input set matched the build package hashes before and after capture. The runtime functional and cleanup checks passed, while strict AC-4 cadence acceptance failed. This failure is a Major gap with an unresolved timing mechanism, so this package is being returned to Admiral for investigation; it is not being submitted to `gatekeeper-build`.

The implementation remains the r2.2 recovery refresh. Native `camerad` stays enabled; `sidecamerad` defaults on under `driverview` with `SIDECAMERAD=0` as opt-out; four position-specific cereal/VisionIPC interfaces publish availability through `SideCameraAvailability`. The focused tests remain hash-valid at 8/8 because the source and test inputs are unchanged; tests were not rerun in this evidence-only update.

## Acceptance status

| Criterion | Status | Evidence and limitation |
|---|---|---|
| AC-1 default-on manager lifecycle | Proven | Focused test confirms default-on and opt-out while native `camerad` remains enabled. |
| AC-2 no configured paths | Proven | Focused test plus earlier 62.041-second no-path runtime check. |
| AC-3 availability contract | Proven for observed transitions | Unit tests cover defaults, transition-only writes, failure recovery, and capture-failure reset; runtime confirms front-left true while live and all false after shutdown. In r2.3 the startup Params sample was taken before initialization and is unverified. |
| AC-4 one-camera publication and cadence | **Failed** | The single authorized r2.3 capture selected 6,050 matching cereal/VisionIPC frame IDs over 302.471744064 seconds. Mean EOF interval rate was 19.998562241636 fps (required >=20.0); maximum EOF gap was 90.981562 ms; 716 of 6,049 intervals exceeded 60 ms (required zero). Frame IDs were contiguous with zero timestamp/validity mismatches. The prior r2.2 failure also missed the strict gap threshold. Mechanism remains unresolved; no source fix was applied. |
| AC-5 explicit map and compatibility | Proven at interface level | Focused tests verify explicit device path validation and the four service/stream/schema mappings; only front-left was physically connected. |
| AC-6 worker isolation and reconnect | Unproven in full | Mocked failure handling passes; physical disconnect/reconnect, child reap/backoff, and sibling continuity were not exercised. |
| AC-7 application resource envelope | Proven by source review | Four streams allocate 22,118,400 bytes (21.09 MiB) of application image payload; FFmpeg internal memory/RSS and four-camera CPU are excluded. |
| AC-8 four-camera performance sign-off | Pending | No four-camera, 30-minute / 144,000-frame soak or CPU-headroom measurement was run; only one attached USB decoder was available. |

## Runtime evidence

The current AC-4 run used the configured front-left path `/dev/v4l/by-path/platform-xhci-hcd.11.auto-usb-0:1:1.0-video-index0` (resolved `/dev/video60`) at MJPG 1280x720 / 20 fps. It started the unchanged `system/sidecamerad.py` entry point with isolated Params, collected cereal and non-conflated VisionIPC metadata, and stopped at 6,050 matched selected IDs over at least five minutes. No image payload or video was saved; `/data/params` was untouched.

Publication and cleanup passed: `sidecamerad` exited 0, the FFmpeg child was reaped, no TCP listeners remained on 9007–9010, and the isolated Params root was removed. The Params sample taken immediately after process start was null, so startup all-false is unverified; pre-shutdown front-left=true and post-shutdown all-false were observed. The raw interval artifact retains `functional_pass=false` and `params_transition_pass=false` because startup Params was unobserved; the typed probe/report classify frame publication and cleanup as functional pass, without claiming the startup state. This is a controller observation gap, not evidence of failed publication.

Evidence:

- Current runtime report: `reports/report_runtime_health-ac4-r2.3.md` (SHA-256 `2118c1a3a04ab3ca82d8fa1ef38fdd82855537520c528f5a0801d5aa1c3ae8d2`; package revision/submission metadata is separate from preserved health-check origin metadata).
- Typed probe: `artifacts/runtime_probe-ac4-r2.3.json` (SHA-256 `e018216e41f7f94f0684e45b21f84ca441cfacf2a954a51888ce1def8610cb35`; input revision and capture fields are unchanged).
- Raw per-frame matching IDs and EOF intervals: `artifacts/runtime_cadence_intervals-r2.3.json` (SHA-256 `626f0881c22814e055d724ce8683c5206d5c3090dd507e582327362202f53696`).
- Runtime smoke log: `evidence/runtime-smoke-ac4-r2.3.log` (SHA-256 `74722d2c6d4ca799f92529c2b2ce256cf7c6531ecaccbf2d8071d3139c7df19b`).
- Bounded GDB diagnostic: `reports/report_gdb-sampling-r7.md`, `evidence/gdb-sampling-r7.log`, and `artifacts/ffmpeg-gdb-sampling-r7.json`; it completed 600/600 full frames with three snapshots and no source drift. The snapshots place the rawvideo mux thread in `write()`, the main thread in condition waits twice and `sws_scale_frame()` once; these are callsite observations, not causal evidence or AC-4 acceptance.
- Earlier current-binding eight-second smoke remains in `artifacts/runtime_probe-r2.1-post-merge.json` and `evidence/runtime-smoke-r2.1-post-merge.log`; it proves functional startup on one camera, not cadence.

## Security and dependency notes

`SEC-SIDEIPC-001` remains verified. Minor `SEC-FFMPEG-RESOURCE-001` remains open because this host cannot enforce a safe per-child CPU/memory limit; its reopen trigger is before AC-8 sign-off or production. Dependency scanners were unavailable, so the retained scan record is `not-run`, not a clean scan. The generated Params binding is identified as build output, not hand-authored source; the three `msgq_repo` changes are inventoried as project-owned submodule content and remain within approved design scope.

## Evidence lineage and unresolved gaps

- Approved design r2: `../design/verdict_design-to-build.cross-stage.json` (APPROVED).
- Implementation change set: `artifacts/implementation-recovery-r2.2.json`; diff: `evidence/implementation-recovery-r2.2.diff`.
- Tests: `artifacts/tests_probe-r2.2.json` and `evidence/tests-pytest-r2.2.log`; unchanged source/test input hashes mean this evidence remains valid.
- Runtime: r2.3 report, typed probe, matched-frame interval data, and executed smoke log named above.
- Security: `artifacts/security_evidence.json` and `reports/report_security.md`.
- Completeness: `reports/report_completeness.md` was refreshed against the r2.3 package and runtime evidence. It finds 6/9 rows proven, zero waived, and AC-4/AC-6/AC-8 unproven; its SHA-256 is `a7d68fb1985a65b53b7b96a44dd1b8118bec6276115b656baa30a0275b1d3bef`.

AC-4 remains the blocking Major. The measured failure is localized to irregular complete-frame delivery before or at producer EOF timestamping by previous evidence, but the mechanism is not settled; the GDB snapshots are callsite observations only. Admiral should route the case to investigation for a bounded mechanism/fix path before further source edits or another acceptance capture. AC-6 remains a hardware deferral requiring controlled physical reconnect with sibling feeds active. AC-8 remains a hardware deferral requiring four cameras for 30 minutes, per-camera cadence, CPU headroom, and resolution of the open Minor FFmpeg resource-cap issue.

## Package validation status

No `gatekeeper-build` submission is made because the current package still fails AC-4. The completeness report has been refreshed and cross-checked: 6/9 rows are proven, with AC-4/AC-6/AC-8 unproven. Python 3.13.15 ran the canonical `build-to-review` self-check successfully with zero failures and zero warnings. The gatekeeper-build package-shape check returned `NEEDS_JUDGMENT` with zero failures and one expected conditional notice because no gate verdict is being submitted. These checks establish package structure and hash integrity only; they do not change the failed acceptance result or constitute a gate verdict. No product test or source change is in scope for this evidence refresh.
