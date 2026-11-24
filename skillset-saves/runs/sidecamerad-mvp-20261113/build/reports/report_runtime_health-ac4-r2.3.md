---
submission_id: sidecamerad-mvp-20261113_build-r2.3-ac4-handoff-c68
revision: "68"
run_id: sidecamerad-mvp-20261113
health_check_submission_id: sidecamerad-mvp-20261113_health-check-ac4-r2.3
health_check_revision: build-ac4-r2.3
---

# Runtime health — AC-4 five-minute acceptance rerun

**Result:** runtime frame publication and cleanup **pass**; strict AC-4 cadence **fail**. This was the single authorized run on the assigned front-left UVC device using unchanged product source and default sidecamerad FFmpeg argv.

## Acceptance measurement

- Selected matched cereal/VisionIPC frames: **6050** over **302.471744064 seconds** of EOF-to-EOF steady-state data (target 6,050 and at least 300 seconds).
- Mean EOF interval rate: **19.998562241636 fps** (required >=20.0).
- Maximum EOF gap: **90.981562 ms**; intervals over 60 ms: **716 of 6049** (required zero).
- Frame-ID gaps: **0**; cereal/VisionIPC SOF/EOF, validity, or `logMonoTime` mismatches: **0**.
- AC-4 outcome: **FAIL**. The frame count/span and ID/timestamp matching criteria passed; mean cadence and max-gap criteria failed.

## Runtime and cleanup

- Assigned target `/dev/v4l/by-path/platform-xhci-hcd.11.auto-usb-0:1:1.0-video-index0` resolved to `/dev/video60`; the V4L2 mode was MJPG 1280x720 at 20 fps.
- The existing start command was `['/usr/local/venv/bin/python3', 'system/sidecamerad.py']` with child environment `ZMQ=1`, an isolated temporary `PARAMS_ROOT`, and only `SIDECAM_FRONT_LEFT` set to the assigned path. Product source and production argv were unchanged.
- Exact controller command: `/usr/local/venv/bin/python3 .harness-state/sidecam-healthcheck-ac4-r2.3.py`; child start argv: `['/usr/local/venv/bin/python3', 'system/sidecamerad.py']`.
- Source input hashes matched the saved package before and after capture; drift: `{}`.
- Params baseline was null because it was sampled before launching sidecamerad; the `STARTUP` log line prints that pre-launch value after `Popen` and does not establish runtime startup state. The run observed front-left=true before shutdown and all camera states=false after shutdown. Startup all-false remains unverified.
- Sidecamerad exit code: **0**; remaining FFmpeg PIDs: `[]`; TCP listeners on 9007–9010 after shutdown: `[]`. The isolated Params root was removed.
- The controller retained frame IDs and timestamps only. No frame payload or video was saved; `/data/params` was untouched.

## Evidence and limits

- Per-frame matched IDs and EOF intervals: `artifacts/runtime_cadence_intervals-r2.3.json` (SHA-256 `626f0881c22814e055d724ce8683c5206d5c3090dd507e582327362202f53696`).
- Runtime smoke log: `evidence/runtime-smoke-ac4-r2.3.log` (SHA-256 `74722d2c6d4ca799f92529c2b2ce256cf7c6531ecaccbf2d8071d3139c7df19b`).
- Typed probe: `artifacts/runtime_probe-ac4-r2.3.json` (SHA-256 `e018216e41f7f94f0684e45b21f84ca441cfacf2a954a51888ce1def8610cb35`).
- The original controller classification of `functional_pass=false` was a label caused by the unobserved runtime startup Params state; the appended post-capture review separates this from the observed passing frame publication and cleanup. The cadence series is unchanged.
- AC-6 physical reconnect/sibling continuity remains unverified because only one USB decoder is attached. AC-8 four-camera 30-minute soak and resource headroom remain unverified; the FFmpeg resource-cap residual remains open.
- Preserve the cadence failure and route the unresolved timing mechanism through Admiral/investigation. No speculative source fix was made.
