# Runtime health — sidecamerad r2

**Run:** `sidecamerad-mvp-20261113` · **Design:** r2 · **Build checkpoint:** 54  
**Observed:** 2026-11-13 08:06 UTC · **Target:** local `/data/openpilot` host

## Result

The module entry point stayed alive with no camera paths, and a configured front-left feed published matching frames through cereal and VisionIPC for five minutes. The runtime smoke is functionally successful. The r2 one-camera cadence acceptance **does not pass**: although the run reached 6,001 frames at a 20.00036 fps mean, its maximum EOF-to-EOF gap was 95.742 ms, over the 60 ms limit. The log retained only the summary statistics and count of over-limit gaps, not every interval or a histogram.

This report does not close the five-minute cadence criterion or four-camera sign-off. AC-8's four-camera, 30-minute soak and CPU/headroom evidence remain pending because only one decoder is available.

## No-path entry-point probe

Ran `python3 -m system.sidecamerad` with all `SIDECAM_*` variables unset and an isolated temporary `PARAMS_ROOT`. The corrected third attempt observed the exact four-key Python dictionary with all booleans false; one missing-feed warning for each of the four service names; no sidecamerad VisionIPC socket or capture worker; and liveness through a 62.000-second bounded wait. SIGTERM produced exit code 0, and the socket remained absent. The process had seven Python/ZMQ runtime threads; none was a side-camera worker. The two earlier attempts in the log stopped on probe-harness assertions (an incorrect one-thread assumption and a missing `/proc/.../children` file); attempt 3 corrected those checks and passed.

## Five-minute live IPC probe

Ran the module entry point with only `SIDECAM_FRONT_LEFT` set to `/dev/v4l/by-path/platform-xhci-hcd.11.auto-usb-0:1:1.0-video-index0`, using an isolated temporary Params root. The concurrent consumers subscribed to `frontLeftCameraState` and `VISION_STREAM_SIDE_FRONT_LEFT`.

| Signal | Observed result |
|---|---|
| VisionIPC endpoint | Connected at 1280×720; stride 1,280; 1,382,400-byte buffers; three buffers |
| Frames | 6,001 unique frames on cereal and 6,001 on VisionIPC during a 300.037-second capture window |
| IDs | 6,001 matching IDs; first matching ID 1; logged matching sample 1–5 |
| Payload | 1,382,400-byte VisionIPC frame; sampled 4,096 bytes and all 4,096 were nonzero; VisionIPC marked the frame valid |
| Param transition | Front-left became true after frame ID 1 was observed on both channels; other three fields remained false |
| Cadence | Mean 20.00036 fps over a 299.99454-second timestamp span; maximum gap 95.741769 ms; 709/6,000 intervals exceeded 60 ms |
| Shutdown | SIGTERM exit code 0; all four Param fields false; VisionIPC socket removed |

The logged cereal and VisionIPC first/last EOF timestamps and cadence summaries matched. The probe retained first/last timestamps and aggregate gap statistics but not each frame's timestamp or a gap histogram. The exact per-channel observations, payload sample, Param values, and process output are in `evidence/runtime-smoke.log`.

## Dependency, cleanup, and limits

- **Reached:** the configured V4L2 capture path, FFmpeg-backed decode path, cereal camera-state channel, and sidecamerad VisionIPC stream; observed frame payloads on both publication paths.
- **Not exercised:** the other three physical feeds, four-worker CPU/RSS behavior, 30-minute four-camera soak, and controlled disconnect/reconnect. The one-camera probe does not establish those results.
- **Isolation and teardown:** both probes used temporary `PARAMS_ROOT` values rather than `/data/params`. The process exited cleanly and the VisionIPC socket was absent afterward. This IPC validation wrote no video file and did not interact with the user's media directory.
- **Discovery helper:** `check_runtime.py --detect-start-command` could not complete because its harness Python requirement was 3.13 while the project environment is Python 3.12.3. The handoff supplied `python3 -m system.sidecamerad`; that command ran successfully in the project's supported Python range.
- **Save path:** the destinations supplied by the active handoff resolved without directory creation. An earlier resolver call with `--mkdir` refused the pre-existing workspace-local `skillset-saves` tree; no save path or symlink was changed.

## Evidence binding

Executed log: `evidence/runtime-smoke.log`  
SHA-256: `fe9df2e3ef6f4b1dc30fa20217bb635b22c2d8471499c03108771e65a3edd1c6`

The typed probe record is `artifacts/runtime_probe.json`. It records a passing functional entry-point/IPC smoke while separately recording the failed cadence threshold; `pass` here does not mean the one-camera cadence criterion or AC-8 passed. Source and binding hashes are in that record.
