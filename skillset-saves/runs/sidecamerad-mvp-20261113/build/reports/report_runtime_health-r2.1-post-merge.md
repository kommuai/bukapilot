# Runtime Health — sidecamerad r2.1 post-merge smoke

**Run:** `sidecamerad-mvp-20261113` · **Build state:** checkpoint 68 · **Target source HEAD:** `6d92d63a2` · **Approved design:** r2

## Outcome

The current rebuilt Params binding started `system/sidecamerad.py` and published side-camera frames over both local cereal/msgq and VisionIPC. The 8-second one-camera smoke observed 122 unique VisionIPC frames and 117 unique cereal messages, with 117 matching frame IDs. VisionIPC reported 1280×720, stride 1280, and 1,382,400 bytes per frame. The availability Param was true only for front-left while the decoder ran and all false after clean SIGTERM shutdown. No TCP listeners appeared on 9007–9010 even though the child inherited `ZMQ=1`.

This is a current-binding functional smoke only. It does not establish the 20fps cadence threshold or four-camera capacity. The earlier separate five-minute capture remains the cadence evidence: 6,000 matching frames, mean 19.999396661241942 fps, maximum gap 123.185415 ms, and 743 of 5,999 intervals over 60 ms; AC-4 cadence acceptance failed.

## Evidence

- Current-binding smoke: `evidence/runtime-smoke-r2.1-post-merge.log` — SHA-256 `f8a67011b9499ad47ab0b4b90f867ef9790eeeee92395c9c50abeda2cd9351e8`.
- Typed probe: `artifacts/runtime_probe-r2.1-post-merge.json`.
- Earlier five-minute cadence capture: `evidence/runtime-smoke-zmq-r2-final.log` — SHA-256 `899ff2ff4758ce4647b03d90436866a6ea5eebc95cbca65b8de57872b32930dd`.
- Earlier no-path capture: `evidence/runtime-no-path-zmq-r2-final.log` — SHA-256 `e32c356b9440dcb86d0fa1cc09f1b9984c8c2c15cc16004265ee0384b6801971`.
- A probe-controller NameError occurred before the first corrected live attempt started a process or accessed the decoder; it is preserved at `evidence/runtime-smoke-r2.1-post-merge-harness-failure.log` (SHA-256 `21f7bb5aa08a602a3880080ec8796f355ae39d3d7f3072fbbc4ff749961ac7a9`). The corrected runtime smoke passed.

## Environment and dependencies

- **V4L2 decoder:** reachable at `/dev/video60` through the assigned stable by-path symlink.
- **FFmpeg:** reachable; produced 1280×720 NV12 frames.
- **cereal/msgq and VisionIPC:** reachable; 117 matching front-left frame IDs observed.
- **Params:** reachable using a temporary isolated root; all four keys reset false on shutdown.
- **Local IPC:** reachable; no listeners on TCP ports 9007–9010.
- **Start-command discovery:** the helper stopped before inspection because it requires Python ≥3.13 and this host provides 3.12.3. The handoff-authorized local entry command `python3 system/sidecamerad.py` was executed.

The probe stopped its process with exit code 0 and removed its temporary Params root. It created no recording and changed no `/data/params` values. No packages were installed.

## Open limitations

One decoder was attached. Physical disconnect/reconnect and sibling-camera continuity, 4-camera 30-minute cadence and CPU soak remain unverified. The prior strict cadence result is a failure, so the implementation is not signed off for a 20fps cadence target.
