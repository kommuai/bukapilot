# Runtime health — sidecamerad r2.1

**Run:** `sidecamerad-mvp-20261113` · **Design:** r2 · **Implementation:** r2.1 local-IPC hardening  
**Observed:** 2026-11-13T08:36:56.941003+00:00 · **Target:** local `/data/openpilot` host

## Outcome

The final `python -m system.sidecamerad` entry point passes the no-path liveness and five-minute dual-publication functional checks. The sidecar ran with inherited `ZMQ=1`, removed it before creating local IPC publishers, and served all 6,000 matching camera-state and VisionIPC frame IDs over the local endpoint. No listeners appeared on TCP ports 9007–9010. Orderly shutdown cleared every availability flag, removed the VisionIPC socket, and exited 0.

The strict one-camera cadence acceptance **fails**. The capture averaged 19.999396661 fps (below the 20.0 target), had a maximum EOF gap of 123.185415 ms, and 743 of 5,999 intervals exceeded 60 ms. The frame count and matching IDs pass; no timestamps were smoothed. This packet therefore claims functional publication and local-only IPC, not AC-4 cadence acceptance or four-camera readiness.

## No-path entry-point probe

Ran the final r2.1 module entry point with all four `SIDECAM_*` variables unset, child `ZMQ=1`, and an isolated temporary Params root. The process stayed alive for 62.041 seconds, long enough to cross its bounded 60-second wait, with exactly the all-false four-key availability dictionary. It warned once for each missing feed, created no VisionIPC socket or TCP listener, then handled SIGTERM with exit 0 and left availability all false.

The complete output is `evidence/runtime-no-path-zmq-r2-final.log` (SHA-256 `e32c356b9440dcb86d0fa1cc09f1b9984c8c2c15cc16004265ee0384b6801971`).

## Five-minute live IPC probe

Ran with only `SIDECAM_FRONT_LEFT` set to `/dev/v4l/by-path/platform-xhci-hcd.11.auto-usb-0:1:1.0-video-index0`, an isolated Params root, and `ZMQ=1` in the sidecamerad child only. The controller and both readers used the local backend. It subscribed concurrently to `frontLeftCameraState` and `VISION_STREAM_SIDE_FRONT_LEFT`.

| Signal | Observed result |
|---|---|
| Endpoint | Local VisionIPC connected; 1280×720, stride 1,280, 1,382,400-byte buffers, three buffers |
| Frames | 6,000 complete unique frames on cereal and 6,000 on VisionIPC over 300.005 seconds |
| IDs | All 6,000 IDs matched; first matching ID 1 |
| Payload | 1,382,400 bytes; sampled 4,096 bytes, all nonzero; marked valid |
| Availability | All false before capture; front-left true after first dual publication; other three false; all four false at shutdown |
| Cadence | Mean 19.999396661 fps; maximum gap 123.185415 ms; 743/5,999 intervals exceeded 60 ms; strict cadence acceptance failed |
| ZMQ security | No TCP listeners on 9007–9010 after startup, at the end of capture, or after shutdown; local socket removed |
| Cleanup | SIGTERM exit 0; no sidecar or FFmpeg process remained |

The live output is `evidence/runtime-smoke-zmq-r2-final.log` (SHA-256 `899ff2ff4758ce4647b03d90436866a6ea5eebc95cbca65b8de57872b32930dd`). The probe records aggregate gap statistics but not the full per-frame interval series.

## Attempts, cleanup, and limits

The first controller attempt ended after the first matched frame with a harness `NameError` (`buf`); it shut down cleanly and did not complete a five-minute capture. That attempt is preserved in `evidence/runtime-smoke-zmq-r2.log`. The corrected controller completed the full five-minute window and supplies the runtime evidence above.

The probes used isolated temporary Params roots and wrote no video file; `/data/params` was untouched. Only one decoder was available. AC-6 disconnect/reconnect and sibling continuity remain untested. AC-8's 30-minute four-camera soak, 144,000 total frames, per-camera cadence, and CPU headroom are pending. The application-level image allocation is 22,118,400 bytes at four cameras; FFmpeg internals, V4L2/kernel buffers, and total RSS are excluded.

## Evidence binding

- Five-minute log: `evidence/runtime-smoke-zmq-r2-final.log` · SHA-256 `899ff2ff4758ce4647b03d90436866a6ea5eebc95cbca65b8de57872b32930dd`
- No-path log: `evidence/runtime-no-path-zmq-r2-final.log` · SHA-256 `e32c356b9440dcb86d0fa1cc09f1b9984c8c2c15cc16004265ee0384b6801971`
- Typed probe: `artifacts/runtime_probe.json`
- Source and binding hashes: recorded in the typed probe, all checked against the live workspace before publication.
