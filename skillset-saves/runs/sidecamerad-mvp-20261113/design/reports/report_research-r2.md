# Research Evidence Index — sidecamerad r2

**Run:** `sidecamerad-mvp-20261113` · **Run revision:** 47  
**Primary research brief:** [`requirements-brief-r2.md`](requirements-brief-r2.md)  
**SHA-256:** `bff3dc6825e257d653772d43d64a3d7f6221ed91bea0dff4c08ce50b82d7f748`

## Findings carried into design

- `sidecamerad` is default-enabled under its existing `driverview` lifecycle; native `camerad` remains independently managed and unchanged. No packages are added, and product edits remain within `/data/openpilot`.
- Keep the four explicit `SIDECAM_FRONT_LEFT`, `SIDECAM_FRONT_RIGHT`, `SIDECAM_BACK_LEFT`, and `SIDECAM_BACK_RIGHT` assignments. Prefer `/dev/v4l/by-path/...-video-index0` for a configured USB port, while continuing to accept `/dev/videoN`; do not infer side labels from USB enumeration. The observed `video-index1` entry is metadata-only.
- The proposed single Params key is `SideCameraAvailability`, a JSON object with stable boolean fields `frontLeftCameraState`, `frontRightCameraState`, `backLeftCameraState`, and `backRightCameraState`. Reset all fields to false at boot, set a field true after that feed's first successful published frame, and false on disconnect.
- A default-on process with no configured paths remains alive in a bounded SIGINT/SIGTERM wait, keeps every field false, and emits one warning per missing feed. Explicitly configured but unavailable paths follow the per-worker retry policy.
- Append new cereal service declarations at the end only as an ordering/minimal-diff choice. The `SERVICE_LIST` index is unused and the C++ registry is keyed by service name; no numeric-ID/ABI shift was found.

## Host and validation evidence

- `build/evidence/usb-camera-baseline.log` records one connected decoder: `/dev/video60` (`534d:2109`) provides MJPEG 1280×720 at 20 fps; `/dev/video61` is its metadata node. The observed by-path `video-index0` resolves to `/dev/video60`, and `video-index1` to `/dev/video61`.
- The r1 one-camera integration baseline published `frontLeftCameraState` and VisionIPC frames with matching frame ID 1 and a 1,382,400-byte NV12 payload containing nonzero image data. This is baseline evidence only; rerun V4L2 capture and the matching cereal/VisionIPC frame-ID check on r2 code.
- Limitations: only one decoder is attached; no four-camera availability, sustained throughput, or disconnect/replug recovery has been demonstrated. Same-port recovery through a by-path symlink is an operational inference to verify during build validation.

**Evidence sources:** `intake/report_scope-change.md` (current SHA-256 `5ab1e0b6bc31e2158f8a9126a27d9f25ed132b776528f522ac7015ebcb1ffb39`); `build/evidence/usb-camera-baseline.log` (SHA-256 `9629e9abadb286f4bd2d90f8d9649575c554ef184b648290331073573d6a59e2`); host `readlink` observation for the two by-path entries; prior approved r1 design and validation lineage.
