# Architecture — sidecamerad MVP

**Run/revision:** `sidecamerad-mvp-20261113` / 11  
**Requirements:** [requirements-brief.md](requirements-brief.md) · SHA-256 `1d0813a7fca5e7cee8d987c80842c22fd52ef8dca7dcdd4197632d3fe3040288`

## Components and data flow

| Component | Boundary and failure behavior |
|---|---|
| Process manager | Runs separate `sidecamerad` only at `SIDECAMERAD=1`, under existing `driverview` lifecycle. Native `camerad` remains unchanged. |
| `sidecamerad` | Owns four optional position workers, their state publications, and VisionIPC streams. Missing path omits that camera; failures retry per worker without stopping siblings. |
| Worker + FFmpeg | One worker, FFmpeg child (one thread), and reusable NV12 buffer per configured V4L2 path. On child exit or short read, close/reap, log, then retry that camera with bounded backoff. |
| VisionIPC / cereal | Publishes 3 VisionIPC buffers per configured stream and matching `FrameData` topic at 20 Hz. Consumers subscribe by position. |

For each configured `SIDECAM_FRONT_LEFT`, `SIDECAM_FRONT_RIGHT`, `SIDECAM_BACK_LEFT`, or `SIDECAM_BACK_RIGHT` path: V4L2 MJPEG 1280×720 at 20 fps → existing FFmpeg executable → raw NV12 → position-specific VisionIPC stream. At read completion, stamp host-monotonic `timestampEof = monotonic_ns`; set nominal `timestampSof = timestampEof - 50,000,000 ns` (one target frame period). Streams have no cross-camera synchronization. FFmpeg is limited to one thread; the app has no frame queue or BGR conversion.

## Invariants and risk

- Each state topic maps to one distinct side stream; append stream IDs 7–10 after occupied IDs 0–6 and retain all existing values. Use server/control name `sidecamerad` and add it to the ZMQ endpoint-name allowlist.
- Payload allocation for four cameras is 16 frames total: `(3 shared + 1 scratch) × 4 × 1,382,400` = **22,118,400 bytes (21.09 MiB)**, below the brief’s 27,648,000-byte ceiling. Decoder/kernel buffers and metadata are excluded.
- Configured device paths and MJPEG are untrusted inputs; validate path shape and isolate device/decode errors per worker. Child exit, EOF, or short read closes/reaps and logs FFmpeg, then retries with bounded exponential backoff: 1, 2, 4, 8, 16, then 30 seconds maximum. Local publication exposes no new network service or secrets. Concurrent decoding’s denial-of-service risk is bounded by four workers, one FFmpeg thread each, fixed buffers, and retry backoff.
- Four-camera 80 fps and CPU headroom are **unmeasured**. The single-camera prior result does not establish this target; measure on target hardware before performance sign-off.

## Decision register

Resolved: separate opt-in sidecar, explicit partial path list, FFmpeg MJPEG→NV12, no new packages, bounded buffers (intake D-1–D-6). Rejected: replace `camerad`, require all four devices, or add image packages (D-10–D-12). Deferred until a consumer is approved: logging/encoding, model/UI use, synchronization/transforms (D-7–D-9). YAGNI: auto-discovery and persistent assignment (D-14). No unresolved architecture decision remains; four-camera performance is a verification item.
