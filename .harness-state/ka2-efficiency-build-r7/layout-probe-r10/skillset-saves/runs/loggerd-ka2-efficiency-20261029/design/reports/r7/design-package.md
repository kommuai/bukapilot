---
submission_id: loggerd-ka2-efficiency-20261029-design-r7
revision: r7
---

# Design package — KA2 encoder efficiency, r7

## Outcome

One bounded build slice evaluates the existing opt-in MPP DMA-import path for full-resolution road, wideRoad, and driver HEVC streams against the existing copy path on an idle KA2/RK3588 target. Copy remains the default. qRoad stays on its downscale/RGA/copy path as a control. Recording remains best effort; the package requires explicit loss, packet usability, and decoded-content reporting and makes no lossless claim.

## Why this candidate

Source arithmetic gives 3,456,000 NV12 bytes/frame at 20 FPS, or 69.12 MB/s of input-copy traffic per eligible encoder and 207.36 MB/s across the three streams. Their six-slot pools have about 62.2 MB aggregate nominal capacity. These are traffic/allocation estimates, not measured CPU, RSS, or DMA savings.

## Build and measurement path

B1 has four ordered gates: (G0) build from a clean detached worktree, verify idle state and isolated temporary roots, and capture image/resource identity; (G1) switch immediately and stickily to copy after any import failure, copying the current frame and preserving fallback through reopen; (G2) run same-binary, matched synthetic copy-versus-import arms and report frame IDs, loss, packet/decode/content quality, timestamps, segments, CPU, RSS, available DMA/CMA, and qRoad control results; (G3) publish an image-specific qualify/reject/exploratory disposition. Fix test ceilings before the first arm. Stop if competing camera, loggerd, encoderd, sensor, or MPP work appears.

The MPP library/API, imported-buffer completion lifetime, EOS/IDR behavior, packet bounds, and DMA/RSS accounting remain unresolved. A synthetic result can only support a claim about the tested image and workload. No live camera/sensor, on-road recording, production-root writes, production default change, or broad capture/storage rearchitecture is included.

## Compatibility and package decisions

Codec/profile, resolution, FPS, camera and audio selection, timestamps, routes, file/message formats, and published interfaces are preserved. The encoder's existing environment switch selects the isolated A/B arm; ENCODER_ZERO_COPY=0 is the rollback. D-22 through D-25 in the intake log define the active r7 scope and best-effort semantics. The security assessment identifies no new or moved adversarial trust boundary and retains output-integrity and DMA-lifetime risks.

Python 3.13.15 is available for package validation. Codex hooks are missing/optional. Project stack detection is incomplete because its byte limit stopped the scan; r7 adds no runtime or framework and records the sanctioned unchanged-stack decision without claiming a complete detected stack.
