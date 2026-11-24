---
run_id: sidecamerad-mvp-20261113
submission_id: sidecamerad-mvp-20261113_investigation-ac4-r8_2026-11-15T14:52:54Z
revision: "8"
---

# Investigation Package — SideCamera AC-4 cadence failure

**Boundary:** `investigation-review` · **Owning phase:** BUILD · **Build input revision:** 68 · **Investigation revision:** 8

## Scope

Investigate the current-source, one-camera AC-4 cadence failure for the assigned front-left UVC camera at 1280×720 MJPEG and requested 20 fps. AC-6 reconnect/sibling continuity and AC-8 four-camera soak/resource headroom remain outside this investigation. No source or production FFmpeg argv change is proposed. The r7 package, manifest, approved verdict, and evidence chain remain preserved under revisioned paths; r8 binds the immutable archived r7 verdict, not the mutable canonical verdict path.

## Reproduction

The existing r2.3 health-check is the executed reproduction, captured on 2026-11-15 from 14:13:11.581561 through 14:18:18.055618 UTC. It selected 6,050 matched cereal/VisionIPC frame IDs over 302.471744064 seconds. Mean EOF cadence was **19.998562241636 fps** against a 20.0 fps minimum. The maximum EOF gap was **90.981562 ms**, with **716/6,049 intervals above 60 ms**. AC-4 therefore fails both cadence thresholds. IDs were contiguous and timestamp/validity comparisons had zero mismatches. Publication and cleanup passed; sidecamerad exited 0, reaped its FFmpeg child, and left no listeners. Startup all-false Params remains unverified.

This is a reused executed health-check, not a new r8 reproduction. The manifest points directly to the original smoke log, typed probe, and per-frame interval artifact under `build/`; no derived text is represented as executed output. Source hash for `system/sidecamerad.py` is unchanged at `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b`.

## Observed evidence chain

The hash-bound r8 chain is `evidence/evidence-chain-r8.log` (SHA-256 `8a6456d11053abf666f3517f9122232bf3213d21e2fe7f04a182c4ed88b11ab8`). It records each observation, its timestamp and source path, then separates inference from observations. Key findings:

- The earlier independent V4L2-only capture had regular post-startup source intervals; a separate FFmpeg-to-raw-pipe capture without SideCamera IPC still showed >60 ms gaps. These were sequential probes, not same-frame attribution.
- In the corrected same-run DQBUF/PTS/output/read correlation, post-startup DQBUF and SOE PTS intervals had no >60 ms gaps, while output-write boundaries and full-read intervals did. The trace had 602 successful DQBUF calls for 600 outputs and no V4L2 sequence fields, so frame identity is not exact. PTS are V4L2 `ts-src-soe` timestamps, not EOF.
- Output write calls were short (25,800 calls, maximum 10.554 ms; none >50 ms), which does not support sustained pipe backpressure. Per-thread schedstat runqueue delay differed little between long and other gaps; only 3/77 long gaps had any thread delay >=5 ms. This does not support primary runqueue delay. Both measurements have observer limits.
- The syscall trace showed futex waits overlapping long gaps. Offline pairing found same-address positive wakes during writer waits, but `FUTEX_WAKE` does not identify the selected waiter. This is synchronization correlation, not causality.
- The sequential traced flush ABBA did not produce a repeatable improvement. The bounded r7 GDB capture read 600 frames and obtained three snapshots: the rawvideo mux thread was in `write()` in all three; the main thread was in condition waits twice and `sws_scale_frame()` once. These are callsites only. GDB pauses perturbed read intervals, and FFmpeg cleanup ended by SIGKILL after the 600 frames; no payload was saved.
- Prior r1–r7 packages/chains progressively superseded the source-PTS-as-EOF hypothesis, tested flush sensitivity, bounded the futex correlation, and preserved the unknown mechanism. The prior r7 approved package, manifest, verdict and chain are hash-preserved in revisioned archive paths.

## Mechanism

**No causal mechanism is established.** The evidence gives a medium-confidence localization of timing variation after the measured source-side cadence and before raw-pipe delivery, but exact frame-to-DQBUF mapping is absent. The bounded suspect set is:

1. FFmpeg decode/filter/output pacing or internal synchronization; and
2. host scheduling around FFmpeg and raw-pipe delivery.

GDB stacks identify callsites only. Futex overlap does not establish waiter identity or that a wake caused a gap. The remaining evidence does not distinguish these suspects or exclude a source-side contribution on the same output frame. Subscriber backlog, sustained pipe backpressure, primary runqueue delay, a repeatable flush-option effect, and using SOE PTS as EOF are unsupported; none is promoted as the cause.

## Bounded path and required observation

**Owner:** BUILD, routed by Admiral only after this `investigation-review` gate. Do not select a source fix from the present evidence.

The observation that would settle the mechanism is a single, same-frame monotonic timeline through V4L2 dequeue **sequence and completion**, FFmpeg decoder output, filter completion, raw-pipe write boundaries and full-read EOF, plus per-thread on-CPU/runqueue scheduling during a greater-than-60 ms gap. Keep frame identity and clock alignment explicit. A maximum 600-frame, no-payload diagnostic on the same camera and unchanged binary/default argv bounds the observation; stop after the first complete >60 ms gap or the frame cap. BUILD must submit the measurement/instrumentation plan to Admiral before any new capture, source instrumentation, or tracer is authorized. If the observation shows the delay before DQBUF, revisit the source/driver boundary; if it appears between decoder/filter/output stages, scope the FFmpeg path; if output completion is delayed by a blocked write or host descheduling, scope that boundary instead. Then propose only the smallest phase-owned change and verify it with a fresh, separately approved five-minute AC-4 run.

Until that observation exists, escalate the causal mechanism as unresolved at `investigation-review`. This r8 package does not route work back to BUILD before Admiral's cross-stage gate.

## Residual uncertainty

- Exact same-frame V4L2 dequeue-to-FFmpeg-output identity is absent; 602 dequeue calls correspond to 600 outputs without sequence metadata.
- Existing PTS values are SOE, not EOF. Source and output probes were separate or lacked sequence mapping.
- Futex wake/wait pairing is temporal same-address association, not kernel-confirmed waiter identity or causal proof; the output writer may be idle awaiting upstream work.
- GDB pauses and strace/per-frame schedstat observation effects are material. GDB stacks are callsites only and cannot measure uninstrumented AC-4 behavior.
- AC-6 and AC-8 remain hardware deferrals; the minor FFmpeg child resource-cap residual remains open.
