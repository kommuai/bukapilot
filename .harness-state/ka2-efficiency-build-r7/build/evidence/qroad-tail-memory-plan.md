# qRoad tail-latency and memory investigation plan

Run: `loggerd-ka2-efficiency-20261029`, revision 146  
Target owner: user-authorized Firefly AIO-3588Q / RK3588 (`kommu-870ce22d4093b701`)  
Scope: synthetic DMA-BUF input only. Do not start or use live cameras, sensors, road data, or production logger paths.

## Question and comparison

The prior candidate-class benchmark showed qRoad median latency unchanged, with qRoad p95/p99 higher when the road encoder used zero-copy. The suspected causes are per-buffer MPP import work on the submitting thread and shared MPP/RGA/memory activity while the road frame remains in flight. These are hypotheses, not findings yet.

Use the same candidate `MppEncoder` object/library and same deterministic 1920x1200 NV12 DMA-BUF ring. Compare five modes, five discarded pilot launches and 10 measured launches each:

1. `copy`: road copy path then qRoad downscale/encode; existing control.
2. `zero`: road zero-copy then qRoad; existing candidate behavior.
3. `zero-preimport`: import all 24 road DMA-BUFs before timed frame submissions, then road zero-copy followed by qRoad. This is an experiment for moving one-time imports out of the frame path.
4. `zero-reverse`: qRoad downscale/encode then road zero-copy. Diagnostic for within-frame call order; not a proposed production change.
5. `qroad-only`: qRoad downscale/encode with no road submissions. Diagnostic baseline for qRoad without road encoder work.

The same harness source will emit per-frame qRoad latency and in-process buffer accounting. It will not alter production encoder code. The pre-import experiment moves setup work before the timed interval, so its frame latency must be read alongside startup cost and total work.

## Environment and load ceiling

- Same RK3588 board, OS/kernel, candidate encoder archive and loaded MPP/RGA libraries as the prior direct-class runs. One harness process at a time; at most two MPP contexts.
- Each launch submits 100 synthetic frames at 20 fps; 24 DMA-BUF inputs total about 79.1 MiB. Maximum live working set: 256 MiB; output goes to `/dev/null`, per-frame timing is printed only after each timed interval, and total logs are capped at 10 MiB.
- Three discarded warm-up launches per mode, then five pilot launches per mode to estimate variance, then 10 measured launches per mode. Interleave modes in rotating order. Total: 90 launches, each paced at 20 fps for about five seconds; the 15-minute ceiling includes setup and teardown. Target limit: 15 minutes total board time.
- No CPU governor, service, thermal, kernel, or cache setting changes. Restore register is empty; only temporary harness/log files are created and then retained as benchmark evidence.
- Abort before or during a launch if another MPP user or camera/sensor/encoderd appears, `MemAvailable` drops below 1.5 GiB, thermal cooling state is nonzero, input allocation/import or packet validation fails, any accepted frame lacks an output packet, any qRoad call stalls over 1 second, or temporary data reaches 256 MiB. Do not merge a partial sample set.

## Measurements

For every launch, record accepted/emitted packets, import count/fallback, per-frame road and qRoad call latencies, p50/p95/p99, CPU time, process RSS/HWM, active `MemAvailable`, CMA counters, DMA-BUF object count/bytes while alive, and encoder pool/imported-buffer sizes. Report qRoad latency separately for frames 1-24 (first use of the ring) and frames 25-100 (steady ring reuse). Decode/content validation is not changed by this diagnostic run; the existing 30-pair and 10-pair decoded-output validations remain the functional signal.

Compute per-launch percentiles first and compare across repeated launches; use run-level bootstrap 95% intervals. Treat five-mode 10-launch results as mechanism isolation, not a production `encoderd` or three-stream performance claim.


## Setup probes excluded from measurements

Two initial harness starts exited before submitting any frame: the first created the subscriber before the encoder publisher, and the second lacked the isolated `/dev/shm/msgq_ka2bench` directory. Both logs are retained with `setup-failure` names and excluded from all samples. The harness now creates publishers before the subscriber; the runner creates the unique MsgQ directory and removes only its two queues and the directory on exit. DMA-BUF accounting returned to zero after each failure.


## Added stage-profile subset

The five-mode final run confirms a steady-state qRoad p95 difference, but does not identify which sub-operation causes it. Add a separate instrumented subset using temporary copies of `mpp_encoder.cc` and the diagnostic harness; do not edit production source. Time the qRoad RGA `imresize(..., IM_SYNC)`, MPP `encode_put_frame`, and post-submit `drain_packets` separately, and print timing rows only after each 100-frame timed interval. Compare `copy` and `zero` with three discarded warm-ups per arm and five alternating paired launches. Keep the same 20 fps, 24-buffer system-DMA-heap ring, one harness process, memory/thermal/device checks, 1-second per-call abort, 256 MiB live limit, and 15-minute overall ceiling. Treat stage timing as diagnostic because instrumentation adds a small timer cost; no production performance claim uses these samples.


## Same-buffer alias diagnostic

Add a third instrumented mode, `zero-disjoint`: preallocate a second 24-buffer system-DMA-heap ring with byte-identical synthetic frames, let the road MPP encoder import that ring, and let qRoad RGA continue reading the original ring. This adds 79.1 MiB of input backing; total live target remains below the 256 MiB ceiling, and `MemAvailable` must stay above 1.5 GiB. It isolates whether both devices reading the exact same DMA-BUF is needed for the RGA tail. The shadow ring is copied during setup and reused, so this is only a cause-isolation probe, not a zero-copy-from-camera implementation. Use three discarded warm-ups and five measured launches per mode across `copy`, `zero`, and `zero-disjoint`, with rotating order and the same guards.
