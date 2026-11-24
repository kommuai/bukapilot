# qRoad tail latency and encoder memory investigation

Run: `loggerd-ka2-efficiency-20261029`, checkpoint revision 147  
Target: Firefly AIO-3588Q / RK3588 (`kommu-870ce22d4093b701`), Linux 5.10.198  
Candidate: committed encoder change `438acf545`; candidate `liblogger.a` SHA-256 `991024bd6c713f4bd3910df1ed3f904e9dcc356d82163bb44191ad24e81e3801`; loaded Rockchip MPP library SHA-256 `ea762b7da71b63a73cdf14d053d84b091654aee5e5c4f26ceb1449b8fb8d92f9`.

## Finding

The qRoad p95 increase is real on this board and workload: enabling zero-copy on the full-resolution road encoder adds about **0.29 ms** to qRoad's p95 call time. The repeat runs show the extra time inside qRoad's synchronous RGA image resize. MPP submission and packet draining do not explain the increase.

The extra time is **not caused by qRoad and road using the exact same DMA-BUF**. A diagnostic with separate, byte-identical input rings still had the qRoad/RGA slowdown. This points to contention somewhere in the shared RGA/MPP/SoC memory or driver path when zero-copy road encoding is active. The current traces do not identify which hardware or kernel mechanism causes that contention, so that part remains an evidence-based hypothesis.

No tested zero-copy variant removes the qRoad tail penalty. Zero-copy remains opt-in in the source (`ENCODER_ZERO_COPY`); it defaults off, and qRoad continues to resize into its own encoder input buffer.

## qRoad latency

Five modes were tested in 10 alternating launches each. Every launch submitted 100 synthetic frames at 20 fps and emitted 100 qRoad packets. Table entries are the mean of each launch's frame percentile; each launch percentile is calculated from 100 qRoad calls. Changes and intervals are paired by launch, using 50,000 bootstrap resamples.

| Mode | qRoad p50 | qRoad p95 | qRoad p99 |
|---|---:|---:|---:|
| Road copy + qRoad | 3.904 ms | 5.176 ms | 5.716 ms |
| Road zero-copy + qRoad | 4.021 ms | 5.464 ms | 6.103 ms |
| Zero-copy, imports preloaded | 4.013 ms | 5.465 ms | 6.277 ms |
| Zero-copy, qRoad submitted first | 4.100 ms | 6.174 ms | 6.693 ms |
| qRoad encoder only | 4.088 ms | 6.406 ms | 6.850 ms |

Zero-copy versus copy increased qRoad p95 by **0.288 ms** (95% CI **0.166–0.436 ms**). The qRoad median changed by only 0.118 ms and was not a clear difference. The p95 increase remains after the first 24 buffer imports: for frames 25–100, p95 was **0.294 ms higher** (95% CI **0.156–0.470 ms**). Moving the 24 imports before the timed loop had effectively no effect (**+0.001 ms** versus normal zero-copy; 95% CI **−0.211 to +0.178 ms**), so import initialization is not the main cause.

The latest 10-launch set's qRoad p99 difference was +0.387 ms, but its interval crossed zero (95% CI **−0.391 to +0.908 ms**). Two earlier direct-class sets did confirm a p99 increase: +0.682 ms in 30 paired runs (95% CI +0.483 to +0.861 ms), and +0.879 ms in a separate 10-pair repeat (95% CI +0.626 to +1.102 ms). The p95 result is consistent across all three sets; the exact p99 size is noisier.

Submitting qRoad before road made qRoad p95 about **0.710 ms worse** than ordinary zero-copy (95% CI +0.509 to +0.912 ms). The qRoad-only control was also slower than road-copy-plus-qRoad. That control shows that encoder context, scheduling, clock state, or other board load affects the result; it is not a clean isolated hardware baseline.

## Where the extra time occurs

An instrumented stage probe measured the qRoad RGA `imresize(..., IM_SYNC)`, MPP `encode_put_frame`, and packet drain separately. Across five paired launches, the RGA stage's per-run p95 increased by **0.306 ms** under zero-copy (95% CI **+0.106 to +0.504 ms**). The qRoad total-stage p95 increased by 0.291 ms (95% CI +0.126 to +0.545 ms). MPP submit changed by +0.063 ms (95% CI −0.021 to +0.154 ms), and packet drain by +0.0006 ms (95% CI −0.0048 to +0.0070 ms); neither is supported as the cause.

The second five-launch stage batch estimated same-ring zero-copy's qRoad call p95 at +0.196 ms versus copy (95% CI +0.057 to +0.335 ms), and its RGA p95 at +0.321 ms (95% CI +0.232 to +0.448 ms). The separate-ring diagnostic then compared copy, same-ring zero-copy, and zero-copy with a second DMA-BUF ring used only by road. In the separate-ring mode, qRoad p95 was still **0.390 ms above copy** (95% CI **+0.294 to +0.503 ms**), while RGA p95 was **0.402 ms above copy** (95% CI **+0.265 to +0.597 ms**). MPP-submit and packet-drain p95 remained statistically unchanged. Since separating the backing buffers did not clear the slowdown, buffer aliasing itself is not the explanation.

Most likely, MPP's use of imported road buffers is competing with qRoad's synchronous RGA resize through a shared hardware, memory, or driver resource. This is an inference from stage timing and the separate-ring test; we did not capture a kernel fence trace or a hardware-engine utilization trace to identify the exact resource.

## Memory comparison

The table separates encoder-owned memory from imported references. The 24 input DMA-BUFs exist in both modes; zero-copy imports their handles instead of allocating another full road input pool.

| Allocation / measurement | Copy mode | Zero-copy mode | Difference |
|---|---:|---:|---:|
| Road MPP input pool | 6 × 3,456,000 B = **19.78 MiB** | **0 MiB** | **19.78 MiB saved** |
| qRoad downscale pool | 6 × 266,112 B = **1.52 MiB** | **1.52 MiB** | No change |
| Road imported DMA-BUF handles | 0 | 24 handles / 82,944,000 B = **79.10 MiB referenced** | References existing input ring; not another 79.10 MiB allocation |
| Paired active `MemAvailable` comparison | — | — | Zero-copy left **20.18 MiB more available** (95% CI **19.31–21.05 MiB**) |

The measured `MemAvailable` saving closely matches the 19.78 MiB road MPP pool removed. Process peak RSS was effectively identical (about 28,800 KiB) in either mode; RSS does not account reliably for these DMA-heap buffers. The board's `debugfs/dma_buf/bufinfo` did not enumerate the input buffers correctly, so the pool/handle accounting and paired `MemAvailable` measurement are the useful memory signals here.

The separate-ring diagnostic intentionally allocated another 79.10 MiB of synthetic input memory. That was only to isolate buffer aliasing; it is not a proposed deployment mode or part of the ordinary before/after memory comparison.

## What this says about a regression-free zero-copy design

The following variants were tried and did not remove qRoad's p95 penalty:

- Pre-importing all road DMA-BUFs before frame submission: no measurable improvement.
- Reversing the per-frame submit order: made qRoad p95 worse.
- Giving road and qRoad separate DMA-BUF rings: slowdown remained, so avoiding exact buffer sharing is insufficient.

So far, **there is no measured zero-copy configuration that preserves the copy-mode qRoad tail**. A plausible next design is to remove the competing resize operation from the shared schedule: use a camera/ISP-provided qRoad-size stream if the board's camera path supports it, or schedule the RGA resize behind a real MPP completion fence and measure the resulting qRoad latency, road latency, and throughput together. Merely swapping call order is not enough because MPP work is asynchronous. Either alternative needs a full `encoderd` three-stream check before it can be called regression-free.

For the current build, keep `ENCODER_ZERO_COPY` disabled when qRoad tail latency must match the copy path. The saved road-pool memory and lower road submit latency are real, but the whole road-plus-qRoad path currently has a confirmed p95 tradeoff.

## Method and limits

- All runs used synthetic NV12 frames from `/dev/dma_heap/system`, one road encoder and one 526×330 qRoad encoder, 20 fps, 24 buffers, and one benchmark process at a time. The five-mode experiment had three warmups, five pilot launches, and 10 final launches per mode. No live camera, sensor, or production logger/encoderd was run.
- The primary latency comparison used 10 launches per mode. Separate instrumentation used five paired launches per stage-profile mode. Modes alternated; runs had warmups and strict packet-count, import, thermal, memory, and stall guards. No board settings or services were changed.
- This isolates the candidate `MppEncoder` class and its direct call path. It does not include production IPC, full `encoderd` scheduling, or the production three-stream workload. The measurements establish a regression in this controlled workload and localize it to RGA timing; they are not a claim about deployed end-to-end camera latency.
- No production code was changed in this investigation. The previous encoder change remains committed as `438acf545`.
- After the last run, the temporary MsgQ directory was absent, no MPP/DMA-heap test process was active, `MemAvailable` was 1,879,852 KiB, and the memory/thermal guards remained satisfied.

## Evidence

- Plan and guard limits: [`qroad-tail-memory-plan.md`](../evidence/qroad-tail-memory-plan.md)
- Five-mode harness and runner: [`qroad-tail-memory-harness.cc`](../evidence/qroad-tail-memory-harness.cc), [`run_qroad_tail_memory.sh`](../evidence/run_qroad_tail_memory.sh)
- Original stage probe: [`qroad-tail-stage-harness.cc`](../evidence/qroad-tail-stage-harness.cc), [`qroad-tail-stage-mpp_encoder.cc`](../evidence/qroad-tail-stage-mpp_encoder.cc), [`run_qroad_tail_stage_probe.sh`](../evidence/run_qroad_tail_stage_probe.sh)
- Separate-ring diagnostic: `qroad-stage-disjoint-profile-{copy,zero,zero-disjoint}-{01..05}.log` in `build/evidence/`; three discarded warmups per mode are retained with `warmup` names.
- Five-mode final logs: `qroad-tail-final-{copy,zero,zero-preimport,zero-reverse,qroad-only}-{01..10}.log` in `build/evidence/`.
- Prior cross-checks: [`report_ka2_encoder_speedup_final_r7.md`](report_ka2_encoder_speedup_final_r7.md), [`report_ka2_encoder_speedup_repeat10_20261031.md`](report_ka2_encoder_speedup_repeat10_20261031.md).
