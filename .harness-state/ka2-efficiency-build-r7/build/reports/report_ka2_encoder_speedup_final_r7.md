# KA2 encoder speedup measurement — final r7

## Result

The candidate's full-resolution road path is faster in zero-copy mode on the tested KA2 board. Across 30 paired runs, zero-copy reduced the road encoder's per-frame submission latency and reduced total process CPU time for the road-plus-qRoad harness. The qRoad control's high-percentile submission latency also increased, so this is a speedup with a measured tail-latency tradeoff, not a no-regression result.

The table shows the **mean of each run's metric** across 30 paired runs. Each run measured 100 frames at 20 fps. Confidence intervals are paired bootstrap 95% intervals over the 30 run pairs (50,000 resamples, seed `20261030`). A negative change means zero-copy was faster; a positive change means it was slower.

| Metric | Copy | Zero-copy | Zero-copy change (95% CI) | Relative change |
|---|---:|---:|---:|---:|
| Road submit latency, per-run median | 5.997 ms | 4.957 ms | −1.040 ms (−1.090 to −0.992) | 17.30% faster |
| Road submit latency, per-run p95 | 8.346 ms | 6.028 ms | −2.318 ms (−2.462 to −2.164) | 27.64% faster |
| Road submit latency, per-run p99 | 9.994 ms | 8.152 ms | −1.842 ms (−2.334 to −1.335) | 17.98% faster |
| Harness process CPU time per 100-frame run | 365.72 ms | 254.00 ms | −111.72 ms (−117.40 to −106.39) | 30.46% less CPU |
| qRoad submit latency, per-run median | 3.945 ms | 3.984 ms | +0.039 ms (−0.010 to +0.089) | No clear change |
| qRoad submit latency, per-run p95 | 5.209 ms | 5.535 ms | +0.326 ms (+0.240 to +0.421) | 6.29% slower |
| qRoad submit latency, per-run p99 | 5.553 ms | 6.235 ms | +0.682 ms (+0.483 to +0.861) | 12.73% slower |

The sender was deliberately paced at 20 fps, so wall time was about 5.03 seconds in both modes. This test measures lower encoder-call latency and lower process CPU use; it does not show a higher end-to-end frame rate.

## What was tested

- Target: Firefly AIO-3588Q / RK3588, board `kommu-870ce22d4093b701`, Ubuntu 24.04.4, Linux 5.10.198, 2026-10-30.
- Comparison: the same candidate `MppEncoder` implementation and harness binary, toggling only `ENCODER_ZERO_COPY=0` versus `1`. This compares the candidate's copy path to its zero-copy path; it is not a deployed-old-binary versus deployed-new-binary comparison.
- Workload: deterministic synthetic 1920×1200 NV12 frames, one road encoder plus the 526×330 qRoad control, 20 fps, 24 input buffers allocated from `/dev/dma_heap/system`. Each sample ran 100 frames and took about five seconds.
- Sampling: eight discarded warm-ups (three per arm with explicit route-status reporting, plus one preliminary warm-up per arm after fixing the queue-size bug), two decoded-output validation runs, five exploratory paired pilot samples, then 30 final pairs with arm order alternating. The final set contains 30 runs per arm, 3,000 input frames per arm, and 3,000 road plus 3,000 qRoad output packets per arm. The measured board run window was 18:57:39–19:11:40 UTC (14 minutes, 1 second).
- CPU time is `CLOCK_PROCESS_CPUTIME_ID` around the 100-frame loop. It includes the harness and both encoder contexts, and excludes encoder creation and close. Submit latency is measured around each `encode_frame` call. Each reported per-run median/p95/p99 summarizes 100 calls; bootstrap resampling uses run pairs, not individual frames.

## Validity and observed limits

- All 60 final runs exited successfully. Every run accepted and emitted 100 road packets and 100 qRoad packets, with matching packet byte counts between modes.
- In every zero-copy run the road encoder retained zero-copy mode, imported all 24 distinct DMA-BUFs, and reported zero import failures and zero fallbacks. Copy runs imported none. qRoad remained on its existing downscale path in both modes.
- One 100-frame road stream from each mode decoded successfully as HEVC at 1920×1200. Both encoded streams had SHA-256 `536f157d8d5e30bb5c981f89403e9f1f235a5bb7de19fe80011addb79571850f`; their decoded pixel hashes also matched: `2af5951a0c1f46a13a299080741448820822773039d8dfedd898b0e49d21ffba`. Final timed runs sent output to `/dev/null`; they validated packet counts and sizes but did not each decode a stream.
- MemAvailable ranged from 1,992,064 to 2,006,244 KiB across final samples. Thermal cooling states stayed at zero. No camera, sensor, production encoder, or other MPP/DMA-heap user was active. CMA reported no free pages (`CmaFree=0`), so the test used the system DMA heap.
- The qRoad p95/p99 increase is consistent across the paired set, while qRoad median is statistically unchanged. Its cause is not established. Because this harness runs road and qRoad encoder contexts together, the result should be treated as a shared-resource interaction to investigate before claiming the change is regression-free.
- This is a synthetic, direct-class test of one full-resolution stream plus qRoad. It does not measure camera IPC, `encoderd` process overhead, three-stream operation, live camera input, or vehicle behavior.

## Artifacts

- Plan and ceilings: [`ka2-speedup-benchmark-plan.md`](../evidence/ka2-speedup-benchmark-plan.md)
- Direct-class harness source: [`mpp_encoder_direct_class_harness.cc`](../evidence/mpp_encoder_direct_class_harness.cc)
- Observed-state harness binary: [`mpp_encoder_direct_class_harness_observed`](../evidence/mpp_encoder_direct_class_harness_observed), SHA-256 `0c4101c4b22daee9f83e782e1a378c1fc8a32c3ce79a19ee23b75f58c6035b4d`
- Final sample runner: [`run_ka2_class_final_pairs.sh`](../evidence/run_ka2_class_final_pairs.sh)
- Validation logs: [`class-validation-copy-v1.log`](../evidence/class-validation-copy-v1.log), [`class-validation-zero-v1.log`](../evidence/class-validation-zero-v1.log)
- Final per-run logs: `build/evidence/class-final-v1-pair-{01..30}-{copy,zero}.log`
- Final log hashes: [`class-final-v1-sample-sha256.txt`](../evidence/class-final-v1-sample-sha256.txt)

## Recommendation

Keep the road-path zero-copy change as a promising CPU and latency optimization. Before calling the whole encoder path regression-free, isolate why qRoad p95/p99 latency rises when the road encoder uses zero-copy, then verify with the full three-stream `encoderd` workload.
