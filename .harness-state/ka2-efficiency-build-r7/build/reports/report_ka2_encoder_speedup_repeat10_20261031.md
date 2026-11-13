# KA2 encoder repeat benchmark — 10 paired runs

## Summary

I repeated the same candidate `MppEncoder` copy-versus-zero-copy test on 2026-10-31 using the exact harness, candidate archive, encoder object, and Rockchip MPP library from the prior 30-pair benchmark.

The repeat is consistent with the previous result for the road encoder:

- Road per-run median submission latency improved **17.97%** (6.009 ms to 4.926 ms; paired bootstrap 95% CI for the reduction: **16.46–19.34%**). The previous 30-pair estimate was 17.30%. The bootstrap CI for the difference between these two estimates is −0.98 to +2.20 percentage points, so the repeat does not show a meaningful change in the effect.
- Process CPU time for the combined road-plus-qRoad loop fell **30.76%** (366.71 ms to 253.71 ms per 100-frame run; 95% CI: **28.52–32.82%**). The previous estimate was 30.46%; the between-run-set CI is −2.24 to +2.66 percentage points.
- The qRoad tail-latency regression appeared again: p95 was **5.73% slower** and p99 **15.53% slower** in zero-copy mode. This supports the previous finding that the road improvement carries a qRoad tail-latency tradeoff in this combined workload.

The table shows the mean of each run's frame-latency percentile over 10 paired runs. Each run contains 100 observations. Confidence intervals are paired bootstrap intervals over the 10 run pairs (50,000 resamples, seed `20261031`). Negative latency change means zero-copy was faster; positive means slower.

| Metric | Repeat copy mean | Repeat zero-copy mean | Zero-copy change (95% CI) | Repeat effect | Prior 30-pair effect |
|---|---:|---:|---:|---:|---:|
| Road submit latency, per-run median | 6.009 ms | 4.926 ms | −1.083 ms (−1.179 to −0.978) | 17.97% faster | 17.30% faster |
| Road submit latency, per-run p95 | 8.375 ms | 5.897 ms | −2.478 ms (−2.696 to −2.264) | 29.51% faster | 27.64% faster |
| Road submit latency, per-run p99 | 9.625 ms | 8.300 ms | −1.326 ms (−1.909 to −0.813) | 13.92% faster | 17.98% faster |
| Process CPU time per 100-frame run | 366.71 ms | 253.71 ms | −112.99 ms (−121.65 to −103.52) | 30.76% less CPU | 30.46% less CPU |
| qRoad submit latency, per-run median | 3.961 ms | 3.980 ms | +0.019 ms (−0.091 to +0.123) | No clear change | No clear change |
| qRoad submit latency, per-run p95 | 5.210 ms | 5.508 ms | +0.298 ms (+0.160 to +0.447) | 5.73% slower | 6.29% slower |
| qRoad submit latency, per-run p99 | 5.681 ms | 6.559 ms | +0.879 ms (+0.626 to +1.102) | 15.53% slower | 12.73% slower |

The sender remained paced at 20 fps, so wall time was about five seconds in both modes. These results measure lower road call latency and lower process CPU use; they do not show higher end-to-end frame rate.

## Method and validity

- Target: the same Firefly AIO-3588Q / RK3588 board, Ubuntu 24.04.4, Linux 5.10.198. MemAvailable ranged from 1,892,060 to 1,937,164 KiB during measured runs; thermal cooling states stayed at zero. `CmaFree` was zero, so the run used the system DMA heap.
- 10 measured pairs: 10 copy-mode runs and 10 zero-copy runs, with arm order alternating. Each run used 100 synthetic 1920×1200 NV12 frames at 20 fps, one road encoder, the qRoad downscale control, and a 24-buffer `/dev/dma_heap/system` ring.
- Three discarded warm-ups per arm and one decoded-output validation run per arm preceded sampling. Every measured run accepted and emitted 100 road packets and 100 qRoad packets. All 10 zero-copy runs imported 24 distinct road DMA-BUFs with no import failure or fallback; copy runs imported none.
- The validation streams decoded as HEVC at 1920×1200 with 100 frames. Copy and zero-copy stream hashes matched (`536f157d8d5e30bb5c981f89403e9f1f235a5bb7de19fe80011addb79571850f`); decoded pixel hashes matched (`2af5951a0c1f46a13a299080741448820822773039d8dfedd898b0e49d21ffba`). Timed runs wrote encoded output to `/dev/null` and checked packet counts and sizes.
- This is the direct `MppEncoder` class comparison, not a comparison of deployed old and new `encoderd` binaries. It covers one full-resolution road stream plus qRoad with synthetic data; it excludes production IPC and three-stream operation.
- No live camera, sensor, or production encoder was used. Board settings and services were left unchanged. Temporary DMA-BUF-backed output streams and isolated MsgQ files were removed after validation and hashing.

## Artifacts

- Prior 30-pair report: [`report_ka2_encoder_speedup_final_r7.md`](report_ka2_encoder_speedup_final_r7.md)
- Repeat harness: [`mpp_encoder_direct_class_harness_observed`](../evidence/mpp_encoder_direct_class_harness_observed), unchanged SHA-256 `0c4101c4b22daee9f83e782e1a378c1fc8a32c3ce79a19ee23b75f58c6035b4d`
- Candidate artifacts were unchanged: `liblogger.a` SHA-256 `991024bd6c713f4bd3910df1ed3f904e9dcc356d82163bb44191ad24e81e3801`; `mpp_encoder.o` SHA-256 `b9934b105ebb85c1347e1bb8b438bf122fbf3e7e13b30ee8567e7eac31e3bf44`; MPP library SHA-256 `ea762b7da71b63a73cdf14d053d84b091654aee5e5c4f26ceb1449b8fb8d92f9`.
- Repeat runner: [`run_ka2_class_repeat10.sh`](../evidence/run_ka2_class_repeat10.sh)
- Repeat validation: [`class-repeat10-validation-copy.log`](../evidence/class-repeat10-validation-copy.log), [`class-repeat10-validation-zero.log`](../evidence/class-repeat10-validation-zero.log)
- Repeat sample hashes: [`class-repeat10-v1-sample-sha256.txt`](../evidence/class-repeat10-v1-sample-sha256.txt)
- Repeat run logs: `build/evidence/class-repeat10-v1-pair-{01..10}-{copy,zero}.log`

## Conclusion

The 10-pair repeat confirms the road-path median-latency and CPU gains at about the same size as the prior 30-pair result. It also reproduces the qRoad p95/p99 regression. Treat zero-copy as a measured road-path optimization with a shared-workload tradeoff; investigate qRoad tail latency before describing the encoder change as regression-free.
