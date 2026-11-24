# Sensord efficiency benchmark report — r6

**Run:** `sensord-refactor-20261029`  
**Code candidate:** r5, captured in `build/artifacts/implementation-r5.diff`  
**Baseline:** `ad334254e36a6694df5e651ef5aecb8e0e7a3085`  
**Decision benchmark:** r6 symmetric-loader comparison  
**Result:** the invalid-data polling path uses **99.04% less CPU** in the synthetic 200 ms window. ICM42670 event construction is **1.98% faster for accelerometer events** and **3.35% faster for gyroscope events**. The same nine production files contain **2 fewer lines** than baseline. No event-path change exceeds the approved 5% limit.

## Method

The r4/r5 event runs exposed a comparison asymmetry: baseline modules were executed from source while candidate modules used normal imports. r5 also reported a slowdown for MMC5603 after that driver had been restored byte-for-byte to baseline. The r6 harness corrects this by executing both arms into isolated modules with the same module name, package, and filename metadata. The r6 run is the decision-quality event comparison; earlier logs remain preserved for audit.

The pinned baseline source is read from Git commit `ad334254e36a6694df5e651ef5aecb8e0e7a3085`. The candidate source hashes were checked before and after sampling. Eight fixed-fixture `get_event` workloads compare serialized event output before timing; dynamic timestamps are excluded from parity. Each sample times 2,000 calls. Arms were interleaved with alternating baseline-first/candidate-first order. Five warmup pairs were discarded, a seven-pair pilot remained below the 20% CV cutoff, and the final set used 40 paired samples per workload. The 95% intervals are paired bootstrap intervals over 5,000 resamples.

The invalid polling workload runs for 200 ms with the real project `Ratekeeper` at 104 Hz and a fake sensor that always reports invalid data. Calibration uses 300 deterministic samples with sleep time excluded. No I2C device, daemon, hardware, network, or application data was used.

## Event construction

Median and tail values are microseconds per event, shown as `median / p95 / p99`. Positive improvement means the candidate is faster. The 95% interval is for paired median improvement.

| Workload | Baseline µs (median / p95 / p99) | Candidate µs (median / p95 / p99) | Improvement (paired 95% CI) |
| --- | ---: | ---: | ---: |
| ICM42670 accelerometer | 13.112 / 13.148 / 13.164 | 12.853 / 12.904 / 12.938 | +1.979% [+1.873%, +2.037%] |
| ICM42670 gyroscope | 10.819 / 10.838 / 10.844 | 10.456 / 10.479 / 10.493 | +3.354% [+3.263%, +3.397%] |
| ICM42670 temperature † | 7.554 / 7.579 / 7.591 | 7.575 / 7.613 / 8.386 | -0.276% [-0.424%, -0.227%] |
| LIS2MDL magnetometer † | 14.769 / 14.792 / 14.802 | 14.760 / 14.784 / 14.798 | +0.059% [+0.012%, +0.100%] |
| LSM6DS3 accelerometer † | 13.437 / 13.509 / 14.498 | 13.420 / 13.483 / 13.505 | +0.130% [+0.065%, +0.174%] |
| LSM6DS3 gyroscope † | 13.478 / 13.497 / 13.502 | 13.459 / 13.484 / 13.489 | +0.144% [+0.049%, +0.154%] |
| LSM6DS3 temperature † | 8.010 / 8.052 / 8.060 | 8.004 / 8.045 / 8.286 | +0.074% [+0.062%, +0.174%] |
| MMC5603NJ magnetometer † | 23.466 / 23.559 / 23.598 | 23.427 / 23.559 / 23.603 | +0.167% [-0.007%, +0.139%] |

† These six driver source files are byte-identical to the pinned baseline, so their sub-0.28% timing offsets are not code-change effects. I interpret them as residual execution-instance and host variation in this isolated microbenchmark; the offsets are below the agreed 5% practical threshold. The MMC5603 interval overlaps zero. The only changed files among these event paths are the ICM42670 accelerometer and gyroscope, and both show repeatable improvements.

## Board-weighted event cost estimate

This estimate sums each path's median per-event CPU time at the repository service frequencies: 104 Hz accelerometer, 104 Hz gyroscope, 2 Hz temperature, and 25 Hz magnetometer. The service rates were read from `SERVICE_LIST`.

| Sensor set | Baseline ms/s | Candidate ms/s | Difference (ms/s saved) | Relative change |
| --- | ---: | ---: | ---: | ---: |
| KA2 | 2.873205 | 2.808296 | +0.064909 | +2.259% |
| TICI (LSM; no MMC) | 2.815202 | 2.811360 | +0.003841 | +0.136% |
| tizi (LSM + MMC) | 3.401851 | 3.397033 | +0.004818 | +0.142% |

These figures describe only mocked `get_event` calls. They exclude bus reads, interrupts, PubMaster work, and the rest of sensord. KA2 includes both changed ICM paths, so its event-only estimate improves 2.26%. The TICI and tizi sets use unchanged event source; their approximately 0.14% point deltas are below the threshold and should be read as no measurable event-cost change, not a product speedup.

## Invalid-data polling

| Metric | Baseline median / p95 / p99 | Candidate median / p95 / p99 | Improvement (paired 95% CI) |
| --- | ---: | ---: | ---: |
| CPU time per 200 ms window (ms) | 199.293 / 199.970 / 199.971 | 1.922 / 2.371 / 2.511 | +99.036% [+99.006%, +99.065%] |
| CPU fraction (fraction) | 0.996 / 1.000 / 1.000 | 0.010 / 0.012 / 0.012 | +99.046% [+99.018%, +99.075%] |
| Invalid loop iterations (count) | 561839.500 / 581492.000 / 582480.000 | 21.000 / 21.000 / 21.000 | +99.996% [+99.995%, +99.996%] |
| Window wall time (ms) | 200.006 / 200.013 / 200.014 | 202.137 / 202.308 / 202.686 | -1.066% [-1.089%, -1.058%] |

The median CPU time fell from 199.293 ms to 1.922 ms per 200 ms window. The baseline made 561,840 invalid reads; the paced candidate made 21, approximately 104/s. The candidate's window wall time is about 1.07% longer because Ratekeeper can finish its final wait after the 200 ms stop deadline. This is a CPU reduction, not a latency improvement.

## Gyro calibration

For one sleep-free calibration over 300 samples, median time was 0.882738 ms baseline and 0.882592 ms candidate. The estimated improvement is +0.017%, with 95% CI -0.281% to +0.347%. This is no measurable change. The accepted tuple-list calibration buffer is unchanged from baseline; the slower compact-array experiment remains rejected.

## Environment, limits, and restore

- Platform: `Linux-5.10.198-aarch64-with-glibc2.39`; aarch64; Python 3.12.3.
- CPU affinity [0, 1, 2, 3, 4]; 5 CPUs visible; CPU0 governor `ondemand` at 1800000 kHz before and after. The governor was not changed.
- One-minute load average was 1.87 before and 1.84 after.
- Declared ceilings: 180 seconds total, 512 MiB benchmark-process RSS, and load average 10. Actual duration was 39.70 seconds and peak RSS was 50.18 MiB; the run stayed within the ceilings.
- No services, host settings, GC settings, or device state were changed. Restore register: empty.

## Conclusion and next action

The main quantitative win is removing busy spinning from invalid polling: about 99% lower CPU use in the measured window. Hoisted ICM constants improve their two changed event paths. Total event-only savings are measurable for KA2 and negligible for the LSM-based sensor sets. The source remains 2 lines shorter, and all tested event outputs match.

No further event-construction refactor is justified by this local data. The remaining gate is a runtime smoke on an identified and authorized target; this benchmark cannot establish daemon or target-device behavior.

**Artifacts:** `build/artifacts/benchmark_sensord_hotpaths.py` (r6; versioned copy: `benchmark_sensord_hotpaths_r6.py`); `build/reports/benchmark-plan-simplification-r6.md`; parity, pilot, and final logs under `build/evidence/`.
