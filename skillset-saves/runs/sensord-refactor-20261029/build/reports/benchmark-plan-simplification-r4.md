# Sensord efficiency comparison plan — source revision 4

**Run:** `sensord-refactor-20261029`  
**Question:** compare the approved r2 candidate, with local event constructors and shorter production source, against pinned HEAD.  
**Baseline Git HEAD:** `ad334254e36a6694df5e651ef5aecb8e0e7a3085`  
**Candidate:** source hashes below. No production source changes are expected during sampling.

## Production source count

The nine tracked production Python files below total 864 physical lines at pinned HEAD and 862 candidate lines (−2, or −0.23%). The count uses `splitlines()` across the same file set and excludes tests and the removed helper. This includes blank and comment lines so the count is reproducible.

## Candidate source hashes

| Path | SHA-256 |
| --- | --- |
| `system/sensord/sensord.py` | `45b0fa7635a541cf2024b1ceda36774fe5faea6fbc1aa5abb9df6e05cf4af35e` |
| `system/sensord/sensors/icm42670_accel.py` | `66046d50a36f3ef7b88577497aa6fb517068c9eb1f1973d040c2df1ea1e95d3d` |
| `system/sensord/sensors/icm42670_gyro.py` | `bcf82f511496ce18a9bf97aafac98ac4da7c6775e48f726b179b21ebb799158e` |
| `system/sensord/sensors/icm42670_temp.py` | `4a41743aaab559f639a4d18615bb789b44c51aa52969135b4fa2b7dc44e163f0` |
| `system/sensord/sensors/lis2mdl_magn.py` | `74141eac8eb58fb43e25bfe591c8a6380cbe39175341224cb91832303bf991d6` |
| `system/sensord/sensors/lsm6ds3_accel.py` | `e3c8c548928b8603f475807be1dc721c0b2b3a304330c844dc793499f869b116` |
| `system/sensord/sensors/lsm6ds3_gyro.py` | `c2d12eb1667cbab14dcdffe0a9d931b859262c1b620e661a1583555b056dc345` |
| `system/sensord/sensors/lsm6ds3_temp.py` | `b27b9cddb0b762bd86dd4e02ee96ab1c7bcee2f35eef6a0a7db180216c3ccb36` |
| `system/sensord/sensors/mmc5603nj_magn.py` | `b983013d00e9df832729cd632ae67a76013c8b5fb56e5cdfa4516a6c593892e2` |

## Workloads and sampling

Use `build/artifacts/benchmark_sensord_hotpaths.py`. It loads baseline source from pinned HEAD and the candidate in one process, checks all event fixtures before timing, interleaves both arms, and verifies candidate source hashes after timing.

1. Invalid polling loop: fake invalid sensor and fake publisher; real 104 Hz `Ratekeeper`; 200 ms windows; record wall time, process CPU, iteration count, and CPU fraction.
2. Driver event construction: deterministic mocked reads for eight ICM42670, LSM6DS3, MMC5603NJ, and LIS2MDL `get_event` paths; 2,000 events per sample; record ns/event and derived throughput.
3. Gyro calibration: the same 300 ordered XYZ samples in the existing tuple-list representation; `time.sleep` is mocked to zero; record elapsed time.

First run parity mode. Then use five warm-up pairs and seven pilot pairs. If any pilot metric's coefficient of variation exceeds 20%, increase the final count to 80; otherwise use 40 paired final samples per workload. Alternate baseline-first/candidate-first order. Report median/p95/p99, paired median delta, bootstrap 95% confidence intervals, and throughput ratio where applicable. Treat event-path changes under 5% as local measurement noise; reject any individual regression above 5%.

The current 12 focused offline tests passed against this exact source snapshot; the runner log is `build/evidence/tests-sensord-efficiency-r4.log`.

## Limits and interpretation

Run locally on Linux aarch64 / Python 3.12.3, sequentially, with GC enabled and no device path. No I2C, daemon startup, PubMaster traffic, cache dropping, service stopping, affinity changes, or governor changes. Abort if combined elapsed time exceeds 180 seconds, process peak RSS exceeds 512 MiB, load average exceeds 10, parity fails, a workload raises, or source hashes change. Record environment and host settings before and after.

This is a mocked-I/O local comparison only; do not infer KA2/TICI end-to-end performance or target CPU/RSS behavior. Hardware runtime remains unverified.
