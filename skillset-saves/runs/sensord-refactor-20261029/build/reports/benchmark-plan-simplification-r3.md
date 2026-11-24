# Sensord simplification comparison plan — source revision 3

**Run:** `sensord-refactor-20261029`  
**Question:** compare the final candidate, with concise call sites and direct Cap'n Proto builder operations, against the pinned original source.  
**Baseline Git HEAD:** `ad334254e36a6694df5e651ef5aecb8e0e7a3085`  
**Candidate:** source hashes below. No production source changes are expected during sampling.

## Candidate source hashes

| Path | SHA-256 |
| --- | --- |
| `system/sensord/sensord.py` | `45b0fa7635a541cf2024b1ceda36774fe5faea6fbc1aa5abb9df6e05cf4af35e` |
| `system/sensord/sensors/icm42670_accel.py` | `920f5000dae46ffdf33b2b758fd7b55f0484f1f3d626433592823ad1b98a8b6a` |
| `system/sensord/sensors/icm42670_gyro.py` | `c7f5cf6b996e5f9d0126db751d6b9d63a3050c60f1ec0c6ca84f276732d30f0c` |
| `system/sensord/sensors/icm42670_temp.py` | `66ad62dfc5ab5d08de1971559b7b071d399d233a039127f31621df11d9808f96` |
| `system/sensord/sensors/lis2mdl_magn.py` | `509b87d2f0f9327526f4bd2b57c33c70d7ca514a3f83d00fa301576d4e330670` |
| `system/sensord/sensors/lsm6ds3_accel.py` | `2ba05e9e360e241403a5934d5de9059e350ba4e5ea151423ef6e9719fddc97ab` |
| `system/sensord/sensors/lsm6ds3_gyro.py` | `08be99246e9a0b1a0beab4ec0ac31810ea74a3d10e97c416ffd13723dfb7ed6f` |
| `system/sensord/sensors/lsm6ds3_temp.py` | `d7c0a2e5aba3232667a03a3e0b04c537c34b007290be6c92b63ee3398b14960a` |
| `system/sensord/sensors/mmc5603nj_magn.py` | `b983013d00e9df832729cd632ae67a76013c8b5fb56e5cdfa4516a6c593892e2` |
| `system/sensord/sensors/sensor_event.py` | `98295ae7436fd4d44540198870b3edc4a3814cd7262224dd6a42dde71446f11d` |

## Workloads and sampling

Use the existing bounded harness `build/artifacts/benchmark_sensord_hotpaths.py`. It loads the pinned baseline source and candidate in one process, checks all event fixtures before timing, interleaves both arms, and verifies the candidate source hashes after timing.

1. Invalid polling loop: fake invalid sensor and fake publisher; real 104 Hz `Ratekeeper`; 200 ms windows; record wall time, process CPU, iteration count, and CPU fraction.
2. Driver event construction: deterministic mocked reads for all eight ICM42670, LSM6DS3, MMC5603NJ, and LIS2MDL `get_event` paths; 2,000 events per sample; record ns/event and derived throughput.
3. Gyro calibration: same 300 ordered XYZ samples; the final candidate uses the tuple-list representation selected for fewer lines and faster calibration; `time.sleep` is mocked to zero; record elapsed time.

Use five warm-up pairs, seven pilot pairs, and 40 final paired samples per workload; alternate baseline-first/candidate-first order. If any pilot metric's coefficient of variation exceeds 20%, increase the final count to 80. Report median/p95/p99, paired median delta and bootstrap 95% CI, and throughput ratio where applicable. Treat event-path changes under 5% as local measurement noise.

## Limits and interpretation

Run locally on Linux aarch64 / Python 3.12.3, sequentially, with GC enabled and no device path. No I2C, daemon startup, PubMaster traffic, cache dropping, service stopping, affinity changes, or governor changes. Abort if combined elapsed time exceeds 180 seconds, process peak RSS exceeds 512 MiB, load average exceeds 10, parity fails, a workload raises, or source hashes change. Record environment and host settings before and after.

The full 12-test offline suite passed against the exact source snapshot before this plan was prepared. This is a mocked-I/O local comparison only; do not infer KA2/TICI end-to-end performance or target CPU/RSS behavior. Hardware runtime remains unverified.
