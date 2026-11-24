# Sensord efficiency benchmark plan — r6

**Run:** `sensord-refactor-20261029`  
**Baseline:** `ad334254e36a6694df5e651ef5aecb8e0e7a3085`  
**Candidate:** current sensord worktree after removing the LSM6DS3/MMC scale hoists  
**Decision rule:** the approved design allows no individual event-path slowdown greater than 5%; report any statistically detectable smaller changes as observed. Production LOC must remain strictly below the same nine-file baseline set.

## Why this rerun exists

The r5 comparison showed a 1.56% slowdown in MMC5603 even though that driver had been restored byte-for-byte to the pinned baseline. Other unchanged paths shifted slightly in both directions. The previous harness executed baseline sources in isolated modules but imported candidate sources through normal import machinery. This r6 harness loads both baseline and candidate source into identically shaped isolated modules with the same module name, package, and synthetic filename before comparing them. r6 is the decision-quality event-path run; r4/r5 results remain preserved as historical evidence.

## Workloads and measurements

- Eight fixed-fixture `get_event` paths, 2,000 calls per sample, with serialized event-field parity checked before timing. Dynamic timestamps are excluded from field equality.
- Invalid-data polling over 200 ms using the project `Ratekeeper` at 104 Hz; record CPU ms, CPU fraction, iterations, and wall time.
- Gyroscope zero-rate calibration over 300 deterministic samples with sleeps mocked out; record sleep-free elapsed time.
- Baseline and candidate samples interleave with alternating arm order; discard five warmup pairs.
- Run a seven-pair pilot first. If all measured CVs remain under the 20% stability bound, collect 40 paired samples per workload. Otherwise increase sample count or stop and report instability.
- Report median, p95, p99, and paired bootstrap 95% confidence interval. Improvement is `(baseline - candidate) / baseline`, so negative values mean slower candidate timing.

## Environment and safety bound

Run locally in the non-production `/data/openpilot` checkout on Linux aarch64, Python 3.12.3, CPU affinity 0–4. Keep the existing `ondemand` governor and host services unchanged; no frequency, service, or GC tuning is permitted. This is a single-process synthetic microbenchmark with no hardware, I2C, daemon, network, or persistent workload data.

- Maximum duration: 180 seconds including warmup and samples.
- Maximum benchmark-process RSS: 512 MiB.
- Maximum one-minute host load average: 10.
- Maximum concurrency: one benchmark process and one timed arm at a time.
- Data volume: fixed synthetic buffers in memory and one JSON evidence record per mode; no application or device data is written.
- Abort on ceiling breach, source-hash or baseline-commit change, process RSS breach, or fixture parity failure. Discard any partial final set.

## Restore register

No host settings or services are changed, so the register starts empty and must remain empty. Confirm the original governor and service state are untouched in the final environment record.
