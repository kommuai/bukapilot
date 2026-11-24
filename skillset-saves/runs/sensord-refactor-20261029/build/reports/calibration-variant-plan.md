# Calibration buffer variant comparison plan

**Question:** does one interleaved signed-short buffer improve on the immediately prior three-array candidate while preserving the approved calibration behavior?

**Arms:** prior candidate source is reconstructed in memory by replacing only `_calibrate_zero_rate_bias`'s current flat-array block with the exact previous three-array block. Candidate source is the current worktree file. Both use the same 300 ordered synthetic XYZ reads and `time.sleep` is mocked to zero. No product files are edited by the benchmark.

**Measurements:** one calibration per sample; record `perf_counter_ns` elapsed time and `process_time_ns`. Use one Python process, five warm-up pairs, seven interleaved pilot pairs, then 40 interleaved final pairs (80 if either arm's pilot CV exceeds 20%). Alternate which arm runs first. Record medians, p95/p99, paired median relative time change, and paired bootstrap 95% CI. Compare exact resulting `bias_counts` per sample. Also record sample-container sizes using `sys.getsizeof` as an estimate, not process RSS.

**Limits:** one local CPU process, total duration under 180 seconds, process peak RSS below 512 MiB, observed load average below 10, no I2C/device path, no service or host-setting changes. Abort if source/parity differs, a workload raises, or a limit is exceeded. Interpret only as this host's sleep-free calibration code path; do not extrapolate to target boot time or sensor behavior.
