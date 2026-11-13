# Direct MppEncoder copy versus DMA-BUF benchmark — r7

**Run:** `loggerd-ka2-efficiency-20261029`  
**Observed:** 2026-10-30 18:57:39–19:14:29 UTC  
**Disposition:** The approved synthetic pilot and 30-pair final set completed. The results support a scoped class-level efficiency improvement for the tested road workload. They do not measure full `encoderd`, multi-camera operation, or an on-road route.

## Workload and binary identity

The comparison used a direct harness around the candidate `MppEncoder` class from the merged candidate `liblogger.a`. The same compiled harness ran both arms, changing only `ENCODER_ZERO_COPY=0` versus `1`. The unchanged merged `encoderd` binary was not started.

- Harness source `build/evidence/mpp_encoder_direct_class_harness.cc`: SHA-256 `d35225929b4f3e919244ec215200d9e1b6c4252e15a5e0cd008aebaad5cb832b`.
- Harness executable `build/evidence/mpp_encoder_direct_class_harness_observed`: SHA-256 `0c4101c4b22daee9f83e782e1a378c1fc8a32c3ce79a19ee23b75f58c6035b4d`, Build ID `4906cd8a31bf2142859e2002d0b60f697bc8581b`.
- `ldd -r` resolved MPP to `/lib/aarch64-linux-gnu/librockchip_mpp.so.1`, SHA-256 `ea762b7da71b63a73cdf14d053d84b091654aee5e5c4f26ceb1449b8fb8d92f9`, and RGA to `/lib/aarch64-linux-gnu/librga.so.2`. No unresolved symbols were reported.
- Candidate MPP archive: `/tmp/ka2-efficiency-r7-mergecheck/system/loggerd/liblogger.a`, SHA-256 `991024bd6c713f4bd3910df1ed3f904e9dcc356d82163bb44191ad24e81e3801`; `mpp_encoder.o` SHA-256 `b9934b105ebb85c1347e1bb8b438bf122fbf3e7e13b30ee8567e7eac31e3bf44`.
- The saved plan is `build/evidence/ka2-speedup-benchmark-plan.md`, SHA-256 `6f3ef32c1f78dc27ec5e629ed7e971adff09d4d3069d673115cc17e11946ab95`.
- Final-arm runner: `build/evidence/run_ka2_class_final_pairs.sh`, SHA-256 `28b12e43628c08e2391bdca632f7d4227d49d55cefefedc2a320e6cf2797ca59`.

Input was deterministic synthetic NV12 at 1920×1200, 20 frames/s, allocated from `/dev/dma_heap/system` and circulated through a 24-buffer DMA-BUF ring. Each five-second interval supplied 100 road frames and 100 qRoad frames to two MPP contexts in one process. qRoad stayed in copy mode in both arms as a control. After the harness correction, the measured run window from 18:57:39 to 19:11:40 UTC was 14 minutes, 1 second. It included one preliminary discarded warmup per arm, the six planned warmups, two decoded-output validation runs, five paired exploratory pilot samples, and 30 alternating final pairs; it stayed within the 15-minute ceiling. The preliminary warmup logs lacked explicit import/fallback counters; the six planned warmup logs and all final logs record them.

## Results

All 30 final pairs completed with exit code 0 and no invariant failures. Every interval reported 100 accepted road frames and 100 road packets, plus 100 accepted qRoad frames and 100 qRoad packets. Per-interval payload and stream sizes matched across arms: road payload `1,215,073` bytes, road stream `1,215,239` bytes, qRoad payload `105,434` bytes. Each copy arm imported zero road DMA-BUFs. Each zero-copy arm imported 24, with no import failure or fallback warning. There were no packet-count losses. The collector did not assert uniqueness or full frame-ID coverage for every interval, so exact per-input correspondence across all 30 pairs is not proven by the count fields alone.

Latency and CPU summaries below are computed from the 30 paired, per-interval summaries. Paired changes are zero-copy minus copy, so negative latency or CPU values indicate a reduction. Bootstrap confidence intervals use 50,000 paired resamples with seed `20261030`. Percentile columns summarize the harness's per-interval p95 and p99 values across the 30 intervals; per-frame raw latency vectors were not retained, so these are not pooled all-frame percentiles.

| Measure | Copy | Zero-copy | Paired zero-copy change |
|---|---:|---:|---:|
| Per-interval submit median, mean across 30 intervals | 5,997 µs | 4,957 µs | −1,040 µs; 95% CI −1,089 to −992 µs |
| Per-interval submit median, median across intervals | 5,984 µs | 4,965 µs | Mean relative reduction 17.30%; 95% CI 16.64–17.95% |
| Range of per-interval submit medians | 5,686–6,357 µs | 4,859–5,031 µs | — |
| Per-interval p95, mean across intervals | 8.346 ms | 6.028 ms | −2.318 ms; 95% CI −2.463 to −2.164 ms |
| Per-interval p99, mean across intervals | 9.994 ms | 8.152 ms | −1.842 ms; 95% CI −2.329 to −1.333 ms |
| Median of interval p95 values | 8,476 µs | 5,957 µs | — |
| Median of interval p99 values | 10,062 µs | 7,768 µs | — |
| Process CPU per five-second interval, mean | 365.72 ms | 254.00 ms | −111.72 ms; 95% CI −117.29 to −106.37 ms; mean relative reduction 30.46% (CI 29.31–31.63%) |
| Process CPU per five-second interval, median | 365.25 ms | 253.36 ms | — |
| qRoad submit median, mean across intervals | 3,945 µs | 3,984 µs | +39 µs; 95% CI −11 to +89 µs (no clear shift) |
| qRoad submit p95, mean across intervals | 5,209 µs | 5,535 µs | +326 µs; 95% CI +240 to +421 µs; higher in 29/30 pairs |
| qRoad submit p99, mean across intervals | 5,553 µs | 6,235 µs | +682 µs; 95% CI +483 to +860 µs; higher in 28/30 pairs |

The qRoad control's median did not shift clearly, but its p95 and p99 latency increased consistently when the road path used zero-copy. The reason is not established. Peak process RSS across all final intervals was 13,308 KiB. This is a measured tail-latency tradeoff, so the result is not regression-free.

This is a measured reduction in submit latency and process CPU for the direct class harness at the tested frame rate. It is not a measurement of wall-clock encode throughput, the production `encoderd` process, all three camera streams, or total system CPU. CPU frequency varied across intervals because the board governor was left unchanged; arm order alternated, and the qRoad control did not show a clear shift, but frequency effects are not eliminated.

## Output validity and remaining limits

The final intervals directed the encoded output to `/dev/null`; they recorded packet counts and byte totals but did not retain or decode every interval's output. Separate copy and zero-copy validation runs on the same harness and deterministic workload each emitted and decoded 100 HEVC frames at 1920×1200. Their stream SHA-256 matched exactly at `536f157d8d5e30bb5c981f89403e9f1f235a5bb7de19fe80011addb79571850f`, and decoded pixel SHA-256 matched at `2af5951a0c1f46a13a299080741448820822773039d8dfedd898b0e49d21ffba`. The final intervals also had identical packet counts and byte totals across arms, but per-interval decoded hashes were not collected; the content-equivalence evidence is therefore a separate representative validation rather than a per-sample assertion.

No live camera, sensor, production logging root, or on-road input was used. The saved runner checked, before every final arm, for `MemAvailable` ≥1.5 GiB, no MPP/DMA-heap device users, no harness/camera/encoderd/sensor process, zero thermal cooling states, and temporary `/dev/shm`/MsgQ use below 256 MiB; it rechecked memory and output/import gates after each arm. All logged cooling states were zero. Start/end `MemAvailable` over the final intervals ranged from 1,992,064 to 2,006,244 KiB. A post-run read-only snapshot at 19:14:29 UTC found `MemAvailable=1,987,204 KiB`, `CmaFree=0`, `/data` free space 1.8 GiB, `/dev/shm` use 34 MiB, and no MPP, DMA-heap, DRM, video, V4L2, camera, sensor, encoder, or harness users. No host settings or services were changed.

The preliminary post-correction warmup logs are `build/evidence/class-warmup-copy-1-corrected.log` (SHA-256 `cea9cc1cc4125fae6691911b3c26d1bc9a58974d8ed2413e6c18dd3fcb4c2d2a`) and `build/evidence/class-warmup-zero-1.log` (SHA-256 `3b334da3bc6fa732f98f0534890a59fc4bf090f1ba3416893009edb124c0d14c`). The validation logs are `build/evidence/class-validation-copy-v1.log` (SHA-256 `922422c307869780ca26f123bdab13cb9bdb48f4d376dce5b7dcc10b5b852624`) and `build/evidence/class-validation-zero-v1.log` (SHA-256 `e93a9c002d6918413f5d30853db9527bb8ba166f7669410612838c72f21b512e`). The 60 final sample log hashes are recorded in `build/evidence/class-final-v1-log-manifest.sha256`, SHA-256 `9da976579f1cc2bfd605424aaee27fe7277a6c2713835fea5da784bb1532739d`.

## Conclusion

Under the bounded synthetic workload, DMA-BUF import remained active for all zero-copy road intervals, all reported packet counts and byte totals matched, and the 30-pair confidence intervals show lower road submit latency and lower harness process CPU in the zero-copy arm. The observed class-level result is about 17.3% lower road submit median and 30.5% lower process CPU, alongside a higher qRoad p95/p99 tail. This supports continuing the encoder optimization work with `ENCODER_ZERO_COPY=0` still the production default, while the qRoad tail interaction is investigated before describing the full path as regression-free.

This result does not establish a gain for `wideRoad`, driver, simultaneous camera streams, the full `encoderd` pipeline, or a production route. It does not prove zero loss for every input frame or per-sample decoded-content identity. Those limits must remain attached to any downstream summary; no broader performance claim is supported by this run.
