# Direct MppEncoder class pilot — r7

**Run:** `loggerd-ka2-efficiency-20261029`  
**Observed:** 2026-10-30 19:00:17–19:03:23 UTC  
**Status:** Five exploratory paired samples completed. The final paired set has not run; no stable speedup claim is supported yet.

## Tested path and safeguards

This is a class-level synthetic benchmark around the candidate `MppEncoder`, not a production `encoderd` run. It used one full-resolution road encoder and one qRoad control at 20 fps with deterministic 1920×1200 NV12 input allocated from `/dev/dma_heap/system`, a 24-buffer DMA-BUF ring, and isolated MsgQ/Params roots. The same harness binary was used for both modes with only `ENCODER_ZERO_COPY` changed. No live camera, sensor, road input, or production log root was used.

- Harness source: `build/evidence/mpp_encoder_direct_class_harness.cc`, SHA-256 `d35225929b4f3e919244ec215200d9e1b6c4252e15a5e0cd008aebaad5cb832b`.
- Harness binary: `build/evidence/mpp_encoder_direct_class_harness_observed`, SHA-256 `0c4101c4b22daee9f83e782e1a378c1fc8a32c3ce79a19ee23b75f58c6035b4d`, Build ID `4906cd8a31bf2142859e2002d0b60f697bc8581b`.
- `ldd -r` resolved `librockchip_mpp.so.1` to `/lib/aarch64-linux-gnu/librockchip_mpp.so.1` and `librga.so.2` to `/lib/aarch64-linux-gnu/librga.so.2`, with no unresolved symbols.
- Benchmark plan: `build/evidence/ka2-speedup-benchmark-plan.md`, SHA-256 `6f3ef32c1f78dc27ec5e629ed7e971adff09d4d3069d673115cc17e11946ab95`.

The harness initially SIGBUSed before producing measurement output. Source inspection found its subscribers had been created with a 256-byte MsgQ segment while each production queue is 10,485,760 bytes; `msgq_new_queue()` truncates the shared queue file on each open. The harness was corrected to use each topic's configured queue size. The setup failure and its evidence are recorded in `report_direct_class_harness_setup_failure_r7.md`. The corrected binary then completed warmups, validation, and pilot samples. This correction is confined to the temporary harness.

## Output validity check

Separate copy and zero-copy validation runs each accepted and emitted 100 road and 100 qRoad packets with no failures. Zero-copy road imported 24 distinct DMA-BUFs with no import failure or fallback; qRoad remained in copy mode in both arms. Both runs produced the same road stream SHA-256 `536f157d8d5e30bb5c981f89403e9f1f235a5bb7de19fe80011addb79571850f`. `ffprobe` identified HEVC at 1920×1200 with 100 decoded frames in each run, and decoded pixel SHA-256 matched at `2af5951a0c1f46a13a299080741448820822773039d8dfedd898b0e49d21ffba`.

Validation logs:

- Copy: `build/evidence/class-validation-copy-v1.log`, SHA-256 `922422c307869780ca26f123bdab13cb9bdb48f4d376dce5b7dcc10b5b852624`.
- Zero-copy: `build/evidence/class-validation-zero-v1.log`, SHA-256 `e93a9c002d6918413f5d30853db9527bb8ba166f7669410612838c72f21b512e`.

The five pilot logs report packet counts, bytes, imports, fallback counters, and submit-time distributions. The pilot streams were sent to `/dev/null`, so the separate validation above establishes mode-to-mode output equivalence but does not provide decoded content hashes for each individual pilot sample.

## Discarded warmups

All six planned 5-second warmups exited successfully. Each arm emitted 100 road and 100 qRoad packets with identical per-run counts and byte totals: road payload/stream `1,215,073`/`1,215,239` bytes and qRoad payload `105,434` bytes. Road imports were 0 in copy mode and 24 in zero-copy mode; there were no import failures or fallback warnings. qRoad remained copy mode in both arms. `MemAvailable` stayed around 2.02 GB. The warmup logs are:

| Log | SHA-256 |
|---|---|
| `class-warmup-copy-1-observed.log` | `cd8b9f19120d46b88f2e45ec723f46ddbefbe90d139a05b527ca84cb564baecd` |
| `class-warmup-copy-2-observed.log` | `cd9be7f0375fe3e52084fdbe53f3f9176c61c127a111d391afb7cf33357ca0d8` |
| `class-warmup-copy-3-observed.log` | `c2bbdb5711cc6ca55f75507649f15b5ad722f413e8a26acc51e68618c03b22ad` |
| `class-warmup-zero-1-observed.log` | `c3add30d9544fe25d7af1ddb00cb6997b39b40f7342910c8ce96d7cdceeed90f` |
| `class-warmup-zero-2-observed.log` | `07889874a518e0ab272641907840439d76378b3211b4bfc4f8f069d40c2f82ad` |
| `class-warmup-zero-3-observed.log` | `3c53d3e0dd7af9b5922911a47a49a523b8039cc8b85e9a652d28354c7ced0e59` |

## Exploratory paired pilot

Each pair used one 5-second, 100-frame interval per arm. All ten intervals exited 0 with 100 accepted road frames, 100 road packets, 100 accepted qRoad frames, and 100 qRoad packets. Road byte totals matched in every interval. Copy mode imported no DMA-BUFs; zero-copy mode imported all 24 ring buffers with no fallback. qRoad stayed in copy mode and served as the control.

| Pair | Road submit median, copy (µs) | Road submit median, zero-copy (µs) | Copy p95 / p99 (µs) | Zero-copy p95 / p99 (µs) | Process CPU, copy / zero-copy (ms) |
|---:|---:|---:|---:|---:|---:|
| 1 | 6,255 | 4,966 | 8,547 / 10,163 | 6,028 / 8,077 | 381.9 / 247.4 |
| 2 | 5,970 | 4,984 | 8,770 / 9,219 | 6,214 / 8,731 | 366.9 / 267.1 |
| 3 | 6,016 | 4,982 | 8,328 / 12,521 | 6,114 / 8,450 | 367.9 / 267.5 |
| 4 | 5,842 | 4,907 | 8,006 / 9,765 | 6,334 / 8,265 | 344.6 / 264.8 |
| 5 | 6,110 | 4,848 | 8,093 / 8,787 | 5,785 / 8,534 | 357.3 / 244.8 |

Across the five pairs, the mean of the paired road median-latency reductions is 1,101 µs, or 18.19% relative to copy mode; the sample standard deviation of paired percentages is 2.26 percentage points. Mean process CPU reduction is 28.87%. These are descriptive pilot results only. The prespecified final paired set and its confidence interval have not been collected, and this class-level harness does not measure `encoderd` IPC or full-system overhead.

Pilot log hashes:

| Pair | Copy log SHA-256 | Zero-copy log SHA-256 |
|---:|---|---|
| 1 | `9d0b972814f9ec3dd98bbd4fc8d4a8cc1182e52a1075d288741eabdd7d1f24a2` | `879383968da9f6e444c21f4c7b9dcede0772b158248b06ada2d4851470de9fe1` |
| 2 | `f6d9151197bf9bf8fa61d0c70576cce59c66b4340815eac50d939ea01325c450` | `8d658df1585e58b67122d1a6672021ea7549b144e13acf4e3f767777faab9f40` |
| 3 | `8ffa048d457e9fafab6acfde9c278ab5e7155d4cde5c93bfe47df67dda3c023c` | `0a57b5090a82166fffc242087e7689762dd6804dbfaf24a40f8f688cfa24dd01` |
| 4 | `e4e1f31205d4d6ca170160a023e381dac910b8d74f3cec2fc76d28aef23581b9` | `4483ce61febf1f6b2a2ad969fd3d79d34377a32bd6d0010906810432eb138584` |
| 5 | `53327d5bec7a2be9ac4f47b4b0d5e296512c7a840c717419c5359f734798b365` | `f4efef9dfb167d618af0981f6553d00c8620e3f7cbf36aa2f314f1dcb328c2bd` |

## Disposition

The corrected direct class harness demonstrates successful DMA-BUF import and valid, matching encoded output for the tested synthetic workload. The exploratory pilot shows lower road submit latency and process CPU in the zero-copy arm, but no final-set confidence interval exists. No claim is made for a stable speedup, the full `encoderd` pipeline, three simultaneous camera streams, zero frame loss outside these samples, or behavior on a production route. Continue only with the bounded final paired set in the approved plan, rechecking all abort guards before the first final sample. The merged `encoderd` binary remains unchanged and was not executed.
