# Runtime health report: KA2 loggerd/encoderd r10, fixture refresh r231

## Outcome

**PASS for the named synthetic DMA-BUF board target.** This refresh reran the production-binary smoke after the DMA-heap publisher and its Python fixture changed. Loggerd and encoderd started, accepted the updated four-buffer publisher, completed two rotations, and produced decodable qRoad video/audio across segments 0–2. The runtime smoke log is `evidence/runtime-r10-smoke-r231.log` (SHA-256 `d5436e9418dbc2eb7cdf861e4a2c6c182ac62de4a69ea672bda9f36cf03a39e3`).

The successful run used workspace HEAD `438acf545a776a63a05a70e42268f1ea32da036c` and run input revision **231**. Production loggerd/encoderd source hashes match the earlier smoke; fixture inputs are refreshed and bound in `artifacts/runtime-r10-probe.json`. This qualifies the synthetic DMA-heap VisionIPC path on this board. It does not qualify live camera/V4L2 export, synchronization fences, live camera-buffer reuse, or the deployed production MPP package.

## Readiness contract and observations

- **Target:** local, non-production KA2/RK3588 test board. The user authorized board testing; no authenticated or network requests were made. Routes, Params and MSGQ used unique isolated paths.
- **Startup window:** 10 seconds from daemon launch for all three camera-state subscriptions, raw audio subscription, and `LoggerdSegment=0`. Process liveness was sampled every 2 seconds during rotation.
- **Lifecycle window:** 24 seconds to complete segments 1 and 2, followed by 20 frames after segment 2.
- **Pass predicate:** both production daemons remain alive; both rotation reports show no missing streams, degradation or drops; segments 0–2 contain all six expected nonempty files; qRoad H.264 video and audio decode in each segment.

The updated publisher reported `backing=dma_heap`, 1920×1200 NV12 streams, stride 1920, UV offset 2,304,000, and four DMA-BUF buffers per stream. All subscriptions and segment 0 were ready about 0.10 seconds after the daemons started. Segment 1 began at 7.13 seconds and segment 2 at 9.28 seconds. The flow sent 105 frames. Rotation reports were healthy: segment 1 committed in 64 ms, segment 2 in 132 ms, both with `missing_streams=[]`, `degraded=false`, and all known-drop counters zero.

After graceful shutdown, each segment contained `rlog.zst`, `qlog.zst`, `qcamera.ts`, `fcamera.hevc`, `dcamera.hevc`, and `ecamera.hevc`, all nonempty. qRoad H.264 and audio decoded in each segment; decoded audio sample counts were 34,816, 36,864, and 17,408. The smoke log records the process, report, segment, decode, and teardown observations.

## Build, dependencies, and teardown

SCons built `system/loggerd/loggerd`, `system/loggerd/encoderd`, and the updated DMA-heap publisher with no `-DLOGGERD_TEST_BUILD` option. The loggerd and encoderd binary scans found no test-only fault-selector, counter, or fake-clock markers. Binary digests and Build IDs:

- `system/loggerd/loggerd` — SHA-256 `9fceb8f103a11687c766eb5b6e48a8086faecd433e76b4fc765a187bb979ee7c`, Build ID `855b9a6081d361ce79ab671cd4af62ed10e61157`
- `system/loggerd/encoderd` — SHA-256 `e7dd7e418fc41d384d0896b0e1d76705e229f48eb6c5d601448f00bece19362e`, Build ID `e0502f8b9154d4042a5b3b26b3823893a2da2533`
- `build/system/loggerd/tests/visionipc_dmabuf_publisher` — SHA-256 `f3a50f36501e4057c34a909bd73e70ad2fd57fe3732a18c1cd99ef8f0595c1ef`, Build ID `f802116f8914511923c7754da42863559ba5cdec`

`LOGGERD_TEST=1` and `LOGGERD_SEGMENT_LENGTH=2` shortened the segment interval only. No MPP fault selector was set. `/dev/dma_heap/system`, `/dev/mpp_service`, and `/dev/rga` were present. Successful HEVC and qRoad output established reachability of the exercised encoder/RGA path; network dependencies are not applicable to this local service pair.

Encoderd and loggerd exited with code 0 after the smoke. The publisher emitted `CLEANUP_OK` and exited 0. Postflight found no active camerad, loggerd or encoderd processes and no users of video, media, V4L-subdevice, MPP or RGA nodes; the isolated VisionIPC socket and MSGQ directory were removed. The run created only disposable synthetic media and isolated Params, all deleted after checks.

## Discovery and preserved attempt

The prescribed non-executing `check_runtime.py --detect-start-command` was previously run for this build but exited 1: host Python is 3.12.3 while the tool requires 3.13, and project inspection exceeded its byte limit. The handoff named the service pair; those exact production binaries were directly built and exercised. Discovery output remains `evidence/runtime-r10-start-discovery.log` (SHA-256 `3f027a3e72324eeaba0d1228ef7c235fc2b22d33b5a94fb4a60045a4ed2b673b`).

The first r231 attempt stopped before daemon launch because the copied one-off runner expected the publisher READY line without the updated `backing=dma_heap` suffix. That failed run is preserved as `runtime-r10-smoke-r231-attempt1.log`; its postflight was clean. The runner expectation was aligned with the current fixture output, and the successful run above passed. Earlier r222 smoke and harness attempts remain preserved; r222 is superseded as the current fixture input changed.

## Evidence hashes

| Artifact | SHA-256 |
|---|---|
| `evidence/runtime-r10-smoke-r231.log` | `d5436e9418dbc2eb7cdf861e4a2c6c182ac62de4a69ea672bda9f36cf03a39e3` |
| `evidence/runtime-r10-startup-r231.log` | `e675fac82f60c311b23c679ed65275c757d0cc22a8e265a653d1c284bba1f71d` |
| `evidence/runtime-r10-readiness-r231.log` | `2f6f3dbb47b0b6e72ced6168ea745528c24c4ee303dfa96e43f7f30df3251999` |
| `evidence/runtime-r10-dependencies-r231.log` | `8d18ee31d3b516281b9d85e96a59b53cff142e938d991c668c30b0bd6bb54352` |
| `evidence/runtime-r10-preflight-r231.log` | `afdb7d33c83bb3be615e1a025719dd34f7d48bada715c4c9772f7d56421cbe8a` |
| `evidence/runtime-r10-postflight-r231.log` | `0b7ccf649b8e3ec83d9e508e0cef8224c6fcc150167f1d491ce881689d2ee8ec` |
| `evidence/runtime-r10-build-r231.log` | `545abdb7484aa1b2089e2b4b513043e3b185b2d9d86bce668ba4bd76e6dc6992` |
| `evidence/runtime-r10-smoke-runner-r231.py` | `65633d1441c34e1778008056f1c370b438763a30dad6677d58bb5782135e8c0c` |
| `evidence/runtime-r10-smoke-r231-attempt1.log` | `22d13b4066733afdd7246e3a51fcfec85e4b7a628d460c226c651fb9a058db0c` |
| `evidence/runtime-r10-preflight-r231-attempt1.log` | `afdb7d33c83bb3be615e1a025719dd34f7d48bada715c4c9772f7d56421cbe8a` |
| `evidence/runtime-r10-postflight-r231-attempt1.log` | `0b7ccf649b8e3ec83d9e508e0cef8224c6fcc150167f1d491ce881689d2ee8ec` |
| `evidence/runtime-r10-start-discovery.log` | `3f027a3e72324eeaba0d1228ef7c235fc2b22d33b5a94fb4a60045a4ed2b673b` |
| `evidence/runtime-r10-smoke.log` | `40bf8754e2a9080955543a53cbaf7f519f706846177f80679a893762ead150cc` |
| `evidence/runtime-r10-smoke-attempt1.log` | `4d25b48dda36dde179e88ee2e2224070fc230fab53155a7e824fa1e39771313c` |
| `evidence/runtime-r10-smoke-attempt2.log` | `3f2e08df92b2cc1addad87af61252d53f4ed39f3cacd479077b1ac5563ae99d4` |
| `evidence/runtime-r10-smoke-attempt3.log` | `4f915259a5c51b51da4a589fb62ad319fc871923ae2487aadf5e3c53627d9ce8` |
| `evidence/runtime-r10-smoke-attempt4.log` | `b6a11d8e357a1ce253f73c1caf6e7a5283a5da70dc0e6a692572df6de2690dff` |
| `artifacts/runtime-r10-probe.json` | `573f13b6c75c8167c850c180dad5754a3f61644a938b9c53d3052d52c640899c` |

## Remaining qualification limit

**Major:** live V4L2 DMA-BUF export, fences/synchronization, camera-buffer reuse, and deployed MPP behavior remain unqualified. Carry this limit into the build package; do not describe this synthetic smoke as live-camera validation.
