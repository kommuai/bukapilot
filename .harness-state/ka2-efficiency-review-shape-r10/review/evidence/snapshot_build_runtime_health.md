# Runtime health report: r10 production smoke at revision 266

## Result

**PASS for the synthetic DMA-heap VisionIPC target on the local KA2/RK3588 board.** The r269 run used production loggerd and encoderd rebuilt from the current source snapshot, including encoderd.cc SHA-256 fde3d8cb1adfd5308279df4c3c37989ebed7a63127a1f51ed1cbdb48e7eab8d9. It completed two rotations with healthy reports and zero known drops, wrote all expected files for segments 0–2, and produced qRoad H.264 video and audio that decoded in every segment.

The passing smoke is evidence/runtime-r10-smoke-r269.log (SHA-256 556f5ff807a2825d20034656b758e6ca5d56701dbeba388416d4e624cacbaf2f). The typed probe binds the current source hashes and input revision 266. The run-save state was coherent and pinned at revision 266 at stage entry. The workspace base is git HEAD 438acf545a776a63a05a70e42268f1ea32da036c; the dirty source snapshot is bound by hashes below. No commit, deployment or save-run checkpoint was made.

The claim is limited to this synthetic DMA-heap path on KA2. It does not qualify live V4L2 camera export, synchronization fences, live camera-buffer reuse, deployed production MPP behavior, or non-KA2 silicon.

## Readiness contract and observations

- **Target:** Local, non-production KA2/RK3588 board. Board testing was authorized; no network or authenticated traffic was used.
- **Startup window:** 10 seconds from daemon launch for road, driver, wide-road and raw-audio subscriptions and LoggerdSegment=0.
- **Lifecycle window:** 24 seconds for segments 1 and 2, then at least 20 additional frames in segment 2. Liveness was sampled every 2 seconds.
- **Pass predicate:** Both production daemons stay alive; rotation reports for segments 1 and 2 are healthy with zero known drops; segments 0–2 contain all six expected nonempty files; qRoad H.264 video and audio decode in each segment.

The fresh preflight found no camerad, loggerd or encoderd processes and no users of the 83 enumerated video, media, V4L-subdevice, MPP and RGA nodes. The publisher reported four 1920×1200 DMA-heap buffers per stream, stride 1920, UV offset 2,304,000. The four required subscriptions and segment 0 were ready about 0.10 seconds after daemon start.

The run sent 106 synchronized frames per camera stream (318 camera frames total) and 106 audio events. Loggerd reached segment 1 at 7.26 seconds and segment 2 at 9.46 seconds after runner start. Both rotation reports had degraded=false, no missing streams and every known-drop counter at zero. Their reported elapsed times were 73 ms and 173 ms. The full runner, including preflight and postflight, took 16.16 seconds by its monotonic clock.

Each segment contained rlog.zst, qlog.zst, qcamera.ts, fcamera.hevc, dcamera.hevc and ecamera.hevc, all nonempty. qRoad H.264 video and audio decoded in segments 0, 1 and 2; decoded audio output contained 34,816, 37,888 and 17,408 samples, respectively.

Loggerd, encoderd and the publisher exited cleanly. Postflight confirmed the test-owned VisionIPC socket and isolated MSGQ directory were removed, with no camera/recording process or device users. Fixture media, Params and temporary data lived under /data and were deleted after inspection. The pre-existing /tmp capacity pressure was left untouched; only the product's transient VisionIPC socket used its fixed /tmp namespace.

## Production build and dependencies

SCons built system/loggerd/loggerd, system/loggerd/encoderd and build/system/loggerd/tests/visionipc_dmabuf_publisher for KA2/RK3588. The build had no LOGGERD_TEST_BUILD compile flag. The current encoderd source change adds test-only all-output-reopen counting inside LOGGERD_TEST_BUILD guards; the production compile omitted that macro. The binary audit found no test-only marker strings and ldd resolved all three binaries without missing libraries.

- loggerd: SHA-256 9fceb8f103a11687c766eb5b6e48a8086faecd433e76b4fc765a187bb979ee7c; Build ID 855b9a6081d361ce79ab671cd4af62ed10e61157.
- encoderd: SHA-256 8871ac0e11dd97049c0218d4a16199911166f46e3ddd0790ffaf2c5f1ffdb9ae; Build ID 9970f52a7ff0b2a980b7937fcc9012488aef07c2.
- DMA-heap publisher: SHA-256 f3a50f36501e4057c34a909bd73e70ad2fd57fe3732a18c1cd99ef8f0595c1ef; Build ID f802116f8914511923c7754da42863559ba5cdec.

The runtime used LOGGERD_TEST=1 and LOGGERD_SEGMENT_LENGTH=2 to shorten the interval. LOGGERD_TEST_DETERMINISTIC=1 selected the production-default RK3588 rotation algorithm despite the test timing guard. ENCODERD_DETERMINISTIC_DISABLE and all fault selectors were unset. No test-only hooks or fault injection were present in the production binaries.

The DMA heap, MPP and RGA nodes were present. ffmpeg and ffprobe 6.1.1 were available; all shared libraries resolved. The publisher reached READY, loggerd subscribed to all camera/audio message services, and encoder output was recorded and decoded. No network dependency applies to this local service-pair smoke. See evidence/runtime-r10-dependencies-r269.log and evidence/runtime-r10-binary-audit-r269.log.

## Preserved runtime history

The earlier r268 production smoke also passed using encoderd.cc SHA-256 a30d52209fbfb79dc8cf3efe1a1bc8c7eb9e76a2dde3db422f0c22684537396e. That report, probe and smoke are preserved as evidence/runtime-r10-report-r268.md, evidence/runtime-r10-probe-r268.json and evidence/runtime-r10-smoke-r268.log. The current r269 smoke revalidates after the guarded test-counter source update and uses current production binaries.

The first r267 attempt, run without LOGGERD_TEST_DETERMINISTIC, advanced through segments 0–2 but failed the harness's report assertion before media inspection. LOGGERD_TEST=1 disables the deterministic path unless the flag is supplied, so no rotation reports were published in that attempt. The daemons and publisher exited cleanly and postflight passed. This was a harness-mode mismatch, not a daemon failure; the attempt remains preserved as evidence/runtime-r10-smoke-r267.log with its clean postflight.

The non-executing start-command discovery was previously run with Python 3.13.15. Its repo scan exceeded the configured byte limit before enumerating candidates. The handoff named the production loggerd and encoderd binary paths; the smoke launched those paths directly. Discovery output is preserved as evidence/runtime-r10-start-discovery-r267.log.

## Input source hashes

| Source | SHA-256 |
|---|---|
| system/loggerd/loggerd.cc | 3a2e621adc50f44f727a12b0d8deb7410a4c502948ae2297069aa40b4ea8f5a9 |
| system/loggerd/loggerd.h | be831a312bedac0b997eaea6a932f63f2ca265f35c5881c96c9c0a17097e1034 |
| system/loggerd/encoderd.cc | fde3d8cb1adfd5308279df4c3c37989ebed7a63127a1f51ed1cbdb48e7eab8d9 |
| system/loggerd/encoder/ffmpeg_encoder.cc | 7124dee947f6eb409e5fc0e720182f077554d470871dc5c400b6430abc1ea8de |
| system/loggerd/encoder/ffmpeg_encoder.h | ca58013144a178daf80b1e483dcfd56a231d3af72129aec5a7052e8df269f8d8 |
| system/loggerd/encoder/mpp_encoder.cc | 24537361363ace4fd11485fca4284938f22e9b73fdb13ff84f100dd8368d5477 |
| system/loggerd/encoder/mpp_encoder.h | 253c02a7c10624d0333f3360cb13281d6edb1ede10efe91e705c03ac284e8f43 |
| system/loggerd/encoder/mpp_encoder_input.h | 965e011ff0676341996a13974f7c86f87a26bc72441765eefc68f115ee068ce3 |
| system/loggerd/SConscript | 9423e0b617e2b99b50ee97e592c65d89925a6bbad0af6f8944eb78fb056c2373 |
| system/loggerd/tests/visionipc_dmabuf_publisher.cc | 0994305f86b1c7c1481d3276ee2dd5b5fd81d033847db822a198568941fefad9 |
| system/loggerd/tests/test_dmabuf_fixture.py | cfc0baf39d00be3871f7e8359083adc86266e29333e3b2dc09cd349e175a7b9c |

## Evidence hashes

| Evidence | SHA-256 |
|---|---|
| evidence/runtime-r10-smoke-r269.log | 556f5ff807a2825d20034656b758e6ca5d56701dbeba388416d4e624cacbaf2f |
| evidence/runtime-r10-smoke-runner-r269.py | 39c1afee7094e3e6fed08934fd7d51181ce25d2bfdc958420f8769d03adc29a7 |
| evidence/runtime-r10-startup-r269.log | 3f7315dd0c803bd549f477a4b20923886a475c221605ab88c1ee6490f01d4aea |
| evidence/runtime-r10-readiness-r269.log | 2103534f30b35275b83841d44448a1c171cbde343d9c857af5c6127417360e13 |
| evidence/runtime-r10-dependencies-r269.log | a5c4d46d68dd77b578d8997114b80b636c0ba75a3cd161ae3ec6253590df1ec5 |
| evidence/runtime-r10-preflight-r269.log | afdb7d33c83bb3be615e1a025719dd34f7d48bada715c4c9772f7d56421cbe8a |
| evidence/runtime-r10-postflight-r269.log | 0b7ccf649b8e3ec83d9e508e0cef8224c6fcc150167f1d491ce881689d2ee8ec |
| evidence/runtime-r10-build-r269.log | 1c7706e78a0ae87a3ac6f7b3fe274a06eb56a904f0a74ba9bafa98491aae5590 |
| evidence/runtime-r10-binary-audit-r269.log | c6b86d4ecf089c786ce5331eb124fab899380654abe4566874fb08374604c7ae |
| evidence/runtime-r10-smoke-r268.log | 6307d13b26184d1ff9c9184e850fe1064cbb38986cf55cfc4239bfd2ba791b92 |
| evidence/runtime-r10-report-r268.md | 732799b2d6b29b2bca668492f4253c7a0034ee428481b4d87291bfa47b001c97 |
| evidence/runtime-r10-probe-r268.json | c38a6bce8811390f334440a96b6a0734c78f1e7a7913f46ebfd6ff271eac7a21 |
| evidence/runtime-r10-smoke-r267.log (preserved failed attempt) | 9581fc87fbd3b8e869dd7a4145ece797800df27c81f76afc0dfa0cf64ea8494a |
| evidence/runtime-r10-postflight-r267.log | 0b7ccf649b8e3ec83d9e508e0cef8224c6fcc150167f1d491ce881689d2ee8ec |
| artifacts/runtime-r10-probe.json | 528ab4e04f008f13fb7417646560641d1ea914727b22b9b2d1ffb5856403e82e |

## Qualification limits

**Major:** This run uses a synthetic DMA-heap publisher. Live V4L2 DMA-BUF export, fences and synchronization, real camera-buffer reuse, deployed production MPP behavior, and non-KA2 silicon remain unqualified. Keep the runtime claim scoped to the synthetic KA2/RK3588 path.
