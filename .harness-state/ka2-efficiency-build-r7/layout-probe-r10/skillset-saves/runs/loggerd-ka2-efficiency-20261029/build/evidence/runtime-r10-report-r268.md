# Runtime health report: r10 KA2 encoder recovery and qRoad refresh

## Result

**PASS for the synthetic DMA-heap VisionIPC target on the local KA2/RK3588 board.** The corrected production-binary smoke ran the current encoderd recovery policy and qRoad FFmpeg changes. Loggerd and encoderd started, advanced through segments 0–2, reported two healthy rotations with zero known drops, and wrote valid qRoad H.264 video and audio in every segment.

The passing smoke is evidence/runtime-r10-smoke-r268.log (SHA-256 6307d13b26184d1ff9c9184e850fe1064cbb38986cf55cfc4239bfd2ba791b92). It ran 2026-10-31 at about 13:14 UTC against the current workspace source snapshot. The run-save status was coherent and pinned at revision 261 when this stage began; the exact source inputs are bound below. The base git HEAD is recorded in the probe. No commit or deployment was performed.

This qualifies the synthetic DMA-heap path on the KA2 board. It does not qualify live V4L2 camera export, synchronization fences, live camera-buffer reuse, or the deployed production MPP package.

## Readiness contract and observations

- **Target:** Local, non-production KA2/RK3588 board. The user authorized board testing. No network or authenticated traffic was used.
- **Startup window:** 10 seconds from daemon launch for road, driver, wide-road and raw-audio subscriptions and LoggerdSegment=0.
- **Lifecycle window:** 24 seconds for segments 1 and 2, followed by at least 20 frames in segment 2. Liveness was sampled every 2 seconds.
- **Pass predicate:** Production loggerd and encoderd remain alive; rotation reports for segments 1 and 2 are healthy with zero known drops; segments 0–2 have all six expected nonempty files; qRoad H.264 video and audio decode in every segment.

The publisher reported a four-buffer, 1920×1200 DMA-heap fixture with stride 1920 and UV offset 2,304,000. The daemons became ready about 0.15 seconds after launch. The run sent 106 synchronized frames to each of the three camera streams (318 camera frames total) and 106 audio events. It reached segment 2 at 9.47 seconds after runner start and completed preflight, smoke and postflight in about 15.8 seconds according to the runner clock.

Both rotation reports had degraded=false, no missing streams and all known-drop counters at zero. Segment 1 and 2 reported 159 ms and 121 ms, respectively. Every segment contained rlog.zst, qlog.zst, qcamera.ts, fcamera.hevc, dcamera.hevc and ecamera.hevc; all were nonempty. qRoad H.264 video and audio decoded in segments 0, 1 and 2. Decoded audio output contained 34,816, 36,864 and 17,408 samples, respectively.

Loggerd, encoderd and the publisher exited cleanly. The postflight found no camera/recording processes or users of video, media, V4L-subdevice, MPP or RGA nodes. The unique VisionIPC socket and MSGQ directory were removed. Logs and media were stored under /data and deleted after inspection; /tmp remained at its pre-run 99% capacity.

## Build and runtime dependencies

The production build command was SCons with targets system/loggerd/loggerd, system/loggerd/encoderd and build/system/loggerd/tests/visionipc_dmabuf_publisher. It used no LOGGERD_TEST_BUILD compile flag. The binary audit found no test-only fault-selector or counter markers, and ldd resolved all three binaries with no missing libraries.

- loggerd: SHA-256 9fceb8f103a11687c766eb5b6e48a8086faecd433e76b4fc765a187bb979ee7c; Build ID 855b9a6081d361ce79ab671cd4af62ed10e61157.
- encoderd: SHA-256 9aaac89d5b8202510247e60b914eba06d10d749a8b1eac6a3a04c6ec1abadfc9; Build ID f585db9949a08f2991f6123e831c2e43ff1d65d9.
- DMA-heap publisher: SHA-256 f3a50f36501e4057c34a909bd73e70ad2fd57fe3732a18c1cd99ef8f0595c1ef; Build ID f802116f8914511923c7754da42863559ba5cdec.

The runtime used LOGGERD_TEST=1 and LOGGERD_SEGMENT_LENGTH=2 to shorten the segment interval. LOGGERD_TEST_DETERMINISTIC=1 selected the same RK3588 deterministic rotation algorithm enabled by default in production when LOGGERD_TEST is absent; this preserves the production rotation behavior while allowing the timing override. ENCODERD_DETERMINISTIC_DISABLE and all fault selectors were unset. Test-only hooks and fault injection remained compiled out.

The DMA heap, MPP and RGA device nodes were present and idle at preflight. Runtime library dependencies resolved. ffmpeg and ffprobe 6.1.1 were available. The fixture publisher became ready, all four expected message subscriptions connected, encoder output was produced, and segment files decoded successfully. See evidence/runtime-r10-dependencies-r268.log and evidence/runtime-r10-binary-audit-r267.log.

## Start-command discovery

The prescribed non-executing check_runtime.py discovery ran with Python 3.13.15 and exited 1 because the repository scan exceeded its total-byte limit before it could enumerate start-command candidates. The authorized handoff named the production loggerd and encoderd binaries; the smoke launched those exact paths and recorded the start commands. The discovery limitation is preserved in evidence/runtime-r10-start-discovery-r267.log.

## Preserved first attempt

The initial r267 smoke was run with LOGGERD_TEST=1 but without LOGGERD_TEST_DETERMINISTIC. Loggerd and encoderd remained alive and rotated through segments 0–2, but the harness stopped at its missing-report assertion before inspecting the segment media. With LOGGERD_TEST enabled, the deterministic rotation path is disabled unless that flag is set, so the production rotation reports were not emitted. The daemons and publisher exited cleanly and postflight passed. This was a harness-mode mismatch, not evidence of a daemon failure. Its log and postflight are preserved as evidence/runtime-r10-smoke-r267.log and evidence/runtime-r10-postflight-r267.log. The corrected r268 invocation set the flag to match the production-default RK3588 path and passed the full predicate.

## Input source hashes

| Source | SHA-256 |
|---|---|
| system/loggerd/loggerd.cc | 3a2e621adc50f44f727a12b0d8deb7410a4c502948ae2297069aa40b4ea8f5a9 |
| system/loggerd/loggerd.h | be831a312bedac0b997eaea6a932f63f2ca265f35c5881c96c9c0a17097e1034 |
| system/loggerd/encoderd.cc | a30d52209fbfb79dc8cf3efe1a1bc8c7eb9e76a2dde3db422f0c22684537396e |
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
| evidence/runtime-r10-smoke-r268.log | 6307d13b26184d1ff9c9184e850fe1064cbb38986cf55cfc4239bfd2ba791b92 |
| evidence/runtime-r10-startup-r268.log | d00ff81cc168007bc81f37958a75847336ffd20f724666e1abaafe395093e35b |
| evidence/runtime-r10-readiness-r268.log | b63e751f2e6a43b7e00273bfa8f94858666d7114ca04b6e7df6a1b9af3b62d54 |
| evidence/runtime-r10-dependencies-r268.log | 946880cbc0f94f783d52ec75de931ec429549cf838dc62385c307546234dba89 |
| evidence/runtime-r10-preflight-r268.log | afdb7d33c83bb3be615e1a025719dd34f7d48bada715c4c9772f7d56421cbe8a |
| evidence/runtime-r10-postflight-r268.log | 0b7ccf649b8e3ec83d9e508e0cef8224c6fcc150167f1d491ce881689d2ee8ec |
| evidence/runtime-r10-build-r267.log | ab2f9b4862d82591695ddaf929102161472137a6c45d2e42f27be5ee4061561a |
| evidence/runtime-r10-binary-audit-r267.log | f8e6c5ca3360e743b8b49e493d5d0988ebddb1e7dbc5eaf922d2ddc29e31e038 |
| evidence/runtime-r10-start-discovery-r267.log | 2b48623631ffe98ce1d34c76dd4cc58ee80458136e1e6f62f0079a72d963bf3a |
| evidence/runtime-r10-smoke-r267.log (preserved failed first attempt) | 9581fc87fbd3b8e869dd7a4145ece797800df27c81f76afc0dfa0cf64ea8494a |
| evidence/runtime-r10-smoke-runner-r268.py | 18301bbfa56ff000b9dc18cb7b8f4fc17cfbdaf83543fb96331fb369b2c75e86 |
| evidence/runtime-r10-smoke-runner-r267.py | cd07857603daa63905da97ba2e7e52b1df996e318333a9e9855754800eb5152e |
| evidence/runtime-r10-startup-r267.log | 79e6bf53e68c805ffe2cd5ff16d9717cc4ba9f3c5eb81e19d35ab83dd2ceaa5a |
| evidence/runtime-r10-readiness-r267.log | e8d444a4dff92d1c585f070d38c6bb07d0bc71eb42897fc8d8b643b6c4f34e77 |
| evidence/runtime-r10-dependencies-r267.log | 3901fead3f3efc9923cc1c584b58fdf09a626eacf57911b79bee6d59659cb06b |
| evidence/runtime-r10-preflight-r267.log | afdb7d33c83bb3be615e1a025719dd34f7d48bada715c4c9772f7d56421cbe8a |
| artifacts/runtime-r10-probe.json | c38a6bce8811390f334440a96b6a0734c78f1e7a7913f46ebfd6ff271eac7a21 |
| evidence/runtime-r10-postflight-r267.log | 0b7ccf649b8e3ec83d9e508e0cef8224c6fcc150167f1d491ce881689d2ee8ec |

## Qualification limits

**Major:** This smoke uses a synthetic DMA-heap publisher. Live V4L2 DMA-BUF export, fences and synchronization, real camera-buffer reuse, and deployed production MPP behavior remain unqualified. Keep the runtime claim scoped to the synthetic KA2/RK3588 path.
