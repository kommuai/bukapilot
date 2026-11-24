# AC-4 FFmpeg pipeline timing diagnostic — r1

**Outcome:** This third bounded capture reproduces a 60.520 ms post-warmup output gap and identifies its immediate mechanism: regular camera/dequeue cadence plus a longer FFmpeg decode/filter service time on the triggering frame. The data points to a software throughput/timing problem, not a camera cadence fault. The measured decode/filter CPU work increased by 11.514 ms over the preceding frame, while the output processing latency increased by 9.642 ms.

## Scope and setup

- User-authorized, metadata-only capture under the continued AC-4 diagnostic request.
- Same camera path resolving to `/dev/video60`; 1280×720 MJPEG, requested 20 fps.
- Same `/usr/bin/ffmpeg` production argv and filter graph from `system/sidecamerad.py`; no source or argv changes.
- Twenty complete frames were warmup. The first subsequent EOF gap over 60 ms triggered stop at output frames 27→28 after 28 complete outputs.
- Preload observers recorded V4L2 DQBUF, `av_read_frame`, `avcodec_send_packet`, frame receive, filter source/sink, `sws_scale_frame`, mux, and parent reader EOF. Only timestamps, PTS, sizes, and CPU/wall durations were retained; no image payload was written.
- All three event rings had zero overflow. `/usr/bin/ffmpeg` and `system/sidecamerad.py` hashes were unchanged. FFmpeg exited on the bounded SIGINT stop (status 255); follow-up found no FFmpeg process or `/dev/video60` holder.

## Same-frame evidence

Contiguous V4L2 sequences 26→27 map exactly to decoder/filter PTS `1,412,031 → 1,460,056` µs. The corresponding complete reader outputs 27→28 were 60.520 ms apart.

| Measurement | Previous frame (seq 26) | Triggering frame (seq 27) | Inter-frame interval |
| --- | ---: | ---: | ---: |
| V4L2 monotonic SOE | 35,206.431279 s | 35,206.479304 s | 48.025 ms |
| DQBUF completion | 35,206.487584 s | 35,206.538462 s | 50.878 ms |
| DQBUF completion → reader EOF | 33.675 ms | 43.317 ms | **+9.642 ms** |
| Reader EOF | 35,206.521259 s | 35,206.581779 s | **60.520 ms** |

The arithmetic is direct: the triggering DQBUF completed 17.203 ms after the prior reader EOF, and its frame reached reader EOF 43.317 ms later, totaling 60.520 ms. The excess over the 50.878 ms DQBUF interval is exactly the 9.642 ms increase in dequeue-to-output service latency.

## Where the extra service time appears

The frame’s matched `av_read_frame` call took 0.062 ms wall / 0.057 ms thread CPU. Once the packet was available, FFmpeg spent substantially longer in decode and filter conversion:

| FFmpeg call | Previous frame CPU | Triggering frame CPU |
| --- | ---: | ---: |
| `avcodec_send_packet` (MJPEG decode work) | 10.010 ms | 15.993 ms |
| `buffersrc_add_frame_flags` (includes scale/format filter work) | 17.855 ms | 23.386 ms |
| **Combined decode + filter CPU** | **27.866 ms** | **39.379 ms** |

For the triggering frame, `sws_scale_frame` itself took 23.683 ms wall / 23.339 ms thread CPU. The matched filter-source call took 23.730 ms wall / 23.386 ms CPU. The following buffer-sink call was 0.007 ms; mux write was 2.790 ms wall / 1.076 ms CPU. These measured durations account for most of the frame’s 43.317 ms dequeue-to-output service time.

The decode-plus-filter CPU increase between the pair was 11.514 ms, close to the 9.642 ms service-latency increase. This directly ties the reproduced excess gap to variable software processing cost. The source SOE interval stayed regular at 48.025 ms, and DQBUF completion cadence stayed at 50.878 ms.

## Conclusion and limits

For this reproduced >60 ms event, the camera continued to produce contiguous frames at a regular cadence. The gap appears because the FFmpeg pipeline’s decode/filter work for the next frame takes longer than for the previous one, after the next camera buffer becomes available. This is a software processing budget issue. The data does not support startup as the explanation: the trigger followed 20 warmup frames. The separate 695.988 ms sequence 0→1 startup timestamp jump from the earlier capture remains distinct.

This identifies the immediate mechanism for the captured 60.520 ms event. It does not prove that every interval in the earlier five-minute tail (716/6,049 over 60 ms; maximum 90.982 ms) has the same exact cause. Observer overhead was not independently calibrated, and the longer 90 ms tail was not captured by this first-gap stop rule. JPEG content complexity and host scheduling may contribute to frame-to-frame work variation; this run does not separate those factors.

## Evidence

- Normalized result: `build/artifacts/ac4-pipeline-time-diagnostic-r1.json` — SHA-256 `390a5b3649f876e9d31dfa52560fb34e3be8a4020a1bf2a4e5400e6e7e64fe3d`
- Per-frame event log: `build/evidence/ac4-pipeline-time-diagnostic-r1.jsonl` — SHA-256 `93a01e9e3938f8b678698533a3db4e235c91041c877c4b3bf34db53e2c68e4a1`
- Pipeline, FFmpeg-stage, and scale observer rings: `build/evidence/ac4-pipeline-time-api-observer-r1.bin` `43951cb4fc442ec072413dc24ef2468cdcb2ef4f67ea246d0be0f77b8beff1df`; `build/evidence/ac4-pipeline-time-ffmpeg-observer-r1.bin` `f5de555b003a74bd8b76ec6b5bd238dd4cf5725836931228594858e2fad90f1c`; `build/evidence/ac4-pipeline-time-scale-observer-r1.bin` `fbd817ac69138ddfdcb87525baacd4334e06b525ed0e115adc8a045362f399bb`
- Observer source/library: `build/artifacts/ac4-pipeline-time-observer-r1.c` — SHA-256 `a044d97f2f33d2e28aee5d54cf4714d2262eeb01fbee71f812a750a6cda818ce`; `.so` — SHA-256 `f71219c574aa5ee5dad3b4e6f5057bf3685a01f9b9fe7a71cdf8dbb0fafc7bbc`
- Runner: `build/artifacts/ac4-pipeline-time-runner-r1.py` — SHA-256 `20fc0836f80b795d1453c3d08d1ad0fa5c2dbb7144a50459123172a0136a1ff1`

## Next step

Keep AC-4 open. If further diagnosis is needed, collect a larger same-argv sample that waits for a more severe tail gap and preserves surrounding per-frame decode/filter CPU records; do not change product source or claim AC-4 passes based on this localization run.
