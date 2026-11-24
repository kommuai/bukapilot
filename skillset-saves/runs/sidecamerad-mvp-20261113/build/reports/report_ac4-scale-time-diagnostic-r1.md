# AC-4 scale timing diagnostic — r1

**Outcome:** The second bounded run reproduced a steady-state 61.032 ms output gap after warmup. V4L2 source cadence remained below 60 ms for the same frame pair; the interval grows across FFmpeg processing. The scale conversion uses substantial CPU, but the matched call itself takes 23.139 ms wall time and 23.136 ms thread CPU time, so it is not a 61 ms blocked or unusually long call. The remaining mechanism is still inside FFmpeg’s pre-filter handoff/decode scheduling or host scheduling.

## Scope and setup

- User-authorized, metadata-only diagnostic following the approved same-frame plan.
- Same camera: `/dev/v4l/by-path/platform-xhci-hcd.11.auto-usb-0:1:1.0-video-index0` → `/dev/video60`; 1280×720 MJPEG at requested 20 fps.
- Exact production `/usr/bin/ffmpeg` argv and filter graph from `system/sidecamerad.py`, with the selected device `/dev/video60`; no product source or argv change.
- Twenty complete outputs were warmup. The first subsequent complete EOF gap over 60 ms triggered stop after 27 complete frames.
- A metadata-only `LD_PRELOAD` observer recorded V4L2 DQBUF, decoder and filter PTS/completion, mux completion, plus `sws_scale_frame` wall and thread CPU time. The output buffer was reused and no image payload was written.
- The observer ring had no overflow. The run stopped via SIGINT and closed the reader pipe; FFmpeg exited with status 255. Follow-up checks found no FFmpeg process or `/dev/video60` holder.
- `/usr/bin/ffmpeg` hash remained `9af5a141b45d32f64fe7bb5cd2c939d56fb9972234705513c45575ef7a1da38e`; `system/sidecamerad.py` remained `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b`.

## Same-frame interval growth

For contiguous camera sequences 25→26, whose timestamps align exactly to decoder PTS `1,939,971 → 1,987,992` µs:

| Boundary | Interval |
| --- | ---: |
| V4L2 source SOE timestamps | 48.021 ms |
| DQBUF completion | 50.928 ms |
| Decoder output completion | 56.561 ms |
| Filter-sink output completion | 62.602 ms |
| Mux packet completion | 61.048 ms |
| Complete reader EOF | **61.032 ms** |

This trigger occurred between reader outputs 26→27, after the 20-frame warmup. The camera source interval for that same frame pair was 48.021 ms. Startup therefore does not explain this event.

## Scale-call timing

The matched `sws_scale_frame` call for PTS `1,987,992` µs took **23.139388 ms wall** and **23.135583 ms thread CPU** (difference 0.003805 ms). Across 28 calls, wall time averaged 21.522 ms (p95 25.578 ms, p99 26.077 ms); thread CPU averaged 21.076 ms (p95 25.231 ms). The conversion is a meaningful software CPU cost, but the measured call is not a long internal wait. The adjacent scale-call start interval was 56.491 ms, versus a 48.021 ms source SOE interval; the extra interval is already present before the next scale call begins.

## Interpretation and limits

This evidence strongly shifts the observed steady-state gap away from camera capture cadence and into the FFmpeg/software path. Scaling consumes about 21 ms CPU per frame, but the >60 ms event is not caused by one >60 ms scale invocation. The first measured interval growth appears before decoder output, with a further increase by the filter-sink boundary. The leading suspects are FFmpeg demux/decode handoff or thread scheduling around those stages; this run does not distinguish them.

`LD_PRELOAD` observer overhead was not independently calibrated. The call timings are localization evidence, not an AC-4 acceptance result. The existing five-minute uninstrumented run remains the strict-cadence failure evidence.

## Evidence

- Normalized result: `build/artifacts/ac4-scale-time-diagnostic-r1.json` — SHA-256 `2b905d0384dce1382f3a86695de2be4f488f397669166631773cf87ae4bed6ea`
- Per-frame event log: `build/evidence/ac4-scale-time-diagnostic-r1.jsonl` — SHA-256 `b579fb90d69db2f57a7f9b41cf1db40174349fb7bd8a3586fcc2030e631176a6`
- Scale observer ring: `build/evidence/ac4-scale-time-observer-r1.bin` — SHA-256 `957e76ed323b1d1b55918b35375ded495b9cbaf9ac9dc6fa5cb8d7a96f72958c`
- FFmpeg observer ring: `build/evidence/ac4-scale-time-ffmpeg-observer-r1.bin` — SHA-256 `fdc21055990876bfc78c9a4c790a7567b60e13da07d7cb74aa3404b73b0fc509`
- Observer source/library: `build/artifacts/ac4-scale-time-observer-r1.c` — SHA-256 `b9b3409de9d5dfeb2df86324df629c7ece7269809f45d4e0e344c63357af0224`; `.so` — SHA-256 `773d123aa5177901fb81524e06d0a4420dee500babfcbe90996c95001ca1da61`
- Runner: `build/artifacts/ac4-scale-time-runner-r1.py` — SHA-256 `87630e64a2ee6da4c910c2e94a6734fc7fa5590a91309d58e24d4d37732570c6`

## Next step

Make one more bounded metadata capture that times `av_read_frame`, `avcodec_send_packet`, and the existing decoder/filter handoffs with both wall and thread CPU clocks. This should show whether the extra interval is spent waiting for input, inside decode submission, or between the FFmpeg stages. Keep the same camera, binary, argv, warmup, 600-frame cap, and first post-warmup >60 ms stop rule; do not change product source.
