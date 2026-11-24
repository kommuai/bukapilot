# AC-4 pipeline correlation diagnostic — r1

**Conclusion:** The repeated steady-state gaps are primarily a software processing problem in FFmpeg’s MJPEG decode and scale/format path, with some additional variation in when FFmpeg dequeues camera buffers. The camera’s source cadence stayed regular. This is not a startup-only effect and the data does not support a sensor cadence fault.

## 600-frame capture

The same camera, FFmpeg binary, production argv, and 20-frame warmup were used. The run continued through 600 complete outputs, retaining only per-frame timestamps, PTS, call durations and sizes. No image payload was saved, and product source and argv were unchanged.

Across 580 post-warmup intervals:

- **77 (13.3%)** reader gaps exceeded 60 ms; the gaps appeared in every 100-frame block after warmup.
- Maximum reader gap: **77.761 ms**; p95 **64.849 ms**; p99 **71.419 ms**.
- Matched camera SOE intervals: mean **50.000 ms**, maximum **52.209 ms**, with **0/580** over 60 ms.
- In this heavier instrumentation run, DQBUF completion intervals exceeded 60 ms 40 times, with maximum **61.207 ms**. Of the 77 slow output intervals, 50 still had a DQBUF interval at or below 60 ms.

## Software processing tracks the output tail

For each frame, I matched the camera timestamp to decoder PTS and compared the output gap against the input dequeue interval and the change in decode/filter CPU time. The correlation between the output gap in excess of DQBUF cadence and the frame-to-frame change in decode-plus-filter CPU was **0.937** across the 580 intervals.

On intervals with output gaps over 60 ms, the next frame used **37.284 ms** average CPU for decode plus filter, compared with **32.556 ms** on intervals at or below 60 ms. Its CPU cost averaged **10.608 ms more than the previous frame**; on normal intervals, that difference averaged **−1.617 ms**. The camera source intervals during slow outputs were slightly shorter on average (48.937 ms) than during normal outputs (50.163 ms).

The heavier work is concentrated in `avcodec_send_packet` and `buffersrc_add_frame_flags` (which runs the scale/format conversion). Their p95 thread CPU times were **17.070 ms** and **25.949 ms** respectively. The mean `av_read_frame` call took 0.021 ms wall time; the output buffer sink and mux calls were small beside decode and filter work.

## Hardware and startup cross-checks

An independent 60-second V4L2-only capture recorded 1,198 steady intervals: mean **49.996 ms**, maximum **52.287 ms**, and none over 60 ms. A separate prior FFmpeg correlation capture also had steady DQBUF intervals no greater than 58.634 ms (0/600 over 60), while 146/598 output-write intervals exceeded 60 ms and reached 89.580 ms. That earlier trace did not prove exact DQBUF-to-output frame identity, but it independently showed that output cadence could be irregular while source timestamps and dequeue cadence remained regular.

The current 600-frame capture’s DQBUF intervals do show some >60 ms timing. Because SOE timestamps remain regular, that points to delay after exposure and before dequeue; these captures cannot separate USB/driver delivery from FFmpeg input-thread scheduling for those intervals. The strong decode/filter CPU correlation and the prior stable DQBUF trace still make software processing the primary cause. The earlier 695.988 ms sequence 0→1 timestamp jump was a separate startup event; current slow intervals recur well after warmup.

## Limits

The observers were not independently calibrated. This run’s maximum was 77.761 ms, below the earlier uninstrumented 90.982 ms maximum, so it does not identify the exact mechanism of that most severe event. The findings establish software processing variability as the leading cause of the reproduced tail, while leaving room for host scheduling and input-path timing to contribute.

## Evidence

- Aggregate correlation analysis: `build/artifacts/ac4-pipeline-correlation-analysis-r1.json` — SHA-256 `95eeeb7d6bdc3c05462b331cb5b679a70eda4c49f47a7895f1537f0e88db57e5`
- Raw capture summary: `build/artifacts/ac4-pipeline-correlation-diagnostic-r1.json` — SHA-256 `31153c9104bf1347c4e7ef641a70c8408fd881dc9107479b4ecadbc2114415da`
- Per-frame events: `build/evidence/ac4-pipeline-correlation-diagnostic-r1.jsonl` — SHA-256 `9a1db04b5bf4d2608b984c88babb3a33d7a4fc242fe678935589c9ee16001a25`
- Observer rings: API `936608726a82f807f317fd024faaf8527034096b20c8fdafbfe285e149ea95cf`; FFmpeg stages `ba46abb2104efc9572696edda37e762511c9aed4d21b4b0cab37301325b3197a`; scale `2bd2deb39abb4875231e268fbaaf4a5d58c35adf84f3efd85d24a7eec0793c2b`
- Runner: `build/artifacts/ac4-pipeline-correlation-runner-r1.py` — SHA-256 `a598c2c1e7bf03429594965874d7d770346bfd8808f07286224d3839d63a8d8a`
- Hardware cross-checks: `investigation/evidence/v4l2-capture-timestamps-60s.log` and `build/artifacts/dqbuf-output-correlation-r2.3-r1.json`

AC-4 remains a cadence failure. No product code was changed in these diagnostics.
