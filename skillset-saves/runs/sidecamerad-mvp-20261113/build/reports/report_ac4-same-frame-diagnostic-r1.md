# AC-4 same-frame diagnostic — r1

**Outcome:** The steady-state >60 ms interval captured in this run appears after camera dequeue, at the FFmpeg filter/output side. It is not explained by the V4L2 source cadence for that frame pair. The result points to a software-path delay, but does not yet distinguish filter work from scheduling around FFmpeg.

## Scope and setup

- User-authorized, metadata-only diagnostic under the approved r1 plan.
- Target: `/dev/v4l/by-path/platform-xhci-hcd.11.auto-usb-0:1:1.0-video-index0`, resolved to `/dev/video60`; 1280×720 MJPEG, requested 20 fps.
- Executed `/usr/bin/ffmpeg` with the exact production sidecamerad argv. The binary hash before and after was `9af5a141b45d32f64fe7bb5cd2c939d56fb9972234705513c45575ef7a1da38e`; `system/sidecamerad.py` hash remained `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b`.
- The external `LD_PRELOAD` observer recorded V4L2 DQBUF sequence/timestamps, decoder frame PTS/completion, filter-sink frame PTS/completion, and mux packet PTS/completion. The parent read complete raw frames into one reusable memory buffer. No image payload was persisted.
- Twenty output frames were treated as warm-up. The run stopped on the first subsequent complete EOF gap over 60 ms, after 26 complete output frames total.

## Same-frame observation

The triggering gap was between output frames 25 and 26. V4L2 sequence numbers were contiguous, and the source timestamps mapped exactly to decoder/filter PTS for those frames:

| Boundary | Frame / sequence 24 → 25 interval |
|---|---:|
| V4L2 source timestamp (monotonic SOE) | 52.023 ms |
| DQBUF completion | 50.732 ms |
| FFmpeg decoder output completion | 55.893 ms |
| FFmpeg filter-sink output completion | **62.067 ms** |
| FFmpeg mux packet completion | **63.028 ms** |
| Complete raw-frame read EOF | **63.049 ms** |

The source PTS values were 1,843,998 µs and 1,896,021 µs; those matched decoder and filter frame PTS. Mux PTS advanced from 37 to 38 in its 1/20-second time base. Thus the camera delivered the corresponding frames about 52 ms apart, while the delay first crossed 60 ms between decoder output and filter-sink output. The mux-to-reader completion difference was about 0.34 ms.

After warm-up, eight consecutive V4L2 source intervals were 47.945–52.029 ms, with none over 60 ms. The initial startup sequence did contain a separate 695.988 ms V4L2 timestamp jump from sequence 0 to 1; the steady-state trigger gap was sequences 24 to 25, not that startup event.

## Interpretation and limits

For this observed steady-state gap, hardware capture timing is not the likely source. The delay is present by the FFmpeg filter-sink boundary, so the leading explanation is FFmpeg filter/output pacing or scheduling around that path. The measurement does not establish whether scaling/filter work, a wait, or host descheduling caused the extra delay.

The observer ring had 109 events and no overflow. It did not capture kernel `write`/`writev` events (`stdout_bytes_recorded=0`) or a scheduler trace; `perf`/BPF tools were unavailable. Observer overhead was not independently calibrated, so this instrumented 63 ms sample is localizing evidence, not an AC-4 pass/fail measurement. The earlier uninstrumented five-minute run already showed persistent gaps (716/6,049 over 60 ms, max 90.982 ms).

At the stop boundary FFmpeg did not exit within the signal cleanup window and was killed with status `-9`, then reaped. Follow-up checks found no remaining FFmpeg/sidecamerad process and no holder on `/dev/video60`. Product source and production argv were unchanged.

## Evidence

- Per-frame and observer metadata: `build/evidence/ac4-same-frame-diagnostic-r1.jsonl`
- Normalized summary: `build/artifacts/ac4-same-frame-diagnostic-r1.json`
- Observer ring: `build/evidence/ac4-same-frame-observer-r1.bin`
- Observer source/library: `build/artifacts/ac4-dqbuf-write-observer-r1.c`, `build/artifacts/ac4-dqbuf-write-observer-r1.so`

## Next step

Measure the CPU time and wall time inside FFmpeg’s `sws_scale_frame` call for the corresponding frames during one more bounded run. Similar CPU and wall durations would support scaling work; a much larger wall duration would support waiting or scheduling. Keep the same 600-frame cap, first post-warm-up >60 ms stop, camera, binary, and argv. Do not change product source or persist payload.
