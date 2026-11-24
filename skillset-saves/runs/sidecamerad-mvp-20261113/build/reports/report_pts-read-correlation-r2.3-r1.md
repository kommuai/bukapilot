# FFmpeg PTS and full-frame read correlation

## Capture

This read-only, same-run capture recorded 600 decoded FFmpeg `showinfo` PTS values and 600 Python `CLOCK_MONOTONIC` timestamps taken immediately after each complete 1,382,400-byte NV12 frame read from `/dev/video60`. The per-frame output-order pairing is contiguous (`n=0..599`); no raw frame payload was stored. The FFmpeg process returned 0, all 600 reads completed, and children were reaped.

`showinfo` uses timebase 1/1,000,000. The PTS series is in the monotonic domain, as supported by the prior same-device clock-correlation diagnostic. V4L2 marks this timestamp source `ts-src-soe` (start of exposure). These PTS identify source exposure time; they are not `timestampEof` under the approved interface and must not be substituted for the host-monotonic full-read completion time.

## Cadence

The first PTS interval includes a 160.006 ms startup gap. Excluding that startup interval, 598 steady PTS intervals averaged 49.999948 ms, with a 52.106 ms maximum and no intervals above 60 ms. Across the same 598 steady intervals, Python full-read completion averaged 49.977012 ms; 89 exceeded 60 ms, with an 89.946 ms maximum.

Across the 600 frame pairs, completion-minus-PTS lag had a 94.883 ms median and 106.003 ms p95. This shows that the irregularity is present after source PTS timestamping and before or at full raw-frame read completion. The capture does not isolate FFmpeg output pacing from host scheduling or pipe behavior.

## Integrity and disposition

Product source remained unchanged: `system/sidecamerad.py` SHA-256 `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b` before and after. This is diagnostic evidence, not a passing AC-4 acceptance run. Keep AC-4 unresolved and preserve `timestampEof` as the full-frame read completion timestamp. AC-6 reconnect/sibling continuity and AC-8 four-camera soak remain hardware deferrals; the Minor FFmpeg resource-cap finding remains open.

Next, if authorized, run one bounded read-only V4L2 DQBUF/FFmpeg output correlation that identifies capture-buffer dequeues without exposing payload. Compare dequeue cadence with FFmpeg output/full-read cadence to locate whether jitter enters before dequeue or afterward. Do not edit source or substitute SOE PTS as EOF.

## Evidence hashes

- JSON: `build/artifacts/pts-read-correlation-r2.3-r1.json` — SHA-256 `067a4b4d3dee1d3d836490dcaa489d393ae916bda52ebb101da5fcde0c822bba`
- FFmpeg showinfo log: `build/evidence/ffmpeg-showinfo-pts-read-correlation-r2.3-r1.log` — SHA-256 `848d42a5562f3145222130badeeda8e477a187b0959f614d135b3dcf4a90e183`
