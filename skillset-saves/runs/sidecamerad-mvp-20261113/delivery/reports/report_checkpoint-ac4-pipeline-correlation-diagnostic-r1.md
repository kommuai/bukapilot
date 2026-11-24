# Admiral checkpoint — AC-4 pipeline correlation diagnostic r1

**Active boundary:** BUILD_ACTIVE. Investigation-review r8 remains APPROVED. User authorization covers continued metadata-only AC-4 diagnostics. Product source and production FFmpeg argv are unchanged.

## Finding

The 600-output capture collected 580 post-warmup intervals. Seventy-seven (13.3%) reader gaps exceeded 60 ms, across all post-warmup frame regions; the maximum was 77.761 ms. Matched camera SOE intervals had a 52.209 ms maximum and zero intervals over 60 ms. A standalone 60-second V4L2-only capture also had zero steady intervals over 60 ms (1,198 intervals; maximum 52.287 ms). This rules against the camera’s frame-generation cadence or startup as the primary cause.

The gap excess beyond DQBUF cadence tracks changes in per-frame decode/filter CPU strongly (Pearson correlation 0.937). For >60 ms output intervals, target-frame decode/filter CPU averaged 37.284 ms versus 32.556 ms on other intervals; target-minus-previous CPU averaged +10.608 ms versus −1.617 ms otherwise. This points to variable software processing cost in FFmpeg’s MJPEG decode and scale/format path.

Caveat: in this heavier observer run, 40/580 DQBUF intervals exceeded 60 ms even though SOE intervals remained regular. This may reflect input-thread scheduling or driver/USB delivery after exposure; these captures do not separate those causes. An earlier less-instrumented same-run correlation had zero steady DQBUF intervals over 60 ms while 146/598 output intervals exceeded 60 ms. Observer overhead is uncalibrated, and the current 600-frame run did not recapture the earlier 90.982 ms maximum. AC-4 remains failed and open.

## Evidence

- Detailed report: `skillset-saves/runs/sidecamerad-mvp-20261113/build/reports/report_ac4-pipeline-correlation-diagnostic-r1.md` — SHA-256 `0ffe730fac4176493ab69c1e832f7c711dfbb6bbdef7108d9ff4fe2fd5c49f90`
- Aggregate analysis: `build/artifacts/ac4-pipeline-correlation-analysis-r1.json` — SHA-256 `95eeeb7d6bdc3c05462b331cb5b679a70eda4c49f47a7895f1537f0e88db57e5`
- Raw capture summary: `build/artifacts/ac4-pipeline-correlation-diagnostic-r1.json` — SHA-256 `31153c9104bf1347c4e7ef641a70c8408fd881dc9107479b4ecadbc2114415da`
- Per-frame event log: `build/evidence/ac4-pipeline-correlation-diagnostic-r1.jsonl` — SHA-256 `9a1db04b5bf4d2608b984c88babb3a33d7a4fc242fe678935589c9ee16001a25`
- Observer rings: API `936608726a82f807f317fd024faaf8527034096b20c8fdafbfe285e149ea95cf`; FFmpeg `ba46abb2104efc9572696edda37e762511c9aed4d21b4b0cab37301325b3197a`; scale `2bd2deb39abb4875231e268fbaaf4a5d58c35adf84f3efd85d24a7eec0793c2b`
- Runner: `build/artifacts/ac4-pipeline-correlation-runner-r1.py` — SHA-256 `a598c2c1e7bf03429594965874d7d770346bfd8808f07286224d3839d63a8d8a`
- Hardware cross-checks: V4L2-only capture `investigation/evidence/v4l2-capture-timestamps-60s.log`; prior same-run FFmpeg/DQBUF correlation `build/artifacts/dqbuf-output-correlation-r2.3-r1.json`.

## Next action

Return the software-primary diagnosis to the user. AC-4 remains failed. No code fix or frame-drop policy was implemented; any mitigation should be scoped separately against freshness, cadence, and dropped-frame acceptance criteria.
