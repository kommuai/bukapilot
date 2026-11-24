# Admiral checkpoint — AC-4 pipeline timing diagnostic r1

**Active boundary:** BUILD_ACTIVE. Investigation-review r8 remains APPROVED. The user authorized continued metadata-only AC-4 diagnosis. This run used the same camera and production FFmpeg argv; no product source changed.

## Finding

The bounded capture reproduced a 60.520 ms output gap after 20 warmup frames. Contiguous V4L2 sequences 26→27 had a regular 48.025 ms source SOE interval and 50.878 ms DQBUF completion interval. The reader gap was 60.520 ms because the next dequeue completed 17.203 ms after the prior reader EOF and the triggering frame then took 43.317 ms to reach EOF.

For the same pair, decode plus filter CPU rose from 27.866 ms to 39.379 ms (+11.514 ms), while dequeue-to-output service latency rose by 9.642 ms. The matched `av_read_frame` call took 0.062 ms wall / 0.057 ms CPU; `avcodec_send_packet` took 15.997 / 15.993 ms; filter source/scale work took 23.730 / 23.386 ms. The measured event therefore points to variable software processing cost, not a camera cadence fault or startup. The 695.988 ms initial sequence 0→1 timestamp jump in the earlier capture remains a separate startup event.

This establishes the immediate mechanism for this reproduced 60.520 ms event, not every interval in the earlier 90.982 ms maximum tail. Observer overhead is not independently calibrated; host scheduling or JPEG content complexity may contribute. AC-4 remains open.

## Evidence

- Detailed report: `skillset-saves/runs/sidecamerad-mvp-20261113/build/reports/report_ac4-pipeline-time-diagnostic-r1.md` — SHA-256 `8e6e8736f0f10e2d32d6d595d462f134b59409529dd2eb9e13cd5dc81d1d7224`
- Normalized result: `build/artifacts/ac4-pipeline-time-diagnostic-r1.json` — SHA-256 `390a5b3649f876e9d31dfa52560fb34e3be8a4020a1bf2a4e5400e6e7e64fe3d`
- Per-frame log: `build/evidence/ac4-pipeline-time-diagnostic-r1.jsonl` — SHA-256 `93a01e9e3938f8b678698533a3db4e235c91041c877c4b3bf34db53e2c68e4a1`
- Observer rings: API `43951cb4fc442ec072413dc24ef2468cdcb2ef4f67ea246d0be0f77b8beff1df`; FFmpeg `f5de555b003a74bd8b76ec6b5bd238dd4cf5725836931228594858e2fad90f1c`; scale `fbd817ac69138ddfdcb87525baacd4334e06b525ed0e115adc8a045362f399bb`
- Observer source/library: C `a044d97f2f33d2e28aee5d54cf4714d2262eeb01fbee71f812a750a6cda818ce`; `.so` `f71219c574aa5ee5dad3b4e6f5057bf3685a01f9b9fe7a71cdf8dbb0fafc7bbc`
- Runner: `build/artifacts/ac4-pipeline-time-runner-r1.py` — SHA-256 `20fc0836f80b795d1453c3d08d1ad0fa5c2dbb7144a50459123172a0136a1ff1`

All rings had zero overflow. FFmpeg was reaped, and no camera holder remained after cleanup.

## Next action

Run one same-argv, metadata-only sample through the 600-frame cap, retaining all frames after the first >60 ms gap. Correlate each frame’s source/dequeue interval and decode/filter CPU with its reader interval, especially the more severe tail. Keep production code and argv unchanged; AC-4 remains open.
