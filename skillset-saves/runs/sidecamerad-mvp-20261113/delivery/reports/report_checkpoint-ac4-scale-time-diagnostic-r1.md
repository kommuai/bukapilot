# Admiral checkpoint — AC-4 scale timing diagnostic r1

**Active boundary:** BUILD_ACTIVE. Investigation-review r8 remains APPROVED. The user authorized continued metadata-only AC-4 diagnosis. This capture made no product source or production argv changes.

## Finding

The run reproduced a 61.032 ms reader gap between outputs 26→27 after the 20-frame warmup. The matching contiguous V4L2 source SOE interval was 48.021 ms. The inter-frame interval grew to 50.928 ms at DQBUF completion, 56.561 ms at decoder output, and 62.602 ms at filter-sink output; mux and reader completion were 61.048 and 61.032 ms. This strongly points to the FFmpeg/software path rather than camera capture cadence, and it is separate from the initial startup timestamp jump found in diagnostic r1.

The matching scale conversion (`sws_scale_frame`, PTS 1,987,992 µs) took 23.139388 ms wall and 23.135583 ms thread CPU. Across 28 calls, mean wall time was 21.522 ms, p95 25.578 ms, p99 26.077 ms. Scaling is a notable software CPU cost, but the measured call itself did not block for 61 ms. The extra interval is already visible before the next scale call begins; whether demux/decode handoff or thread scheduling causes it remains open.

## Evidence

- Detailed report: `skillset-saves/runs/sidecamerad-mvp-20261113/build/reports/report_ac4-scale-time-diagnostic-r1.md` — SHA-256 `decabe09417145562cbd96295117f6865d814c5fd48401e9775f78d197fb5299`
- Normalized result: `skillset-saves/runs/sidecamerad-mvp-20261113/build/artifacts/ac4-scale-time-diagnostic-r1.json` — SHA-256 `2b905d0384dce1382f3a86695de2be4f488f397669166631773cf87ae4bed6ea`
- Per-frame event log: `skillset-saves/runs/sidecamerad-mvp-20261113/build/evidence/ac4-scale-time-diagnostic-r1.jsonl` — SHA-256 `b579fb90d69db2f57a7f9b41cf1db40174349fb7bd8a3586fcc2030e631176a6`
- Scale observer ring: `build/evidence/ac4-scale-time-observer-r1.bin` — SHA-256 `957e76ed323b1d1b55918b35375ded495b9cbaf9ac9dc6fa5cb8d7a96f72958c`
- FFmpeg observer ring: `build/evidence/ac4-scale-time-ffmpeg-observer-r1.bin` — SHA-256 `fdc21055990876bfc78c9a4c790a7567b60e13da07d7cb74aa3404b73b0fc509`
- Observer source/library: `build/artifacts/ac4-scale-time-observer-r1.c` `b9b3409de9d5dfeb2df86324df629c7ece7269809f45d4e0e344c63357af0224`; `.so` `773d123aa5177901fb81524e06d0a4420dee500babfcbe90996c95001ca1da61`
- Runner: `build/artifacts/ac4-scale-time-runner-r1.py` — SHA-256 `87630e64a2ee6da4c910c2e94a6734fc7fa5590a91309d58e24d4d37732570c6`

Observer overhead is not calibrated; this is localization evidence, not an AC-4 acceptance run. FFmpeg exited after the bounded stop (status 255); no process or camera holder remained.

## Next action

Run one bounded metadata capture timing `av_read_frame` and `avcodec_send_packet` with wall and thread CPU clocks, alongside the existing frame/stage markers. This distinguishes input wait from decode handoff and later FFmpeg work. Keep the same camera, binary, argv, 20-frame warmup, 600-frame cap, and first post-warmup >60 ms stop rule. Product source remains unchanged.
