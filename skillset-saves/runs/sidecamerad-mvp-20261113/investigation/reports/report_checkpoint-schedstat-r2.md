# Checkpoint — schedstat probe completed

- Run: `sidecamerad-mvp-20261113`; investigation revision 2; resumed from run revision 86.
- Active stage: scheduler probe complete; evidence-chain revision 2 next.
- Read-only bounded capture: 600 of 600 FFmpeg output frames, 599 read intervals, 77 over 60 ms, maximum 80.383947 ms. The exact five-minute reproduction was reused and not rerun.
- Per-thread `/proc/<pid>/task/*/schedstat` was available for four FFmpeg threads. Long-gap intervals had 1.469 ms mean summed runqueue delay versus 1.273 ms on other intervals; only 3/77 long gaps had a thread delay of at least 5 ms. CPU runqueue scheduling is not supported as the primary cause.
- Corrected output trace reports 25,800 writes: median duration 0.058 ms, p95 0.153 ms, p99 0.256 ms, maximum 10.554 ms, no write over 50 ms. Sustained pipe backpressure is not supported.
- Limitations: post-read `/proc` snapshots add observer cost to the next interval; writer-thread attribution is not proven; DQBUF had 602 successful calls for 600 frames and raw ioctl omitted buffer sequence numbers. FFmpeg PTS are V4L2 `ts-src-soe`, not EOF.
- Product source remained unchanged at SHA-256 `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b`. No payloads or source edits.
- Registered evidence: `investigation/artifacts/ffmpeg-schedstat-600-r2.json`, `investigation/evidence/ffmpeg-schedstat-600-r2.log`.
- Next action: write the revised observed evidence chain and bounded fix path, then update manifest and self-check.
