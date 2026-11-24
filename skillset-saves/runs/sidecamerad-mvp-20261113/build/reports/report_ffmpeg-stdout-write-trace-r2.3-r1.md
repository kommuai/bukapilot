# FFmpeg stdout write timing trace

## Result

A read-only `strace -ttt -T` capture traced raw write syscall arguments only, with payload capture disabled. It followed FFmpeg threads and mapped 600 cumulative output byte boundaries to Python `CLOCK_MONOTONIC` timestamps taken after each full 1,382,400-byte NV12 frame read. The capture completed 600/600 frames, read 829,440,000 bytes, returned status 0, and correlated all 600 frame boundaries. FFmpeg's selected output descriptor was fd 12; 25,800 writes returned exactly 829,440,000 bytes.

The `-ttt` realtime timestamps were aligned with Python monotonic time using before/after clock brackets. Estimated realtime-minus-monotonic offset was 1794726953573887050 ns, with estimated uncertainty ±3551 ns. Bracket widths were 3500 ns before and 4375 ns after.

## Cadence and correlation

| Series | Mean interval | p95 | p99 | Maximum | Gaps >60 ms |
| --- | ---: | ---: | ---: | ---: | ---: |
| FFmpeg write-boundary intervals | 49.986795 ms | 67.000 ms | 73.159 ms | 81.639 ms | 121 / 599 |
| Python full-frame read completion | 49.986931 ms | 67.026 ms | 73.128 ms | 81.607 ms | 122 / 599 |

The median time from the crossing write syscall start to full-frame read completion was 0.128 ms; p95 was 0.249 ms. `strace -T` syscall-return time is only an interval endpoint: bytes from a large write may be available to Python before the syscall returns, so return-time lag is not treated as causal evidence.

The traced mean interval differed from the saved untraced 1,200-frame A/B mean (~50.003379 ms pooled) by about -0.016448 ms (-0.033%). The traced >60 ms share was 20.37%, versus 13.18% pooled in the untraced A/B. These runs were sequential and unpaired; this is only an approximate observer-effect comparison. Mean and maximum intervals were similar, while the short traced sample had a higher >60 ms share.

The close match between output-boundary and read-completion cadence, plus the small write-start-to-read completion delay, does not show a separate sustained Python reader backlog. It bounds the jitter to the producer-to-pipe/read interface. The writes-only trace does not distinguish FFmpeg decode/output pacing from host scheduling or pipe backpressure; the exact mechanism remains unproven. Keep `timestampEof` at host-monotonic full-frame read completion. V4L2 timestamps are marked `ts-src-soe` and remain unsuitable as EOF.

## Integrity and open items

Source SHA-256 before and after was `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b`; no drift. The trace subprocesses were reaped. No source edits were made. AC-6 reconnect/sibling continuity and AC-8 four-camera soak remain hardware deferrals. The Minor FFmpeg resource-cap finding remains open. The AC-4 cadence failure remains unresolved; this trace does not authorize a source change or a passing claim.

## Evidence

- JSON artifact: `build/artifacts/ffmpeg-stdout-write-trace-r2.3-r1.json` — SHA-256 `5dede62fa4529206a576e6241723f9c4c6b5aa4cd55eebbe45d23da3833fe49f`
- Raw writes-only trace: `build/evidence/ffmpeg-stdout-write-trace-r2.3-r1.strace` — SHA-256 `9878b91cb3fde20172cd524c399a7220f245ee2c6c3561f2e567bf30143a9cc4`
- Source probe: `.harness-state/sidecam-write-trace-follow-threads-20261115T094800Z/trace.json` and `strace-writes.log` (copied byte-for-byte into the build paths above)
