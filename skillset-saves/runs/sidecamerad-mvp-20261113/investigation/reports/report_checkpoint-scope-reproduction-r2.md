# Investigation revision 2 checkpoint — scope and reproduction

- Run: `sidecamerad-mvp-20261113`
- Stage: scope-and-reproduction verified
- Prior investigation revision: 1 (APPROVED)
- Current investigation revision: 2
- Build source: unchanged; `system/sidecamerad.py` SHA-256 `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b`

The accepted five-minute reproduction is reused without rerunning it because the source hash is unchanged. `investigation/evidence/reproduction.log` SHA-256 `de804d35326c6925862d2c8b36708d4a14a10b05120076924f523397976e8c55` matches the build runtime capture and records 6,050 selected matched frames over 302.506666 seconds, mean 20.000523 fps, maximum EOF gap 89.163715 ms, and 753 of 6,049 intervals above 60 ms. The failure remains reproduced.

New build observations change the prior proposed fix path: FFmpeg PTS are monotonic-domain V4L2 start-of-exposure timestamps, not EOF. After startup, PTS and successful V4L2 dequeue cadence are regular, while raw-pipe write boundaries and Python full-read completions retain gaps above 60 ms. Write-to-reader delay is small. The remaining mechanism question is FFmpeg decode/output pacing versus FFmpeg-thread scheduling or pipe backpressure. The existing full-read-completion meaning of `timestampEof` is preserved; no source timestamp substitution is authorized by this evidence.

Next action: run the bounded untraced 600-frame per-thread `schedstat` sample, recording monotonic full-read completion times and each FFmpeg thread's runtime/runqueue-delay deltas. Store no frame payloads and make no product source edits. If thread attribution remains unavailable, state the residual uncertainty explicitly.
