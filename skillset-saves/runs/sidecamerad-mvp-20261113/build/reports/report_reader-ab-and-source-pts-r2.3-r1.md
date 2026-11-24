# Raw-pipe reader A/B and source-PTS semantics

## Scope

Read-only diagnostics for SideCamera under Admiral run revision 80. The build evidence lineage remains separate from the Admiral run revision; the prior build package evidence used phase input revision 68. No product source changed during these diagnostics. `system/sidecamerad.py` SHA-256 before and after the reader A/B was `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b`.

## Source timestamp evidence

The 60-frame FFmpeg alignment capture contains 60 `showinfo` records and 60 `framecrc` rows. Frame indices are contiguous and the per-frame checksums match in order. `showinfo` reports a 1/1,000,000 timebase; `framecrc` PTS is rescaled to 1/20 and is not used as the precision timestamp source. This demonstrates decoded-frame order alignment for that diagnostic.

The V4L2 capture log labels the timestamp source `ts-src-soe` (start of exposure). This is a source-exposure timestamp, not the approved `timestampEof` value, which remains host-monotonic time immediately after the complete 1,382,400-byte raw NV12 frame read. The PTS value must not be relabeled or published as EOF.

A separate same-run PTS-to-Python correlation record was also retained for context, but its Python zlib CRC values do not match FFmpeg `showinfo` checksums. That CRC comparison is inconclusive for per-frame byte mapping and is not used as evidence of verified raw-byte-to-PTS mapping. The matching showinfo/framecrc capture is the valid alignment evidence; SOE semantics still prevent it from serving as EOF.

## Reader A/B

Two sequential 1,200-frame captures used the same FFmpeg raw-pipe command and `/dev/video60`. Each measured `time.monotonic_ns()` immediately after reading all 1,382,400 bytes of a frame. The A arm reproduced the current per-chunk `poll` plus `readv` loop. The B arm removed `poll` and used blocking `readv` until the complete frame was read.

| Reader | Frames | Mean interval | Max interval | Gaps >60 ms | Poll calls | readv calls |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Current poll + readv | 1,200 | 49.9943 ms | 80.672 ms | 155 / 1,199 (12.93%) | 31,387 | 31,386 |
| Blocking readv | 1,200 | 50.0125 ms | 81.591 ms | 161 / 1,199 (13.43%) | 0 | 32,965 |

Removing per-chunk polling did not reduce >60 ms intervals; it produced 6 more in this pair (0.50 percentage points). The read-path substitution is not a supported fix for the AC-4 cadence failure.

Both capture processes were terminated and reaped. A post-run process check found no remaining FFmpeg or sidecamerad process. The camera was temporarily opened by the diagnostic as intended; no camera configuration was changed. No product source hash drift occurred.

## Disposition and next action

Do not change the reader loop or source timestamps based on these results. Keep `timestampEof` tied to full-frame read completion, retain the AC-4 cadence failure, and do not use V4L2 SOE PTS as EOF. Next run a bounded read-only `strace -ttt -T` capture of FFmpeg stdout writes only, while Python records full-frame completion times. Correlate cumulative stdout bytes at each 1,382,400-byte boundary with completion timestamps, note tracing overhead, and determine whether the observed gaps come from producer write stalls or host-side reader scheduling. No source edit is authorized until that mechanism is evidenced.

AC-6 reconnect/sibling continuity and AC-8 four-camera soak remain hardware deferrals. The open Minor FFmpeg resource-cap item remains open.

## Evidence hashes

| Artifact | SHA-256 |
| --- | --- |
| `build/artifacts/reader-poll-blocking-ab-r2.3-r1.json` | `107045fbfed7c078205711a7631cd5a1c573e1d2a85305a80cf09a43afa0f6b0` |
| `build/evidence/reader-poll-blocking-ab-r2.3-r1.log` | `f466d8a43b50a58b3cbde361f38c57373dcf71be70af053cb5fe95693deb6a8a` |
| `build/evidence/ffmpeg-source-pts-alignment-r1.log` | `d5239c4656501b5cefd4d91f1afdf73855baa09e6470461ab3971bb2ebbd4e57` |
| `investigation/evidence/v4l2-capture-timestamps-60s.log` | `bf3ea3aee6ea62d9b3b7010db2ed89e6c4755a536eb2202c0790ae8f72b21999` |
| `build/evidence/ffmpeg-pts-monotonic-correlation-r2.3-r1.log` (inconclusive CRC comparison) | `a57914213c48116947286439781b38710e6fde95219059b86f4d470f834d8029` |
| `build/artifacts/frame-source-pts-monotonic-r2.3-r1.json` (inconclusive CRC comparison) | `2139a473d8f93d7efb87752cff48250f50e62bba8f09f2a0c21d18acc81399df` |
