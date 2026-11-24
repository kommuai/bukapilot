# V4L2 DQBUF and FFmpeg output timing correlation

## Capture integrity

This is the corrected compact parse of a bounded read-only syscall trace. It pairs strace unfinished/resumed syscall records: 0 unmatched resumes and 0 unparsed relevant lines. Raw syscall arguments only were recorded; no frame payload was captured. The preliminary `dqbuf-output-correlation.json` is excluded because its parser did not pair unfinished/resumed calls.

The run read 600/600 complete NV12 frames (829,440,000 bytes), recorded 600 showinfo PTS and 600 output byte boundaries, and returned 0. There were 602 successful V4L2 DQBUF calls. The trace records 25,800 output writes on fd 12 totaling 829,440,000 bytes. Child processes were reaped. Product source hash remained `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b` before and after.

## Cadence after startup

| Stream | Intervals | Mean | Maximum | Gaps >60 ms |
| --- | ---: | ---: | ---: | ---: |
| V4L2 DQBUF | 600 | 49.994824 ms | 58.634 ms | 0 |
| FFmpeg showinfo PTS | 598 | 50.006737 ms | 52.128 ms | 0 |
| FFmpeg raw output write boundaries | 598 | 49.957065 ms | 89.580 ms | 146 |
| Python full-frame read completion | 598 | 49.957056 ms | 89.587 ms | 147 |

The median time from the write crossing a frame boundary to full-frame read completion was 0.116 ms; p95 was 0.196 ms. Write-boundary and read-completion cadence closely match, while DQBUF and SOE PTS cadence stay regular after startup. This narrows the observed jitter to after V4L2 buffer dequeue and before raw-pipe delivery.

DQBUF successes (602) do not map one-to-one to the 600 outputs: the raw ioctl trace omits the V4L2 buffer sequence field, so exact dequeue-to-output frame identity is not claimed. The timing comparison therefore localizes the interval where jitter appears without asserting per-frame identity. It also cannot distinguish FFmpeg decode/output pacing from FFmpeg-thread host scheduling or pipe backpressure.

## Timestamp semantics and disposition

The 1/1,000,000-timebase PTS values are in the monotonic clock domain per the prior same-device check, and V4L2 marks their source `ts-src-soe` (start of exposure). SOE is not EOF. Do not substitute PTS for `timestampEof`, which remains host-monotonic time after the complete raw-frame read. This capture does not support the approved source-EOF timestamp fix path and does not pass AC-4.

Route this corrected evidence to investigation for a revised, bounded mechanism analysis before any source edits. AC-6 reconnect/sibling continuity and AC-8 four-camera soak remain hardware deferrals; the Minor FFmpeg resource-cap finding remains open.

## Evidence hashes

- JSON: `build/artifacts/dqbuf-output-correlation-r2.3-r1.json` — SHA-256 `29c587a2bf121c8e35c16481400f621686505cf3ad3afe0f95cb7f0ffab130b2`
- Corrected raw trace: `build/evidence/v4l2-dqbuf-ffmpeg-output-r2.3-r1.strace` — SHA-256 `5eb1ed5f3f73c01d88e6745a2c4c7df170f9d0324319ac374c1f4a84b98fd1b5`
- FFmpeg showinfo: `build/evidence/ffmpeg-showinfo-dqbuf-correlation-r2.3-r1.log` — SHA-256 `1db9bc2ce055045e0b971fc895f95c4b759bf352c7ae7cfdb68141db271ca70e`
