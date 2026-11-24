# FFmpeg output-flush A/B diagnostic

Run revision 93; device `/dev/video60`; source SHA-256 `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b` before capture. The same blocking `os.readv` reader timestamps each full-frame completion with `time.monotonic_ns()`. No frame payload was saved; strace records raw write/writev arguments only.

The only FFmpeg argv difference is output-side `-flush_packets 1`. Shared diagnostic frame bound is 600 frames.

## default

Command: `/usr/bin/ffmpeg -nostdin -hide_banner -loglevel error -thread_queue_size 1 -threads 1 -filter_threads 1 -f v4l2 -input_format mjpeg -framerate 20 -video_size 1280x720 -i /dev/video60 -an -sn -dn -vf scale=1280:720:flags=neighbor,format=nv12 -pix_fmt nv12 -threads 1 -fps_mode passthrough -frames:v 600 -f rawvideo pipe:1`

Capture 600/600; bytes 829440000/829440000; return 0; error None.
Output fd 12: 25800 writes, 829440000 bytes; unparsed write lines 0.
Read intervals: {'count': 599, 'mean_ms': 50.01252162771286, 'median_ms': 51.119084, 'p95_ms': 69.085185, 'p99_ms': 78.061077, 'max_ms': 88.425002, 'over_60ms': 122, 'under_40ms': 124}.
Write-boundary start gaps: {'count': 599, 'mean_ms': 50.0123121869783, 'median_ms': 51.173, 'p95_ms': 69.023, 'p99_ms': 77.996, 'max_ms': 88.367, 'over_60ms': 120, 'under_40ms': 125}; end gaps: {'count': 599, 'mean_ms': 50.01238397328881, 'median_ms': 51.162, 'p95_ms': 69.065, 'p99_ms': 78.054, 'max_ms': 88.406, 'over_60ms': 122, 'under_40ms': 125}.
Output syscall durations: {'count': 25800, 'mean_ms': 0.07607011627906977, 'median_ms': 0.059, 'p95_ms': 0.144, 'p99_ms': 0.229, 'max_ms': 9.632, 'over_60ms': 0, 'under_40ms': 25800}.
Clock offset uncertainty: 1895 ns.

## flush_packets_1

Command: `/usr/bin/ffmpeg -nostdin -hide_banner -loglevel error -thread_queue_size 1 -threads 1 -filter_threads 1 -f v4l2 -input_format mjpeg -framerate 20 -video_size 1280x720 -i /dev/video60 -an -sn -dn -vf scale=1280:720:flags=neighbor,format=nv12 -pix_fmt nv12 -threads 1 -fps_mode passthrough -frames:v 600 -flush_packets 1 -f rawvideo pipe:1`

Capture 600/600; bytes 829440000/829440000; return 0; error None.
Output fd 12: 25800 writes, 829440000 bytes; unparsed write lines 0.
Read intervals: {'count': 599, 'mean_ms': 49.975462589315526, 'median_ms': 50.337447, 'p95_ms': 67.755504, 'p99_ms': 77.073519, 'max_ms': 86.67944, 'over_60ms': 107, 'under_40ms': 110}.
Write-boundary start gaps: {'count': 599, 'mean_ms': 49.97541402337229, 'median_ms': 50.372, 'p95_ms': 67.732, 'p99_ms': 77.104, 'max_ms': 86.696, 'over_60ms': 106, 'under_40ms': 107}; end gaps: {'count': 599, 'mean_ms': 49.9753856427379, 'median_ms': 50.326, 'p95_ms': 67.736, 'p99_ms': 77.094, 'max_ms': 86.65, 'over_60ms': 107, 'under_40ms': 109}.
Output syscall durations: {'count': 25800, 'mean_ms': 0.07859961240310077, 'median_ms': 0.059, 'p95_ms': 0.15, 'p99_ms': 0.242, 'max_ms': 11.402, 'over_60ms': 0, 'under_40ms': 25800}.
Clock offset uncertainty: 1458 ns.

## Comparison

Read >60 ms count/max: 122/88.425002 ms → 107/86.67944 ms.
Write-boundary start >60 ms count/max: 120/88.367 ms → 106/86.696 ms.
This is diagnostic only; the sequential traced captures do not prove the internal FFmpeg wait mechanism or pass AC-4. No production argv/source change was made.

## Evidence

- JSON `build/artifacts/ffmpeg-flush-pacing-ab-r2.3-r1.json` SHA-256 `913a57ae11491fe31ee103884b98a2c1df357b9d8dbc8f8c89c519f0d1786f3c`
- Default trace `build/evidence/ffmpeg-flush-default-r2.3-r1.strace` SHA-256 `8de6089eff1a1557e552d325e8dd051774485451d69ad953c40c3ee6be1506fb`
- Flush trace `build/evidence/ffmpeg-flush-packets-1-r2.3-r1.strace` SHA-256 `cfde2108693799f2f87222ff8058a6fededba3c955c4ee09288e973f3ca4bc75`
- Log `build/evidence/ffmpeg-flush-pacing-ab-r2.3-r1.log` SHA-256 `510294d1de93851644766d6fd7394f836abd8bdba5b3059d38507b1f5e72717f`


## Finding and scope

With `-flush_packets 1`, this single sequential sample had 15 fewer Python read gaps above 60 ms (122 to 107 of 599) and a 1.746 ms lower maximum (88.425 to 86.679 ms). Write-boundary starts showed 14 fewer such gaps (120 to 106) and a 1.671 ms lower maximum (88.367 to 86.696 ms). Both arms still have substantial >60 ms cadence gaps. Output write call counts remained 25,800; mean write syscall duration changed from 0.076070 ms to 0.078600 ms, with maxima 9.632 ms and 11.402 ms. This one unpaired A/B suggests a small diagnostic shift, not a supported remediation or a proven wait mechanism. Do not change production argv from this result.

Hardware scope: this host exposes one USB Video camera at `/dev/video60` (with sibling node `/dev/video61`). Both arms used the same camera. This round does not add multi-camera evidence; AC-6 reconnect/sibling continuity and AC-8 four-camera soak remain deferred.

Next finding: route the measured but insufficient `-flush_packets 1` effect to investigation for a bounded repeatability/mechanism assessment. Preserve AC-4 as unresolved; only the required five-minute acceptance can decide AC-4 after a separately approved source change.


## Trace caveat

`strace` can perturb thread scheduling. These were sequential, unpaired traced samples, so the small difference can include observer overhead and run-to-run variation. The JSON retains each output write duration in trace order. A frame byte boundary falls within the successful write syscall that crosses its cumulative byte threshold; syscall entry and exit bracket when those bytes become available, and neither endpoint alone gives the exact byte-availability instant. Treat the boundary comparison as diagnostic timing, not causal proof.
