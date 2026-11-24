# FFmpeg output-flush ABBA diagnostic

Admiral run revision at start: 104; device `/dev/video60`; source SHA-256 `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b` before capture. Each arm reads 600 full 1,382,400-byte frames with the same blocking `os.readv` reader and timestamps completion with `time.monotonic_ns()`. No frame payload was saved. Tracing is identical across arms (`strace -f -ttt -T -qq -e trace=write,writev -e raw=write,writev`); raw write arguments/timings only.

The arm order is A1 default, B1 `-flush_packets 1`, B2 `-flush_packets 1`, A2 default. Each A/B pair uses the same FFmpeg command, with B adding only output-side `-flush_packets 1`; the 600-frame bound is common to all.

## A1-default

Exact FFmpeg argv: `/usr/bin/ffmpeg -nostdin -hide_banner -loglevel error -thread_queue_size 1 -threads 1 -filter_threads 1 -f v4l2 -input_format mjpeg -framerate 20 -video_size 1280x720 -i /dev/video60 -an -sn -dn -vf scale=1280:720:flags=neighbor,format=nv12 -pix_fmt nv12 -threads 1 -fps_mode passthrough -frames:v 600 -f rawvideo pipe:1`

Capture 600/600 frames; 829440000/829440000 bytes; return 0; error None.
Output fd 12: 25800 writes, 829440000 bytes; unmatched resumed calls 0; unparsed relevant lines 0.
Read interval distribution: {'count': 599, 'mean_ms': 49.95538937395659, 'median_ms': 50.192394, 'p95_ms': 68.705993, 'p99_ms': 79.196897, 'max_ms': 90.373815, 'over_60ms': 123, 'under_40ms': 124}.
Write-boundary start gaps: {'count': 599, 'mean_ms': 49.955322203672786, 'median_ms': 50.263, 'p95_ms': 68.729, 'p99_ms': 79.154, 'max_ms': 90.28, 'over_60ms': 122, 'under_40ms': 123}; end gaps: {'count': 599, 'mean_ms': 49.95536393989983, 'median_ms': 50.232, 'p95_ms': 68.724, 'p99_ms': 79.162, 'max_ms': 90.334, 'over_60ms': 123, 'under_40ms': 123}.
Per-write syscall duration distribution (full per-write array is in JSON): {'count': 25800, 'mean_ms': 0.08019406976744187, 'median_ms': 0.058, 'p95_ms': 0.151, 'p99_ms': 0.247, 'max_ms': 12.11, 'over_60ms': 0, 'under_40ms': 25800}.
Realtime-to-monotonic bracket uncertainty: 2625 ns.

## B1-flush_packets_1

Exact FFmpeg argv: `/usr/bin/ffmpeg -nostdin -hide_banner -loglevel error -thread_queue_size 1 -threads 1 -filter_threads 1 -f v4l2 -input_format mjpeg -framerate 20 -video_size 1280x720 -i /dev/video60 -an -sn -dn -vf scale=1280:720:flags=neighbor,format=nv12 -pix_fmt nv12 -threads 1 -fps_mode passthrough -frames:v 600 -flush_packets 1 -f rawvideo pipe:1`

Capture 600/600 frames; 829440000/829440000 bytes; return 0; error None.
Output fd 12: 25800 writes, 829440000 bytes; unmatched resumed calls 0; unparsed relevant lines 0.
Read interval distribution: {'count': 599, 'mean_ms': 49.96540717195325, 'median_ms': 50.643945, 'p95_ms': 69.306076, 'p99_ms': 75.749046, 'max_ms': 85.298357, 'over_60ms': 107, 'under_40ms': 118}.
Write-boundary start gaps: {'count': 599, 'mean_ms': 49.96527212020033, 'median_ms': 50.671, 'p95_ms': 69.232, 'p99_ms': 75.878, 'max_ms': 85.316, 'over_60ms': 108, 'under_40ms': 118}; end gaps: {'count': 599, 'mean_ms': 49.965333889816364, 'median_ms': 50.678, 'p95_ms': 69.261, 'p99_ms': 75.784, 'max_ms': 85.341, 'over_60ms': 108, 'under_40ms': 118}.
Per-write syscall duration distribution (full per-write array is in JSON): {'count': 25800, 'mean_ms': 0.07629135658914729, 'median_ms': 0.057, 'p95_ms': 0.144, 'p99_ms': 0.229, 'max_ms': 12.04, 'over_60ms': 0, 'under_40ms': 25800}.
Realtime-to-monotonic bracket uncertainty: 2042 ns.

## B2-flush_packets_1

Exact FFmpeg argv: `/usr/bin/ffmpeg -nostdin -hide_banner -loglevel error -thread_queue_size 1 -threads 1 -filter_threads 1 -f v4l2 -input_format mjpeg -framerate 20 -video_size 1280x720 -i /dev/video60 -an -sn -dn -vf scale=1280:720:flags=neighbor,format=nv12 -pix_fmt nv12 -threads 1 -fps_mode passthrough -frames:v 600 -flush_packets 1 -f rawvideo pipe:1`

Capture 600/600 frames; 829440000/829440000 bytes; return 0; error None.
Output fd 12: 25800 writes, 829440000 bytes; unmatched resumed calls 0; unparsed relevant lines 0.
Read interval distribution: {'count': 599, 'mean_ms': 49.95574270951586, 'median_ms': 49.924421, 'p95_ms': 69.138546, 'p99_ms': 75.912239, 'max_ms': 96.676936, 'over_60ms': 120, 'under_40ms': 110}.
Write-boundary start gaps: {'count': 599, 'mean_ms': 49.955921535893154, 'median_ms': 49.959, 'p95_ms': 69.158, 'p99_ms': 75.889, 'max_ms': 96.578, 'over_60ms': 120, 'under_40ms': 110}; end gaps: {'count': 599, 'mean_ms': 49.955858096828045, 'median_ms': 49.786, 'p95_ms': 69.163, 'p99_ms': 75.906, 'max_ms': 96.644, 'over_60ms': 121, 'under_40ms': 110}.
Per-write syscall duration distribution (full per-write array is in JSON): {'count': 25800, 'mean_ms': 0.07687531007751938, 'median_ms': 0.058, 'p95_ms': 0.14, 'p99_ms': 0.228, 'max_ms': 11.794, 'over_60ms': 0, 'under_40ms': 25800}.
Realtime-to-monotonic bracket uncertainty: 1530 ns.

## A2-default

Exact FFmpeg argv: `/usr/bin/ffmpeg -nostdin -hide_banner -loglevel error -thread_queue_size 1 -threads 1 -filter_threads 1 -f v4l2 -input_format mjpeg -framerate 20 -video_size 1280x720 -i /dev/video60 -an -sn -dn -vf scale=1280:720:flags=neighbor,format=nv12 -pix_fmt nv12 -threads 1 -fps_mode passthrough -frames:v 600 -f rawvideo pipe:1`

Capture 600/600 frames; 829440000/829440000 bytes; return 0; error None.
Output fd 12: 25800 writes, 829440000 bytes; unmatched resumed calls 0; unparsed relevant lines 0.
Read interval distribution: {'count': 599, 'mean_ms': 49.99203123372287, 'median_ms': 50.330187, 'p95_ms': 67.140702, 'p99_ms': 74.359346, 'max_ms': 81.195158, 'over_60ms': 105, 'under_40ms': 100}.
Write-boundary start gaps: {'count': 599, 'mean_ms': 49.99213689482471, 'median_ms': 50.263, 'p95_ms': 67.136, 'p99_ms': 74.346, 'max_ms': 81.22, 'over_60ms': 106, 'under_40ms': 99}; end gaps: {'count': 599, 'mean_ms': 49.992095158597664, 'median_ms': 50.285, 'p95_ms': 67.134, 'p99_ms': 74.354, 'max_ms': 81.221, 'over_60ms': 105, 'under_40ms': 98}.
Per-write syscall duration distribution (full per-write array is in JSON): {'count': 25800, 'mean_ms': 0.07778178294573643, 'median_ms': 0.058, 'p95_ms': 0.149, 'p99_ms': 0.238, 'max_ms': 10.792, 'over_60ms': 0, 'under_40ms': 25800}.
Realtime-to-monotonic bracket uncertainty: 1312 ns.

## Pooled A/B comparison

Read-gap A: {'count': 1198, 'mean_ms': 49.97371030383973, 'median_ms': 50.253453, 'p95_ms': 68.135584, 'p99_ms': 75.681088, 'max_ms': 90.373815, 'over_60ms': 228, 'under_40ms': 224}; B: {'count': 1198, 'mean_ms': 49.96057494073456, 'median_ms': 50.341698, 'p95_ms': 69.240343, 'p99_ms': 75.912239, 'max_ms': 96.676936, 'over_60ms': 227, 'under_40ms': 228}.
Write-boundary A: {'count': 1198, 'mean_ms': 49.973729549248745, 'median_ms': 50.263, 'p95_ms': 68.086, 'p99_ms': 75.677, 'max_ms': 90.28, 'over_60ms': 228, 'under_40ms': 222}; B: {'count': 1198, 'mean_ms': 49.96059682804675, 'median_ms': 50.3535, 'p95_ms': 69.181, 'p99_ms': 75.889, 'max_ms': 96.578, 'over_60ms': 228, 'under_40ms': 228}.

This is diagnostic only; AC-4 remains open. Tracing can affect scheduling. ABBA order bounds simple drift, but the runs are still sequential and unpaired. A frame boundary may fall within a successful write syscall; syscall entry/exit bracket byte availability and are not exact byte-arrival timestamps. No production argv or source change is authorized by this diagnostic.

## Hardware scope

This host exposes one USB Video camera at `/dev/video60` (with sibling node `/dev/video61`). This ABBA uses that same camera only; AC-6 reconnect/sibling continuity and AC-8 four-camera soak remain deferred.

## Evidence

- JSON artifact: `build/artifacts/ffmpeg-flush-abba-r3.json` — SHA-256 `db777e8aa2d4b63155e5595cda47cfa863367805885c270f518df3dde7389671`
- Raw trace abba-a1: `build/evidence/ffmpeg-flush-abba-a1-r3.strace` — SHA-256 `e9609a2446d0fe009059919d8f6a363e3f73d3263b35d65b9993b9b9225be0e3`
- Raw trace abba-b1: `build/evidence/ffmpeg-flush-abba-b1-r3.strace` — SHA-256 `4dac35c98aed5615a9ad45e32748ffd79835ce1231b3191dc3bfc215353c132a`
- Raw trace abba-b2: `build/evidence/ffmpeg-flush-abba-b2-r3.strace` — SHA-256 `8e686f905fbf4021ab9b717d469d17d48794922a3c5c74e27e8f0649a0ae0d23`
- Raw trace abba-a2: `build/evidence/ffmpeg-flush-abba-a2-r3.strace` — SHA-256 `6d6e937c8fc8c19a889d9311be01416cf0531d3625cf0abe1eabeba1fe784044`
- Summary log: `build/evidence/ffmpeg-flush-abba-r3.log` — SHA-256 `7816977983473254c5fc01747f84a1d33d07b35869b3021a425128363ace0064`


## Finding

The ABBA sequence does not reproduce a repeatable cadence improvement from `-flush_packets 1`. Pooled read gaps above 60 ms are effectively unchanged (A 228/1,198; B 227/1,198), while B's maximum is higher (96.677 ms versus 90.374 ms). Write-boundary gaps above 60 ms are identical (228/1,198 for both); B's maximum is also higher (96.578 ms versus 90.280 ms). The first B arm looked better than the first A arm, but the second B arm was worse than the following A arm. Treat the earlier single-pair reduction as run-to-run variation; this option is not a supported cadence fix. Route the result to investigation for an alternative bounded mechanism path.
