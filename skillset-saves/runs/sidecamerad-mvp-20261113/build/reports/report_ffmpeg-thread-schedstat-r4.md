# AC-4 r4 syscall and scheduler timing diagnostic

## Result

This bounded read-only capture completed **600/600 frames** (829,440,000/829,440,000 bytes) from `/dev/video60`. FFmpeg exited with code `0` and was reaped; source hash was unchanged (`494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b`). No frame payload was saved.

The 599 full-read intervals averaged 49.944 ms (p95 71.231 ms, p99 81.756 ms, max 95.742 ms); **133 exceeded 60 ms**. AC-4 remains open; this instrumented short capture is diagnostic only.

## Correlation

The raw trace has 50,115 lines. The parser paired all 8,306 `<unfinished ...>` entries with 8,306 resumed records: unmatched resumes 0, unfinished at EOF 0, unparsed lines 0. It extracted 41,809 completed calls (`{'futex': 8544, 'ioctl': 7235, 'read': 230, 'write': 25800}`).

Of 133 read gaps over 60 ms, 133 overlap a selected syscall lasting at least 1 ms, and 133 overlap one lasting at least 10 ms. By syscall type, ≥1 ms overlaps were {'futex': 133, 'ioctl': 8, 'write': 41}; ≥10 ms overlaps were {'futex': 133, 'ioctl': 1, 'write': 2}. Per-gap syscall IDs/TIDs and runtime/runqueue deltas are recorded in the correlation artifact and summary log.

Futex waits dominate the long overlaps in this capture. That does not establish that a futex wait caused a camera gap: an idle worker can wait while another pipeline stage controls delivery. A gap with no selected blocking syscall would also not prove CPU work. The trace excludes unselected syscalls and does not measure all host scheduling activity.

`strace` ptrace instrumentation and a `/proc/<pid>/task/*/schedstat` snapshot after each frame can perturb cadence. Their overhead was not independently calibrated. Realtime-to-monotonic offset was bracketed before and after; offset values were 1794726953573887123 ns and 1794726953573888436 ns, with largest half-bracket uncertainty ≤6563 ns. `-T` durations were anchored at syscall entry; resumed syscalls were paired by TID and syscall name.

## Reproduction and evidence

FFmpeg argv and strace argv are in the capture and correlation JSON. The fixed camera mode was `/dev/video60`, V4L2 MJPEG 1280×720 at 20 fps, queue size 1, one decode/filter thread, existing scale+NV12 filter. `strace` selected `ioctl,futex,poll,ppoll,select,pselect6,epoll_wait,read,readv,write,writev`, with read/write calls in raw-argument mode so payload bytes were not logged.

- Raw trace: `skillset-saves/runs/sidecamerad-mvp-20261113/build/evidence/ffmpeg-thread-syscall-schedstat-r4-final2.strace` — SHA-256 `0de7b3e50ff49d676085ff34e36e6fad8b9fb36bcbd7200d9df251cb0b9e7037`.
- Capture JSON: `skillset-saves/runs/sidecamerad-mvp-20261113/build/artifacts/ffmpeg-thread-syscall-schedstat-r4-final2.json` — SHA-256 `b64db8453b48b731dafbec8b33db5d40a0c577ab698ff29d204c2127e176cba8`.
- Correlation JSON: `skillset-saves/runs/sidecamerad-mvp-20261113/build/artifacts/ffmpeg-thread-schedstat-correlation-r4.json` — SHA-256 `48d0a6517c05a9b40a377de87a358ee93ac12b70a44a47d04bfdafc44bedfb5a`.
- Summary log: `skillset-saves/runs/sidecamerad-mvp-20261113/build/evidence/ffmpeg-thread-schedstat-correlation-r4.log` — SHA-256 `d8c1e1602585d14a118a82f49c297778186c6f221142d9ec0d3afced65a0b712`.
- Source: `system/sidecamerad.py` — SHA-256 `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b` before and after.

Two earlier PID-lookup runner attempts remain preserved; their trace paths and hashes are listed in the summary log and artifact. The successful run found FFmpeg using its `/proc` executable/cmdline and ancestry after strace spawn. The diagnostic used only the single visible `/dev/video60` camera and proves neither reconnect behavior nor four-camera readiness.
