# GDB sampling diagnostic — investigation revision 7

## Result

Status: **600-frame capture complete; process cleanup ended by signal 9**. The controller read 600/600 complete raw frames (829,440,000/829,440,000 bytes) from the configured camera `/dev/v4l/by-path/platform-xhci-hcd.11.auto-usb-0:1:1.0-video-index0` (resolved to `/dev/video60`). After the requested bytes were read, FFmpeg returned `-9` (signal 9 / SIGKILL); the record does not show a graceful exit. The child was reaped and no frame payload was saved. This cleanup result is a limitation of the attempt, although it did not truncate the requested frame capture.

The controller mirrored `CameraWorker._read_frames`: `poll(250 ms)` then `readv` into the remaining full-frame buffer, with a 1,382,400-byte frame size. The default FFmpeg argv was unchanged; the controller stopped its capture after frame 600, without `-frames:v` or other argv additions. Frame 0 was startup; subsequent reads armed a persistent condition-based monotonic one-shot watchdog at 60 ms. Maximum GDB snapshots: three. Trigger, attach/detach and full-read completion markers, pause durations, verified PID/exe/cmdline, thread/LWP mappings, and stacks are in the JSON and log.

Successful GDB snapshots: 3; >60 ms watchdog triggers: 63; budget-suppressed triggers: 60. Instrumented read intervals: {'count': 599, 'mean': 49.974, 'p95': 64.672, 'max': 1098.139, 'gt60ms': 65}. The sampled stacks showed the rawvideo mux thread in `write()` in all three snapshots; the main thread was in condition waits in snapshots 1 and 3, and in `sws_scale_frame()` in snapshot 2. These are callsite observations only. GDB startup/attach and process pauses perturb cadence, so the timings do not measure uninstrumented performance and do not establish causality or AC-4 acceptance. AC-4 remains failed/open.

## Integrity and prior attempt

Source hash before/after: `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b` / `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b`; drift `False`. Attempt 1 remains preserved byte-for-byte at the separate `*-attempt1.*` files with its failed-controller hashes listed in the log and JSON. Attempt 1 used `/dev/video60` and `-frames:v 600`; this corrected attempt used the configured by-path path and no argv changes.

- Report: `build/reports/report_gdb-sampling-r7.md`
- Command log: `build/evidence/gdb-sampling-r7.log` — SHA-256 `adb417c4888e65b16e13cb100370a4cb961307469bee7599696580bdf2ce24a9`
- Capture JSON: `build/artifacts/ffmpeg-gdb-sampling-r7.json` (its final digest is registered in the run checkpoint)

No frame payload was saved. No strace or other tracer was used. No source edits or product tests were run. This diagnostic does not replace the five-minute AC-4 acceptance capture. No further camera attempt is included in this diagnostic.
