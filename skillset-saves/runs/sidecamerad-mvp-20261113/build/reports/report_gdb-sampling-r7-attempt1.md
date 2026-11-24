# GDB sampling diagnostic — investigation revision 7

## Result: aborted before any debugger snapshot

One attempt started FFmpeg on `/dev/video60` and completed **2/600 frames** (2,764,800/829,440,000 bytes). FFmpeg exited `255` after cleanup. No watchdog trigger occurred, no PID verification for GDB was reached, and **no GDB attach or backtrace was attempted**. The child and pipes were cleaned up, and all timer threads were joined. The attempt was not retried.

The attempt aborted because frame 1 completed before its 60 ms watchdog threshold. After cancelling the timer, the runner observed that the timer thread had not yet exited and treated that brief cancellation race as fatal. This is a runner bookkeeping failure, not evidence about FFmpeg timing or ptrace feasibility.

## Command-scope deviation

The attempted argv below included `-frames:v 600` as a finite-run cap. Later steering explicitly prohibited adding that option or changing the default argv. This attempt therefore does not satisfy the approved r7 capture scope. No replacement capture was made.

Exact attempted FFmpeg argv: `['/usr/bin/ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'error', '-thread_queue_size', '1', '-threads', '1', '-filter_threads', '1', '-f', 'v4l2', '-input_format', 'mjpeg', '-framerate', '20', '-video_size', '1280x720', '-i', '/dev/video60', '-an', '-sn', '-dn', '-vf', 'scale=1280:720:flags=neighbor,format=nv12', '-pix_fmt', 'nv12', '-threads', '1', '-fps_mode', 'passthrough', '-frames:v', '600', '-f', 'rawvideo', 'pipe:1']`

## Evidence and integrity

- Capture JSON: `build/artifacts/ffmpeg-gdb-sampling-r7.json` — SHA-256 `449a90ffacefd409befce3f8758b81c3aa522fdab2e0c941eecfc878ae6b5e12`.
- Command and cleanup log: `build/evidence/gdb-sampling-r7.log` — SHA-256 `825c970caea2cb44ff0ead6a57db25a73b08884067b00dd343b28e41b59f770f`.
- Source SHA-256 before/after: `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b` / `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b`; unchanged.
- The log preserves the FFmpeg stderr produced during termination. No frame payload was saved; no GDB attach, strace, or other tracer was used. No source edits or product tests occurred.

AC-4 remains failed/open. This partial run is not acceptance evidence and yielded no callsite or causal finding.
