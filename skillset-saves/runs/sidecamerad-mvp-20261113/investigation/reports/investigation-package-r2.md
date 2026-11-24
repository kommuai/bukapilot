---
run_id: sidecamerad-mvp-20261113
submission_id: sidecamerad-mvp-20261113_investigation-ac4-timing-r2
revision: "2"
---

# Investigation Package — SideCamera AC-4 Timing Jitter

**Boundary:** `investigation-review` · **Owning phase:** BUILD · **Build input revision:** 68 · **Investigation revision:** 2

## Scope and impact

Investigate the reproduced single-camera AC-4 cadence failure in the source-to-publication timing path for one 1280×720 MJPEG camera requested at 20 fps. AC-6 physical reconnect/sibling continuity and AC-8 four-camera performance/CPU soak are outside this investigation. No product source was edited. The approved revision-1 package is preserved byte-for-byte at `reports/investigation-package-r1.md` (SHA-256 `b4afe53167fb5ff722d3b8174f7e339f98d3cc66578d84d420e12acb8e6bc796`). Revision 1's proposed source-EOF PTS path is superseded: the newer same-run evidence identifies PTS as V4L2 `ts-src-soe`, not EOF.

AC-4 requires at least 6,000 complete published frames over five steady-state minutes, mean cadence of at least 20 fps, no EOF-to-EOF timestamp interval above 60 ms, and matching cereal/VisionIPC frame IDs.

## Reproduction

Reuse the accepted current-source five-minute capture; it was not rerun for this revision. It records 6,050 matched frames over 302.506666 seconds, 6,049 EOF intervals, mean interval 49.998692 ms (20.000523 fps), maximum 89.163715 ms, and 753 intervals above 60 ms. The strict max-gap condition still fails. IDs were contiguous and cereal/VisionIPC timestamps matched.

`evidence/reproduction.log` is byte-identical to the build runtime capture, SHA-256 `de804d35326c6925862d2c8b36708d4a14a10b05120076924f523397976e8c55`. The accepted build probe/report carries its input lineage. `system/sidecamerad.py` remained SHA-256 `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b` through the saved reproductions and new diagnostics. The revised typed record and hashes are in `manifest.json`.

## Observed evidence chain

The timestamped trace is `evidence/evidence-chain-r2.log` (SHA-256 `4f48970cac9f2f18217cbdce31323ae435e59b77294b7c04f91e46ff2459d8ec`). It separates observations from inference and names the captured source for each observation.

- The same-run corrected DQBUF/PTS/write/read trace has regular post-startup DQBUF and SOE PTS cadence: 600 DQBUF intervals averaged 49.994824 ms, max 58.634 ms, with none over 60 ms; 598 PTS intervals averaged 50.006737 ms, max 52.128 ms, with none over 60 ms.
- Over those steady intervals, FFmpeg raw-output write boundaries averaged 49.957065 ms, max 89.580 ms, with 146/598 over 60 ms. Python full-frame read completions averaged 49.957056 ms, max 89.587 ms, with 147/598 over 60 ms. Write-start to read completion was median 0.116 ms and p95 0.196 ms. The series place the cadence divergence after dequeue timestamps and before raw-pipe delivery. The trace counted 602 successful DQBUF calls for 600 output frames and omitted V4L2 sequence fields, so exact dequeue-to-output frame identity is not proven.
- The corrected DQBUF trace recorded 25,800 raw-output writes. Their syscall durations had median 0.058 ms, p95 0.153 ms, p99 0.256 ms, and maximum 10.554 ms; no write exceeded 50 ms. This does not support sustained pipe backpressure. The separate writes-only trace has observer overhead and was unpaired with DQBUF; preserve that limitation. The corrected same-run syscall trace is also traced evidence, so its timing may be affected by tracing.
- An untraced 600-frame FFmpeg read with per-thread `/proc/<pid>/task/*/schedstat` snapshots completed 600/600 frames. Of 599 read intervals, 77 exceeded 60 ms and the maximum was 80.383947 ms. Four FFmpeg threads exposed schedstat. Long gaps had mean summed runqueue delay 1.469 ms and mean maximum single-thread delay 0.988 ms; only 3/77 had any thread delay of at least 5 ms. Other intervals had means 1.273 ms and 0.848 ms, with 13/522 at least 5 ms. This does not support CPU runqueue delay as the primary cause. The `/proc` reads occurred after each frame and their cost is included in the next interval; exact writer-thread attribution is not established.
- FFmpeg PTS are monotonic-domain V4L2 SOE timestamps (`ts-src-soe`), confirmed in the same-device source correlation. SOE is exposure start, not EOF. Keep `timestampEof` as host-monotonic time sampled after a complete raw NV12 frame read.
- The earlier five-minute capture and same-run reader evidence show contiguous IDs, matching cereal/VisionIPC times, low producer-to-reader lag, and bursty output at both producer and reader. The separate FFmpeg-to-raw-pipe capture reproduced gaps without SideCamera IPC. Reader-side backlog is not supported.

Supporting saved build reports: `../build/reports/report_pts-read-correlation-r2.3-r1.md`, `../build/reports/report_dqbuf-output-correlation-r2.3-r1.md`, and `../build/reports/report_ffmpeg-stdout-write-trace-r2.3-r1.md`. Their source artifacts and hashes are bound in the manifest.

## Mechanism

**Medium confidence:** cadence jitter appears after V4L2 buffer dequeue and inside FFmpeg processing/output before raw-pipe delivery. The matching write-boundary/read-completion cadence and sub-millisecond write-start-to-read lag do not support a separate Python reader backlog. Scheduler-delay deltas do not support CPU runqueue scheduling as the primary cause. Short output-write syscall durations do not support sustained pipe backpressure.

The exact FFmpeg internal decode/filter/output pacing or synchronization wait remains uncertain. The available traces do not identify the exact raw-output writer thread, and missing DQBUF sequence fields prevent exact per-frame dequeue-to-output matching. This is a bounded suspect boundary, not a claim of a proven internal FFmpeg submechanism.

## Bounded BUILD path

**Owner:** BUILD, through `build/build-management`.

Before any source change, run one read-only, one-variable 600-frame FFmpeg output-pacing A/B on the assigned camera: compare the current output-flush default with a second fixed argv that adds only output-side `-flush_packets 1`. Keep the existing executable, 1280×720 MJPEG-to-NV12 mode, `-threads 1`, `-filter_threads 1`, `-thread_queue_size 1`, frame count, reader, and every other argument fixed. This diagnostic adds no decoder threads or application queues. Save no image payload and make no source edit. Record complete-frame counts, write-boundary gaps, full-read completion intervals, and write durations for both arms.

Preserve the approved timestamp contract during the probe: `timestampEof` remains host-monotonic at complete raw-frame read completion; SOE PTS remains diagnostic data and is not substituted for EOF. Compare the number of >60 ms gaps and maximum gap. Any change is diagnostic evidence only, not an AC-4 pass or authorization to change production arguments. If BUILD/security determines that this fixed diagnostic argv variant crosses the approved fixed-argv/resource contract, stop before running it and escalate that exact contract decision to the design/security owner. A later product change requires its own BUILD review and the existing five-minute AC-4 acceptance.

## Residual uncertainty

- The DQBUF trace has 602 successful calls for 600 outputs and no sequence fields; exact frame identity is unproven.
- `/proc` schedstat snapshots are post-read and add observer cost to the following interval; the exact FFmpeg output writer thread and internal decode/filter wait were not isolated.
- The writes-only strace was unpaired and observer-intrusive. The corrected DQBUF trace correlates DQBUF, PTS, writes, and reads in one run, but strace itself can affect scheduling.
- The proposed 600-frame A/B has not been run. Its result can narrow output-buffer pacing only; it cannot establish five-minute AC-4 acceptance. AC-4 remains failed on the reused accepted reproduction.
- AC-6 reconnect/sibling continuity and AC-8 four-camera soak/CPU headroom remain unverified hardware deferrals. The existing minor FFmpeg child resource-limit finding remains open.
