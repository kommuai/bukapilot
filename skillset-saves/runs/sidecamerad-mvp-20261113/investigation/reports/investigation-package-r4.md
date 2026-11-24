---
run_id: sidecamerad-mvp-20261113
submission_id: sidecamerad-mvp-20261113_investigation-ac4-timing-r4
revision: "4"
---

# Investigation Package — SideCamera AC-4 Timing Jitter

**Boundary:** `investigation-review` · **Owning phase:** BUILD · **Build input revision:** 68 · **Investigation revision:** 4

## Scope and impact

Investigate the reproduced single-camera AC-4 cadence failure in the source-to-publication timing path for one 1280×720 MJPEG camera requested at 20 fps. AC-6 physical reconnect/sibling continuity and AC-8 four-camera performance/CPU soak remain outside scope. No product source or production argv was changed. Earlier approved packages remain byte-for-byte preserved at `reports/investigation-package-r1.md` (SHA-256 `b4afe53167fb5ff722d3b8174f7e339f98d3cc66578d84d420e12acb8e6bc796`), `reports/investigation-package-r2.md` (SHA-256 `b35b75239d451b80d9cf7015028d4598a821f0e110debd3927213fab67e44cfd`), and `reports/investigation-package-r3.md` (SHA-256 `e83647146bf85fb14b60b44c2aa8c6d209882ded91a2e7f3e5991a9106a1d573`). Revision 1's source-EOF PTS proposal remains superseded: FFmpeg PTS are V4L2 `ts-src-soe`, not EOF.

AC-4 requires at least 6,000 complete published frames over five steady-state minutes, mean cadence of at least 20 fps, no EOF-to-EOF timestamp interval above 60 ms, and matching cereal/VisionIPC frame IDs.

## Reproduction

Reuse the accepted current-source five-minute capture; it was not rerun in revisions 2, 3, or 4. It records 6,050 matched frames over 302.506666 seconds, 6,049 EOF intervals, mean interval 49.998692 ms (20.000523 fps), maximum 89.163715 ms, and 753 intervals above 60 ms. This remains a strict max-gap failure. IDs were contiguous and cereal/VisionIPC timestamps matched.

`evidence/reproduction.log` is byte-identical to the build runtime capture, SHA-256 `de804d35326c6925862d2c8b36708d4a14a10b05120076924f523397976e8c55`. The accepted build probe/report carries source lineage. `system/sidecamerad.py` remained SHA-256 `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b` through all saved captures and diagnostics. The typed reproduction record and hash are in `manifest.json`.

## Observed evidence chain

The timestamped evidence trace is `evidence/evidence-chain-r4.log` (SHA-256 `efd09d6751444a40bdc5d370d0816c2de2061f259cb188201cd5c2df6884ca40`). It retains prior DQBUF, PTS, pipe, and scheduler observations and adds the four-arm ABBA result.

- In the corrected same-run DQBUF/PTS/write/read trace, post-startup DQBUF intervals averaged 49.994824 ms (max 58.634 ms; none >60 ms) and SOE PTS intervals averaged 50.006737 ms (max 52.128 ms; none >60 ms). Output write-boundary gaps averaged 49.957065 ms (max 89.580 ms; 146/598 >60 ms); full-read gaps averaged 49.957056 ms (max 89.587 ms; 147/598 >60 ms). Write-start to full-read completion was median 0.116 ms, p95 0.196 ms. There were 602 successful DQBUF calls for 600 outputs and no sequence fields, so exact dequeue-to-output frame identity is unproven.
- Corrected output write syscall durations were 25,800 writes, median 0.058 ms, p95 0.153 ms, p99 0.256 ms, maximum 10.554 ms, none >50 ms. This does not support sustained pipe backpressure. The separate writes-only trace was observer-intrusive and unpaired with DQBUF.
- Per-thread schedstat on a 600-frame capture showed 77/599 read gaps >60 ms, maximum 80.384 ms. Long-gap intervals averaged 1.469 ms summed runqueue delay versus 1.273 ms on other intervals; only 3/77 long intervals had any thread delay >=5 ms. This does not support CPU runqueue delay as the primary cause. Post-read `/proc` samples add overhead to the next interval and did not identify the exact writer thread.
- The output-flush ABBA had four sequential, traced 600-frame arms, 25,800 output writes per arm, and no payload saved. Pooled read gaps >60 ms were 228/1,198 default versus 227/1,198 `-flush_packets 1`, while the flush maximum was higher (96.677 ms versus 90.374 ms). Pooled write-boundary starts >60 ms were 228/1,198 in both groups, with flush maximum higher (96.578 ms versus 90.280 ms). B1 looked better than A1, but B2 was worse than A2. The pooled result shows no repeatable improvement; the earlier single-pair change is treated as run-to-run variation. ABBA ordering bounds simple drift, but strace and sequential-run effects remain.
- FFmpeg PTS are monotonic-domain V4L2 SOE timestamps (`ts-src-soe`), not EOF. Keep `timestampEof` as host-monotonic time sampled after complete raw NV12 frame read.

ABBA report: `../build/reports/report_ffmpeg-flush-abba-r3.md` SHA-256 `e904cc3db831e98eb200e7bdd988b333082ccacfa4df4a85daa2410d94259ca2`. Its JSON, summary log, and four raw trace hashes are bound in the manifest.

## Mechanism

**Medium confidence:** cadence jitter remains after V4L2 buffer dequeue and inside FFmpeg processing/output before raw-pipe delivery. The close write-boundary/read-completion cadence and sub-millisecond write-to-read lag do not support a separate Python reader backlog. Scheduler deltas do not support CPU runqueue delay as the primary cause, and output write syscall durations do not support sustained pipe backpressure.

The ABBA result does not reproduce a consistent improvement from `-flush_packets 1`, so output flushing is not a supported cadence fix. The exact FFmpeg internal decode/filter/output pacing or synchronization wait remains unknown. Existing captures do not determine whether a particular >60 ms output gap aligns with a V4L2/input wait, FFmpeg synchronization wait, or user-space processing. Exact writer-thread attribution and DQBUF-to-output identity remain unproven.

## Bounded BUILD path

**Owner:** BUILD through `build/build-management`.

Run one read-only, 600-frame default-argv FFmpeg trace on the same camera, with no source or argv changes. Keep the existing fixed executable, 1280×720 MJPEG-to-NV12 mode, one decoder thread, one filter thread, queue size 1, reader, and all other settings fixed. Use per-thread `strace -f -ttt -T` to record ioctl, futex, poll/ppoll, select/pselect, epoll wait, and read/write syscall IDs and durations. Keep raw-frame pipe read/write arguments raw-only so no payload bytes are logged; capture V4L2 ioctl metadata and request identity only. In the Python controller, record monotonic completion times after every full frame and per-thread `/proc/<ffmpeg-pid>/task/*/schedstat` runtime/runqueue deltas at the same boundaries; bracket realtime and monotonic clocks for correlation. Save only trace metadata and timing, not frame payloads.

For each >60 ms full-read interval, report overlapping per-thread blocking syscall IDs/durations, runtime deltas, and runqueue-delay deltas. Long DQBUF/poll/futex waits overlapping a gap would support a kernel-input or synchronization wait. Increased runqueue delay would support runnable scheduling. Runtime accumulation without a long blocking syscall would be consistent with user-space processing but would not prove it; no matching wait or scheduler signal leaves the internal boundary unresolved. This one capture is bounded to 600 frames and is diagnostic only. `strace` perturbs scheduling, and the post-read `/proc` snapshots add overhead to the next interval, so any attribution must carry those limits. Do not change production argv or claim AC-4 passes.

Preserve the approved timestamp contract throughout: `timestampEof` remains host-monotonic at complete raw-frame read; PTS remains SOE and is not substituted for EOF. The prescribed five-minute acceptance remains the decision point after any separately approved product change.

## Residual uncertainty

- The A/B ABBA does not show repeatable sensitivity to `-flush_packets 1`; traced sequential runs still include observer and run-order effects.
- No capture has yet aligned each long full-read interval with per-thread ioctl/futex/poll/epoll waits and scheduler runtime/runqueue deltas. The proposed trace has not been run. Its syscall evidence can distinguish observed blocking waits from intervals without such waits, but absent a long wait cannot by itself prove CPU work over scheduling or an unobserved wait.
- DQBUF counted 602 successful calls for 600 outputs and omitted sequence fields; exact frame identity remains unproven.
- The accepted five-minute reproduction still fails AC-4; 600-frame diagnostic traces do not replace it. AC-6 reconnect/sibling continuity and AC-8 four-camera soak/CPU headroom remain unverified hardware deferrals. The minor FFmpeg child resource-limit finding remains open.
