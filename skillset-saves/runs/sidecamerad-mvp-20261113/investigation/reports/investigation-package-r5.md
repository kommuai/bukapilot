---
run_id: sidecamerad-mvp-20261113
submission_id: sidecamerad-mvp-20261113_investigation-ac4-timing-r5
revision: "5"
---

# Investigation Package — SideCamera AC-4 Timing Jitter

**Boundary:** `investigation-review` · **Owning phase:** BUILD · **Build input revision:** 68 · **Investigation revision:** 5

## Scope and impact

Investigate the reproduced single-camera AC-4 cadence failure in the source-to-publication timing path for one 1280×720 MJPEG camera requested at 20 fps. AC-6 physical reconnect/sibling continuity and AC-8 four-camera performance/CPU soak remain outside scope. No product source or production argv was changed. Earlier approved packages remain byte-for-byte preserved at `reports/investigation-package-r1.md` (SHA-256 `b4afe53167fb5ff722d3b8174f7e339f98d3cc66578d84d420e12acb8e6bc796`), `reports/investigation-package-r2.md` (SHA-256 `b35b75239d451b80d9cf7015028d4598a821f0e110debd3927213fab67e44cfd`), and `reports/investigation-package-r3.md` (SHA-256 `e83647146bf85fb14b60b44c2aa8c6d209882ded91a2e7f3e5991a9106a1d573`), and `reports/investigation-package-r4.md` (SHA-256 `39f762cc1f8002a5ea694cf59fe638e359a856ac8e206a8b1c52209900da08bc`). Revision 1's source-EOF PTS proposal remains superseded: FFmpeg PTS are V4L2 `ts-src-soe`, not EOF.

AC-4 requires at least 6,000 complete published frames over five steady-state minutes, mean cadence of at least 20 fps, no EOF-to-EOF timestamp interval above 60 ms, and matching cereal/VisionIPC frame IDs.

## Reproduction

The accepted current-source five-minute capture was reused in revision 5 and was not rerun. It records 6,050 matched frames over 302.506666 seconds, 6,049 EOF intervals, mean interval 49.998692 ms (20.000523 fps), maximum 89.163715 ms, and 753 intervals above 60 ms. This remains a strict max-gap failure. IDs were contiguous and cereal/VisionIPC timestamps matched.

`evidence/reproduction.log` is byte-identical to the build runtime capture, SHA-256 `de804d35326c6925862d2c8b36708d4a14a10b05120076924f523397976e8c55`. The accepted build probe/report carries source lineage. `system/sidecamerad.py` remained SHA-256 `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b` through all saved captures and diagnostics. The typed reproduction record and hash are in `manifest.json`.

## Observed evidence chain

The timestamped evidence trace is `evidence/evidence-chain-r5.log` (SHA-256 `88939d93590a991028c66fa788980cbfef5b572152faac320f32a38f06acdf3c`). It preserves the previous reproduction, DQBUF/PTS/write/read, schedstat, and ABBA findings and adds offline futex wait/wake pairing from the successful final2 trace. No new capture was run.

- The accepted five-minute current-source reproduction remains unchanged: 6,050 matched frames, 6,049 EOF intervals, 20.000523 fps mean, maximum EOF interval 89.163715 ms, and 753 intervals above 60 ms. It fails the strict AC-4 max-gap requirement. `evidence/reproduction.log` remains hash-identical to the build runtime log. Product source remains SHA-256 `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b`.
- In the corrected same-run DQBUF/PTS/write/read trace, post-startup DQBUF gaps had mean 49.994824 ms, max 58.634 ms, and none above 60 ms. V4L2 SOE PTS gaps had mean 50.006737 ms, max 52.128 ms, and none above 60 ms. Output write-boundary gaps had mean 49.957065 ms, max 89.580 ms, 146/598 above 60 ms; full-read gaps had mean 49.957056 ms, max 89.587 ms, 147/598 above 60 ms. Write-start to full-read completion was median 0.116 ms, p95 0.196 ms. There were 602 successful DQBUF calls for 600 outputs and the trace omitted sequence fields, so exact dequeue-to-output identity remains unproven.
- Output writes were fast in the corrected DQBUF trace: 25,800 calls, median duration 0.058 ms, p95 0.153 ms, p99 0.256 ms, max 10.554 ms, none above 50 ms. This does not support sustained pipe backpressure. The earlier writes-only trace was observer-intrusive and unpaired; retain it as corroboration only.
- The 600-frame schedstat probe found 77/599 read gaps above 60 ms, max 80.384 ms. Long-gap intervals averaged 1.469 ms summed runqueue delay versus 1.273 ms on other intervals; only 3/77 had any thread delay at least 5 ms. This does not support CPU runqueue delay as the primary cause. Post-read `/proc` snapshots add observer cost to the next interval.
- The sequential four-arm flush ABBA found 228/1,198 read gaps above 60 ms in default versus 227/1,198 with `-flush_packets 1`; maxima were 90.374 ms and 96.677 ms. Write-boundary counts were 228/1,198 in both groups; maxima were 90.280 ms and 96.578 ms. B1 looked better than A1, but B2 was worse than A2. The small difference is not repeatable and is not causal evidence; sequential run order and strace overhead remain.
- The final2 syscall/schedstat capture recorded 600/600 frames, with 133/599 full-read intervals above 60 ms (max 95.742 ms). All 8,306 unfinished/resumed calls were paired, with zero unmatched or unparsed calls. The saved per-gap correlation reports all 133 long gaps overlapping at least 1 ms futex waits by each of TIDs 863769, 863808, and 863811; separately, all 133 gaps overlap at least one futex wait of 10 ms or longer. These counts describe overlapping waits, not unique gap causes. TID 863811 made all 25,800 writes to fd 12; successful DQBUF calls were on TIDs 863769 and 863809.
- Offline parsing of the raw futex trace found 8,544 futex calls with no pending or unparsed records. Of 2,481 successful waits and 2,478 positive wakes, all 598 successful output-writer waits lasting at least 1 ms (592 at least 10 ms) had a positive same-address FUTEX_WAKE syscall from FFmpeg leader TID 863769 temporally inside the wait interval. This is temporal pairing, not kernel-confirmed waiter identity: FUTEX_WAKE does not report which waiter it woke, and a concurrent same-address waiter may have been selected. The temporal same-address pairings split evenly across two futex addresses, 299 each; median wake-to-wait completion was 0.201 ms. This is an observed synchronization correlation, not evidence that the wake caused a delayed frame or that it represents input readiness. The writer may be idle awaiting upstream work. No stack, instruction pointer, or wchan was captured. Strace and `/proc` sampling overhead were not independently calibrated.
- FFmpeg PTS are V4L2 `ts-src-soe` timestamps (start of exposure), not EOF. Preserve `timestampEof` as host-monotonic time sampled after the complete raw-frame read.

The new pairing record is `evidence/ffmpeg-futex-wake-pairing-r5.log` (SHA-256 `b4da13d544b42b0acb8a1b79c2668ad1fa51af68f6bce1460ccb8f015562ceca`) with structured counts at `artifacts/ffmpeg-futex-wake-pairing-r5.json` (SHA-256 `5c69e8b7f52201fef1ec9a4958bbd3e0b485b1a3f9ec58882847e0e1fdfc07f3`). Its input raw trace is `../build/evidence/ffmpeg-thread-syscall-schedstat-r4-final2.strace` (SHA-256 `0de7b3e50ff49d676085ff34e36e6fad8b9fb36bcbd7200d9df251cb0b9e7037`); associated report, correlation JSON, and capture JSON are registered in the manifest.

## Mechanism

**Medium confidence:** cadence jitter remains after V4L2 buffer dequeue and inside FFmpeg processing/output before raw-pipe delivery. The futex analysis strengthens the internal-synchronization correlation: the output-writer TID blocks on two alternating futex addresses, and each of its 598 successful waits of at least 1 ms has a positive same-address FUTEX_WAKE syscall from the FFmpeg leader TID temporally inside the wait interval. This temporal same-address pairing does not establish waiter identity; FUTEX_WAKE does not report which waiter it woke, and a concurrent same-address waiter could have been selected. The leader also issues DQBUF calls. Across the traced run, all 133 >60 ms read gaps overlap writer-thread futex waits.

This correlation does not identify the futex condition or prove causality. The writer may be idle waiting for upstream work; no stack, instruction pointer, or wchan was captured. It is unresolved whether the upstream signal represents input readiness, decode/filter progress, another synchronization condition, or unrelated work. Reader backlog, CPU runqueue delay as the primary cause, and sustained pipe backpressure remain unsupported. The sequential ABBA did not find a repeatable `-flush_packets 1` effect. No source-time fix is supported: PTS is SOE, not EOF.

## Bounded BUILD path

**Owner:** BUILD through `build/build-management`.

The next BUILD-owned step is a bounded GDB/ptrace feasibility check on the same host and user context, not another capture. Inspect GDB availability and the current ptrace policy, then attempt a non-elevated GDB attach to a disposable same-user child, capture its thread backtrace, and detach cleanly. The check passes only if the attach succeeds, a usable backtrace is returned, and detach completes. If attach is denied, symbols are unusable, or detach fails, stop and route the specific host access/symbolization decision through Admiral; do not bypass policy, substitute another tracer, or start a 600-frame capture. If it passes, return the capability evidence to Admiral before any separately scoped FFmpeg sampling. This probe changes no source, production argv, or timestamp semantics; debugger pauses are observer effects. It is diagnostic only and does not establish AC-4 acceptance.

Preserve the approved timestamp contract: `timestampEof` remains host-monotonic at complete raw-frame read; FFmpeg PTS remains SOE and is not substituted for EOF. No source edit or production argv change is proposed in this investigation.

## Residual uncertainty

- Futex wake/wait pairing is strong internal synchronization correlation, but without stacks, instruction pointers, or wchan it cannot identify the callsite or prove that the leader wake causes the output gap. The output writer may be asleep while awaiting upstream work.
- The host's GDB/ptrace feasibility check is not yet run. If unavailable, the follow-on trace stops; there is no alternate tracer proposed. Any permitted debugger sampling would perturb scheduling and require separately bounded interpretation.
- `strace` and post-read `/proc` sampling overhead were not independently calibrated, so the 600-frame trace's timing and overlap counts are diagnostic only.
- DQBUF counted 602 successful calls for 600 outputs and omitted sequence fields; exact frame identity remains unproven. FFmpeg PTS are SOE, not EOF. The approved `timestampEof` remains host-monotonic at complete raw-frame read.
- The accepted five-minute reproduction still fails AC-4: 753/6,049 EOF intervals exceed 60 ms. The 600-frame diagnostics do not replace the acceptance run. AC-6 reconnect/sibling continuity and AC-8 four-camera soak/CPU headroom remain hardware deferrals; the minor FFmpeg child resource-limit finding remains open.
