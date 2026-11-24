---
run_id: sidecamerad-mvp-20261113
submission_id: sidecamerad-mvp-20261113_investigation-ac4-timing-r6
revision: "6"
---

# Investigation Package — SideCamera AC-4 Timing Jitter

**Boundary:** `investigation-review` · **Owning phase:** BUILD · **Build input revision:** 68 · **Investigation revision:** 6

## Scope and impact

Investigate the reproduced single-camera AC-4 cadence failure in the source-to-publication timing path for one 1280×720 MJPEG camera requested at 20 fps. AC-6 physical reconnect/sibling continuity and AC-8 four-camera performance/CPU soak remain outside scope. No product source or production argv was changed. Earlier approved packages remain byte-for-byte preserved at `reports/investigation-package-r1.md` (SHA-256 `b4afe53167fb5ff722d3b8174f7e339f98d3cc66578d84d420e12acb8e6bc796`), `reports/investigation-package-r2.md` (SHA-256 `b35b75239d451b80d9cf7015028d4598a821f0e110debd3927213fab67e44cfd`), and `reports/investigation-package-r3.md` (SHA-256 `e83647146bf85fb14b60b44c2aa8c6d209882ded91a2e7f3e5991a9106a1d573`), and `reports/investigation-package-r4.md` (SHA-256 `39f762cc1f8002a5ea694cf59fe638e359a856ac8e206a8b1c52209900da08bc`), and `reports/investigation-package-r5.md` (SHA-256 `e67cd4b4290b38f022bb5f1a164b3285ff0bd2f7db413bdec3024178909c5ac0`). Revision 1's source-EOF PTS proposal remains superseded: FFmpeg PTS are V4L2 `ts-src-soe`, not EOF.

AC-4 requires at least 6,000 complete published frames over five steady-state minutes, mean cadence of at least 20 fps, no EOF-to-EOF timestamp interval above 60 ms, and matching cereal/VisionIPC frame IDs.

## Reproduction

The accepted current-source five-minute capture was reused in revision 6 and was not rerun. It records 6,050 matched frames over 302.506666 seconds, 6,049 EOF intervals, mean interval 49.998692 ms (20.000523 fps), maximum 89.163715 ms, and 753 intervals above 60 ms. This remains a strict max-gap failure. IDs were contiguous and cereal/VisionIPC timestamps matched.

`evidence/reproduction.log` is byte-identical to the build runtime capture, SHA-256 `de804d35326c6925862d2c8b36708d4a14a10b05120076924f523397976e8c55`. The accepted build probe/report carries source lineage. `system/sidecamerad.py` remained SHA-256 `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b` through all saved captures and diagnostics. The typed reproduction record and hash are in `manifest.json`.

## Observed evidence chain

The revision-6 trace is `evidence/evidence-chain-r6.log` (SHA-256 `380e84f2791cc6741d741beb18bbca36209bdcb46a262e3fa6903547f895afc2`). It carries the accepted reproduction, r4 DQBUF/PTS/write/read and syscall/schedstat observations, r5 offline futex pairing, and the new GDB feasibility result. No capture or test was run in revision 6.

- The accepted five-minute reproduction remains the unchanged current-source capture: 6,050 matched frames over 302.506666 seconds, mean 20.000523 fps, maximum EOF interval 89.163715 ms, and 753/6,049 intervals above 60 ms. AC-4 remains failed. Product source SHA-256 remains `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b`.
- Existing r4 DQBUF/PTS correlation found no >60 ms post-startup DQBUF or SOE PTS gaps, while output write-boundary and full-read gaps exceeded 60 ms. There were 602 successful DQBUF calls for 600 outputs with no sequence fields, so exact dequeue-to-output identity is unproven. Output write durations (25,800 calls; p95 0.153 ms; max 10.554 ms) do not support sustained pipe backpressure. Per-thread schedstat did not support CPU runqueue delay as the primary cause. The sequential flush ABBA did not show a repeatable effect.
- Existing r4 syscall/schedstat capture reported 133/599 full-read intervals >60 ms (max 95.742 ms). The offline r5 futex analysis found 598 successful writer waits >=1 ms with positive same-address wakes from TID 863769 temporally inside each wait, but `FUTEX_WAKE` does not identify which waiter it woke. All 133 >60 ms gaps separately overlapped >=1 ms waits by each of TIDs 863769, 863808, and 863811, and all 133 overlapped at least one futex wait >=10 ms. This remains synchronization correlation, not causal proof; the writer may be idle waiting for upstream work.
- The saved GDB feasibility probe used GDB 15.1 without elevation on a disposable same-UID Python process. It obtained four readable thread stacks totaling 86 frame lines, with one main-thread unwind warning, then detached cleanly; the child remained alive and was reaped. The host ptrace sysctl files were not available. The feasibility result is GO only for this disposable Python target. It does not prove GDB can attach to FFmpeg or reveal SideCamera source behavior. The initial report classifier was corrected against preserved output; the attach was not repeated.

GDB report: `../build/reports/report_gdb-ptrace-feasibility-r5.md` (SHA-256 `7d58e08a8383fbee735c4b9cd411542801fdbbccf9a441957486304f1c4a481d`). Command log: `../build/evidence/gdb-ptrace-feasibility-r5.log` (SHA-256 `eb03bdede78cd20e69b7e8dc12241fda9f087a79b2c3220ad10461fa48351732`). Prior r5 package is preserved byte-for-byte at `reports/investigation-package-r5.md` (SHA-256 `e67cd4b4290b38f022bb5f1a164b3285ff0bd2f7db413bdec3024178909c5ac0`). Keep `timestampEof` as host-monotonic time sampled after full raw-frame read; FFmpeg PTS are V4L2 SOE timestamps, not EOF.

## Mechanism

**Medium confidence:** cadence jitter remains after V4L2 buffer dequeue and inside FFmpeg processing/output before raw-pipe delivery. Futex waits correlate with long gaps, and the output-writer wait intervals have temporal same-address positive wake calls from the FFmpeg leader. This does not identify the awakened waiter or prove that a wake caused a gap. The GDB feasibility check established non-elevated attachment only to a disposable same-UID Python process; it adds no FFmpeg stack or callsite evidence. Reader-side backlog, primary CPU runqueue delay, sustained pipe backpressure, and a repeatable `-flush_packets 1` effect remain unsupported. The exact FFmpeg wait/processing cause remains unresolved.

## Bounded BUILD path

**Owner:** BUILD through `build/build-management`, after the revision-6 investigation-review cross-stage approval. No capture is authorized before that approval.

After r6 cross-stage approval, run one 600-frame diagnostic on the same assigned camera using the fixed FFmpeg executable and unchanged default production argv. Keep the 1280×720 MJPEG-to-NV12 mode, one decoder thread, one filter thread, queue size 1, and the same camera and reader fixed. Save no frame payload. After startup, arm a one-shot monotonic watchdog for each in-flight full-frame read; if that read remains incomplete beyond 60 ms, trigger at most one snapshot for it. Do not re-arm until that full frame completes, and cap the run at three snapshots total. Verify the target PID is the live FFmpeg child launched by the controller and that its `/proc/<pid>/exe` and command line match the fixed executable and expected argv. For each trigger, record monotonic trigger, attach, detach, and full-read-completion times plus debugger pause duration. Use the bounded batch invocation `gdb --quiet --batch --nx --pid <pid> -ex 'set pagination off' -ex 'set confirm off' -ex 'set debuginfod enabled off' -ex 'info threads' -ex 'thread apply all bt 16' -ex 'detach' -ex 'quit'`; retain its TID/LWP mapping and stacks. Do not use strace or another tracer. Abort and clean up if PID verification, attach, backtrace collection, or detach fails.

GDB stops FFmpeg threads while collecting each snapshot, so these pauses perturb cadence and may extend the triggering read gap. The snapshots are for callsite localization only; their timings cannot measure AC-4 and do not establish causality. Keep the timestamp contract unchanged: `timestampEof` remains host-monotonic at complete raw-frame read, and source PTS remain SOE. A separate accepted five-minute AC-4 run remains the acceptance measure after any future approved product change.

## Residual uncertainty

- The GDB feasibility evidence is limited to a disposable same-UID Python target. It does not prove attach permission to FFmpeg, provide an FFmpeg callsite, or say anything about SideCamera source behavior. Host ptrace sysctl values were unavailable; one Python main-thread unwind produced a warning.
- Futex wake/wait evidence is temporal same-address association, not kernel-confirmed waiter identity or causal proof. Concurrent same-address waiters may exist, and the output writer may be idle awaiting upstream work. No FFmpeg stacks, instruction pointers, or wchan were captured.
- Any approved future GDB snapshot pauses FFmpeg threads and perturbs the triggering interval; the maximum-three-snapshot plan is for callsite localization, not AC-4 measurement. It must not run before r6 cross-stage approval.
- DQBUF counted 602 successful calls for 600 outputs without sequence fields, so exact dequeue-to-output identity remains unproven. FFmpeg PTS are SOE, not EOF; `timestampEof` remains host-monotonic at full raw-frame read completion.
- The accepted five-minute reproduction still fails AC-4: 753/6,049 EOF intervals exceed 60 ms. AC-6 reconnect/sibling continuity and AC-8 four-camera soak/CPU headroom remain hardware deferrals; the minor FFmpeg child resource-limit finding remains open.
