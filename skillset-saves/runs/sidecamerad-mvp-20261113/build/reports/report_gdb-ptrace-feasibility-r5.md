# GDB/ptrace feasibility — investigation revision 5

## Result: GO for the bounded feasibility check

GDB was available at `/usr/bin/gdb` (GNU GDB 15.1). The process ran as UID/EUID 1000/1000 with no elevation; effective capabilities were zero. This host namespace did not expose `/proc/sys/kernel/yama/ptrace_scope` or `/proc/sys/kernel/ptrace_scope`. `/proc/self/status` reported `TracerPid: 0` and `Seccomp: 0`.

GDB attached to a disposable same-user Python child (PID 1030953, UID 1000) without elevated privileges. It captured four thread backtraces with 86 frame lines; each thread had frames, including symbolized Python and libc frames. The main-thread unwind ended with GDB's `previous frame identical` warning, while all four stacks remained readable. GDB detached cleanly (`[Inferior 1 (process 1030953) detached]`), and the child remained alive afterward. The harness then terminated and reaped the child. No process remains from the probe.

The first report classifier did not recognize GDB 15.1's attach/detach wording. I corrected the classification from the preserved command output; **the attach was not repeated**. The retained output shows three new LWP records, `libthread_db` initialization, four thread stacks, and the detached inferior message.

This is a GO only for GDB/ptrace availability on a disposable same-user target. It authorizes no FFmpeg attach, camera capture, or production change. AC-4 remains open; any separate FFmpeg sampling plan needs its own bounded scope and Admiral authorization.

## Existing r4 trace context

Admiral's cross-check of the already-saved r4 trace found TID 863811 performed all 25,800 writes on fd `0xc` and also issued futex calls. TIDs 863769 and 863809 issued `VIDIOC_DQBUF` on fd `0xb`. These are observed thread roles in the saved trace; they do not establish that any futex wait caused a frame gap. This feasibility probe did not attach to FFmpeg or capture camera data.

## Evidence

- Report: `skillset-saves/runs/sidecamerad-mvp-20261113/build/reports/report_gdb-ptrace-feasibility-r5.md`
- Command log: `skillset-saves/runs/sidecamerad-mvp-20261113/build/evidence/gdb-ptrace-feasibility-r5.log` — SHA-256 `eb03bdede78cd20e69b7e8dc12241fda9f087a79b2c3220ad10461fa48351732`

The command log records the exact GDB argv, policy observations, child identity, attach/backtrace/detach results, full GDB output, and cleanup status. No source edits or product tests were performed. GDB used `/usr/bin/gdb --quiet --batch --nx --pid 1030953`, then `thread apply all bt`, `detach`, and `quit`.
