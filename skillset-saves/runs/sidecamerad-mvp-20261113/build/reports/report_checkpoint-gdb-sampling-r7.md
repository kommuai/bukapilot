# Checkpoint — corrected GDB sampling diagnostic

## Boundary and verified state

The run remains in BUILD after the `build-to-review` revise finding. The investigation r7 handoff remains approved. The r7 corrected diagnostic read all 600 requested complete frames (829,440,000 bytes) with the configured by-path camera and unchanged default FFmpeg argv. Three GDB snapshots succeeded. No payload was saved, the source hash did not drift, the child was reaped, and no FFmpeg/GDB process remained after cleanup.

The process returned `-9` (signal 9) after all requested frames had been read; the evidence does not show a graceful FFmpeg exit. The rawvideo mux thread was in `write()` for each snapshot; the main thread was in condition waits in two snapshots and `sws_scale_frame()` in one. These are callsite observations only. Debugger startup and pauses perturb the sampled cadence; they do not establish cause or AC-4 acceptance.

## Open requirement

AC-4 remains failed/open. The 600-frame diagnostic is not the five-minute acceptance capture required by the build revise finding. AC-6 and AC-8 remain unverified hardware deferrals. The first malformed GDB attempt remains preserved separately, and no additional GDB capture is part of this checkpoint.

## Evidence

- `build/reports/report_gdb-sampling-r7.md`
- `build/evidence/gdb-sampling-r7.log`
- `build/artifacts/ffmpeg-gdb-sampling-r7.json`
- Attempt 1 archives: `build/reports/report_gdb-sampling-r7-attempt1.md`, `build/evidence/gdb-sampling-r7-attempt1.log`, and `build/artifacts/ffmpeg-gdb-sampling-r7-attempt1.json`

## Next action

Resume `build/build-management`. It should carry the corrected diagnostic into the current build package and resolve the AC-4 revise finding against the approved scope. The five-minute acceptance run remains a distinct required item; use existing approval lineage if it authorizes that run, otherwise return the exact authorization gap to Admiral. Do not repeat the GDB sampling diagnostic.
