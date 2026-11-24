# Checkpoint — AC-4 acceptance rerun

## Verified boundary

The run is active in BUILD at Admiral revision 134, pinned, with the build-to-review verdict still REVISE for `AC-4-CADENCE-FAIL` (submission c68, package revision 68, round 1). Investigation-review revision 7 is approved. No FFmpeg/GDB process remains from the corrected GDB diagnostic.

The corrected GDB diagnostic captured 600/600 full frames (829,440,000 bytes), used the configured by-path camera and unchanged default FFmpeg argv, took three GDB snapshots, saved no frame payload, and recorded no product-source hash drift. FFmpeg returned -9 after all requested bytes were read; its child was reaped. The stacks are callsite observations only and are not AC-4 acceptance evidence.

## Open gate finding

The revise packet requires one current-source five-minute acceptance capture on the assigned camera: at least 6,000 successfully published frames, mean cadence >=20.0 fps, every EOF-to-EOF interval <=60 ms, and matching cereal/VisionIPC frame IDs. The prior five-minute run failed (6,000 matched IDs; mean 19.999396661241942 fps; max gap 123.185415 ms; 743/5,999 intervals >60 ms). The eight-second smoke and the 600-frame GDB diagnostic cannot substitute.

## Next action

Delegate one five-minute run to health-check under the existing authorized assigned-camera scope, using the current source hash and preserving per-frame EOF timestamps plus dual-publication IDs. Do not rerun GDB. If AC-4 fails again, preserve the measurements and route the unresolved mechanism through Admiral/investigation; do not apply a speculative source fix. AC-6 remains a hardware deferral and AC-8 remains the open FFmpeg resource-control residual.

## Evidence

- Corrected GDB report: `build/reports/report_gdb-sampling-r7.md` (d7412919734b279918b23d0ef265f44a1117a2cbabe1904ace84c834565d0fb8).
- Corrected GDB log: `build/evidence/gdb-sampling-r7.log` (adb417c4888e65b16e13cb100370a4cb961307469bee7599696580bdf2ce24a9).
- Corrected GDB capture JSON: `build/artifacts/ffmpeg-gdb-sampling-r7.json` (c059e3000c236a6f05cb336fe9afbdecdadc393228f4b77c7f05ec5e029e4782).
- Prior REVISE verdict and judgment: `build/verdict_build-to-review.json` (043042434bb69b167bc757fca7f201d2393ff52680c3ccba023fef899fa0a85f); `build/verdict_build-to-review.judgment.json` (a329d0451a4e2396b9085b80f74ac31a25a82d1be816b5d8a0f9593e1b0c746f).
- Attempt-1 artifacts remain archived separately and hash-verified.
