---
submission_id: loggerd-ka2-efficiency-20261029-build-r1
revision: r1
---

# Build status: KA2 loggerd rotation regression

## Outcome

The corrected focused integration test ran on the user-authorized idle KA2 at 2026-10-30 02:30 UTC and failed its exact-frame assertion. All three inputs submitted IDs 1..145, while every encoded output emitted 137 unique IDs. There are no duplicate or extra indices; eight IDs per stream are missing. The build does not prove zero-drop behavior and is not ready for `build-to-review`.

## Product changes

The follow-up changed only `system/loggerd/tests/test_loggerd.py`. It reads direct `EncodeIndex` values from the logged `*EncodeIdx` union fields and compares each output set with all generated frame IDs, reporting missing, extra, and duplicate IDs.

Current test file SHA-256: `fc55ae0129598d5a0dff0e62160522bffa86502410392dbd37202d4ca01cf6ac`.

The earlier approved test-only loggerd seam remains in `system/loggerd/loggerd.cc`, SHA-256 `4a9191f40d1cdbfa1300a9b3a0e24991901ebb63f0bcd1a8647c89aa60b17324`. No runtime behavior fix, build-system edit, encoder-mode/default change, manager change, or unrelated test change was made in this follow-up.

## Runtime evidence

- Command: `pytest system/loggerd/tests/test_loggerd.py::TestLoggerd::test_ka2_deterministic_rotation_alignment -q`.
- 02:30 run result: exit 1; 12.77 seconds total, 8.83 seconds in test call. The exact-ID assertion failed at wideRoad. Remaining stream counts were independently parsed offline from the saved qlogs.
- Inputs: road/wideRoad/driver each submitted 145 IDs, 1..145; 145 raw audio events.
- Outputs: each stream emitted 137 unique IDs, with zero duplicates or extras. Missing road/driver/qRoad IDs: `[1,2,100,101,102,104,105,106]`; missing wideRoad IDs: `[1,2,100,101,102,103,105,106]`. Each stream emitted 97 IDs in segment 0 and 40 in segment 1.
- Target preflight and postflight: the parent records aarch64, `/KA2` marker, MPP device present, and no loggerd/encoderd/camerad process or MPP holder before or after the run.
- The exact test-source and executable hashes for this run are recorded in `build/evidence/rerun_20261030_023059/test_run_summary.json`.

The second-run summary hash is `3c4cd258f8b8a5f9e0f7d4c1f8d7087c0967491338d71af2b2198a97f6a75179`; its 580-row ledger hash is `27625d6c70829ffd8fb8b46cfceaa5843d712cc3bc5ed6946598a70a6d0557c5`. Raw qlog/rlog files and hashes are preserved in the same run directory. Earlier parser-mismatch run evidence remains under `build/evidence/raw_rotation_failure/`.

## Checks and limits

- In-memory Python syntax parse and scoped `git diff --check` passed after the correction. This is static evidence, separate from the failed integration run.
- No runtime fix was applied. The failure provenance, source-supported mechanism, and bounded proposal are in `build/evidence/loggerd-focused-rotation-failure.log` and `build/reports/report_rotation-gap.md`.
- No CPU, RSS, DMA-memory, or efficiency benchmark was run. No zero-drop, no-regression, or speedup claim is made.

## Blocker and next action

The active blocker is the reproduced missing-frame behavior. The implementation/design response remains with Commander under the new design scope. Do not submit `build-to-review` or resume performance sampling until the approved correction passes the exact frame-ID and other correctness gates.
