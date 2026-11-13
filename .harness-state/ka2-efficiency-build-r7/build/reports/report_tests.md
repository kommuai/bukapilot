# Test report: KA2 loggerd rotation alignment

## Test changes

`TestLoggerd.test_ka2_deterministic_rotation_alignment` uses the paired test flags `LOGGERD_TEST=1` and `LOGGERD_TEST_DETERMINISTIC=1`, temporary log/Params roots, explicit RecordFront/RecordAudio, synthetic VisionIPC frames and audio, a short loggerd boundary, and cleanup that stops encoderd before loggerd.

The test parser reads `roadEncodeIdx`, `wideRoadEncodeIdx`, `driverEncodeIdx`, and `qRoadEncodeIdx` directly as `EncodeIndex` values. It checks each output's complete frame-ID set against generated IDs and reports missing, unexpected, and duplicate IDs. It retains segment, file, sentinel, audio, and stream-order assertions.

Corrected test source SHA-256: `fc55ae0129598d5a0dff0e62160522bffa86502410392dbd37202d4ca01cf6ac`.

## Authorized runtime attempts

The test received explicit user authorization for an idle KA2 target. The parent reported aarch64, `/KA2` marker, MPP device present, and no loggerd/encoderd/camerad processes or MPP holder before the runs. Postflight checks likewise found no such processes or holder.

### Earlier parser-mismatch run

An earlier run crossed the boundary and failed because the test searched for `*EncodeData` qlog fields while loggerd writes `*EncodeIdx`. Its raw logs are preserved in `build/evidence/raw_rotation_failure/`; see `loggerd-focused-rotation-failure.log` and `loggerd-rotation-frame-id-summary.txt`. The test's redundant prefix override had caused a prior harness setup failure and was removed before this process run.

### Corrected exact-ID run — 2026-10-30 02:30 UTC

Command: `pytest system/loggerd/tests/test_loggerd.py::TestLoggerd::test_ka2_deterministic_rotation_alignment -q`.

Pytest exited 1 after 12.77 seconds total (8.83 seconds in the test call). The corrected exact-ID assertion failed at `wideRoadEncodeIdx`. All three camera input services submitted IDs 1..145, and rlog contains 145 raw audio events. Offline parsing of both qlogs found 137 unique output IDs for each stream, with no duplicates or extras. Missing IDs are:

| Output | Expected | Emitted | Missing IDs | Segment counts |
|---|---:|---:|---|---|
| roadEncodeIdx | 145 | 137 | 1, 2, 100, 101, 102, 104, 105, 106 | 97 / 40 |
| driverEncodeIdx | 145 | 137 | 1, 2, 100, 101, 102, 104, 105, 106 | 97 / 40 |
| qRoadEncodeIdx | 145 | 137 | 1, 2, 100, 101, 102, 104, 105, 106 | 97 / 40 |
| wideRoadEncodeIdx | 145 | 137 | 1, 2, 100, 101, 102, 103, 105, 106 | 97 / 40 |

The failure stopped at wideRoad; counts for the other outputs came from the saved summary and offline qlog parsing. Segment membership is observed from this elapsed-time-triggered run, not imposed as a frame-ID schedule. No runtime fix was applied, and this run is a correctness failure, not a passing regression.

The run summary records the exact test-source hash (`fc55ae0129598d5a0dff0e62160522bffa86502410392dbd37202d4ca01cf6ac`), loggerd binary hash (`6c6c13656f4f5abd1de1f31b9c3acb4debc6cc58c4d1aab9a790c46b5b367ed8`), and encoderd binary hash (`a46f1c853094b8cf85cfd6533f874cb65047d6c99fd87e215aa0c762ba860b6d`).

## Evidence

- Second run summary: `build/evidence/rerun_20261030_023059/test_run_summary.json` — SHA-256 `3c4cd258f8b8a5f9e0f7d4c1f8d7087c0967491338d71af2b2198a97f6a75179`.
- Second run's 580-row input/output ledger: `build/evidence/rerun_20261030_023059/frame_ledger.csv` — SHA-256 `27625d6c70829ffd8fb8b46cfceaa5843d712cc3bc5ed6946598a70a6d0557c5`.
- Second run raw qlog/rlog files: `build/evidence/rerun_20261030_023059/`; raw file hashes are in the JSON summary and `loggerd-rotation-evidence-hashes.txt`.
- Earlier run counts and boundary timestamps: `build/evidence/loggerd-rotation-frame-id-summary.txt`, `loggerd-expected-emitted-frame-ledger.csv`, and `loggerd-rotation-boundary-timeline.txt`.
- Combined failure provenance and process-close checks: `build/evidence/loggerd-focused-rotation-failure.log`.
- Source-supported bounded rotation-gap proposal: `build/reports/report_rotation-gap.md`.

## Static checks and validation limits

- In-memory Python syntax parse and scoped `git diff --check` passed after the test correction; commands and results are in `build/evidence/loggerd-test-static-check.log`.
- The corrected pytest was executed in the 02:30 run and failed on the exact-ID assertion.
- The raw pytest console transcript and exact build command are not present in the evidence set; the structured runtime summary and raw qlog/rlog files are preserved.
- No production runtime source was changed. No CPU/RSS/DMA-memory or efficiency benchmark was run. No zero-drop, regression-free, or speedup claim is made.

## Status

The corrected regression detects real frame loss across startup and rotation. The new design scope and production fix remain with Commander. Do not treat this build as accepted or resume performance sampling until the approved correctness gates pass.
