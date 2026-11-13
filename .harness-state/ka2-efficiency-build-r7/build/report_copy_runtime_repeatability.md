# Copy-mode synthetic repeatability check

## Result

Three isolated executions of `TestLoggerd.test_ka2_deterministic_rotation_alignment` passed: the previously saved R1, followed by R2 and R3. In each run, every submitted camera frame ID appeared once in each corresponding encoded-index stream, with no missing or duplicate IDs. The first ID carried flags value 8 on every output stream.

Exact run-to-run repeatability was **not demonstrated**. R1 and R2 both submitted IDs 1–148 but assigned different IDs to the two log segments. R3 submitted IDs 1–147 and used another segment split. This is evidence for per-run frame-ID conservation, not for a fixed workload length or deterministic per-ID segment mapping.

## Run results

| Run | Input IDs per camera stream | Encoded IDs per output stream | Driver / road / qRoad segments | Wide-road segments | Pytest |
|---|---:|---:|---:|---:|---|
| R1 | 1–148 (148) | 1–148 (148), no duplicates | 100 / 48 | 100 / 48 | passed in 14.31s |
| R2 | 1–148 (148) | 1–148 (148), no duplicates | 101 / 47 | 101 / 47 | passed in 14.02s |
| R3 | 1–147 (147) | 1–147 (147), no duplicates | 99 / 48 | 100 / 47 | passed in 14.08s |

The paired comparison confirms the segment mapping differs for all four encoded streams between R1/R2 and R2/R3. The recorded per-segment media hashes also differ between adjacent runs.

## Boundary behavior

The focused test sets a five-second segment length and sends synthetic frames at 20 fps. It reads the live segment parameter after publishing each frame and stops after 45 post-boundary frames. The boundary and stop condition are therefore paced by wall-clock scheduling. A one-frame shift in which submitted frame observes the rotation changes the workload length and segment assignment. The test source records this stimulus at `system/loggerd/tests/test_loggerd.py:214-295`.

This explains why the current test does not establish an identical workload across runs. It does not, by itself, show a lost frame: each run's output ID set exactly matches that run's submitted ID set. It also does not prove media-content equivalence, production behavior under concurrent shutdown, or a runtime performance gain.

## Isolation and cleanup

R2 and R3 each had a fresh KA2 preflight immediately before execution: no pre-existing loggerd/encoderd/camerad process or MPP holder, isolated temporary output roots, memory and temperature below stop limits, and more than 768 MiB free. Both runs completed under the 60-second timeout. Their postflights found no remaining target process or MPP holder. Combined R1–R3 generated output was 428,431 bytes, below the 768 MiB cap. Production log and params roots were not touched.

No source files were changed during this repeatability attempt.

## Evidence

- R1 test: `build/evidence/loggerd-copy-runtime-regression.log`; frame summary: `build/evidence/loggerd-copy-runtime-frame-summary.json`.
- R2 preflight, test, summary, postflight: `build/evidence/loggerd-copy-preflight-r2.log`, `build/evidence/loggerd-copy-runtime-r2.log`, `build/evidence/loggerd-copy-runtime-r2-summary.json`, `build/evidence/loggerd-copy-postflight-r2.log`.
- R3 preflight, test, summary, postflight: `build/evidence/loggerd-copy-preflight-r3.log`, `build/evidence/loggerd-copy-runtime-r3.log`, `build/evidence/loggerd-copy-runtime-r3-summary.json`, `build/evidence/loggerd-copy-postflight-r3.log`.
- Pairwise and per-run analysis: `build/evidence/loggerd-copy-repeatability-comparison.json`.
- Input evidence hashes: `build/evidence/loggerd-copy-repeatability-r1-r3-hashes.txt`.
