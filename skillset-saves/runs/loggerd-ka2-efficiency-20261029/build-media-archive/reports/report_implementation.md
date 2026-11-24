# Implementation report: KA2 test rotation seam

## Outcome

Implemented the approved test-only deterministic rotation gate in `system/loggerd/loggerd.cc`. On KA2/RK3588, the existing deterministic clock rotation remains active in production. Under `LOGGERD_TEST=1`, it is enabled only when `LOGGERD_TEST_DETERMINISTIC=1` is also present. With the new flag absent or set to another value, existing `LOGGERD_TEST` fallback behavior remains unchanged. The new flag by itself does not alter production behavior.

The flag is read once during process initialization, and the new mode requires the pre-existing `LOGGERD_TEST` gate. No encoder, manager, build, writer, schema, MPP, or production-default behavior was changed.

## Changed artifact

- `system/loggerd/loggerd.cc` — SHA-256 `4a9191f40d1cdbfa1300a9b3a0e24991901ebb63f0bcd1a8647c89aa60b17324`
- Scoped binary diff — SHA-256 `9ab248b85c14b4993f884c7703f4d54b615962ffb46693779b1491097490400f`

## Static check

- `git diff --check -- system/loggerd/loggerd.cc` — exit 0; log: `build/evidence/loggerd-diff-check.log`.
- `clang++ -fsyntax-only` with the RK3588/SCons-derived flags — exit 0; this was syntax/type checking only and produced no object. Command and result: `build/evidence/loggerd-syntax-check.log`.

## Validation limits

The parent reports rebuilding `system/loggerd/loggerd` and running the focused KA2 synthetic regression after user authorization and target preflight. The build command/transcript and launch-time binary identity were not captured. The focused regression failed; see `report_tests.md` and `report_rotation-gap.md`. Its parser and exact-frame assertion have since been corrected, but not rerun. The current on-disk loggerd binary hash is recorded in `build/evidence/loggerd-focused-rotation-failure.log` and must not be treated as a launch-time fingerprint.

D-6 was resolved for that one authorized synthetic test. No performance sample or benchmark was run, and no CPU/memory improvement or regression-free claim is made. The build remains blocked by the observed frame losses and pending design approval for a production fix.
