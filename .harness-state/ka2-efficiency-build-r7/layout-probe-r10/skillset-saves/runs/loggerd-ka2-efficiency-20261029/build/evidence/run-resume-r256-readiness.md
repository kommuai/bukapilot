# Run resume and readiness checkpoint

## Verified recovery

- Run `loggerd-ka2-efficiency-20261029` is coherent, pinned, and active at revision 256. `save_run.py status --run-id ...` returned `result: ok`, owner `admiral`, no interruption, and revision 256.
- The stale-run recovery was completed by Admiral and is reflected in `_state.md` as recovery from the prior Admiral lock. This checkpoint records that recovery as a completed event; it does not alter or re-run the recovery operation.
- Readiness is degraded: the available `python3` is 3.12.3, and Codex hook registration is missing. The supported Python 3.13.15 interpreter is available for the build/package validators. These conditions are recorded, not treated as blockers.

## Investigation status

- The first corrected-scope GDB attempt showed a loggerd SIGABRT in `ZstdFileWriter::flushCache`; that run used a pytest temporary directory on `/tmp`, which the investigation found to be 790/800 MiB full with about 11 MiB free.
- A corrected legacy fixture is present at SHA-256 `4e629bf55715eca9341dc48547757accbda5249f892ad95dcb33345135202a18`.
- The next authorized action is one confirmation run using a unique pytest `--basetemp` under `/data`, with a fresh immediate process/device preflight and clean postflight. Capture `written`, `output.pos`, errno, and `ferror(file_)` if the short write recurs, or establish that the corrected lifecycle passes. Preserve existing attempts; do not change production writer code until this distinguishes `/tmp` exhaustion from another cause.
- The investigation agent exhausted its usage limit before this confirmation. Build-management is resuming the bounded action. No commit or deployment is requested.

## Readiness observations

- `save_run.py status` at revision 256: coherent, pinned, active; lock owner `admiral`.
- Python fallback: `python3 --version` reports 3.12.3. Use the installed 3.13.15 path for shape/gate validators.
- Codex hook registration: missing, as reported by readiness; record as degraded readiness.
- Board test authorization remains available from the user. The prior attempt's postflight was clean; the confirmation runner must perform its own immediate preflight and postflight.
