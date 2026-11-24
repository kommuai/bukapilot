# Admiral intake brief: sensord efficiency refactor

## Request and endpoint
Investigate and refactor `/data/openpilot/system/sensord` to reduce CPU and memory use and substantially reduce implementation code while preserving behavior. Route through design, build, and review; stop before merge, deployment, or hardware rollout.

## Scope
Cover KA2 ICM42670 accelerometer, gyro, temperature and LIS2MDL polling, plus TICI/tizi LSM6DS3 interrupt-driven accel/gyro, temperature polling, and tizi’s optional MMC5603 polling. Keep current sensor selection, service frequencies, data mapping, message fields and metadata, timestamp sources, warmup validity, calibration algorithm and thresholds, error handling, IRQ handling, and shutdown.

## Intended bounded slice
Rate-limit invalid polling iterations using the already-configured service Ratekeeper; hoist fixed transforms/scales out of sample paths; reduce calibration sample-buffer memory without changing sample values or the existing `fmean`/`pstdev` calculations; and centralize repeated sensor-event construction where omitted/default fields remain identical.

## Constraints and non-goals
Do not change cereal schemas, sensor topology, bus ownership, register settings, sample rates, thread topology, sensor identities, or published behavior. Do not alter unrelated existing work in the checkout. Do not claim measured target-device performance without a KA2/TICI baseline.

## Acceptance
Focused offline tests prove invalid polling iterations are rate-limited without warmup publication, event metadata/payloads remain equal, and gyro calibration results remain identical for fixed samples. Run the affected offline tests and `git diff --check`. Hardware integration and comparative CPU/RSS measurements are deferred: `/dev/i2c-*` nodes are present, but their board/sensor identity and functional access are unverified, no KA2/TICI rig is confirmed, and no before/after baseline is available.

## Intake evidence
`system/sensord/sensord.py` and sensor drivers show that invalid polling samples currently `continue` before `Ratekeeper.keep_time()`. Configured service rates are in `cereal/services.py`. The existing integration test is marked TICI and starts the managed hardware process. No project `AGENTS.md` exists. On resume, readiness reports Python 3.12.3 below the harness minimum of 3.13 and missing Codex hooks; the project `pyproject.toml` permits Python >=3.11,<3.13. No MCP-backed data or application is needed.
