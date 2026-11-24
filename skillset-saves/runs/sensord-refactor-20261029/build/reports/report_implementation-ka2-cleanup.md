# Implementation report: KA2-only sensord cleanup

## Outcome

Implemented the approved production/configuration/CI slice. `sensord.main()` now fails immediately when `KA2` is false, then configures realtime and starts only the four KA2 sensor drivers on bus 3 using polling. The accepted KA2 polling, event and shutdown behavior is retained.

## Changed paths

- `system/sensord/sensord.py` — removed the TICI/tizi composition, GPIO/IRQ loop, and non-KA2 imports; added the first-action KA2 guard and fixed bus 3.
- `system/manager/process_config.py` — set `sensord` registration to `enabled=KA2`; retained `only_onroad`.
- `selfdrive/test/test_onroad.py` — removed the TICI sensord CPU budget.
- `system/loggerd/tests/test_encoder.py` — removed sensord start/stop from the loggerd rotation test.
- `Jenkinsfile` — removed the dedicated LSM + MMC `tizi-lsmc` stage and added the offline sensord suite to `HW + Unit Tests`, selected by `system/sensord/` changes.
- Deleted `system/sensord/sensors/lsm6ds3_accel.py`, `lsm6ds3_gyro.py`, `lsm6ds3_temp.py`, and `mmc5603nj_magn.py`.
- The two pre-implementation contract tests are in `system/manager/test/test_manager.py` and `system/sensord/tests/test_sensord.py`; they were authored by test-builder and remain subject to the final test-builder pass. The initial red run is not pass evidence.

All changed product paths are first-party repository files. No generated or vendored path was touched. The authored unified diff is `build/artifacts/implementation-ka2-cleanup.diff` (SHA-256 `68e956530937f49c02187f9da1787432922d37761d31390d3024f45966753db1`).

## Verification

- Python AST parsing of the four changed Python implementation files passed.
- Scoped `git diff --check` passed.
- Jenkins selection assertions passed: the old hardware stage is absent and the offline suite is selected by the requested diff path.
- Runtime reference scan and deleted-driver absence check passed.

Detailed command logs are in `build/evidence/implementation-ka2-cleanup.log` and its four referenced check logs. The final offline suite and loggerd test remain for test-builder; no final tests are claimed here.

## Risks

- KA2 physical startup, cadence, shutdown, and target CPU/RSS were not tested; no identified KA2 board, authorized rig, or target baseline was available.
- Local Python 3.12.3 is below the harness minimum. The focused tests used that project environment; the isolated Python 3.13.15 harness interpreter lacked pytest.
- The interim test-first file still contains non-KA2 constructor names solely in the new guard test's patch list; test-builder must remove those stale names when finalizing the test suite after these driver modules are deleted.

## Return

Implementation diff and check evidence are returned to `build/build-management`. No commit was created.
