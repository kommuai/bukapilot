# Test Results — sidecamerad r2.2

## Outcome

**PASS.** The focused suite passed all 8 tests in 5.85 seconds against the recovered `SideCameraAvailability` Params key and rebuilt binding. The runner log is `evidence/tests-pytest-r2.2.log` (SHA-256 `6483593c0ab82eaf9e22a0e2ce423a35bf206b7325f94aa8ba37772aa3fff519`). The typed probe `artifacts/tests_probe-r2.2.json` binds this run to the source and binding hashes exercised.

## Runner and command

The existing project configuration and previous discovery identify pytest as the runner; no packages were installed.

```text
python3 -m pytest -vv system/manager/test/test_sidecamerad.py
exit code: 0
8 passed in 5.85s
```

The suite passed the eight existing selectors covering the Params default and JSON dictionary round-trip, no-path acknowledgement and signal wait, manager default-on behavior with native camerad retained, inherited-ZMQ removal, stable cereal/VisionIPC mapping, device path validation, capture-failure availability reset, and transition-only Param updates with write-failure recovery.

## Coverage and limits

- **AC-1:** `sidecamerad` defaults on and accepts `SIDECAMERAD=0` as opt-out; native `camerad` remains enabled under its existing `driverview` condition.
- **AC-2:** all four missing-feed warnings, all-false availability initialization, no stream or publisher creation, and signal-driven exit are covered with mocks.
- **AC-3:** the Param default, dictionary serialization, transition-only writes, write-failure recovery, and reset after capture failure are covered.
- **AC-4:** the local IPC setup clears inherited `ZMQ` before constructing Params, VisionIPC, and cereal publishers; hardware capture is mocked here.
- **AC-5:** all four service/stream/schema mappings, accepted device path forms, and invalid resolved paths are covered.

The suite does not access a physical camera or verify live publication. The separate runtime-health evidence owns that check. Physical reconnect and sibling continuity (AC-6), four-camera cadence/CPU headroom (AC-8), and resource measurements remain outside this suite. No test was quarantined and no coverage collection was run.

## Current source lineage

The typed probe includes the current hashes for `common/params_keys.h` (`c2925ebd8f9f26cc04951afb0e3c67ac7cb20a1c370b79aa93a25a4aaef2a031`) and rebuilt `common/params_pyx.so` (`3108e29407f31b9bd86a86ccafc82a4f98a225884412987a24be5c357dbfb586`), along with the test and implementation inputs. This refresh was produced after the merge recovery; it does not advance the shared run state or edit the build manifest.
