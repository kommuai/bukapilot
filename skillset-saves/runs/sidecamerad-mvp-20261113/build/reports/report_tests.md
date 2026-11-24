# Test Results — sidecamerad r2.1

## Outcome

**PASS.** The focused pytest run completed with 8 tests passing in 7.06 seconds. The runner log is `evidence/tests-pytest.log` (SHA-256 `07e3923ff525579662c15880adb258bf83a877b54cf20c54f1bc03bed89ca21b`). The typed probe at `artifacts/tests_probe.json` binds the log to the current test and implementation inputs.

## Runner and command

The project config declares pytest; discovery previously reported pytest 9.0.2. No packages were installed.

```text
python3 -m pytest -vv system/manager/test/test_sidecamerad.py
exit code: 0
8 passed in 7.06s
```

The verbose log lists and passes all eight selectors: `test_params_default_and_json_dict_round_trip`, `test_no_paths_acknowledges_each_feed_and_waits_for_signal`, `test_manager_enables_sidecamerad_by_default_and_keeps_camerad`, `test_main_removes_inherited_zmq_before_ipc_and_pubmaster`, `test_side_camera_service_stream_and_schema_mapping_is_stable`, `test_explicit_device_paths_resolve_only_to_video_character_nodes`, `test_worker_clears_availability_after_capture_failure`, and `test_availability_writes_only_state_transitions_and_recovers_write_failure`.

## Coverage

- **AC-1:** `sidecamerad` is enabled when `SIDECAMERAD` is unset and disabled at `0`; native `camerad` remains enabled under its existing `driverview` condition.
- **AC-2:** all four missing-feed warnings, all-false Params initialization, no stream or publisher creation, and idle waits ending through the registered SIGTERM handler. The wait is mocked, so no real 60-second delay occurs.
- **AC-3:** the Params default and Python-dict round trip, transition-only writes, recovery after a failed Params write, and clearing a previously available feed after capture failure.
- **AC-4:** with `ZMQ=1` and one syntactically valid `/dev/video60` feed, the test records `ZMQ` absent during Params, `VisionIpcServer`, and `PubMaster` construction. The worker and IPC hardware are mocked. Shutdown sets the stop event and joins the worker.
- **AC-5:** direct `/dev/videoN` and by-path `video-index0` acceptance, `video-index1` rejection, unsafe resolved target rejection, and all four service/stream/schema mappings.

The test does not access a physical camera or run a real worker. It does not exercise a live disconnect/reconnect, sibling-stream continuity, or four-camera cadence/CPU headroom. The runtime-health stage owns the one-camera end-to-end capture and five-minute cadence check. AC-6 reconnect/sibling behavior and AC-8 remain unverified on this one-decoder host. The 22,118,400-byte four-stream allocation is source-reviewed, not measured by this suite.

## Prior evidence and coverage data

The previous quiet eight-test log and probe were archived as `evidence/tests-pytest-r2.1-quiet.log` (SHA-256 `d409836d2f76ade319d2e10bffcfbf13984f5332856b9a9798a25e0b00a85ae1`) and `artifacts/tests_probe-r2.1-quiet.json` (SHA-256 `34480e67361e7f1a0c4275499d641e47b9655375210733a91dfdb73780b0efbc`). The initial fixture failure is retained as `evidence/tests-pytest-initial-failure.log` (SHA-256 `2770ca81740ecb71c709c0c23fd7dc07d282e36e8fb3252662d6a08f1e571b8c`); the prior seven-test r2.0 log and probe remain archived at their `r2-pre-zmq` paths. The final run passed; no tests were quarantined. Coverage collection was not requested or run. No `.coverage*`, `htmlcov`, or `.nyc_output` residue was created at project root.
