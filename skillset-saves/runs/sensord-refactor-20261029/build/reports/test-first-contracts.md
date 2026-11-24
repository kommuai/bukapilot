# Test-first KA2 platform contracts

**Outcome:** The two approved offline regression tests fail against the pre-change code as expected. This is a test-first implementation precondition, not final test evidence.

**Runner discovery:** The approved implementation specification names `python3 -m pytest`; the project manifest also declares pytest. The isolated Python 3.13.15 harness interpreter does not have pytest installed, so that attempt is preserved in `evidence/test-first-contracts-interpreter-probe.log`. The focused red run used the project Python 3.12.3 environment with pytest 9.0.2.

**Command:**

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -n 0 -q \
  system/manager/test/test_manager.py::test_sensord_enabled_only_for_ka2_and_stays_onroad \
  system/sensord/tests/test_sensord.py::test_non_ka2_fails_before_setup_sensor_access_threads_or_publication
```

**Observed:** Exit 1, 2 failed in 0.19s. Manager selection reports `sensord.enabled` is `True` when KA2 is patched false; direct non-KA2 startup returns without raising `RuntimeError`. Both are the expected pre-change failures. No manager process, sensor constructor, I2C access, or hardware test ran; the sensor constructors, thread creation, realtime setup, and publication APIs were mocked.

**Coverage:** Manager selection is exercised for both KA2 values and the existing on-road predicate is asserted. Direct-entry failure ordering covers realtime setup, all retained KA2 constructors, removed-platform constructors, thread creation, and publication. Full KA2 behavior and the complete offline suite remain for the post-implementation test stage.

**Environment limitation:** The harness's Python 3.13.15 interpreter lacks pytest. This focused run used the project's Python 3.12.3 / pytest 9.0.2 environment, which is below the harness runtime minimum noted in the design package.
