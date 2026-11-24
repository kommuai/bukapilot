# Test Results — sidecamerad r2

## Outcome

**PASS.** The project-declared pytest runner completed the focused sidecamerad suite: 7 tests passed in 6.11 seconds. The final runner log is `evidence/tests-pytest.log` (SHA-256 `1eb3a3be2d74096c77cd5c14dd2757111dca4e2424deabbb1a0cc2e2938ffa82`); its typed probe record is `artifacts/tests_probe.json` (SHA-256 `183b259e34c2d3036ae9fee2dbfed5dd4746710c4b9aff8d5153de373cc006c6`).

## Runner discovery and command

`pyproject.toml` declares pytest in the test dependencies and configures the project pytest runner. `python3 -m pytest --version` reported pytest 9.0.2. The project inspection probe found `pyproject.toml`; its broader walk was incomplete at the repository byte limit, so runner selection relied on the declared project config. No packages were installed.

```text
python3 -m pytest -q system/manager/test/test_sidecamerad.py
exit code: 0
```

## Coverage

- **AC-1:** sidecamerad is enabled when `SIDECAMERAD` is unset and disabled only at `0`; the `camerad` enabled flag and `driverview` condition are unchanged.
- **AC-2:** all four missing-feed warnings, all-false Params initialization, no stream/publisher creation, and bounded 60-second idle waits ending through the registered SIGTERM handler (mocked event; no real 60-second sleep).
- **AC-3:** exact all-false Params default and Python dict round-trip; transition-only writes; recovery after a failed Params write; worker capture failure clears a previously available feed.
- **AC-5:** direct `/dev/videoN` and explicit by-path `video-index0` acceptance, `video-index1` rejection, rejection of an unsafe resolved target, and four service/stream/schema ordinal mappings.

The suite does not exercise live camera capture, a physical disconnect/reconnect, sibling-stream continuity, or four-camera cadence/CPU headroom. The runtime-health stage owns the one-camera end-to-end publication and five-minute cadence checks. AC-6 reconnect/sibling behavior and AC-8 remain unverified on this one-decoder host. The 22,118,400-byte four-stream allocation is verified by source review, not measured by this test suite.

## Prior run and coverage data

The first run exposed a test fixture setup omission: it seeded Params availability true but did not mark the mock worker as having published a frame. The fixture was corrected, and the first failure log is retained as `evidence/tests-pytest-initial-failure.log` (SHA-256 `2770ca81740ecb71c709c0c23fd7dc07d282e36e8fb3252662d6a08f1e571b8c`). The final rerun passed. No tests were quarantined. Coverage collection was not requested or run; no `.coverage*`, `htmlcov`, or `.nyc_output` files were left at project root.
