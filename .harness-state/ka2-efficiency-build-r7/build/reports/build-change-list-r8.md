# Build change list — KA2 rotation liveness r8

**Owner:** build-management (Admiral)  
**Preamble tier:** Tier 2 — this is a multi-file implementation with delegated code/test/runtime work and board-backed synthetic DMA-BUF verification.  
**Approved design:** r8, `design/verdict_design-to-build-r8.json` plus cross-stage approval `design/verdict_design-to-build-r8-attempt-2.cross-stage.json`.

## Writable first-party paths

Production behavior:

- `system/loggerd/loggerd.cc` — deterministic-KA2 rotation deadline, epoch readiness, diagnostic report, writer/audio/queue finalization.
- `system/loggerd/encoderd.cc` — deterministic-KA2 per-output packet-progress monitoring, bounded recovery/backoff, and isolated segment drain handling.
- `system/loggerd/encoder/mpp_encoder.cc` and `.h` — actual-packet progress and pending-frame accounting while preserving the existing mandatory one-validate/one-import DMA-BUF path.
- `system/loggerd/encoder/mpp_encoder_input.h` — preserve the existing DMA-BUF validator; modify only if required by the accepted input contract.

Test infrastructure and regression tests:

- `system/loggerd/SConscript` — register the test-only DMA-BUF VisionIPC publisher if required for the fixture runner; no production target or runtime behavior changes.
- `system/loggerd/tests/visionipc_dmabuf_publisher.cc` and `test_dmabuf_fixture.py` — system DMA-heap-backed external VisionIPC buffers and fixture smoke.
- `system/loggerd/tests/test_loggerd.py` — replace the KA2 SHM skips by the DMA-BUF publisher fixture for the existing rotation and record-front tests.
- New `system/loggerd/tests/test_loggerd_rotation.cc` and `test_loggerd_rotation.py` — deterministic clock, readiness epoch, writer-finalization, segment queue, and bounded audio contracts.
- New `system/loggerd/tests/test_encoderd_recovery.cc` and `test_encoderd_recovery.py` — packet progress, per-output recovery/backoff, pending identity, and drain timeout contracts.
- New `system/loggerd/tests/test_loggerd_liveness.py` — isolated valid-DMA-BUF, rejected-input/qRoad, silent-output, drain-failure, diagnostics, cleanup, and legacy-mode lifecycle coverage.
- Existing in-scope `system/loggerd/tests/test_mpp_encoder.cc` and `.py` — maintain actual DMA-BUF validation coverage and no-copy invariants.

The existing working-tree no-copy edits in the MPP adapter and its focused test files are intentional inputs to this approved build and must be preserved. The final implementation record must hash their final contents and the scoped diff.

## Exclusions

- No changes to cereal schemas, file formats, route/segment authority, production camera allocation, V4L2 exporter code, or live camera settings.
- No full-resolution copy path, fallback, pointer retry, mode selector, or production test switch.
- No edits to `common/params_pyx.so` or `selfdrive/appbridged/appbridged.py`; these are unrelated pre-existing workspace changes.
- No changes to manager-owned processes or production `Params`, log roots, or routes during tests.
- No new external dependency or non-first-party/vendor/generated-file edits.

## Verification boundary

Run deterministic KA2 tests only with explicit test mode, synthetic `/dev/dma_heap/system` inputs, and unique temporary `LOG_ROOT`, `PARAMS_ROOT`, and `OPENPILOT_PREFIX`. Recheck camera/loggerd/encoderd/video-device/MPP occupancy immediately before every board test. Do not stop manager, uploader, or deleter. Live V4L2 DMA-BUF fences and camera buffer lifetime/reuse remain unqualified and must not be represented as verified.
