# Build change list — KA2 rotation liveness r9

**Owner:** build-management (Admiral)  
**Preamble tier:** Tier 2 — multi-file runtime and test changes with isolated, board-backed DMA-BUF lifecycle verification.  
**Approved design:** r9, phase verdict `design/verdict_design-to-build-r9.json` SHA-256 `2cf3489fcd5191b78125f7886e4540fe95abb6f7682cd61bad276d0b4ed79c7b`; cross-stage verdict `design/verdict_design-to-build-r9.cross-stage.json` SHA-256 `ff3541635c6edff9e9539d20567516f436b20c6cfed610865e1b1053c694a7bc`.

## Writable first-party paths

Production behavior:

- `system/loggerd/loggerd.cc` — deterministic-KA2 timer-driven rotation, epoch-scoped readiness, exact diagnostic report, and segment-safe writer/audio/queue finalization.
- `system/loggerd/encoderd.cc` — deterministic-KA2 per-output packet-progress monitoring, bounded retry/backoff, fair segment drain, and output-local recovery.
- `system/loggerd/encoder/mpp_encoder.cc` and `.h` — actual-packet progress and pending-frame disposition while preserving mandatory one-validation/one-DMA-BUF-import behavior.
- `system/loggerd/encoder/mpp_encoder_input.h` — preserve the current DMA-BUF validator; modify only if an accepted contract requires it.

Test infrastructure and regression tests:

- `system/loggerd/SConscript` — test-only DMA-BUF publisher target; no production target or runtime switch.
- `system/loggerd/tests/visionipc_dmabuf_publisher.cc` and `test_dmabuf_fixture.py` — four-buffer system DMA-heap external VisionIPC fixture and transport/lifetime smoke. This M0 slice is already implemented and board-verified; preserve it.
- `system/loggerd/tests/test_loggerd.py` — replace KA2 skips that require camera/encoder rotation coverage with the DMA-BUF-backed fixture or delegate to the new valid-input lifecycle test.
- New `system/loggerd/tests/test_loggerd_rotation.cc` and `test_loggerd_rotation.py` — fake-monotonic deadline, readiness epoch, report schema, writer finalization, segment queue, and bounded audio contracts.
- New `system/loggerd/tests/test_encoderd_recovery.cc` and `test_encoderd_recovery.py` — packet progress, per-output recovery/backoff, pending identity, fair drain, and drain-timeout contracts.
- New `system/loggerd/tests/test_loggerd_liveness.py` — isolated valid-DMA-BUF rotation, rejected input/qRoad continuity, silent output, drain failure, diagnostics, cleanup, and legacy-mode lifecycle coverage.
- Existing in-scope `system/loggerd/tests/test_mpp_encoder.cc` and `.py` — preserve direct DMA-BUF validation and no-copy regression coverage.

The existing working-tree MPP no-copy edits and M0 DMA-BUF fixture edits are approved inputs to this build, not new changes to overwrite. Final implementation evidence must hash their resulting contents and the complete scoped diff.

## Exclusions

- No cereal schema, file format, route naming, segment authority keys, production camera allocation, V4L2 exporter, or live camera setting changes.
- No full-resolution copy path, pointer retry, mode selector, production fault switch, or alternate MPP import route. qRoad keeps its required RGA resize path.
- No edits to unrelated pre-existing `common/params_pyx.so` or `selfdrive/appbridged/appbridged.py` changes.
- No manager-owned process changes or production `Params`, log roots, routes, or live camera use during tests.
- No external dependency or non-first-party/vendor/generated-file edits.

## Verification boundary

Run deterministic KA2 lifecycle tests with `LOGGERD_TEST_DETERMINISTIC=1`, synthetic `/dev/dma_heap/system` inputs, and unique temporary `LOG_ROOT`, `PARAMS_ROOT`, and `OPENPILOT_PREFIX`. Recheck camera/loggerd/encoderd and video/MPP device occupancy immediately before each board test. Do not stop unrelated manager/uploader/deleter processes. The full loggerd+encoderd DMA-BUF rotation must be demonstrated before claiming this fixture gap closed. Live V4L2 DMA-BUF fences, exporter behavior, and camera-buffer reuse remain unqualified.
