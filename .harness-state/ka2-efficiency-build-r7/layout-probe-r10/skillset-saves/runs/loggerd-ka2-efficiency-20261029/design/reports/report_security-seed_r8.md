# Security seed — KA2 rotation liveness, r8

**Stage:** design `security-seed`  
**Return boundary:** `design-to-build`  
**Architecture:** `report_architecture_r8.md`, r8, SHA-256 `5b8ccd34e8015af00c9c81d785b424a015177551baa79d93af132531b5a3e177`  
**Decision:** assessed no trust-boundary movement (`applicable: false`, `boundaries: []`). This is a determination, not a waiver.

## Assessment

The corrected architecture applies the liveness, writer, audio, diagnostic, and recovery changes only while the deterministic KA2 protocol is active; non-KA2 and explicitly disabled deterministic runs retain their legacy behavior. Within that scope, the design retains the existing local camera/VisionIPC → encoderd → MPP input path and the existing encoded-message path into loggerd. It does not add a remote or user-controlled input source, credential, public endpoint, privilege transition, user-data flow, or cereal schema. Per-output supervision and recovery stay inside encoderd. LoggerdRotationReport is written only after a deterministic-KA2 runtime commit, is a local diagnostic, and is explicitly non-authoritative: the existing LoggerdRotateState, LoggerdSegment, and LoggerdSegmentEpoch remain the only rotation controls. The DMA-heap publisher and rejection/fault seams exist only in tests and must use isolated temporary roots.

The direct DMA-BUF import path still has meaningful integrity and availability risk. This r8 design does not establish that the live V4L2 exporter, MPP driver, synchronization fences, and DMA-BUF lifetime are safe on every image. The synthetic DMA-heap lifecycle case can qualify the tested fixture/image combination but cannot prove the production camera exporter contract.

## Controls the build must preserve

- Keep D-28's single route for full-resolution MPP input: validate the DMA-BUF fd, make one MPP import attempt, and drop a rejected frame. Do not copy rejected data, restore an environment selector, or attach a pointer for another import attempt. Keep qRoad's RGA resize as its normal downscaled path.
- Keep imported-fd and pending-frame ownership tied to MPP completion. Every input identity must be emitted once or retired once with an explicit drop reason. Empty or invalid packets must not leave an unbounded pending identity or cause a later packet to inherit the wrong frame ID.
- Bound each per-output pending queue, retry schedule, diagnostic payload, loggerd future-packet queue, and the segment-tagged qRoad audio queue. Apply the specified backoff and fail one output at a time; do not allow a persistent invalid fd or output stall to churn a healthy sibling.
- Compare every queued encode index with the authoritative committed segment before writing. Drop and count stale items, retain only correctly mapped future items, and never relabel a packet or audio sample into a different segment. Do not create placeholder media for a failed encoder.
- Construct diagnostic stream names from the static encoder inventory, validate them against that inventory, and never use packet-provided text or a diagnostic name to construct a filesystem path. Invalid diagnostic contents must not alter segment authority.
- In the test fixture, set `LOG_ROOT`, `PARAMS_ROOT`, and `OPENPILOT_PREFIX` before constructing Params or launching processes. The C++ publisher owns its DMA-heap fds and mappings until VisionIPC and encoderd stop, brackets CPU writes with DMA_BUF_IOCTL_SYNC, and calls `VisionIpcServer::send(..., sync=false)` to avoid the existing ION-specific sync method. The fixture must remain test-only; no production camera or route roots are used.
- Report unavailable streams and dropped inputs through bounded, rate-limited logs and the non-authoritative LoggerdRotationReport. Limit that report to the defined fields and configured stream names; it must not contain frame payloads, secrets, or filesystem paths.

## Residual risks and scope limits

- **Info:** DMA-heap end-to-end tests do not qualify live V4L2 exporter fencing or buffer reuse lifetime. Keep that limitation visible in build evidence; do not generalize synthetic success to an untested camera/driver image.
- **Info:** Whether RGA accepts the ordinary SHM fixture's mapped address is target-specific. If it does not, use the architected test-only per-output MPP rejection seam to prove sibling containment with a valid DMA-BUF source, while retaining a direct invalid-fd/SHM assertion that proves no-copy rejection.
- **Info:** A timed-out or rejected stream may have absent/partial media and counted audio loss after the bounded qRoad audio buffer expires. That is the documented best-effort failure contract; it must not be presented as a successful complete track.

This assessment is design-level only. It does not grade implementation controls, claim a clean security scan, or qualify the live camera path; those claims require build evidence and the later security review.
