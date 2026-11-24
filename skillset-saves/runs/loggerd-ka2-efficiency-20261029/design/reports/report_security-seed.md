# Security seed — KA2 rotation liveness, r10

**Stage:** design `security-seed`  
**Return boundary:** `design-to-build`  
**Architecture:** `report_architecture_r10.md`, SHA-256 `c16611d69d733e0f4cc5e0011842497f9ec75fe5db4eb61fcbf70c9a7969d189`  
**Interfaces:** `report_interfaces_r10.md`, SHA-256 `eb46c44219eec740f2d0a9df2614716418a7f3d22888cb3675a024d9dc8622c4`  
**Typed interface set:** `interface_set_r10.json`, SHA-256 `aa031f73013890199ca60b2d57d786d6750a2a453aff3c13f21d39fefb846b8f`  
**Decision:** No trust-boundary movement was introduced (`applicable: false`, `boundaries: []`). This is an assessed determination, not a waiver.

## Assessment

The r10 correction distinguishes an ordinary deterministic-KA2 segment reset from a failure recovery. At a normal segment boundary, each healthy MPP output receives its planned EOS/drain/close/open. If the shared drain deadline expires, only outputs still pending are quarantined and their pending frame identities are retired once; segment advancement continues. A healthy qRoad sibling receives its one planned boundary reset, stays active, and must not receive an additional failure-recovery reopen or retry because another stream failed.

This changes local output availability and recovery behavior inside the existing loggerd/encoderd and MPP paths. It adds no external actor, credential, public endpoint, privilege transition, cereal schema, or IPC mechanism. Existing local Params remain the segment authority. The current DMA-BUF input boundary is unchanged: full-resolution MPP input must validate and import the DMA-BUF once, rejected frames are dropped, and no full-resolution copy fallback or pointer retry is allowed. qRoad keeps its existing RGA resize path.

The seven-field `LoggerdRotationReport` remains local diagnostic data. Its fields cannot set segment state, select paths, or carry frame payloads, secrets, or filesystem paths. Encoder-side MPP rejection, stalls, drain failures, pending-frame drops, and retries remain in per-output logs/counters and do not become loggerd report counts. The DMA-heap VisionIPC publisher and rejected-input/fault seams remain test-only, use isolated roots, and do not introduce a production control surface.

Therefore the r10 architecture introduces or moves no adversarial trust boundary. The typed record is `artifacts/security_seed_r10.json`.

## Controls the build must preserve

- Keep D-28's DMA-BUF-only full-resolution MPP input: validate the fd, make one MPP DMA import attempt, and drop rejected input. Do not add a copy path, environment selector, or pointer-based retry. Retain qRoad's existing RGA resize path.
- Keep the ordinary segment close/open separate from output failure recovery. At a shared 900 ms drain deadline, quarantine only still-pending outputs, retire each pending identity once, continue healthy siblings, and do not route one stream's local failure through an all-output recovery helper.
- Track planned segment transitions separately from failure recovery. For a healthy qRoad sibling at one ordinary boundary, require exactly one `segment_transition_reopen_count` and zero sibling-caused `failure_recovery_reopen_count` or `failure_retry_count`; keep qRoad active and verify valid packets for the new segment.
- Keep `LoggerdRotateState`, `LoggerdSegment`, and `LoggerdSegmentEpoch` as the only segment authority. Treat `LoggerdRotationReport` as diagnostic only, validate its schema and configured stream names, ignore stale-route reports, and never use report fields to control rotation or paths.
- Keep synthetic DMA-heap publishing and fault injection test-only. Set isolated `LOG_ROOT`, `PARAMS_ROOT`, and `OPENPILOT_PREFIX` before Params construction/process startup; retain and close DMA-BUF resources according to the fixture lifecycle.

## Residual risks and qualification limits

- **Info:** The deployed MPP version/build is not pinned. The local host's `MPP_ENC_SET_IDR_FRAME` declaration and upstream next-frame IDR documentation do not prove that an encoder context can resume safely after EOS. R10 therefore retains the planned close/open transition and does not qualify in-place EOS continuation.
- **Info:** The synthetic DMA-heap fixture does not qualify the live V4L2 exporter, its DMA-BUF fence/synchronization semantics, or camera-buffer reuse lifetime. Keep those limits visible; do not generalize synthetic success to production-camera behavior.
- **Info:** MPP behavior on live camera buffers, SHM-to-RGA compatibility, and downstream handling of absent or partial stream media remain unqualified. The security seed does not claim those behaviors are safe or tested.
- Reassess if the report is exposed beyond existing local Params readers, if diagnostics begin to control behavior, or if the design adds a new external input or privilege boundary.

This is design-level evidence only. It does not grade an implementation, claim a clean scan, or qualify the live camera/driver path.
