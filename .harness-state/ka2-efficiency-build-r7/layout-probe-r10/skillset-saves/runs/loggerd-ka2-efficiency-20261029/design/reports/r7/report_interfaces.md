# Interface inventory — KA2 MPP efficiency, r7

**Decision:** no external interface change.

The experiment uses the existing ENCODER_ZERO_COPY environment switch in encoderd and the existing MPP buffer-import path. It does not change the VideoEncoder public method signatures, VisionIPC packet layout, cereal schemas, encoded-data service names, EncodeIndex fields, Params keys, route/file naming, Manager process protocol, or downstream consumers.

The scoped implementation may add private state and diagnostics inside MppEncoder to make import failure fall back immediately to the existing copy path. It must preserve the same input frame metadata and published encoded event shape. No new runtime dependency or framework is introduced.

The test-only invocation sets ENCODER_ZERO_COPY per arm. Production remains copy mode by default until the tested target passes the acceptance criteria. Any later change to default selection belongs in the same scoped design revision and must retain the environment override as rollback.

The affected paths are KA2/RK3588 MPP only. The qRoad encoder downsizes through RGA, so it remains on its current copy path. Other platform adapters and service contracts are outside this revision.
