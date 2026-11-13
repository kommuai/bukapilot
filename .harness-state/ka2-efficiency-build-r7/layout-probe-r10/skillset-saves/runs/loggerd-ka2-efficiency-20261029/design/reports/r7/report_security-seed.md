# Security seed — KA2 MPP efficiency, r7

**Decision:** no new or moved adversarial trust boundary is introduced in this scoped design (`applicable: false`). This is an assessed design determination, not a waiver and not a finding that the existing encoder or MPP stack is secure.

## Scope assessed

The scope is the isolated synthetic KA2/RK3588 A/B candidate using the existing `ENCODER_ZERO_COPY` MPP DMA-import path for full-resolution `road`, `wideRoad`, and `driver`, with immediate fallback to the existing copy path on import failure. `qRoad` remains unchanged and recording remains best-effort. Copy stays the default pending target-specific qualification.

The architecture and interface inventory show no new service, public method, message/schema, Params key, route format, Manager protocol, credential flow, or external input actor. The same encoder-to-MPP processing boundary already handles frame content in copy mode. The test harness supplies synthetic frames through isolated IPC, Params, and log roots. The candidate changes the allocation MPP reads and buffer lifetime/ownership handling, but not the trust domain in this test scope.

## Integrity and resource risks retained

DMA ownership and lifetime are not proven: VisionIPC may reuse its ring allocation before MPP has finished reading it. That can produce stale or corrupted frames. The saved architecture also reports that some MPP builds may emit zero-length HEVC packets. These are output-integrity and availability/resource risks; they do not, by themselves, establish a new adversarial trust boundary for this isolated synthetic test.

The design must keep copy mode as default, switch immediately to copy mode on import failure, and qualify the exact KA2 software image with changing synthetic frame patterns, decoded-content/output checks, per-stream loss and unusable-packet counts, and CPU/RSS/DMA measurements. A failed fallback, empty or undecodable output, or unacceptable output loss rejects the candidate. Testing cannot prove a general MPP completion bound or behavior on other images.

No dependency/security scan or implementation review was performed as part of this forward-looking seed. Reassess the boundary if zero-copy becomes production-default, attacker-controlled frame data is introduced, privilege or interface exposure changes, or the MPP/driver image changes.

## Typed `security_seed`

See [`security_seed.json`](../../artifacts/r7/security_seed.json). It records `applicable: false`, the exact r7 scope, the architecture revision/hash, the decision owner, and an empty `boundaries` list.
