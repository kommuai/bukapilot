---
submission_id: loggerd-ka2-efficiency-20261029-design-r10-mpp-segment-recovery-contract
revision: r10
---

# Stack lock — KA2 rotation liveness, r10

## Decision

```json
{
  "applicable": false,
  "reason": "no new runtime or framework - existing stack unchanged",
  "scope": "design-to-build",
  "decided_by": "commander"
}
```

The r10 architecture revises only internal loggerd/encoderd contracts and the diagnostic contents of an existing local Params key. It adds no runtime, framework, package, build dependency, cereal schema, or public interface. It continues to use the existing C++/KA2/Mpp/RGA/VisionIPC stack and proposes only a test-only C++ DMA-heap VisionIPC publisher using dependencies already present in the project.

## Detection evidence and limit

The earlier runtime detector used Python 3.13.15 and met its minimum, but a full repository scan and a focused `system/loggerd` scan stopped at the inspection total-byte limit before examining every manifest and stack. The partial empty stack list is not treated as a complete classification. The current host readiness probe also reports Python 3.12.3 below the harness minimum of 3.13; gate commands for this package are run explicitly with Python 3.13.15. Comparing the r10 architecture with the unchanged r9 stack decision confirms no runtime, framework, or package was added, so the existing unchanged-stack decision remains applicable. The detected C++ toolchain, MPP ABI, and DMA-heap behavior remain target-build observations, not locked version claims. If implementation needs a new runtime, framework, package, or build dependency, stop and return to Commander to resolve the stack before build.
