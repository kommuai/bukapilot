---
submission_id: loggerd-ka2-efficiency-20261029-design-r8
revision: r8
---

# Stack lock — KA2 rotation liveness, r8

## Decision

```json
{
  "applicable": false,
  "reason": "no new runtime or framework - existing stack unchanged",
  "scope": "design-to-build",
  "decided_by": "commander"
}
```

The r8 architecture extends existing C++ loggerd/encoderd, MPP and VisionIPC code and proposes a C++ test-only publisher using existing build dependencies. It introduces no runtime, framework, package, or build-system change. The existing C++ toolchain, MPP ABI and DMA-heap behavior remain target-build observations; this decision does not claim their versions were fully detected.

## Detection evidence and limit

The project runtime detector was run with Python 3.13.15, meeting its minimum version. A full repository scan and a focused `system/loggerd` scan both stopped at the inspection total-byte limit before examining every manifest and stack. The partial empty stack list is not treated as a complete classification. The r7 design already locked the same unchanged-stack decision, and r8 adds no runtime/framework/dependency in the approved architecture. If implementation needs one, stop and return to Commander to resolve the stack before build.
