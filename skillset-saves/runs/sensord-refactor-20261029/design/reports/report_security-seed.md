---
submission_id: sensord-refactor-20261029-design-r4
revision: r4
---

# Security seed — KA2-only `system/sensord` cleanup

**Phase:** design  
**Return boundary:** `design-to-build`  
**Architecture:** revision `r4`, SHA-256 `a1a64320fd8a6b8995d08c5ba86d8c7a83f2f00630bdda1da0cf1f6981475b16`  
**Interface contract:** IF-1, SHA-256 `481ff241761b6b35d83837e74bbee78c4893b6800f03ba217b322fd9ac8b1763`  
**Confirmed intake:** D-20 in `../intake/report_grilling.md`, SHA-256 `a51313a34983497a27a5c9ddc5cdc0c9f82d725f6c8cdf382ab2dae2c75edd4e`; `../intake/intake-brief.md`, SHA-256 `c2e026d40ff71df11fd3d0277704255c51df061bbf3ffc5c7b81187513b9dc75`  
**Assessment owner:** `security-builder`

## Assessment

The r4 design introduces or moves no security trust boundary. The retained sensors continue to read local KA2 I2C devices through the existing `Sensor` and driver path on bus 3, and the existing cereal services and event payloads remain unchanged. The obsolete TICI/tizi device and GPIO 84 interrupt inputs are removed rather than replaced. No remote caller, credential, sensitive-data store, authorization surface, dependency execution, or privilege transition is added.

D-20 removes the r3 platform checks: the manager process registration uses its unconditional enabled default and retains `only_onroad`, and `sensord.main()` has no KA2 guard. This changes process eligibility outside the confirmed deployment target, but the platform constants and process-selection checks were not authentication or authorization boundaries. The change adds no new trust domain or data crossing. The build owes no additional security control for a new boundary; it must implement the accepted manager lifecycle and KA2 runtime contract without adding a platform guard or changing the existing local I2C and cereal paths.

There is an explicit residual scope risk: a non-KA2 managed or direct invocation is not rejected and may reach the fixed bus-3 path. D-20 confirms the deployment target as KA2 and deliberately removes these checks. Non-KA2 behavior is outside this design contract; this assessment does not claim that off-target execution is safe and does not reintroduce a guard. The existing `only_onroad` predicate remains required for managed startup.

This is a design-delta assessment, not an audit of the existing I2C drivers, manager process privileges, physical platform identity, or runtime deployment. No hardware or I2C was accessed. Physical sensor identity and target startup remain deferred.

## Typed `security_seed`

```yaml
applicable: false
scope: design-to-build
decided_by: security-builder
architecture_revision: r4
reason: "The r4 design keeps the existing local I2C and cereal boundaries, removes obsolete local sensor/IRQ inputs, and adds no authentication, privilege transition, external input, or data crossing. D-20 removes platform selection checks for the confirmed KA2 deployment; off-target access to bus 3 is an explicit scope risk, not a new trust boundary or a guard requirement."
boundaries: []
```
