# Runtime health assessment — r7 MPP copy fallback

**Run:** `loggerd-ka2-efficiency-20261029`  
**Target:** `/tmp/ka2-efficiency-r7/system/loggerd/encoderd`  
**Source revision:** `103d166829b3be04d735005ee83c24bd119764e2`  
**Binary SHA-256:** `655d3449384899c05f33ac98955a96671e1aae9fe1883b91946f98d416a44798`  
**ELF Build ID:** `f9eea4e181c7d0bfad3ddb5d41c5e9f1ff169723`

## Result

**Unavailable / unverified.** No runtime startup or readiness smoke was performed. The handoff explicitly forbids invoking `encoderd`/MPP, camera, sensors, or on-road paths. I did not launch the target or any dependent process. There is no executed runtime smoke log and no typed `runtime` probe record; this report must not be used as a passing runtime artifact.

## Evidence and dependency status

- The candidate binary was produced by the final SCons build. Read-only `readelf` and `ldd -r` inspection is recorded in [impl-binary-inspection.log](../evidence/impl-binary-inspection.log). It resolves `librockchip_mpp.so.1` to `/lib/aarch64-linux-gnu/librockchip_mpp.so.1` and reports no unresolved symbols. This establishes loader resolution only; it does not establish process startup, encoder readiness, or dependency reachability during use.
- G0 preflight records `CmaFree=0 kB` and `/proc/zoneinfo nr_free_cma=0`; an earlier non-encoding MPP API/version probe reported `client 12 driver is not ready` and `client 18 driver is not ready`. See [g0-preflight.md](../evidence/g0-preflight.md). Userspace MPP readiness is unproven, and the same-binary hardware A/B was marked unavailable.
- MPP runtime behavior, CMA availability during encode, camera/sensor readiness, and full-resolution/qRoad data flow remain **unexercised**. The alternate `/usr/local` MPP library is present, but the candidate currently resolves the older `/lib` copy.

## Probe effects and gate impact

No runtime probe was attempted, no process was started, no request or data was produced, and no teardown was needed. The isolated pytest helper test and static dependency inspection do not substitute for the required startup smoke.

**Severity: Major — build-to-review blocker.** The `runtime` key has no sanctioned fallback. The build package cannot advance to `build-to-review` until a safe, authorized runtime target is available and a real startup/readiness smoke passes. The current no-encoder boundary remains in force.

## Next action

Provide an authorized isolated runtime path that can start the candidate without camera, sensor, on-road, or live MPP hardware activity, or use a non-production board with suitable CMA headroom and proven MPP readiness. Repeat the preflight, then rerun this health check against that named target. Keep the hardware A/B separate until the MPP/CMA readiness conditions pass.
