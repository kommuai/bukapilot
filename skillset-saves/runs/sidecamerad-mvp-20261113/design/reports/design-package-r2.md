---
revision: r2
submission_id: sidecamerad-mvp-20261113_design-revision-2
run_id: sidecamerad-mvp-20261113
---

# Design package — sidecamerad default-on availability MVP

**Boundary:** `design-to-build` · **Owner:** `commander` · **Revision:** `r2`

## Scope and outcome

Run `sidecamerad` by default under the existing `driverview` lifecycle while leaving native `camerad` registration and behavior unchanged. `SIDECAMERAD=0` remains an explicit opt-out. Keep the four position-to-device assignments explicit; recommend `/dev/v4l/by-path/...-video-index0`, accept `/dev/videoN`, reject the observed metadata-only `video-index1`, and never infer camera sides from USB enumeration order.

Add one `SideCameraAvailability` Params key: `JSON`, `CLEAR_ON_MANAGER_START`, with this compact C++ default literal:

```json
{"frontLeftCameraState":false,"frontRightCameraState":false,"backLeftCameraState":false,"backRightCameraState":false}
```

Python callers pass a dictionary directly to synchronous `Params.put()` and read a dictionary; no JSON text string goes through the Python binding. Manager startup initializes the declared all-false default, and sidecamerad writes all false before capture. Update the value only on availability transitions. Mark a feed true after the same complete frame ID has been sent to its VisionIPC stream and cereal state; mark it false on loss, failure, disconnect, or orderly shutdown.

With no configured paths, warn once per missing feed and keep the default-enabled process alive in bounded waits for SIGINT/SIGTERM, with all four fields false. Keep the existing 1280×720 MJPEG-to-NV12 path and requested 20 fps, fixed FFmpeg arguments, per-camera retry, and bounded buffer design. Append the four cereal service declarations at the end for minimal ordering only; the list index is unused and no numeric service-ID or ABI shift is claimed. Add no package, image queue, BGR conversion, remote listener, or persistent image sink.

## Design evidence

| Artifact | SHA-256 | Purpose |
|---|---|---|
| `../intake/report_grilling.md` | `84a7e874f86ba50a6e4c5b3e3938e4a534099e518c4dea8c9ba8a8f4f2d97a0b` | User decisions, rejected options, and deferred work |
| `../intake/report_scope-change.md` | `5ab1e0b6bc31e2158f8a9126a27d9f25ed132b776528f522ac7015ebcb1ffb39` | Latest scope and operating requirements |
| `../build/evidence/usb-camera-baseline.log` | `9629e9abadb286f4bd2d90f8d9649575c554ef184b648290331073573d6a59e2` | One-device V4L2 and prior dual-publication baseline |
| `reports/report_research-r2.md` | `da91b350598a06d66650ce8d945c3dae708c14e16c4ff211b596a61f98262901` | Research evidence index |
| `reports/requirements-brief-r2.md` | `bff3dc6825e257d653772d43d64a3d7f6221ed91bea0dff4c08ce50b82d7f748` | Requirements, constraints, hardware evidence, and limitations |
| `reports/report_architecture-r2.md` | `9f4cb41838303bd661048d62ddbcfea0ced765d4fc10ec228ef2c453e1a94991` | Process, capture, availability, and failure boundaries |
| `reports/report_interfaces-r2.md` | `2ae1cfbe0746ae3a03df5a682e2dbe02d11337270809f3ace4dd220a6b0c05ad` | IF-1 through IF-7 contracts |
| `reports/api-endpoint-contracts-r2.md` | `d5e5b5b9a43658eca70ad26b8131b81f026355fe185b72ef1c06c77945101bb7` | Local Params, cereal, VisionIPC, and configuration contracts |
| `reports/security-seed-r2.md` | `6d780ee790d464047c360934cb0baef86131c129f01ea14a44707fb8b4b28652` | Trust boundaries and controls owed by build |
| `reports/delivery-plan-r2.md` | `0ef16ba626a8fb8cd117b710375d63d5d46e677cae4925abe2af919eb22eeffc` | One build slice, AC-1–AC-8, rollout, and risks |
| `reports/implementation-spec-r2.md` | `447269713326cd2a99224014d2c145a5b49d9b13dd51eb145fac7193dd00126b` | Ordered implementation work, proof plan, rollback, and AC mapping |
| `reports/report_stack-lock-r2.md` | `f06b7157bf7c2b57f48473395098461bb1c9e3ef3db5415d0fe1dde4b51cff1d` | Existing stack retained; no packages |
| `artifacts/taste_snapshot-r2.md` | `7be5216f7b5b9d5bfb05417acf7886cbbd04618de09648495c468f6f848a0741` | Fresh read-only effective-profile result; both scopes absent at revision 0 |
| `packages/design-r1-archive.zip` | `b72d655619a6e09d5d63d97cc1e978eff2429908164aca66932c7af9def7186a` | Verified r1 manifest, artifacts, and design verdict lineage |

## Acceptance and limitations

The r2 build must verify default-on manager startup, unchanged native `camerad`, no-path all-false signal-wait behavior, exact Params schema and transitions, explicit path validation, and worker isolation. With the one attached decoder, AC-4 requires a five-minute V4L2 capture at 1280×720 MJPEG and requested 20 fps, at least 6,000 complete NV12 frames, 20.0 fps mean, no EOF-to-EOF interval above 60 ms, nonzero 1,382,400-byte payloads, and matching frame IDs in cereal and VisionIPC. R1's prior matching-frame capture is baseline evidence only; repeat this check on r2 code.

Only one USB decoder is attached in the current evidence. Four-camera cadence, CPU headroom, and disconnect/replug recovery have not been demonstrated. The 30-minute four-camera soak in AC-8 remains required before any four-camera performance sign-off. Target support for a hard FFmpeg child resource limit also remains a build/security verification item.

The r1 `design-to-build` cross-stage approval was `APPROVED` at SHA-256 `36b70ad825fe8f036460fb8d7eb994b40195d3d1661e548f09610d4d81275832`. That approval applies to r1 only; this r2 package awaits its own validation and gate.
