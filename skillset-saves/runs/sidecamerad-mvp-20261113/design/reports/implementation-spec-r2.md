# Implementation Spec — sidecamerad default-on availability MVP

**Revision:** r2 · **Design package revision:** r2  
**Target stack:** Existing openpilot Python (`>=3.11,<3.13`), C++/Cap’n Proto/msgq, and the installed FFmpeg executable. No new runtime, framework, or package.

## Approved design inputs

| Input | Revision | SHA-256 |
|---|---:|---|
| Requirements brief | r2 | `bff3dc6825e257d653772d43d64a3d7f6221ed91bea0dff4c08ce50b82d7f748` |
| Architecture | r2 | `9f4cb41838303bd661048d62ddbcfea0ced765d4fc10ec228ef2c453e1a94991` |
| Interface contracts | r2 | `2ae1cfbe0746ae3a03df5a682e2dbe02d11337270809f3ace4dd220a6b0c05ad` |
| Local endpoint contracts | r2 | `d5e5b5b9a43658eca70ad26b8131b81f026355fe185b72ef1c06c77945101bb7` |
| Delivery plan | r2 | `0ef16ba626a8fb8cd117b710375d63d5d46e677cae4925abe2af919eb22eeffc` |
| Security seed | r2 | `6d780ee790d464047c360934cb0baef86131c129f01ea14a44707fb8b4b28652` |

The prior runtime scan was incomplete at its bounded walk limit. This slice keeps the existing supported runtime range and does not add or upgrade dependencies.

## Slice order

| # | Slice id | Depends on | Ships behind | Acceptance owner |
|---:|---|---|---|---|
| 1 | `slice.sidecamerad.default-on-availability` | No earlier source slice; implement the approved Params, cereal, VisionIPC, process, and worker contracts in this order. | Enabled under `driverview` when `SIDECAMERAD` is unset; `SIDECAMERAD=0` is the opt-out. | Side-camera feature owner |

AC-8 is a later four-camera performance sign-off gate, not a second source slice.

## Slice record

### `slice.sidecamerad.default-on-availability`

**Modules owned:** `common/params_keys.h`; `system/manager/process_config.py`; `system/sidecamerad.py`; `cereal/log.capnp`; `cereal/services.py`; `msgq_repo/msgq/visionipc/visionbuf.h`; `msgq_repo/msgq/visionipc/visionipc_server.cc`; `msgq_repo/msgq/visionipc/visionipc_pyx.pyx`; focused tests in `system/manager/test/test_sidecamerad.py` (new), `system/manager/test/test_manager.py`, `common/tests/test_params.py`, `cereal/messaging/tests/test_services.py`, and `msgq_repo/msgq/visionipc/tests/test_visionipc.py`. Keep `camerad`, generic `common/params_pyx.pyx`, and unrelated dirty files unchanged.

**Depends on:** No separate slice. Within S1, add and test the message/Params declarations first, register the default-on process second, then implement startup, idle, worker availability, and publication behavior against those contracts.

**Data / migration changes:** Add one Params key, `SideCameraAvailability`, in `common/params_keys.h` with type `JSON`, `CLEAR_ON_MANAGER_START`, and this compact C++ default literal:

```text
{"frontLeftCameraState":false,"frontRightCameraState":false,"backLeftCameraState":false,"backRightCameraState":false}
```

Manager initialization already clears keys with `CLEAR_ON_MANAGER_START` and writes declared defaults; the generic JSON binding converts the default to a Python dictionary. In `sidecamerad`, pass a Python `dict` directly to synchronous `Params.put()` and read it back as a `dict`; never pass a JSON text string to the Python Params API. Initialize all four fields to `False` and write that complete dictionary before starting workers. Guard the shared dictionary and transition writes with one lock. Write only when availability changes, not per frame. Set a field `True` only after the same full frame ID has completed both its VisionIPC and cereal sends; set it `False` on worker startup/read failure, timeout, decoder exit, disconnect, stop, and orderly process shutdown. This is a runtime status key, so no backfill or persistent-data migration is needed.

Append the four `FrameData` arms at ordinals 150–153 after `audioFeedback @149`, the four non-logged 20 Hz service declarations at the end of `cereal/services.py`, and VisionIPC stream IDs 7–10 in both C++ and Cython definitions. Preserve IDs 0–6, existing cereal ordinals, and native `camerad`. Appending service declarations is a minimal-ordering choice only: the enumerated index is unused and C++ lookup is by name; there is no numeric service-ID/ABI shift.

**Contract tests:** Before implementation, add failing cases for (1) unset/default-on and explicit `SIDECAMERAD=0` with unchanged `camerad`; (2) manager-start default and Python dict round-trip for the exact four-field Param; (3) no-path warning, all-false state, bounded idle wait, and SIGINT/SIGTERM exit; (4) each explicit side-to-path mapping, accepted `/dev/v4l/by-path/...-video-index0` and `/dev/videoN`, and rejected `video-index1`/invalid resolved targets; (5) one complete frame sent to both interfaces with equal frame IDs before availability becomes true; (6) transition to false on failure and back to true after successful retry; (7) service rates, stream IDs, ordinals, and preservation of prior registry values; and (8) worker-local close/reap/backoff and sibling continuity. Use mocks for worker tests so they do not require four devices. Keep image bytes out of Params and logs.

**Proof plan:** First land the failing unit/contract cases, then make the Params and cereal/VisionIPC declarations pass their focused checks, then implement manager registration and the module entry point, and finally implement the sidecar state transitions and worker lifecycle. Add `if __name__ == "__main__": main()` so the manager’s `python -m system.sidecamerad` launch invokes the daemon instead of exiting normally. The minimum green target is the single new focused suite plus the existing manager, Params, service, and VisionIPC tests. Refactor only within the owned modules while those contracts stay green. The one focused test command is:

```bash
pytest -q system/manager/test/test_sidecamerad.py system/manager/test/test_manager.py common/tests/test_params.py cereal/messaging/tests/test_services.py msgq_repo/msgq/visionipc/tests/test_visionipc.py
```

The build phase must also rebuild the existing bindings that include the changed declarations, for example `scons -j2 common/params_pyx.so msgq_repo/msgq/visionipc/visionipc_pyx.so`. These commands are planned verification, not runs performed during design.

**Rollback:** Set `SIDECAMERAD=0` and restart the manager to disable the sidecar while native `camerad` keeps its prior lifecycle. If the code slice itself must be withdrawn before adoption, revert this additive slice as one unit; no data compensation is needed. The manager will clear the availability key at its next startup and restore the declared all-false default.

**Acceptance evidence:** Close the slice against the planner’s AC-1–AC-7 rows below. The feature owner accepts S1. AC-8 remains a separate performance sign-off.

## Acceptance evidence mapping

| Plan criterion | Evidence required from build |
|---|---|
| AC-1 — default-on manager lifecycle | With `SIDECAMERAD` unset and `driverview` true, show `sidecamerad` starts; with value `0`, show it is disabled; verify the `camerad` entry and state are unchanged. |
| AC-2 — no configured paths | Start with all four `SIDECAM_*` variables absent. Show one missing-feed warning per service, exact all-false Params dictionary, no worker or VisionIPC buffers, process alive beyond one 60-second wait interval, then SIGTERM/SIGINT orderly shutdown. The main path must not return just because no cameras are configured. |
| AC-3 — availability contract | Read `SideCameraAvailability` as a Python dictionary before capture, after first dual publication, after controlled failure/disconnect, and after orderly shutdown. Prove exact keys/types, false→true→false transitions, same-frame IDs on both transports, and no per-frame Params writes. |
| AC-4 — one-camera capture and publication | With the attached image node explicitly assigned through its `video-index0` by-path, run five steady-state minutes of 1280×720 MJPEG at requested 20 fps. Capture at least 6,000 complete NV12 frames, 20.0 fps mean, no EOF-to-EOF interval above 60 ms, nonzero 1,382,400-byte payloads, and at least one matching cereal/VisionIPC frame ID. Rerun against r2; r1 capture is baseline only. |
| AC-5 — explicit map and compatibility | Exercise each position mapping where devices are available; show by-path index0 and `/dev/videoN` acceptance, metadata index1 rejection, IDs 7–10 and ordinals 150–153, service declarations appended at the end, and existing stream IDs 0–6/ordinals through 149 unchanged. Do not claim an ABI or numeric service-ID concern. |
| AC-6 — worker isolation and reconnect | Make one configured path unavailable or stop its decoder; show only that field becomes false, child closes/reaps, retries at 1/2/4/8/16/30 seconds maximum, siblings and native `camerad` continue, and a reconnected by-path feed returns to true after a complete dual publication. With only one decoder, record sibling/replug proof as pending. |
| AC-7 — resource envelope | Inspect four-worker maximum, one FFmpeg thread per worker, three VisionIPC buffers plus one reusable scratch frame per camera, and no BGR/app queue. Confirm 16 × 1,382,400 = 22,118,400 bytes (21.09 MiB) image payload; exclude FFmpeg internals, V4L2/kernel buffers, and other RSS. |
| AC-8 — four-camera performance sign-off | Pending four cameras and target hardware. Before claiming 20 fps per camera, run a 30-minute four-camera soak with at least 36,000 published frames per camera/144,000 total and record per-camera cadence, gaps, and CPU utilization/headroom. One-camera evidence cannot close AC-8. |

## Operational constraints

| Constraint | Slices affected | Owner | Removal criterion |
|---|---|---|---|
| Default-on behavior is scoped to the current `driverview` lifecycle; retain `SIDECAMERAD=0` as operational opt-out. | S1 | Side-camera feature owner | Change only through a later approved policy revision. |
| Empty configuration must keep the default-enabled process alive, warn once for each missing feed, and wait for SIGINT/SIGTERM in bounded intervals. | S1 | Side-camera feature owner | Permanent while the process is default-enabled and manager normal exits are not restarted. |
| Use explicit `SIDECAM_*` values; recommend physical-port by-path index0, retain `/dev/videoN`, reject metadata nodes, and never infer side from USB order. | S1 | Integrator | Reopen only after a labeled, stable discovery contract is approved. |
| `Params.put()` is synchronous; use one locked, four-field dictionary and write only initial state and transitions. Keep all pixels in ephemeral VisionIPC buffers. | S1 | Side-camera feature owner | Permanent for this interface. |
| Cap workers at four, one FFmpeg thread each, fixed buffer sizes and bounded retries; add no package, conversion copy, or application queue. | S1 | Build/security owner | Reopen only if measured target evidence requires a new architecture. |
| Check whether the target can safely enforce FFmpeg child CPU/memory limits; do not use unsafe `preexec_fn` or install a dependency. Record remaining exposure if no supported control is available. | S1 | Build/security owner | Resolve or record residual risk before build security sign-off. |

## Non-goals

- Four-camera cadence or CPU headroom is not proven by the current one-decoder evidence.
- No USB auto-discovery, persistent side assignment, or side inference.
- No changes to native `camerad`; no model, UI, loggerd, encoderd, network, or persistent-image integration.
- No BGR conversion, app frame queue, cross-camera synchronization, or camera transforms.

## Blocking decisions

| Decision | Owner | Blocks | Latest safe decision point |
|---|---|---|---|
| Four-camera sustained cadence and CPU headroom | Feature owner on target hardware | AC-8 performance sign-off only; not the one-camera MVP slice | Before any four-camera performance claim. |
| Target-supported FFmpeg child resource limits | Build/security owner | Security sign-off if no supported limit or residual-risk record exists | Before build-to-review. |
| Physical USB-port to camera-side assignment | Integrator | Correct per-side deployment mapping; code remains explicit and buildable | Before each path is assigned in deployment configuration. |

Return this spec to `design/commander` for package assembly; the commander owns the stack-lock record and design-to-build submission.
