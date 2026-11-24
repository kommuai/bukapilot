# Implementation Spec — sidecamerad MVP

**Revision**: 18 · **Design package revision**: 18 · **Stack**: openpilot Python (`>=3.11,<3.13`), C++/Cap’n Proto/msgq; KA2/RK3588 target; existing FFmpeg. Runtime detection was inconclusive in this workspace (tool requires Python 3.13; full scan exceeded its byte limit); commander should confirm deployed runtime at stack-lock.

**Approved inputs**: architecture `reports/report_architecture.md` @ `da35a916fdf28e9ab1a5a5849f598c9783232026fb9d4e8e465d47033ee68ca5`; interfaces `reports/report_interfaces.md` @ `7b6299040d93ab96a875639f24656988ef3ac97f0643987c146faf4855a62755`; plan `reports/delivery-plan.md` @ `baf61b9ca8b384fc5eed94fad0a8d99b9bff13a65f02fc023386fd695648151e`; security seed @ `edf6e4c50d7d125a710a14b68182d32a3d929eec3a2cae6a496d9e30bf798a64`.

## Slice order

| # | Slice id | Depends on | Ships behind | Acceptance owner |
| --- | --- | --- | --- | --- |
| 1 | `slice.sidecamerad.opt-in-feed` | Root slice; approved IF-1–IF-5 | `SIDECAMERAD` unset/`0` by default; runs only at `1` | side-camera feature owner |

## Slice record

### `slice.sidecamerad.opt-in-feed`

**Modules owned**: `msgq_repo/msgq/visionipc/visionbuf.h`, `msgq_repo/msgq/visionipc/visionipc_server.cc`, `msgq_repo/msgq/visionipc/visionipc_pyx.pyx`, `cereal/log.capnp`, `cereal/services.py`, `system/manager/process_config.py`, new `system/sidecamerad.py`; focused tests in `system/tests/test_sidecamerad.py`, `cereal/messaging/tests/test_services.py`, and `msgq_repo/msgq/visionipc/tests/test_visionipc.py`.

**Depends on**: None between slices. Inside this slice: append cereal/VisionIPC contracts and the endpoint allowance → register the opt-in process → implement isolated workers and publication.

**Data / migration changes**: No persistent data or migration. Append `FrameData` arms at ordinals 150–153 after `audioFeedback @149`; add four 20 Hz services with `should_log=False`; append stream IDs 7–10 in both enum surfaces and allow `sidecamerad` in the ZMQ endpoint-name check. Preserve IDs 0–6 and ordinals through 149.

**Contract tests**: Prove IF-1–IF-5: gate/lifecycle without changing `camerad`; partial path configuration and fixed topic↔stream mapping; service rate and non-logging; stable IDs/ordinals and local/ZMQ endpoint resolution; validated path as one data argv with no shell; exact 1,382,400-byte frame acceptance; per-worker child reaping/backoff, sibling continuity, monotonic frame IDs and timestamps. No external API is added.

**Proof plan**: Add failing behavior tests first; minimum green is the contract additions, manager registration, then one isolated worker per configured path reading directly into a reusable NV12 frame and publishing via three VisionIPC buffers. Keep fixed FFmpeg argv, one FFmpeg thread, four-worker cap, exact-size reads, path validation, local-only IPC, no BGR/app queue/package. Refactor only owned files. Focused command: `pytest -q system/tests/test_sidecamerad.py cereal/messaging/tests/test_services.py msgq_repo/msgq/visionipc/tests/test_visionipc.py`; rebuild changed Cython/native interface with `scons msgq_repo/msgq/visionipc/visionipc_pyx.so`.

**Rollback**: Unset `SIDECAMERAD` or set it to `0` and restart manager. For code rollback before downstream adoption, revert this additive slice as one unit; no data compensation is needed. Native `camerad` remains under its current policy.

**Acceptance evidence**: Close against the approved plan IDs verbatim; the feature owner accepts each. AC-1: verify both flag states and unchanged `camerad`. AC-2: verify each/partial mapping, absent-worker silence, and preserved IDs/ordinals. AC-3: separate one-camera five-minute smoke: ≥6,000 published frames, 20.0 fps mean, no interval >60 ms. AC-4: separate 30-minute four-camera target soak: ≥36,000 frames per camera / 144,000 total plus cadence, gaps, CPU/headroom; no four-camera sign-off before pass. AC-5: verify `(3 shared + 1 scratch) × 4 × 1,382,400 = 22,118,400` bytes (21.09 MiB) image payload. AC-6: verify monotonic IDs/EOF and SOF exactly 50,000,000 ns before EOF, with no cross-camera sync claim. AC-7: inject one worker failure and observe close/reap, retries at 1, 2, 4, 8, 16, then ≤30 seconds, while sibling and `camerad` continue.

## Operational constraints

| Constraint | Owner / removal criterion |
| --- | --- |
| Keep opt-in default off; start with one configured path; rollback by unsetting the flag and restarting manager. Retain the flag until an owner-approved policy change. | Feature owner |
| Log per-worker lifecycle/failure/retry only; bound/drain FFmpeg diagnostics, never log frame bytes. No side-camera logging, encoder, or external sink. | Feature owner; permanent while supported |
| Four-camera performance and CPU headroom remain unmeasured until AC-4 passes. | Feature owner; gate removed only by AC-4 evidence |
| No enforceable child memory/CPU limit is established. Build/security must check for a target-supported limit; do not use unsafe `preexec_fn` or add a dependency. If unavailable, retain worker/thread/buffer caps and record residual decoder-resource exposure for owner/security review. | Build/security owner; residual acceptance before security sign-off |

## Non-goals

No changes to native `camerad`, model/UI/driving behavior, loggerd/encoderd, auto-discovery, persistent assignment, cross-camera sync/transforms, BGR conversion, app queue, new packages, network listener, or persistent image storage. No side-camera drive logging; services are `should_log=False`.

## Blocking decisions

| Decision | Owner | Blocks | Latest safe point |
| --- | --- | --- | --- |
| Whether target OS/runtime can enforce child resource limits without `preexec_fn` or a new dependency; otherwise record residual exposure. | Build/security owner; feature owner accepts residual | Coding may proceed; security acceptance needs the finding resolved or accepted | Before build-to-review/security sign-off |

Return to `design/commander` for stack-lock and design-to-build packaging.
