# Implementation report — sidecamerad default-on availability MVP

**Run:** `sidecamerad-mvp-20261113` · **Design:** r2 · **Build revision:** r2.1 implementation (local IPC hardening)

## Outcome

The r2 source delta is implemented. `sidecamerad` now starts under the existing `driverview` lifecycle when `SIDECAMERAD` is unset; only the explicit value `0` disables it. Native `camerad` is unchanged. An all-false `SideCameraAvailability` JSON Param tracks all four cereal camera-state services and changes only on availability transitions.

With no valid side-camera paths, startup warns once for each missing service, creates no workers or VisionIPC buffers, writes all four fields false, and waits for SIGINT/SIGTERM in bounded 60-second intervals. Configured capture paths accept `/dev/videoN` and `/dev/v4l/by-path/...-video-index0`; every retry resolves the path to a `/dev/videoN` character device before launching FFmpeg. `video-index1` is rejected. A worker sets availability true only after it sends the same complete frame ID through VisionIPC and cereal, sets it false on failure/stop, and uses the existing bounded retry loop. Params writes are guarded and transition-only; failed writes roll back the in-memory transition. Direct `python -m system.sidecamerad` launch is enabled.

At the start of `main()`, the sidecar clears inherited `ZMQ` before constructing Params, PubMaster, or VisionIPC objects. This keeps sidecamerad on the host-local IPC backend even when its parent manager environment enables the generic ZMQ backend. The final focused suite passes 8/8, including an explicit regression test that sets `ZMQ=1` and verifies it is absent before any IPC or PubMaster constructor runs.

## Changed artifact set

The r2 production delta is limited to three text files and the generated Params extension, alongside the focused first-party test file. The complete side-camera implementation diff also carries the earlier approved r1 cereal and msgq stream declarations needed by this r2 process:

| Path | SHA-256 | Status / role |
|---|---|---|
| `system/sidecamerad.py` | `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b` | New sidecar process and capture workers; removes inherited ZMQ before local IPC objects are constructed |
| `system/manager/test/test_sidecamerad.py` | `cf9fb49f514ac90458d47d825716a30b296b8cb25880167695c6a86a6660a059` | Eight focused cases for process configuration, Params, no-path lifecycle, ZMQ clearing, path validation, registry compatibility, and availability transitions |
| `system/manager/process_config.py` | `b7fdfda2878cae1fa6b354ed512bc33ad66f1abe1ed25c90ed980b547a8aa02d` | Default-on registration, `0` opt-out; native `camerad` line unchanged |
| `common/params_keys.h` | `87d6e7cc658daaac97f0aff94706cec6b6a545b3d2413d22fcec4b9d2577c035` | Adds `SideCameraAvailability` as JSON with `CLEAR_ON_MANAGER_START` and all-false default |
| `common/params_pyx.so` | `1ba74ad9f9831623e373dd3077102899e9efc49b3fee452b649795304d3c6722` | SCons-generated Params binding required for the new key; not hand-edited |
| `cereal/log.capnp` | `d5c0372d714e65461942874c91c522b205ce208261b382283839ed03b9731f64` | Carried r1 frame-state arms at ordinals 150–153 |
| `cereal/services.py` | `cacd4b749d9a0a3e19e3c466784dcf34925f263fdca3c26af72578c47d740a6b` | Carried r1 side-camera service declarations |
| `msgq_repo/msgq/visionipc/visionbuf.h` | `c971a1afa601f5bb8f560de6fb1dd78c2121cee61ea41c169f00a4154566e870` | Carried r1 stream IDs 7–10 |
| `msgq_repo/msgq/visionipc/visionipc_pyx.pyx` | `36247e353901eb80e52735c91f466151b3026e77152dde9ad9e3bcd682c66f32` | Carried r1 VisionIPC validity metadata forwarding |
| `msgq_repo/msgq/visionipc/visionipc_server.cc` | `36b383bfcaf10eaa69f6844a579c86204b4a07464a889208fe059a6c1202d198` | Carried r1 sidecamerad endpoint support |

The worktree already contained Nav changes in `common/params_keys.h`; they were preserved and are excluded from [the r2 implementation diff](../evidence/implementation-r2.diff). The generated Params binary is rebuilt from the full preserved header, so its hash reflects both the existing header contents and the new key. No unrelated `opendbc_repo`, hook-state, external-save, or media files were edited.

The image-payload allocation remains 22,118,400 bytes (21.09 MiB) at four cameras: three VisionIPC buffers plus one reusable 1,382,400-byte scratch frame per worker. The implementation adds no image queue, BGR conversion, or package dependency.

## Implementation-owned checks

| Check | Result | Evidence |
|---|---|---|
| Python source compile (`system/sidecamerad.py`) | Exit 0 | `evidence/impl-syntax-sidecamerad-final.log` · SHA-256 `db7860bcaf62fe6fa461b01e585c5e6b95931484265a1a7beea45de9590dbec2` |
| Workspace whitespace/diff check | Exit 0 | `evidence/impl-diff-check-final.log` · SHA-256 `454ec192458a24d5e77d622244462c695bcdf1cd1b6ec5e29588ce9697f13067` |
| Params binding/default probe in a temporary isolated Params root | Pass: compiled binding contains the key; default is the exact four-key all-false Python dictionary; Python dict put/get JSON round-trip passes | `evidence/impl-params-default-check.log` · SHA-256 `90a88c6fde650a715171000a3ae1d6904ba12674d6337950ed44a60928cb29c5` |
| Params binding build | SCons exit 0; artifact hash recorded | `evidence/native-binding-build-r2.log` · SHA-256 `4cfd63e34c733a66d6c0b4331b6804e7cbe3ab4274069eacd648b3fd49040c85` |
| Focused sidecamerad test suite | 8/8 passed | `evidence/tests-pytest.log` · SHA-256 `07e3923ff525579662c15880adb258bf83a877b54cf20c54f1bc03bed89ca21b`; typed probe `artifacts/tests_probe.json` · SHA-256 `1ec1a06d84ab1a33b427d2e631e94ed1c83c9a4fd1e8c07fc7deb63c15b13712` |

The final five-minute run used `ZMQ=1` in the sidecamerad child and passed functional publication: 6,000 matching cereal/VisionIPC frame IDs, no TCP listeners on ports 9007–9010, and clean shutdown. The strict AC-4 cadence check failed: mean 19.9993967 fps, maximum gap 123.185415 ms, and 743 of 5,999 intervals over 60 ms. Runtime log SHA-256 `899ff2ff4758ce4647b03d90436866a6ea5eebc95cbca65b8de57872b32930dd`; typed runtime probe `45d3e9c681702264d54db257ff96b4d54994dc56a64c23f0e567d9cb5cf5fcd7`; runtime report `8d19014eb89c81fe2f42df04e442e3e594e946e1c36836ccf2a5357cb7765d26`.

The supplemental no-path run with `ZMQ=1` stayed alive for 62.041 seconds, kept all four availability flags false, created no socket or TCP listener, and exited cleanly on SIGTERM. Log SHA-256 `e32c356b9440dcb86d0fa1cc09f1b9984c8c2c15cc16004265ee0384b6801971`.

Security verified Major `SEC-SIDEIPC-001` closed after confirming the final child run stayed on local IPC with `ZMQ=1`. Minor `SEC-FFMPEG-RESOURCE-001` remains open because this host has no available safe per-child CPU/memory limit. Security report SHA-256 `9285604f91b1e2af64229ed1b4591b0554be7bd70eaa7e1650322c027805b97e`; findings artifact SHA-256 `218259045da268b160ec7be24f60be302abbcb8558a049a2ea77dec846830688`.

AC-6 reconnect and sibling-continuity proof and AC-8 four-camera cadence/CPU sign-off remain pending because only one decoder is available. Four-camera headroom is not claimed.

The short, sequential FFmpeg filter probe drained two 20-second captures from the same device and checked complete nonzero NV12 frames. The existing identity-scale variant yielded 392 frames at 20.0159 fps mean, 71.52% FFmpeg CPU, 86.119 ms maximum gap, and 42 gaps over 60 ms. The output-pixfmt-only candidate yielded 395 frames at 19.9801 fps mean, 73.63% CPU, 82.256 ms maximum gap, and 80 gaps over 60 ms. The candidate showed no measurable improvement, so the production scale filter is retained. Both experimental FFmpeg processes needed SIGKILL after their 2-second SIGTERM cleanup wait expired; the measured frame windows were complete, while shutdown behavior is not claimed by this probe. Log: `evidence/ffmpeg-filter-ab-r2.log` · SHA-256 `abe3e708aa12eebb82dc0a05e2db58d0a93c5f9d02a77968ce239d094877`.

No placeholder or unowned follow-up marker remains in the implementation.
