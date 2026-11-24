# Security checkpoint — sidecamerad r2.1

**Run:** `sidecamerad-mvp-20261113`  
**Phase / return boundary:** build / `build-to-review`  
**Scope:** approved r2.1 source, compiled msgq bindings, and the r2 security seed. This review was read-only; the reviewer changed no source, installed no packages, and did not open or stop the camera device.

## Verdict

The seeded trust boundaries are applicable. The earlier Major finding for conditional ZMQ exposure is **verified closed** for sidecamerad: the entry point removes inherited `ZMQ` before constructing Params, VisionIPC, or PubMaster, and the live five-minute capture ran with `ZMQ=1` in the child while using the local VisionIPC socket and opening no TCP listener on ports 9007–9010 at startup, during capture, before shutdown, or after shutdown. The socket was removed at orderly exit.

One Minor residual remains: the host provides no safe per-child CPU/memory cgroup limit, so FFmpeg's internal resource use is not hard-bounded. Dependency advisory scanning was unavailable and is recorded as `not-run`, not clean. A supplemental 62.041-second no-path run with child `ZMQ=1` also stayed all-false, created no socket/listener, and exited cleanly on SIGTERM.

## Findings

| ID | Severity / status | Owner / trigger | Evidence and disposition |
|---|---|---|---|
| `SEC-SIDEIPC-001` | **Major / verified** | Closed for the sidecamerad boundary in r2.1 | r2.1 calls `os.environ.pop("ZMQ", None)` at `system/sidecamerad.py:240-243`, before Params, VisionIPC, or PubMaster construction. Test `test_main_removes_inherited_zmq_before_ipc_and_pubmaster` passes with `ZMQ=1` and observes it absent at each constructor. The live probe `runtime-smoke-zmq-r2-final.log` (SHA-256 `899ff2ff4758ce4647b03d90436866a6ea5eebc95cbca65b8de57872b32930dd`) independently records child `ZMQ=1`, a connected local 1280×720 VisionIPC socket, no TCP listeners on 9007–9010 at startup/during/end, 6,000 matching cereal/VisionIPC frame IDs, all-false Param after shutdown, local socket removed, and exit 0. The supplemental no-path probe `runtime-no-path-zmq-r2-final.log` (SHA-256 `e32c356b9440dcb86d0fa1cc09f1b9984c8c2c15cc16004265ee0384b6801971`) held all four Param fields false, remained alive 62.041 seconds, logged one missing-feed warning per service, created no socket or TCP listener, then handled SIGTERM with exit 0. These runs close the violation of the security seed's `sidecar-to-local-ipc` control (§14 row and §23) and implementation-spec AC-2/AC-4/AC-5 for the sidecar. The generic ZMQ backend still binds wildcard TCP when used elsewhere; this fix confines sidecamerad to the host-local backend. |
| `SEC-FFMPEG-RESOURCE-001` | **Minor / open residual** | Owner: build/security owner. Reopen before AC-8/four-camera sign-off or production use to establish a target-supported child limit or record explicit residual-risk acceptance. | FFmpeg has fixed dimensions and one decoder/filter thread per configured feed; the sidecar has at most four workers and fixed application buffers. There is no hard child CPU or memory cap. The service cgroup exposes only `memory,pids`, has `memory.max=max`, no CPU controller, an empty `cgroup.subtree_control`, and is not writable by this process; `systemctl show` reports unlimited CPU quota and memory. `Popen` adds no resource limit. Malformed MJPEG can still consume host memory or CPU within the configured worker/thread count. The design explicitly requires this exposure to be recorded when a safe limit is unavailable; no unsafe `preexec_fn` or package install was attempted. |

The gate-typed findings record is [security_evidence.json](../artifacts/security_evidence.json). The environment limits and residual above are the evidence required by the implementation spec's resource-limit decision; four-camera headroom remains unproven.

## Control review

| Seeded boundary | Result | Evidence |
|---|---|---|
| `default-on-lifecycle` | **Verified in source and runtime.** Sidecamerad remains separate from native `camerad`, follows `driverview`, defaults enabled, and has the `SIDECAMERAD=0` opt-out. The no-path probe with child `ZMQ=1` stayed alive 62.041 seconds, kept all four fields false, logged exactly one missing-feed warning per service, created no socket/listener, and exited on SIGTERM. It creates workers only for configured paths; no-path mode writes all-false state, creates no streams/workers, and waits for shutdown. | `system/manager/process_config.py:73-75`; `system/sidecamerad.py:240-279`; focused tests in `build/reports/report_tests.md`. |
| `device-config` | **Verified with a bounded limitation.** Only `/dev/videoN` and the safe by-path `video-index0` form pass the regex. Each retry resolves the path, requires `/dev/videoN`, and checks character-device type. This rejects escaping/by-path index1 targets and revalidates reappearing by-path links. The original path is passed to FFmpeg for stable-port retry behavior. Python does not issue a V4L2 capability ioctl or pin the checked node by file descriptor; validation relies on the fixed V4L2 decoder open/read path before any complete frame is published. A path replacement between `realpath/stat` and FFmpeg open remains a TOCTOU window bounded by the root-managed `/dev` namespace and intended local-device trust boundary; no path escape was observed. | `system/sidecamerad.py:29-57,112-126,164-202`; AC-5 path tests in `build/reports/report_tests.md`. |
| `peripheral-to-decoder` | **Partially verified.** Input is decoded in a child with fixed 1280×720 MJPEG→NV12 mode, one thread, fixed-size output, and only complete 1,382,400-byte frames published. Startup and frame-stall deadlines trigger cleanup/retry. They do not cap FFmpeg's internal CPU or memory; see `SEC-FFMPEG-RESOURCE-001`. Four-camera resource headroom and reconnect remain pending. | `system/sidecamerad.py:17-25,115-126,164-202,204-237`; implementation spec AC-7/AC-8. |
| `sidecar-to-ffmpeg-process` | **Verified in source.** The executable and codec/filter/output arguments are constants; the operator path is one argv element; the `Popen` call does not enable a shell. stdin is `/dev/null`, stderr is discarded, and `close_fds=True`. Exit/EOF/stall causes terminate, timed wait, kill if needed, and reap. Logs contain bounded lifecycle messages and the configured path, never frame bytes or FFmpeg stderr. The child inherits the manager identity; sidecamerad does not change identity or request additional privileges. | `system/sidecamerad.py:27,112-147,209-237`. |
| `sidecar-to-local-status` | **Verified.** `SideCameraAvailability` is a `CLEAR_ON_MANAGER_START` JSON key with exactly four boolean fields and an all-false default. Writes pass a Python dictionary, are lock-protected and transition-only after initialization; a field becomes true only after both same-frame sends. No image or device path is stored in this Param. Orderly shutdown resets availability; an uncatchable kill may leave the last value until manager initialization, as the seed allows. | `common/params_keys.h:167-168`; `system/sidecamerad.py:60-94,150-162,248-288`; live Param reset in runtime log. |
| `sidecar-to-local-ipc` | **Verified for sidecamerad in r2.1.** The normal MSGQ branch uses the local `/tmp/visionipc_sidecamerad` endpoint. When launched with inherited `ZMQ=1`, sidecamerad removes it before creating the IPC publishers; live capture confirms local VisionIPC delivery and no routable side-camera listeners. The generic backend remains capable of wildcard TCP binding for other processes. | `system/sidecamerad.py:240-243,270-275`; `runtime-smoke-zmq-r2-final.log`; generic path at `msgq_repo/msgq/ipc.cc:10-25`, `msgq_repo/msgq/visionipc/visionipc_server.cc:19-34`, `msgq_repo/msgq/impl_zmq.cc:116-132`. |

## Runtime and coverage limits

The final five-minute live probe proved dual publication and the local-only ZMQ defense: 6,000 frames were received on each transport with matching IDs, and the local socket disappeared after exit. The log labels `DUAL_PUBLICATION_ACCEPTANCE=PASS` but `CADENCE_ACCEPTANCE=FAIL`: measured mean was `19.999396661241942` fps, with 743 EOF intervals over 60 ms and a maximum interval of `123.185415` ms. This cadence failure is a runtime acceptance issue, not a security finding; AC-4 is not fully closed by this probe.

Only one decoder was available. AC-6 disconnect/reconnect and sibling continuity remain unverified, and AC-8's 30-minute four-camera soak/CPU headroom sign-off remains pending. The source-reviewed four-camera application image allocation is 22,118,400 bytes; FFmpeg internals, V4L2/kernel buffers, and total RSS are excluded.

## Build configuration and scan coverage

- Compiled `msgq/ipc_pyx.so` and `msgq/visionipc/visionipc_pyx.so` link `libzmq.so.5`; the generic ZMQ backend is present. The live probe set `ZMQ=1` only in the sidecamerad child and verified sidecamerad stayed on local IPC.
- `osv-scanner`, `trivy`, `grype`, `pip-audit`, `safety`, and `semgrep` were unavailable; no packages were installed. The typed record [security-scan.json](../evidence/security-scan.json) is `not-run`, not a clean scan; it binds `uv.lock` and `msgq_repo/pyproject.toml` by SHA-256. `/usr/bin/ffmpeg` reports `6.1.1-3ubuntu5+git20240717.8164ff7d~noble`, but `dpkg-query` does not map the binary to an installed package, so it and its runtime libraries were not advisory-scanned.
- The focused r2.1 suite passed 8 tests. Final test log SHA-256: `07e3923ff525579662c15880adb258bf83a877b54cf20c54f1bc03bed89ca21b`; typed probe SHA-256: `1ec1a06d84ab1a33b427d2e631e94ed1c83c9a4fd1e8c07fc7deb63c15b13712`.

## Source/evidence fingerprints

| Surface | SHA-256 |
|---|---|
| Approved security seed r2 | `6d780ee790d464047c360934cb0baef86131c129f01ea14a44707fb8b4b28652` |
| Implementation spec r2 | `447269713326cd2a99224014d2c145a5b49d9b13dd51eb145fac7193dd00126b` |
| `system/sidecamerad.py` r2.1 | `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b` |
| `system/manager/test/test_sidecamerad.py` | `cf9fb49f514ac90458d47d825716a30b296b8cb25880167695c6a86a6660a059` |
| `system/manager/process_config.py` | `b7fdfda2878cae1fa6b354ed512bc33ad66f1abe1ed25c90ed980b547a8aa02d` |
| `common/params_keys.h` | `87d6e7cc658daaac97f0aff94706cec6b6a545b3d2413d22fcec4b9d2577c035` |
| `msgq_repo/msgq/ipc.cc` | `cf7970c949f02b60ea6bfd4e4fdaf5f36f6b0c270d4ccba2707ffe7adb8ab64a` |
| `msgq_repo/msgq/impl_zmq.cc` | `25395c7141c33a3828009aad53b9f27a600b9593862795e0ecf07b898ff52bec` |
| `msgq_repo/msgq/visionipc/visionipc_server.cc` | `36b383bfcaf10eaa69f6844a579c86204b4a07464a889208fe059a6c1202d198` |
| Compiled `msgq/ipc_pyx.so` | `037f0afcb5ddeaf41e2c1e24d39fb7bbc4027f2cffc5fe287209ee95c7cfb933` |
| Compiled `msgq/visionipc/visionipc_pyx.so` | `361d6587044e71fb0e9d2c8f2ad63a3d940e8444adde71a5350a1353f5da2e35` |
| `evidence/tests-pytest.log` | `07e3923ff525579662c15880adb258bf83a877b54cf20c54f1bc03bed89ca21b` |
| `evidence/runtime-smoke-zmq-r2-final.log` | `899ff2ff4758ce4647b03d90436866a6ea5eebc95cbca65b8de57872b32930dd` |
| `evidence/runtime-no-path-zmq-r2-final.log` | `e32c356b9440dcb86d0fa1cc09f1b9984c8c2c15cc16004265ee0384b6801971` |
| `evidence/security-scan.json` | `077684993c9cc2bc57ec7619773b1278f9cde319f6d1e06a99faddb49a16d4a5` |
