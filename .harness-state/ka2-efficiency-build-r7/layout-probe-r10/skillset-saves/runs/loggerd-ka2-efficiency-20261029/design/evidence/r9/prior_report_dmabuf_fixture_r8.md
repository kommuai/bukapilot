# DMA-BUF VisionIPC Fixture — r8 M0

## Outcome

The test-only DMA-heap VisionIPC fixture built and passed twice on the pinned KA2 aarch64 board: once with pytest serial (`-n 0`) and once with xdist enabled (`-n 1`). The latter rerun proves the initial xdist crash is fixed. This qualifies external DMA-BUF registration, sending, client import and lifetime cleanup for synthetic loggerd rotation inputs; it does not qualify the production encoder route or live camera buffers.

The fixture uses four `/dev/dma_heap/system` NV12 buffers for each of Road, Driver and WideRoad at 1920×1200. It creates a fresh `OPENPILOT_PREFIX`, `LOG_ROOT` and `PARAMS_ROOT` before starting either process, and removes only its uniquely named MSGQ endpoints and socket.

## Commands and results

| Command | Result |
| --- | --- |
| `scons -j2 build/system/loggerd/tests/visionipc_dmabuf_publisher` | Exit 0; target available and up to date after the test's compile step. |
| `pytest -q -s -n 0 system/loggerd/tests/test_dmabuf_fixture.py::test_visionipc_dmabuf_fixture` | Exit 0; 1 passed. |
| `pytest -q -s -n 1 system/loggerd/tests/test_dmabuf_fixture.py::test_visionipc_dmabuf_fixture` | Exit 0; 1 passed with xdist enabled. |
| `python -m py_compile system/loggerd/tests/test_dmabuf_fixture.py` | Exit 0. |
| `git diff --check -- system/loggerd/SConscript system/loggerd/tests/test_dmabuf_fixture.py system/loggerd/tests/visionipc_dmabuf_publisher.cc` | Exit 0. |

Each pytest case performs a fresh SCons build check before launching the helper. The repository defaults to automatic xdist workers; the serial run isolates the board process lifecycle, and the `-n 1` run explicitly confirms the test also works under xdist.

Runner logs:

- `evidence/dmabuf_fixture_smoke_r8.log` — SHA-256 `9647414b0e4c517282f2f5de441547e5a72907aff0bb43395da757af2a95e98d`.
- `evidence/dmabuf_fixture_xdist_r8.log` — SHA-256 `480da392e472d4a7b0974ee4260a4a817a2f5390cb6ace087947283c1a0d3d86`.

## What the smoke proves

- All three named streams register four externally owned DMA-BUFs with the expected dimensions, stride and UV offset.
- The publisher brackets each CPU write with `DMA_BUF_IOCTL_SYNC`, sends with `sync=false`, and changes frame content across the buffer ring.
- The clients receive expected stream metadata, buffer indices, frame IDs, timestamps, and Y/U/V content at image boundaries. Received DMA-BUF identities match descriptors still owned by the publisher.
- Closing clients releases their received fds and imported mappings while the publisher retains the original buffers. On `STOP`, the helper joins the listener, releases its buffers, scans the process FD table for each captured DMA-BUF device/inode, and verifies each mapping is gone. The test also checks the isolated listener socket and removes the fixture's three MSGQ backing files.
- Immediately before each board smoke, the fixture confirms that camerad, encoderd and loggerd are absent and that the video, media, V4L-subdevice, MPP and RGA device nodes are unused. The manager-owned uploader and deleter were left running.

## Superseded diagnostic attempts

The first default pytest invocation used the repository's automatic xdist configuration. Its worker crashed while constructing `VisionIpcClient`: a unique `OPENPILOT_PREFIX` had no corresponding `/dev/shm/msgq_<prefix>` directory, which MSGQ expects the caller to create. The test now creates that fresh directory and removes only its three stream queues. The serial rerun passed, followed by the xdist-enabled `-n 1` rerun, also passing.

A process-wide fd-count assertion initially reported four eventfds and two epoll descriptors after helper teardown. The exact captured `/proc/self/fd` snapshot was `0=pipe:[21014836], 1=pipe:[21014837], 2=pipe:[21014838]` before server startup, and those same three entries plus `3=anon_inode:[eventfd], 16=anon_inode:[eventfd], 17=anon_inode:[eventpoll], 18=anon_inode:[eventfd], 19=anon_inode:[eventpoll], 20=anon_inode:[eventfd]` after teardown. A traced standalone helper run (`strace -f -e trace=eventfd,eventfd2,epoll_create1,epoll_create,close,openat ... visionipc_dmabuf_publisher --serve`, with `STOP` on stdin) showed listener TID 2970089 opening `/etc/protocols`, then creating eventfds 3, 16, 18 and 20 and epoll descriptors 17 and 19. These descriptors are in the publisher process, not the pytest parent. The listener's `LOGD` enters `common/swaglog.cc`, which lazily creates its process-global ZeroMQ logger context; those descriptors live for the helper process lifetime. The final cleanup check is scoped to fixture-owned DMA-BUF device/inode identities and mappings, and the test reaps the helper before asserting IPC cleanup. This preserves detection of DMA-BUF leaks without treating the logger's process-lifetime resources as fixture-owned.

`CEREAL_FAKE` was absent on the board; the fixture explicitly removes it from both helper and test environments so client construction always uses real MSGQ.

## Coverage boundary

This M0 proves the test fixture's DMA-heap-backed VisionIPC transport only. It does not start camerad, loggerd, encoderd, MPP or a live camera; it does not exercise V4L2-exported buffers, fences or buffer reuse. Full encoderd rotation with valid DMA-BUF input and the live exporter path remain unverified by this fixture and must be covered in the subsequent integration slice.

## Bound inputs

| Path | SHA-256 |
| --- | --- |
| `system/loggerd/SConscript` | `fd193f0302d79f3d0a8bc09c683bda3b1ac54c22bf1db9e8b56c3b4b21180f45` |
| `system/loggerd/tests/visionipc_dmabuf_publisher.cc` | `18bc9128b67b83f397ac32a9675cc3f37173c0a23cedbedc0c7f695738ab0443` |
| `system/loggerd/tests/test_dmabuf_fixture.py` | `02eb27a13c44c995433816c77a64ca30fbd6cf9307feefaf1211a9f3ce586449` |
