# KA2 MPP input: no copy fallback

**Run:** `loggerd-ka2-efficiency-20261029`  
**Base:** `438acf545`  
**Decision:** D-28, intake revision 152  
**Scope:** KA2 MPP encoder input path and affected loggerd regression coverage.

## Change

Full-resolution MPP input now has one route: validate the fd with `DMA_BUF_IOCTL_SYNC`, make one `MPP_BUFFER_TYPE_EXT_DMA` import attempt, then submit the imported buffer. A failed check or import returns an encode error; it does not copy the frame. The encoder no longer reads `ENCODER_ZERO_COPY`, keeps no copy-mode state, and makes no pointer-attached import retry. Failed fds are cached as failed until encoder close, so the same invalid buffer is not probed on every frame.

Downscaled qRoad still resizes with RGA into its preallocated MPP buffer pool. That is qRoad's normal format conversion and remains required. The shared frame-submit and packet-drain code stays common to both input routes. Existing encoder reopen and segment-drain code remains in place.

## Evidence

- KA2 `encoderd` built with RK3588 flags and `-Werror`. The captured SCons result is [here](../evidence/encoder_no_fallback_scons_build.log); the source build completed immediately before the captured up-to-date check.
- The DMA-BUF validator rejects `/dev/null` and an invalid fd: **1 passed**. See [pytest log](../evidence/mpp_encoder_dma_buf_validation_pytest.log).
- A fresh board smoke used the current `MppEncoder` class with 24 DMA-heap inputs and no copy-mode switch. Road HEVC and qRoad H.264 each accepted and emitted 100 packets with zero failures. Road imported 24 fds; qRoad imported none. The road decoded-frame hash was `0ef875c179a3008b0f314e9701c55df473c3cdc5f92ca6471e8ae9eccdeb4761`, matching the earlier copy and zero-copy smoke outputs. The run reported road submit p95 5.836 ms, qRoad submit p95 5.658 ms, and peak RSS 29,368 KiB; this single run is a functional smoke, not a latency or memory comparison. See [smoke log](../evidence/mpp_encoder_no_fallback_smoke.log), [runner](../evidence/run_mpp_encoder_no_fallback_smoke.sh), and [harness](../evidence/mpp_encoder_no_fallback_smoke.cc).
- The existing KA2 loggerd rotation fixture creates ordinary shared-memory buffers. Against the no-copy code, full-resolution streams correctly rejected those fds, so no packets arrived to complete the fixture's all-encoder rotation handshake; its run stopped with zero post-boundary frames. I marked the KA2 rotation and `RecordFront` cases skipped until the fixture can provide DMA-BUF-backed VisionIPC buffers. The focused skip selection reports **4 skipped**; see [pytest log](../evidence/loggerd_shm_rotation_not_applicable_pytest.log). This leaves full encoderd rotation with real DMA-BUF inputs unverified in this revision.
- `git diff --check` passed. A source scan found no `ENCODER_ZERO_COPY`, copy selector, copy fallback, or pointer-attached retry in `system/loggerd/encoder` or its tests.

## Simplification

Line counts compare `HEAD` with the current worktree for these files:

| File | Before | After | Change |
|---|---:|---:|---:|
| `mpp_encoder.cc` | 583 | 524 | −59 |
| `mpp_encoder.h` | 75 | 63 | −12 |
| `mpp_encoder_input.h` | 61 | 27 | −34 |
| `test_mpp_encoder.cc` | 340 | 28 | −312 |
| `test_loggerd.py` | 501 | 509 | +8 |
| **Total** | **1,560** | **1,151** | **−409 (26.2%)** |

Product code across the first three files shrank by 105 lines (14.6%). The native helper test now covers the actual DMA-BUF gate rather than simulating copy/fallback state.

## Tradeoffs and limits

Invalid input fds and MPP import failures now lose that frame. If they persist, encoderd uses its existing failure counter and reopen policy. The board smoke shows the current system DMA-heap path imports and emits valid output, but it does not qualify the live camera V4L2 exporter, fences, or full loggerd segment rotation. No camera, sensor, or on-road capture was used.

The previous matched 10-launch qRoad result remains the known performance comparison: +0.288 ms qRoad p95 for the full-resolution zero-copy configuration, with 19.78 MiB lower road MPP pool allocation. This revision did not rerun that comparison. See [the qRoad tail and memory report](report_qroad_tail_memory_investigation.md).

No r8 design or build gate has been run. This report records implementation evidence and the remaining integration-test gap; it is not gate approval.
