# Default-on zero-copy and MPP encoder simplification

**Run:** `loggerd-ka2-efficiency-20261029`  
**Base:** `438acf545`  
**User decisions:** default zero-copy for eligible full-resolution MPP inputs; recursively simplify the encoder while preserving frame output, fallback, and segment lifecycle.

## Result

The MPP encoder now tries zero-copy by default for full-resolution inputs. `ENCODER_ZERO_COPY=0` forces the copy path; downscaled streams such as qRoad remain on the RGA resize and MPP-pool path. A failed import copies and submits that same frame, then latches the encoder in copy mode.

The board rotation test exposed an unsafe input type: its synthetic `VisionIpcServer.create_buffers` path uses `visionbuf_cl.cc`, which backs buffers with ordinary shared-memory files. MPP accepted one of those file descriptors but produced almost no HEVC frames, preventing segment rotation. The encoder now checks `DMA_BUF_IOCTL_SYNC` before MPP import. A non-DMA-BUF fd immediately takes the same-frame copy fallback; valid DMA-BUF inputs still import. The default and forced-copy versions of the three-camera rotation test now both pass.

The real KA2 camera source already supports external DMA-BUFs: `system/camerad/camera/rk.cc` registers its V4L2-exported camera buffers with VisionIPC when `VIDIOC_EXPBUF` succeeds for every camera buffer, and otherwise uses its existing copy path. With this encoder default, camera-exported DMA-BUFs can use zero-copy; if the camera path supplies a generic shared-memory fd, the encoder falls back safely. The synthetic rotation test proves that fallback, while the direct DMA-heap smoke proves the import path. Neither substitutes for checking live V4L2 export and fence behavior, which was outside the authorized synthetic-only hardware tests.

The import and copy paths now share one frame-submit, queue, and packet-drain path in `mpp_encoder.cc`. H.264 and HEVC initialization share common MPP setup while keeping codec-specific settings. Dead buffer state and an impossible empty check on a fixed-size pool were removed. `encoderd.cc`'s segment, drain, and recovery control flow remains unchanged.

## Code size

| File | Before | After | Change |
|---|---:|---:|---:|
| `mpp_encoder.cc` | 583 | 575 | −8 |
| `mpp_encoder.h` | 75 | 64 | −11 |
| `mpp_encoder_input.h` | 61 | 22 | −39 |
| `test_mpp_encoder.cc` | 340 | 77 | −263 |
| `test_loggerd.py` | 501 | 505 | +4 |
| **Total** | **1,560** | **1,243** | **−317 (20.3%)** |

Product code alone is 58 lines shorter (8.1%). The small native helper tests cover default/override selection, import success, current-frame fallback after import failure, sticky copy mode, and copy failure. `test_loggerd.py` now runs the synthetic rotation case both with the default and with `ENCODER_ZERO_COPY=0`.

## Verification

- Focused helper test: **1 passed**. See [pytest log](../evidence/mpp_encoder_refactor_dma_guard_pytest.log).
- Full KA2 `encoderd` target rebuilt with RK3588 flags and `-Werror` — **passed**. See [final build log](../evidence/encoder_default_guard_final_build.log).
- Synthetic three-camera loggerd/encoderd rotation test, using the default and forced-copy modes: **2 passed**. It validates stream output IDs and segment mapping while using synthetic 320×240 VisionIPC inputs. The earlier default run without the DMA-BUF check failed at rotation; that retained transcript is [here](../evidence/encoder_default_and_copy_rotation_pytest.log). The guarded rerun is [here](../evidence/encoder_dma_guard_rotation_pytest.log).
- Rebuilt direct-class board smoke used 100 frames at 20 fps on full-resolution road HEVC and downscaled qRoad H.264. Default mode imported all 24 DMA-heap buffers; forced copy imported none. Both modes accepted and emitted 100 packets per stream with zero failures, and decoded road frame hashes matched (`0ef875c179a3008b0f314e9701c55df473c3cdc5f92ca6471e8ae9eccdeb4761`). See [default run](../evidence/mpp_encoder_zero_default_dma_guard_smoke-default.log), [copy run](../evidence/mpp_encoder_zero_default_dma_guard_smoke-copy.log), and [runner](../evidence/run_mpp_encoder_zero_default_dma_guard_smoke.sh).
- `git diff --check` and runner `bash -n` passed. After the hardware runs, no loggerd/encoderd/camera/sensor process, MPP/DMA device holder, or test MsgQ directory remained; `MemAvailable` was 1,897,708 KiB.

## Known tradeoff and limits

The earlier 10-launch qRoad comparison measured **+0.288 ms qRoad p95** while the full-resolution road encoder used zero-copy. Stage measurements placed the added time in qRoad's synchronous RGA resize. The DMA-heap direct-class test confirms equal decoded output but does not remove that cost. That measurement also found **19.78 MiB** less road MPP pool allocation and **20.18 MiB** more available memory with zero-copy; qRoad's 1.52 MiB pool remains. These memory and tail figures apply to the tested DMA-heap path, not to generic shared-memory inputs that fall back to copy. See [the qRoad tail and memory report](report_qroad_tail_memory_investigation.md).

The full segment test uses synthetic shared-memory VisionIPC frames, not live camera/ISP buffers, and the DMA-heap smoke uses a direct MPP class harness. No live camera, sensor, or on-road scenario was used. The test proves default fallback preserves synthetic three-stream rotation and proves zero-copy on the tested DMA heap; it does not prove the V4L2 camera exporter, MPP import, and camera/encoder fence lifetime together in production.

The previously approved r7 design specified copy-by-default; D-26/D-27 from the user's updated request supersede that default and authorize this refactor. No separate r8 design gate or build gate has been run, so this report does not claim gate approval.
