---
submission_id: loggerd-ka2-efficiency-20261029-design-r10-mpp-segment-recovery-contract
revision: r10
---

# Design package — KA2 rotation liveness and encoder fault containment, r10

## Outcome

This r10 package resolves the AC-6 conflict between ordinary MPP segment transitions and per-output failure containment. It keeps the planned EOS/drain/close/open at normal boundaries while making a drain failure local to the output that remains pending. qRoad may receive its one expected segment-transition reopen, but a sibling failure cannot add a failure-recovery reopen or retry for qRoad.

The objective remains D-29: keep deterministic-KA2 recording moving when a no-copy MPP input is rejected or an encoder stops publishing packets, while containing one encoder failure so healthy siblings continue. D-28 remains mandatory: full-resolution MPP validates and imports a DMA-BUF once; rejected input is dropped and counted with no full-resolution copy fallback, pointer retry, selector, or alternate input path. qRoad keeps its existing RGA resize path.

## MPP evidence and decision

The repository's KA2 SCons entry links the generic `rockchip_mpp` library name but does not pin an MPP release, commit, ABI, or deployed target build. The current encoder submits an EOS frame before draining; the ordinary segment-advance path then closes and opens each encoder context. The unversioned host header declares `MPP_ENC_SET_IDR_FRAME` as requesting an intra frame on the next input, but it does not identify the board's runtime MPP. The upstream developer guide documents that next-frame IDR command; it does not promise that an encoder context may safely resume after EOS. Its EOS/reset continuation text concerns the decoder API. Therefore r10 does not use force-IDR as evidence for post-EOS context reuse.

The architect's source observations are recorded in `reports/report_architecture_r10.md`. The relevant current file hashes are `system/loggerd/SConscript` `fd193f0302d79f3d0a8bc09c683bda3b1ac54c22bf1db9e8b56c3b4b21180f45`, `system/loggerd/encoderd.cc` `37e72f8678e0585cc26dc7b15301254166a8d7279c0a1a983ec56791f0a2d72f`, `system/loggerd/encoder/mpp_encoder.cc` `9d20be320c54d34dafa93b01517777bd8125a6cf2fdf98be34fd32b4b7574fa4`, and local host header `/usr/local/include/rockchip/rk_mpi_cmd.h` `4f35750f6ad5b18fdaee5b097f315c85e826037785340f0f7fa7cc72ae03cc3b`. The upstream reference is the [Rockchip MPP developer guide](https://github.com/rockchip-linux/mpp/blob/develop/doc/Rockchip_Developer_Guide_MPP_EN.md#L858-L872); its moving branch is contextual documentation, not a target-version pin.

## Selected segment and failure behavior

- At every ordinary deterministic-KA2 segment boundary, submit EOS for active outputs with pending work, drain outputs fairly in round-robin slices under one shared 900 ms monotonic deadline, then close/open each healthy output exactly once for the new authoritative segment. This is planned segment-transition work, not failure recovery.
- A poll slice that expires is interim. At the shared deadline, quarantine and close only outputs that still have pending frames, retire those frame identities exactly once, and continue segment advancement without setting process-wide `do_exit`.
- A healthy qRoad sibling receives exactly one `segment_transition_reopen_count` for the ordinary boundary, with no sibling-triggered delta to `failure_recovery_reopen_count` or `failure_retry_count`. qRoad remains active and emits valid packets tagged to the new segment. The timed-out output alone follows its own bounded retry schedule at the current authoritative segment.
- Camera/VisionIPC reconnect remains the all-output reset exception because producer fd generations change. Non-KA2 and explicitly disabled deterministic mode retain their prior recovery behavior.

This resolves D-32 and D-46 in the r10 delivery plan. AC-6 owns the executable proof: inject one output's drain timeout during an ordinary boundary and assert encoderd remains alive, only that output is quarantined, pending IDs are retired once, qRoad gets one planned transition reopen and no failure-recovery reopen/retry, and qRoad emits new-segment packets.

## Carried r9 behavior

R10 carries the approved r9 contracts unchanged outside AC-6 and its dependent engineering details: the deterministic-KA2 5,000 ms loggerd deadline and per-epoch readiness; the exact seven-field diagnostic `LoggerdRotationReport` and five loggerd-observed counters; bounded FIFO qRoad audio with atomic newest-event rejection at the 256 KiB cap; deterministic-KA2-only runtime writer finalization; startup writer-free behavior; isolated synthetic DMA-BUF VisionIPC lifecycle cases; and AC-12 legacy-mode compatibility. The full acceptance table is in `reports/delivery-plan_r10.md`, SHA-256 `8089708235dd82e888a08f53576c7973bc0544e0cfcf12f670ca004ab9277d70`.

## Security, stack, and Taste

Security-builder reassessed the exact r10 architecture/interface revision and found no new trust boundary. The typed seed is `artifacts/security_seed_r10.json`, SHA-256 `02ffba1aa0ebc6810e16d287619a7cea8cebb3eb895b07bad452cbe470d1e420`; its report is `reports/report_security-seed_r10.md`, SHA-256 `ae9dc0d52f30fa07e8a35da42d21e957bda14b83ca6932114cccee00de33da90`. Existing Params and DMA-BUF trust boundaries remain unchanged; diagnostics are local and non-authoritative; the test fixture and fault seams remain test-only and isolated.

No runtime or dependency was added. The existing C++/KA2/MPP/RGA/VisionIPC stack remains unchanged. The r9 Taste snapshot remains effective and unchanged; there is no user-facing surface.

## Prior history and open qualification limits

The complete approved r9 package and its phase/cross-stage verdicts are preserved in `packages/design-r9-approved-before-r10.tar.gz`, SHA-256 `06ca8b15925e51cbd447dea9e8a029e3dd00085a2015e3a6f6eac60c83ff5810`. The r9 manifest SHA-256 was `f6a7e61ea1e2fae2347beaaa3f123a2ae13aaa0d6c248eee8e91c1462a20bac1`; its phase verdict was `APPROVED` (`2cf3489fcd5191b78125f7886e4540fe95abb6f7682cd61bad276d0b4ed79c7b`) and its cross-stage verdict was `APPROVED` (`ff3541635c6edff9e9539d20567516f436b20c6cfed610865e1b1053c694a7bc`). R9 remains prior history and is not overwritten.

The deployed MPP version/build and safe post-EOS encoder-context continuation remain unqualified; r10 uses the conservative existing planned transition. Live RK3588 V4L2 DMA-BUF exporter behavior, synchronization/fences, MPP import against live camera buffers, and camera buffer lifetime/reuse also remain unqualified. The M0 synthetic DMA-heap VisionIPC fixture proves isolated transport and fixture-owned lifetime only, not the full MPP loggerd/encoderd lifecycle. SHM-to-RGA compatibility and downstream handling of absent or partial stream files remain open for build/release evidence.

No r10 product-source edits or product tests were performed as part of this design correction. The implementation change list remains paused until the r10 phase and cross-stage approvals are recorded.
