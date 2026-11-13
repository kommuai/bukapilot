# Fresh G0 preflight — r7 MPP efficiency slice

**Observed:** 2026-10-30 18:14–18:20 UTC (fresh read-only checks after the user's readiness update)  
**Approved implementation spec:** `design/reports/implementation-spec_r7.md`, SHA-256 `a7f06c00f8c671aa8c6a980c75e1abe1ae6b62f11df8db5bb66651033bfcb347`  
**Candidate:** merged overlay binary `/tmp/ka2-efficiency-r7-mergecheck/system/loggerd/encoderd`, SHA-256 `c30b920d13b072d271122408822192e6e427b8ecac4ed289e3cecd64b7d37b47`, Build ID `a956b9960bf8e8ebb08b7e560570ff4c85449b1d`  
**Pinned superproject base:** `103d166829b3be04d735005ee83c24bd119764e2`

## Fresh board and idle state

The target is still a Firefly AIO-3588Q HDMI / RK3588, Ubuntu 24.04.4 LTS, Linux `5.10.198`, `aarch64`. `/dev/mpp_service`, `/dev/dri/renderD128`, `/dev/dri/renderD129`, and V4L2 nodes exist. The fresh process scan found no user-space camerad, encoderd, sensord, RKISP, MPP, or RGA workload; only non-recording loggerd deleter/uploader processes and kernel RGA IRQ workers were present. `fuser` reported no users of MPP, render, video, or V4L2 nodes. A first scripted guard conservatively matched the three kernel RGA IRQ workers and did not launch the probe; the corrected guard excludes empty-cmdline `irq/*` kernel threads and rechecked before the probe.

`MemAvailable` was about 2.03 GB. `CmaTotal` was 262,144 kB, while all five fresh samples reported `CmaFree=0` and both `/proc/zoneinfo` entries reported `nr_free_cma=0`. `/data` had about 1.8 GB free and `/data/media` about 5.8 GB free. `/tmp` had only about 16 MB free, so no A/B data or pytest temp root is planned there; generated output stays under the run's evidence tree or another bounded `/data` location.

## Driver and exact runtime MPP library

Kernel logs show `mpp_service` probe success and both RKVENC cores probe success. They also show IOMMU domain setup falling back to DMA domains and the `venc` regulator missing, which disables RKVENC devfreq. These are kernel-probe observations, not proof that full-resolution encoding and imported-buffer reads work.

The merged candidate's `ldd -r` resolves `librockchip_mpp.so.1` to `/lib/aarch64-linux-gnu/librockchip_mpp.so.1` → `/usr/lib/aarch64-linux-gnu/librockchip_mpp.so.0`, SHA-256 `ea762b7da71b63a73cdf14d053d84b091654aee5e5c4f26ceb1449b8fb8d92f9`, Build ID `b333b160bf1f80ad4f935d9794d2b8956b90bb14`. A separate `/usr/local/lib/librockchip_mpp.so.0` exists (SHA-256 `b2a64906d97585475a2f1130b7de3276ae712100772ef412cb991c7198c9fe20`, Build ID `665b598627076bffe4e6cedebc7c4baa73e5c739`). The runtime candidate resolves the older `/usr/lib` copy; neither library is dpkg-owned. The only installed `rk_mpi.h` is under `/usr/local/include/rockchip` (SHA-256 `96dbae7d90e91baaac38821fc06c74a327cac888007841fe5937798259193646`), so the header/runtime version skew remains explicit.

## Bounded non-encoding readiness probe

A small probe compiled against the installed public header and was explicitly linked to the exact library the candidate loads. `ldd -r` confirmed it resolves the same `/lib/.../librockchip_mpp.so.1`. With no competing user-space process or device-node user, it created and initialized one HEVC encoder context, immediately destroyed it, and submitted no frame: `mpp_create=0`, `mpp_init_encoder_hevc=0`, `mpp_destroy=0`, `frames_submitted=0`, exit 0. The probe took under 0.1 seconds. Pre/post checks found no retained MPP/device user and no material `MemAvailable` change.

This establishes that the loaded userspace library can initialize and destroy an HEVC MPP context on this image. It does **not** establish DMA-BUF import acceptance, frame encoding, decoded output, zero-copy use, or performance. The probe source, build log, and run log are `build/evidence/g0-mpp-init-probe.cc`, `g0-mpp-init-probe-build.log`, and `g0-mpp-init-probe-run-final.log`; the executable is also hash-bound at `build/evidence/g0-mpp-init-probe`.

## G0 disposition

Fresh idleness and device ownership checks passed, and the exact candidate runtime's MPP HEVC context init/destroy passed. Full A/B readiness is still conditional: CMA remains at zero, and the repository's ordinary synthetic `VisionIpcServer` allocates `/dev/shm/msgq_visionbuf_*` memfd buffers rather than DMA-BUFs, while production import uses `MPP_BUFFER_TYPE_EXT_DMA`. Therefore the next authorized step is only the saved one-buffer DMA-BUF import acceptance probe, using `/dev/dma_heap/system`, bounded to a single 1920×1200 NV12 buffer (<4 MiB allocation) and no encoded frames. A failed heap allocation, DMA-BUF identity check, or MPP import stops the experiment. No speed measurement has started.

Raw fresh snapshot: [g0-preflight-r7-fresh.log](../evidence/g0-preflight-r7-fresh.log), SHA-256 `93763696580543a7a57a79ef05bc626768dcb8e1b885b9263e4353a3bc2ba046`. The init-only probe plan is [report_mpp_import_probe_plan_r7.md](report_mpp_import_probe_plan_r7.md).
