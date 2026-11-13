# G0 preflight — KA2 MPP efficiency slice

**Observed:** 2026-10-30 16:56 UTC  
**Approved design:** r7 implementation spec SHA-256 `a7f06c00f8c671aa8c6a980c75e1abe1ae6b62f11df8db5bb66651033bfcb347`  
**Worktree:** `/tmp/ka2-efficiency-r7` at pinned HEAD `103d166829b3be04d735005ee83c24bd119764e2`

## Clean candidate and baseline binary

The detached worktree is at the pinned commit with clean superproject status. The target build used the exact pinned `rednose_repo`, `msgq_repo`, `opendbc_repo`, and `panda` gitlinks. The shared `/data/openpilot` checkout remains dirty and is not part of this candidate.

`OPENPILOT_BUILD_GUI=0 scons -j2 system/loggerd/encoderd` completed for the unchanged pinned source. SCons restored the objects and executable from its build cache, so this proves a target artifact is available for the pinned inputs but is not a fresh compiler run. No tests or encoder invocation were performed.

| Artifact | Evidence |
| --- | --- |
| Baseline executable | `/tmp/ka2-efficiency-r7/system/loggerd/encoderd` |
| SHA-256 | `a46f1c853094b8cf85cfd6533f874cb65047d6c99fd87e215aa0c762ba860b6d` |
| ELF Build ID | `482674b5359998e26e982f4d8ea1cfaa3ec3b2af` |
| MPP dependency | `librockchip_mpp.so.1`; no RPATH/RUNPATH |

`ldd` under the current environment resolves this uninvoked binary's MPP dependency to `/lib/aarch64-linux-gnu/librockchip_mpp.so.1`, which resolves to `/usr/lib/aarch64-linux-gnu/librockchip_mpp.so.0`. That file has SHA-256 `ea762b7da71b63a73cdf14d053d84b091654aee5e5c4f26ceb1449b8fb8d92f9`, ELF Build ID `b333b160bf1f80ad4f935d9794d2b8956b90bb14`, and reports missing VCS version metadata. `LD_LIBRARY_PATH` and `LD_PRELOAD` were unset. The alternative `/usr/local/lib/librockchip_mpp.so.0` is a distinct unowned build (SHA-256 `b2a64906d97585475a2f1130b7de3276ae712100772ef412cb991c7198c9fe20`, Build ID `665b598627076bffe4e6cedebc7c4baa73e5c739`, version `0986d01` dated 2026-08-26). `ldconfig` lists `/lib` before `/usr/local`; neither MPP library is owned by a Debian package. The candidate's runtime resolution must be checked again after the post-change build.

## Target and readiness

- Image: Firefly AIO-3588Q HDMI, RK3588; Ubuntu 24.04.4 LTS, Linux 5.10.198, aarch64.
- `/dev/mpp_service`, `/dev/dri/renderD128`, and `/dev/dri/renderD129` exist. The MPP platform driver is bound; boot logs report service, RKVENC, and RGA probes completed. IOMMU passthrough-domain setup fell back to DMA domains, and RKVENC devfreq setup reported a missing `venc` regulator.
- The latest idle check found no camera, encoder, sensor, RKISP, or MPP workload. `fuser` reported no open MPP or DRM render devices. The only loggerd-named processes were the deleter and uploader. The process scan also matched its own shell/`rg` command.
- Current resources: `MemAvailable=2,496,312 kB`, `MemFree=686,628 kB`; `CmaTotal=262,144 kB`, `CmaAllocated=6,676 kB`, `CmaFree=0 kB`, `/proc/zoneinfo nr_free_cma=0`. `/data` has 1.8 GB free; `/data/media` has 5.8 GB free.
- A prior non-encoding MPP version/API probe logged `client 12 driver is not ready` and `client 18 driver is not ready`. Kernel probe success therefore does not establish userspace encoder readiness.

## G0 disposition

The board was idle and ordinary RAM/storage headroom was available, but no CMA pages were free and userspace MPP readiness remains unproven. A hardware synthetic copy-vs-import A/B would require invoking MPP and could consume scarce DMA/CMA resources; **the bounded synthetic A/B is unavailable on this board state**. Do not run it or relax the preflight. Deterministic code-level fallback tests may proceed in isolation. Reconsider hardware A/B only after a fresh preflight shows suitable CMA headroom and the exact candidate binary's MPP runtime is ready; repeat the idle/device checks before every arm.

No code was changed and no encoder or encoder test was run during G0.
