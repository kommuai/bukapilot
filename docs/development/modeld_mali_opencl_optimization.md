# Mali / OpenCL optimization for modeld

Consolidated from: 30 ms roadmap, KA2 kernel tuning log, per-kernel profile, hypothesis tests, reduction local override test, Mali vs Adreno, aggressive fusion. Also notes on removed features (CL_LOCAL_TUNE, MALI_KERNEL_PROFILE).

---

## 1. Why Mali “loses” to Adreno (TICI) in practice

- **Backend:** TICI runs native Adreno (Mesa IR3, kgsl); RK runs **generic OpenCL**. Same ops, different codegen and driver; no Mali-specific tuning.
- **Launch overhead:** Many small `clEnqueueNDRangeKernel` calls; Mali’s OpenCL driver has high per-launch cost. Command buffers are not used on Mali (queue poisoning).
- **Sync/copy:** Two full syncs per frame and policy CPU→CL copy — mitigated on KA2 by single queue and policy-on-GPU.
- **No Winograd in image path:** IMAGE=2 uses `image_conv2d` without Winograd; IMAGE=0 (buffer path) is used on KA2 for better latency.

So the gap is **backend and stack**, not raw silicon. Levers: Mali-tuned OpenCL (or ACL/Vulkan), fewer syncs/copies, fewer coarser kernel launches.

---

## 2. Roadmap: 57 ms → 30 ms (same model)

**Where time goes:** Vision kernel ~27–28 ms, policy kernel ~6 ms, cl_sync ~24–26 ms; host enqueue and copy/parse small. Inefficiency is **GPU compute**: generic conv/matmul, untuned work-group/local, many kernel launches.

**Tier 1 (highest impact):**  
(1) Profile hot kernels (MODELD_TIMING=1 and per-stage breakdown).  
(2) BEAM at capture (JITBEAM=1 or 2) for vision.  
(3) Winograd in image conv path (or Mali-only path).  
(4) Mali-specific schedules for top ~10 conv/matmul shapes.  
(5) Fusion of glue/elementwise ops to reduce dispatch count.

**Tier 2:** CL_BUILD_OPTIONS (e.g. `-cl-fast-relaxed-math -cl-mad-enable`), MALI_PROFILE=1 build, wider BEAM search (VISION_BEAM_LOCAL_MAX etc.).

**Tier 3:** Vulkan backend or RKNN/offload (different model/stack).

**Order of work:** Profile → BEAM at capture → fast-math → Mali heuristics → target hot shapes + fusion → Winograd in image path.

---

## 3. KA2 kernel tuning log (experiments)

- **Baseline (KA2):** Total ~60–64 ms; vision kernel ~27–28 ms, policy ~6 ms, cl_sync ~24–26 ms; copy/parse &lt;1 ms.
- **JIT_BATCH_SIZE** sweep (0, 4, 16): No meaningful win; default 0 kept.
- **NOLOCALS=0:** ~4 ms slower; reverted. **BEAM_LOCAL_MAX=512:** No benefit.
- **MALI_LOCAL_SMALL:** Negligible; reverted.
- **Timing instrumentation:** Kept in `ModelState.run` (vision_kernel_ms, policy_kernel_ms, cl_sync_ms, copy, parse).
- **dmonitoringmodeld:** JIT device mismatch fix (tensor device vs pkl DEV) so it runs on KA2.
- **Per-kernel profiling:** Was MALI_KERNEL_PROFILE=1; **removed** from tinygrad. Use MODELD_TIMING=1 for per-stage timing.

---

## 4. Per-kernel profile (historical)

- **Removed:** MALI_KERNEL_PROFILE and per-kernel et_ms logging. Use MODELD_TIMING=1 for kernel vs sync breakdown.
- **Historical result:** Mix of few heavy kernels and many small/medium; top shapes (e.g. r_32_64_4_4_192_4, r_32_192_4_4_64_4) good targets for BEAM or shape-specific schedules; ~70% of kernels &lt;0.2 ms → fusion helps.

---

## 5. Aggressive fusion

- **AGGRESSIVE_FUSION=1:** Relax buffer-inlining limit (AGGRESSIVE_FUSION_MAX_BUFS=6), iterate remove_bufferize to fixpoint, relax reduce-path ratio (AGGRESSIVE_FUSION_MIN_RATIO=2). No scons rebuild; schedule built at runtime on first run. Use **fresh process** when toggling.
- **Effect:** Kernel count reduced by ~2.7% (~5 fewer per frame). Caveat: more register pressure; if slower, try lower MAX_BUFS.

---

## 6. Hypothesis tests (code-level)

- Command buffer disabled for Mali; IMAGE path uses image_conv2d (no Winograd); CLGraph uses many kernel calls. MODELD_TIMING=1 validates vision-dominated, GPU-bound total. BEAM default 0 at capture. (Test file and CL_ENQUEUE_TIMING / MALI_KERNEL_PROFILE removed.)

---

## 7. Mali reduction local override (removed)

- **Removed:** MALI_REDUCTION_LOCAL_OVERRIDE. When it existed, override only applied when current local size had &lt;64 threads; on tested runs the hot kernels already had ≥64, so no change and ~0.66% difference was noise. Use MODELD_TIMING=1 for timing.

---

## 8. CL local tune (removed)

- **Removed:** CL_LOCAL_TUNE, CL_ENQUEUE_TIMING, MALI_REDUCTION_LOCAL_OVERRIDE. Use MODELD_TIMING=1 for high-level timing.
- **Historical:** CL_LOCAL_TUNE=1 could reduce GPU kernel time ~10.5% on some runs; on some Mali devices codegen defaults were better and tune could make total **worse** (+10–15%). Production did not enable it on KA2. Confirm 55 ms was device-dependent; no longer testable.

---

## 9. How to measure progress

- **MODELD_TIMING=1** — Per-stage and kernel vs sync in logs.
- **Regression** — After fast-math or schedule changes, compare model outputs vs baseline (e.g. fp16 tolerance).

---

## 10. JITBEAM (vision)

- Set **JITBEAM=1** in the **compile** environment (e.g. `JITBEAM=1 OPENPILOT_DEVICE=KA2 scons ... driving_vision_tinygrad.pkl`). BEAM is read at capture time. Clear vision pkl before rebuilding.
