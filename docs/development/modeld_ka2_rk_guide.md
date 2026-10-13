# modeld on KA2/RK: guide (timing, bottlenecks, profiling, optimization)

Consolidated from: timing TICI vs RK, vision/policy why slow, inference bottlenecks, optimization plan, KA2 profiling, KA2 CL path changes.

---

## 1. Why TICI ~20 ms vs RK ~70 ms

- **Vision input path:** TICI uses zero-copy (`qcom_tensor_from_opencl_address`); RK historically did GPU→CPU→GPU (768 KB read + write). With shared context and `Tensor.from_blob(..., device='CL')`, zero-copy is now used on KA2.
- **Backend:** TICI uses `DEV=QCOM` (native Adreno/Mesa/kgsl); RK uses `DEV=CL` (generic OpenCL on Mali). Same graph, different codegen and driver; CL path has higher launch overhead and less tuning.
- **Sync/copy:** Two full queue syncs per frame (after vision and policy readback); policy inputs were copied CPU→CL each frame — mitigated by keeping policy on GPU and one-sync path (see below).
- **Queues:** Single shared command queue for C++ prepare and tinygrad is now used on KA2; prepare no longer does blocking `clFinish(q)` when using shared queue.

**Rough breakdown:** Input path and backend explain most of the gap; zero-copy + single queue + policy-on-CL reduce it. Remaining gap is generic CL vs QCOM tuning and hardware.

---

## 2. Remaining bottlenecks (after zero-copy and single queue)

1. **Vision run** — Bulk of time: many tinygrad CL kernels (conv/matmul) on Mali; generic schedules, no Winograd in image path.
2. **Vision/policy readback** — `.numpy()` implies sync + GPU→CPU; one sync per frame if vision+policy copyout is combined.
3. **Policy inputs** — If not kept on CL: ~200 KB CPU→CL each frame. With “policy on GPU” this is avoided.
4. **C++ prepare** — With single queue, finish is skipped; prepare time is small (~0.7–1.1 ms).
5. **JIT / kernel launch** — Many small kernel launches; mobile GPUs have high per-launch cost.
6. **Build flags** — `NOLOCALS=1`, `IMAGE=0/2` affect performance; IMAGE=0 was kept on KA2 (~45–50 ms vs ~59–63 ms with IMAGE=2). IMAGE=1 can crash (CL_INVALID_IMAGE_DESCRIPTOR).

---

## 3. How to measure (KA2)

- **MODELD_TIMING=1**  
  `MODELD_TIMING=1 python selfdrive/modeld/modeld.py --demo`  
  Logs every 100 frames: prepare, input_to_tensor, vision, policy, total; and kernel vs sync (vision_kernel, vision_numpy, policy_kernel, policy_numpy).

- **MODELD_SKIP_PREPARE_FINISH=1**  
  Benchmark-only; may be unsafe if queues differ. With single queue this is not needed.

- **MODELD_MEMORY_STATS=1**  
  Logs traced memory and RSS every 100 runs (for leak checks).

---

## 4. Zero-copy and single queue (done on KA2)

- **Zero-copy vision:** `set_external_cl_context(context_ptr, device_id_ptr, queue_ptr)` so tinygrad uses the same CL context/queue as C++. Vision inputs from `Tensor.from_blob(..., device='CL')` on the buffer from prepare(); tensors created once and reused.
- **Single queue:** C++ `ModelFrame` takes optional `shared_q`; when set, prepare() does not call `clFinish`. Tinygrad uses the same queue via `set_external_cl_context(..., queue_ptr)`.
- **Policy on GPU:** Policy inputs from CL (`from_blob`), one sync then copy both vision and policy outputs. Removes second sync and policy NPY→CL copy.

---

## 5. Tinygrad/SConscript flags (larch64)

- Default (kept): `DEV=CL FLOAT16=1 NOLOCALS=1 IMAGE=0 JIT_BATCH_SIZE=0`.
- **IMAGE=0** (buffer path for conv): ~45–50 ms total (IMAGE=2 was ~59–63 ms). **NOLOCALS=0** was tried and reverted (worse). Rebuild `.pkl` after changing flags in SConscript.

---

## 6. Optimization plan (maximal, for reference)

- **Prerequisite:** One shared OpenCL context (tinygrad’s context used by C++ via wrapper `CLContext.from_handle`).
- **Zero-copy:** External CL buffer support in allocator; vision tensors from `from_blob` and reuse.
- **Optional:** Event-based sync instead of global finish; keep policy inputs on CL; fuse or reduce sync points.

Files: `modeld.py` (init and run), `commonmodel_pyx.pyx` (CLContext), `tinygrad_repo/tinygrad/runtime/ops_cl.py` (external_ptr, set_external_cl_context).

---

## 7. References

- Why vision/policy are slow and speedups: see **modeld_mali_opencl_optimization.md**.
- RKNN path: see **modeld_rknn.md**.
