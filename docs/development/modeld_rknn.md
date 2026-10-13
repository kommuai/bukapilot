# RKNN (NPU) path for modeld and dmonitoring

Consolidated from: RKNN driving, dmonitoring RKNN, NPU cores and crash, efficiency/correctness, RKNN vs tinygrad accuracy report.

---

## 1. Driving model (vision + policy)

- When `driving_vision.rknn` and `driving_policy.rknn` are in `selfdrive/modeld/models/`, **RKNN is the default** on KA2. Inputs cast to float16 before inference.
- **Use tinygrad instead:** `USE_RKNN=0` before starting modeld. Log: `using tinygrad driving runner (USE_RKNN=0)`.
- **Accuracy:** RKNN and tinygrad are expected to be similar within float16 tolerance. Verify with `USE_RKNN=0` or frame capture (`MODELD_CAPTURE_FRAMES`, `MODELD_CAPTURE_PATH`).

---

## 2. DMonitoring

- When `dmonitoring_model.rknn` is present, dmonitoringmodeld uses RKNN on **NPU cores 0 and 1**. modeld (driving) uses **core 2** — cores are disjoint.
- **C++ path (default):** `DMonitoringRKNNRunnerCpp` / `dmonitoring_rknnmodel.cc`; preallocated output, uint8→fp16 and calib→fp16 in C++. Fast path.
- **Python path:** Used only if C++ extension fails to import (slower). Ensure extension is built: `scons selfdrive/modeld/runners/dmonitoring_rknnmodel_pyx.so`.
- **Use tinygrad:** `USE_RKNN_DM=0`.  
- **Cores 0 and 1 vs 1+2:** If driver is old, it may not support cores 1+2; upgrade NPU driver (see below). See **modeld_rknn_npu_cores_and_crash** section.

---

## 3. NPU core assignment and multi-process crash

| Process            | Model(s)       | NPU cores   |
|--------------------|----------------|------------|
| modeld             | vision + policy| Core 2     |
| dmonitoringmodeld  | dmonitoring    | Cores 0 and 1 |

- **Driver:** Use **0.9.6** or newer (e.g. rknpu_driver_0.9.6_20240322) for RK3588/RK3588S. Check `deviceState.npuDriverVersion` or `cat /dev/shm/rknpu_drv_version` (after modeld/dmonitoringmodeld has run once).
- **Why “0 and 1” for dmon:** Old driver may not support `RKNN_NPU_CORE_1_2`; upgrade to 0.9.6+. Ensure C++ runner is used (log: “using C++ runner (NPU cores 1+2)” or equivalent).
- **Crashes with both modeld and dmonitoringmodeld:** Check dmesg for NPU/rknn errors; upgrade NPU driver; try staggered startup; confirm C++ path; check resource limits (OOM, file descriptors).

---

## 4. Efficiency and correctness (RKNN C++ path)

**Correctness:**  
- Vision: uint8 → /255 → float16; policy: float32 → float16; input order fixed (desire_pulse, traffic_convention, features_buffer). Output handling and parsing shared with tinygrad. Differences vs tinygrad from fp16/backend, not wrong pipeline.

**Efficiency:**  
- One NPU context per model; reused input buffers; **preallocated output** (is_prealloc=1, no extra memcpy after rknn_outputs_get). No redundant copies in C++; Python returns `.copy()` so caller can hold result across runs. **Unavoidable:** GPU→CPU read for vision input when using NPU.

Same efficiency ideas apply to dmonitoring C++ runner (cores 0+1, prealloc output, uint8/calib→fp16 in C++).

---

## 5. RKNN vs tinygrad accuracy (10 unique real frames)

- **Tolerance used:** rtol=0.01, atol=0.01.  
- **Vision:** max_diff ~12–17; all 10 frames failed strict tolerance (float16/quantization).  
- **Policy:** max_diff &lt;2; mean_abs_diff ~0.08–0.12; also above 1% / 0.01 threshold.  
- Uniqueness enforced by only saving frames whose (img, big_img) differed from all previous. For production, ensure .rknn models match tinygrad/ONNX and input convention (e.g. [0,1] after /255).

---

## 6. Capture and compare

- **Driving:** `MODELD_CAPTURE_FRAMES`, `MODELD_CAPTURE_PATH` for offline comparison.  
- **DMonitoring:** `DMONITORING_CAPTURE_FRAMES`, `DMONITORING_CAPTURE_PATH`; replay with driver camera for replay.
