"""
DMonitoring model runner using RKNN (Python rknnlite). Inputs cast to float16 before inference.
Requires: dmonitoring_model.rknn in the model folder.
"""
from __future__ import annotations

import pickle
from pathlib import Path

import numpy as np

try:
  from rknnlite.api import RKNNLite
except ImportError:
  RKNNLite = None  # type: ignore


def _to_fp16(x: np.ndarray) -> np.ndarray:
  return x.astype(np.float16)


class DMonitoringRKNNRunner:
  """Runs dmonitoring model via RKNN. input_img and calib cast to float16 before inference."""

  def __init__(self, model_dir: Path):
    self.model_dir = Path(model_dir)
    meta_path = self.model_dir / "dmonitoring_model_metadata.pkl"
    rknn_path = self.model_dir / "dmonitoring_model.rknn"
    if not rknn_path.exists():
      raise FileNotFoundError(f"RKNN model not found: {rknn_path}")
    if RKNNLite is None:
      raise ImportError("rknnlite required (pip install rknn-toolkit2-lite)")

    with open(meta_path, "rb") as f:
      meta = pickle.load(f)
    self.input_shapes = meta["input_shapes"]
    self.output_slices = meta["output_slices"]
    # output_shapes is dict from ONNX output name(s); take first for single-output model
    self.output_shape = tuple(next(iter(meta["output_shapes"].values())))
    self.output_size = int(np.prod(self.output_shape))
    self.input_names = list(self.input_shapes.keys())

    self._rknn = RKNNLite(verbose=False)
    self._rknn.load_rknn(str(rknn_path))
    # Use NPU cores 0 and 1 (same as C++ path). modeld uses core 2 so cores are disjoint.
    core_mask_0_1 = getattr(RKNNLite, "NPU_CORE_0_1", 3)
    self._rknn.init_runtime(core_mask=core_mask_0_1)
    n_in = len(self.input_names)
    self._pass_through = [0] * n_in
    self._data_format = ["nchw"] * n_in

  def run(self, input_img: np.ndarray, calib: np.ndarray) -> np.ndarray:
    """input_img: uint8. calib: float32. Returns float32 output."""
    img_fp16 = _to_fp16(input_img.reshape(self.input_shapes["input_img"]))
    calib_fp16 = _to_fp16(calib.reshape(self.input_shapes["calib"]))
    inputs = [img_fp16, calib_fp16]
    outputs = self._rknn.inference(
      inputs=inputs,
      data_type="float16",
      inputs_pass_through=self._pass_through,
      data_format=self._data_format,
    )
    assert len(outputs) == 1
    out = outputs[0]
    if out.dtype != np.float32:
      out = out.astype(np.float32)
    return out.reshape(self.output_shape)
