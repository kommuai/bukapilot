"""
Driving model runner using RKNN (vision + policy). All inputs are cast to float16 before inference.
Requires: driving_vision.rknn, driving_policy.rknn in the model folder and rknnlite.
"""
from __future__ import annotations

import os
import pickle
from pathlib import Path

import numpy as np

try:
  from rknnlite.api import RKNNLite
except ImportError:
  RKNNLite = None  # type: ignore


def _to_fp16(x: np.ndarray) -> np.ndarray:
  """Cast to float16 without changing value range."""
  return x.astype(np.float16)


class DrivingRKNNRunner:
  """Runs driving vision and policy models via RKNN. Inputs are cast to float16 before inference."""

  def __init__(self, model_dir: Path):
    self.model_dir = Path(model_dir)
    vision_meta_path = self.model_dir / "driving_vision_metadata.pkl"
    policy_meta_path = self.model_dir / "driving_policy_metadata.pkl"
    vision_rknn_path = self.model_dir / "driving_vision.rknn"
    policy_rknn_path = self.model_dir / "driving_policy.rknn"

    if not vision_rknn_path.exists() or not policy_rknn_path.exists():
      raise FileNotFoundError(
        f"RKNN models not found: need {vision_rknn_path.name} and {policy_rknn_path.name}"
      )
    if RKNNLite is None:
      raise ImportError("rknnlite is required for RKNN driving runner (pip install rknn-toolkit2-lite)")

    with open(vision_meta_path, "rb") as f:
      vision_meta = pickle.load(f)
    with open(policy_meta_path, "rb") as f:
      policy_meta = pickle.load(f)

    self.vision_input_shapes = vision_meta["input_shapes"]
    self.vision_output_slices = vision_meta["output_slices"]
    self.vision_output_shape = vision_meta["output_shapes"]["outputs"]
    self.vision_input_names = list(self.vision_input_shapes.keys())  # e.g. ['img', 'big_img']

    self.policy_input_shapes = policy_meta["input_shapes"]
    self.policy_output_slices = policy_meta["output_slices"]
    self.policy_output_shape = policy_meta["output_shapes"]["outputs"]
    self.policy_input_names = list(self.policy_input_shapes.keys())
    # Output sizes (flat)
    self.vision_output_size = int(np.prod(self.vision_output_shape))
    self.policy_output_size = int(np.prod(self.policy_output_shape))

    # Vision RKNN
    self._vision_rknn = RKNNLite(verbose=False)
    self._vision_rknn.load_rknn(str(vision_rknn_path))
    self._vision_rknn.init_runtime()
    n_vision_in = len(self.vision_input_names)
    vision_pt = int(os.getenv("RKNN_PY_VISION_PT", "0"))
    vision_fmt = os.getenv("RKNN_PY_VISION_FORMAT", "nchw").lower()
    vision_layout = os.getenv("RKNN_PY_VISION_LAYOUT", "nhwc").lower()
    if vision_layout not in ("nchw", "nhwc"):
      vision_layout = "nhwc"
    self._vision_enforce_nchw = os.getenv("RKNN_ENFORCE_VISION_NCHW", "0") != "0"
    if self._vision_enforce_nchw:
      vision_layout = "nchw"
    self._vision_layout = vision_layout
    self._vision_pass_through = [vision_pt] * n_vision_in
    self._vision_data_format = [vision_fmt] * n_vision_in
    self._vision_use_explicit_format = os.getenv("RKNN_PY_VISION_EXPLICIT", "0") != "0"
    self._nhwc_bigimg_affine_enable = os.getenv("RKNN_NHWC_BIGIMG_AFFINE_ENABLE", "1") != "0"
    self._nhwc_bigimg_scale = float(os.getenv("RKNN_NHWC_BIGIMG_SCALE", "0.55"))
    self._nhwc_bigimg_bias = float(os.getenv("RKNN_NHWC_BIGIMG_BIAS", "-6.0"))

    # Policy RKNN
    self._policy_rknn = RKNNLite(verbose=False)
    self._policy_rknn.load_rknn(str(policy_rknn_path))
    self._policy_rknn.init_runtime()
    n_policy_in = len(self.policy_input_names)
    policy_pt = int(os.getenv("RKNN_PY_POLICY_PT", "0"))
    policy_fmt = os.getenv("RKNN_PY_POLICY_FORMAT", "nchw").lower()
    self._policy_pass_through = [policy_pt] * n_policy_in
    self._policy_data_format = [policy_fmt] * n_policy_in
    self._policy_use_explicit_format = os.getenv("RKNN_PY_POLICY_EXPLICIT", "0") != "0"

  def run_vision(self, img: np.ndarray, big_img: np.ndarray) -> np.ndarray:
    """Run vision model. img and big_img are uint8; cast to float16 and run. Returns float32 (1, 1576)."""
    img_fp16 = np.ascontiguousarray(_to_fp16(img.reshape(self.vision_input_shapes["img"])))
    big_img_arr = big_img.reshape(self.vision_input_shapes["big_img"])
    if self._vision_layout == "nhwc" and self._nhwc_bigimg_affine_enable:
      big_img_arr = np.clip(big_img_arr.astype(np.float32) * self._nhwc_bigimg_scale + self._nhwc_bigimg_bias, 0.0, 255.0).astype(np.uint8)
    big_img_fp16 = np.ascontiguousarray(_to_fp16(big_img_arr))
    if self._vision_layout == "nhwc":
      img_fp16 = np.transpose(img_fp16, (0, 2, 3, 1))
      big_img_fp16 = np.transpose(big_img_fp16, (0, 2, 3, 1))
      img_fp16 = np.ascontiguousarray(img_fp16)
      big_img_fp16 = np.ascontiguousarray(big_img_fp16)
    inputs = [img_fp16, big_img_fp16]
    outputs = None
    if self._vision_use_explicit_format:
      try:
        outputs = self._vision_rknn.inference(
          inputs=inputs,
          data_type="float16",
          inputs_pass_through=self._vision_pass_through,
          data_format=self._vision_data_format,
        )
      except Exception:
        # Some RKNNLite builds reject explicit format enums. Switch once to stable default path.
        self._vision_use_explicit_format = False
      if outputs is None:
        # RKNNLite may log internal set_inputs error and return None without raising.
        # Disable explicit mode once to avoid per-frame error spam.
        self._vision_use_explicit_format = False

    if outputs is None:
      outputs = self._vision_rknn.inference(
        inputs=inputs,
        data_type="float16",
      )
    if outputs is None:
      raise RuntimeError("RKNNLite vision inference returned None")
    assert len(outputs) == 1
    out = outputs[0]
    if out.dtype != np.float32:
      out = out.astype(np.float32)
    return out.reshape(self.vision_output_shape)

  def run_policy(
    self,
    desire_pulse: np.ndarray,
    traffic_convention: np.ndarray,
    features_buffer: np.ndarray,
  ) -> np.ndarray:
    """Run policy model. All inputs cast to float16. Returns float32 (1, 1000)."""
    dp = _to_fp16(desire_pulse.reshape(self.policy_input_shapes["desire_pulse"]))
    tc = _to_fp16(traffic_convention.reshape(self.policy_input_shapes["traffic_convention"]))
    fb = _to_fp16(features_buffer.reshape(self.policy_input_shapes["features_buffer"]))
    inputs = [np.ascontiguousarray(dp), np.ascontiguousarray(tc), np.ascontiguousarray(fb)]
    outputs = None
    if self._policy_use_explicit_format:
      try:
        outputs = self._policy_rknn.inference(
          inputs=inputs,
          data_type="float16",
          inputs_pass_through=self._policy_pass_through,
          data_format=self._policy_data_format,
        )
      except Exception:
        self._policy_use_explicit_format = False
      if outputs is None:
        self._policy_use_explicit_format = False

    if outputs is None:
      outputs = self._policy_rknn.inference(
        inputs=inputs,
        data_type="float16",
      )
    if outputs is None:
      raise RuntimeError("RKNNLite policy inference returned None")
    assert len(outputs) == 1
    out = outputs[0]
    if out.dtype != np.float32:
      out = out.astype(np.float32)
    return out.reshape(self.policy_output_shape)
