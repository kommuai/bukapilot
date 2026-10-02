#!/usr/bin/env python3
import atexit
import os
import sys
from collections import deque
from openpilot.system.hardware import TICI, KA2, HARDWARE
os.environ['DEV'] = 'QCOM' if TICI else ('CL' if KA2 else 'CPU')
USBGPU = "USBGPU" in os.environ
if USBGPU:
  os.environ['DEV'] = 'AMD'
  os.environ['AMD_IFACE'] = 'USB'

# Performance tune (main-repo only): runtime env for tinygrad. No submodule changes.
# AGGRESSIVE_FUSION=1 reduces kernel count (~2.7%); schedule built on first run.
if os.environ.get('DEV') in ('CL', 'QCOM'):
  os.environ.setdefault('AGGRESSIVE_FUSION', '1')
  os.environ.setdefault('AGGRESSIVE_FUSION_MAX_BUFS', '6')
  os.environ.setdefault('AGGRESSIVE_FUSION_MIN_RATIO', '2')

from tinygrad.tensor import Tensor
from tinygrad.dtype import dtypes
import time
import pickle
import numpy as np
import cereal.messaging as messaging
from cereal import car, log
from pathlib import Path
from cereal.messaging import PubMaster, SubMaster
from msgq.visionipc import VisionIpcClient, VisionStreamType, VisionBuf
from opendbc.car.car_helpers import get_demo_car_params
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import config_realtime_process, DT_MDL
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.common.transformations.model import get_warp_matrix, medmodel_fl, sbigmodel_fl
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.controls.lib.drive_helpers import get_accel_from_plan, smooth_value, get_curvature_from_plan
from openpilot.selfdrive.modeld.parse_model_outputs import Parser
from openpilot.selfdrive.modeld.fill_model_msg import fill_model_msg, fill_pose_msg, PublishState
from openpilot.selfdrive.modeld.constants import ModelConstants, Plan
from openpilot.selfdrive.modeld.models.commonmodel_pyx import DrivingModelFrame, CLContext
from openpilot.selfdrive.modeld.runners.tinygrad_helpers import qcom_tensor_from_opencl_address
from tinygrad import Device
from tinygrad.device import Buffer, BufferSpec
from tinygrad.runtime.ops_cl import set_external_cl_context, get_cl_buffer_ptr, enqueue_copy_buffer, enqueue_write_buffer


PROCESS_NAME = "selfdrive.modeld.modeld"
SEND_RAW_PRED = os.getenv('SEND_RAW_PRED')
KA2_MODEL_X_OFFSET_PIX = -30.0

MODEL_DIR = Path(__file__).parent / 'models'
VISION_PKL_PATH = MODEL_DIR / 'driving_vision_tinygrad.pkl'
POLICY_PKL_PATH = MODEL_DIR / 'driving_policy_tinygrad.pkl'
VISION_METADATA_PATH = MODEL_DIR / 'driving_vision_metadata.pkl'
POLICY_METADATA_PATH = MODEL_DIR / 'driving_policy_metadata.pkl'
VISION_RKNN_PATH = MODEL_DIR / os.getenv("RKNN_VISION_MODEL", "driving_vision.rknn")
POLICY_RKNN_PATH = MODEL_DIR / os.getenv("RKNN_POLICY_MODEL", "driving_policy.rknn")

def _use_rknn_driving() -> bool:
  """Use RKNN for driving model when configured .rknn files exist (default). Set USE_RKNN=0 to force tinygrad."""
  if not (VISION_RKNN_PATH.exists() and POLICY_RKNN_PATH.exists()):
    return False
  return os.getenv('USE_RKNN', '1') != '0'

LAT_SMOOTH_SECONDS = 0.0
LONG_SMOOTH_SECONDS = 0.3
MIN_LAT_CONTROL_SPEED = 0.3

DRIVE_PATH_OFFSET_LIMIT_M = 0.25
DRIVE_PATH_OFFSET_STEP = 0.05
DRIVE_PATH_OFFSET_REF_X_M = 20.0


def drive_path_offset_pix(offset_m: float, model_fl: float) -> float:
  return 0.0 if not offset_m else model_fl * offset_m / DRIVE_PATH_OFFSET_REF_X_M


def apply_parallel_output_shift(model_output: dict[str, np.ndarray], offset_m: float) -> None:
  if offset_m and (delta := offset_m - (plan := model_output['plan'][0])[0, (yc := Plan.POSITION.start + 1)]):
    plan[:, yc] += delta
    (ll := model_output['lane_lines'][0])[1, :, 1] += delta
    ll[2, :, 1] += delta


def read_drive_path_offset(params: Params) -> float:
  raw = params.get("DrivePathOffset")
  try:
    val = float(raw or "0.0")
    if not -DRIVE_PATH_OFFSET_LIMIT_M <= val <= DRIVE_PATH_OFFSET_LIMIT_M:
      raise ValueError
    steps = int(round(2 * DRIVE_PATH_OFFSET_LIMIT_M / DRIVE_PATH_OFFSET_STEP))
    if not 0 <= (idx := int(round((val + DRIVE_PATH_OFFSET_LIMIT_M) / DRIVE_PATH_OFFSET_STEP))) <= steps:
      raise ValueError
    val = round(-DRIVE_PATH_OFFSET_LIMIT_M + idx * DRIVE_PATH_OFFSET_STEP, 2)
  except (TypeError, ValueError):
    cloudlog.warning("DrivePathOffset invalid (%r), resetting to 0.0", raw)
    val = 0.0

  if (stored := "0.0" if val == 0.0 else f"{val:.2f}") != raw:
    params.put("DrivePathOffset", stored)
  return val

def get_action_from_model(model_output: dict[str, np.ndarray], prev_action: log.ModelDataV2.Action,
                          lat_action_t: float, long_action_t: float, v_ego: float) -> log.ModelDataV2.Action:
    plan = model_output['plan'][0]
    desired_accel, should_stop = get_accel_from_plan(plan[:,Plan.VELOCITY][:,0],
                                                     plan[:,Plan.ACCELERATION][:,0],
                                                     ModelConstants.T_IDXS,
                                                     action_t=long_action_t)
    desired_accel = smooth_value(desired_accel, prev_action.desiredAcceleration, LONG_SMOOTH_SECONDS)

    desired_curvature = get_curvature_from_plan(plan[:,Plan.T_FROM_CURRENT_EULER][:,2],
                                                plan[:,Plan.ORIENTATION_RATE][:,2],
                                                ModelConstants.T_IDXS,
                                                v_ego,
                                                lat_action_t)
    if v_ego > MIN_LAT_CONTROL_SPEED:
      desired_curvature = smooth_value(desired_curvature, prev_action.desiredCurvature, LAT_SMOOTH_SECONDS)
    else:
      desired_curvature = prev_action.desiredCurvature

    return log.ModelDataV2.Action(desiredCurvature=float(desired_curvature),
                                  desiredAcceleration=float(desired_accel),
                                  shouldStop=bool(should_stop))

class FrameMeta:
  frame_id: int = 0
  timestamp_sof: int = 0
  timestamp_eof: int = 0

  def __init__(self, vipc=None):
    if vipc is not None:
      self.frame_id, self.timestamp_sof, self.timestamp_eof = vipc.frame_id, vipc.timestamp_sof, vipc.timestamp_eof

class InputQueues:
  def __init__ (self, model_fps, env_fps, n_frames_input):
    assert env_fps % model_fps == 0
    assert env_fps >= model_fps
    self.model_fps = model_fps
    self.env_fps = env_fps
    self.n_frames_input = n_frames_input

    self.dtypes = {}
    self.shapes = {}
    self.q = {}

  def update_dtypes_and_shapes(self, input_dtypes, input_shapes) -> None:
    self.dtypes.update(input_dtypes)
    if self.env_fps == self.model_fps:
      self.shapes.update(input_shapes)
    else:
      for k in input_shapes:
        shape = list(input_shapes[k])
        if 'img' in k:
          n_channels = shape[1] // self.n_frames_input
          shape[1] = (self.env_fps // self.model_fps + (self.n_frames_input - 1)) * n_channels
        else:
          shape[1] = (self.env_fps // self.model_fps) * shape[1]
        self.shapes[k] = tuple(shape)

  def reset(self) -> None:
    self.q = {k: np.zeros(self.shapes[k], dtype=self.dtypes[k]) for k in self.dtypes.keys()}

  def enqueue(self, inputs:dict[str, np.ndarray]) -> None:
    for k in inputs.keys():
      if inputs[k].dtype != self.dtypes[k]:
        raise ValueError(f'supplied input <{k}({inputs[k].dtype})> has wrong dtype, expected {self.dtypes[k]}')
      input_shape = list(self.shapes[k])
      input_shape[1] = -1
      single_input = inputs[k].reshape(tuple(input_shape))
      sz = single_input.shape[1]
      self.q[k][:,:-sz] = self.q[k][:,sz:]
      self.q[k][:,-sz:] = single_input

  def get(self, *names) -> dict[str, np.ndarray]:
    if self.env_fps == self.model_fps:
      return {k: self.q[k] for k in names}
    else:
      out = {}
      for k in names:
        shape = self.shapes[k]
        if 'img' in k:
          n_channels = shape[1] // (self.env_fps // self.model_fps + (self.n_frames_input - 1))
          out[k] = np.concatenate([self.q[k][:, s:s+n_channels] for s in np.linspace(0, shape[1] - n_channels, self.n_frames_input, dtype=int)], axis=1)
        elif 'pulse' in k:
          # any pulse within interval counts
          out[k] = self.q[k].reshape((shape[0], shape[1] * self.model_fps // self.env_fps, self.env_fps // self.model_fps, -1)).max(axis=2)
        else:
          idxs = np.arange(-1, -shape[1], -self.env_fps // self.model_fps)[::-1]
          out[k] = self.q[k][:, idxs]
      return out

class ModelState:
  frames: dict[str, DrivingModelFrame]
  inputs: dict[str, np.ndarray]
  output: np.ndarray
  prev_desire: np.ndarray  # for tracking the rising edge of the pulse

  def __init__(self, context: CLContext):
    with open(VISION_METADATA_PATH, 'rb') as f:
      vision_metadata = pickle.load(f)
      self.vision_input_shapes =  vision_metadata['input_shapes']
      self.vision_input_names = list(self.vision_input_shapes.keys())
      self.vision_output_slices = vision_metadata['output_slices']
      vision_output_size = vision_metadata['output_shapes']['outputs'][1]

    with open(POLICY_METADATA_PATH, 'rb') as f:
      policy_metadata = pickle.load(f)
      self.policy_input_shapes =  policy_metadata['input_shapes']
      self.policy_output_slices = policy_metadata['output_slices']
      policy_output_size = policy_metadata['output_shapes']['outputs'][1]

    self.frames = {name: DrivingModelFrame(context, ModelConstants.MODEL_RUN_FREQ//ModelConstants.MODEL_CONTEXT_FREQ) for name in self.vision_input_names}
    self.prev_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)

    # policy inputs
    self.numpy_inputs = {k: np.zeros(self.policy_input_shapes[k], dtype=np.float32) for k in self.policy_input_shapes}
    self.full_input_queues = InputQueues(ModelConstants.MODEL_CONTEXT_FREQ, ModelConstants.MODEL_RUN_FREQ, ModelConstants.N_FRAMES)
    for k in ['desire_pulse', 'features_buffer']:
      self.full_input_queues.update_dtypes_and_shapes({k: self.numpy_inputs[k].dtype}, {k: self.numpy_inputs[k].shape})
    self.full_input_queues.reset()

    # img buffers are managed in openCL transform code
    self.vision_inputs: dict[str, Tensor] = {}
    self.vision_output = np.zeros(vision_output_size, dtype=np.float32)
    self.policy_inputs = {k: Tensor(v, device='NPY').realize() for k,v in self.numpy_inputs.items()}
    self.policy_output = np.zeros(policy_output_size, dtype=np.float32)
    self.parser = Parser()

    # KA2: keep policy inputs on GPU (one sync per frame). Requires policy pickle saved without JIT cache so first run can capture CL.
    self._cl_rolling = KA2 and not USBGPU
    if self._cl_rolling:
      # Allocate CL buffers directly so we have Buffer refs for enqueue_*; zero via copyin
      alloc = Device['CL'].allocator
      opts = getattr(alloc, 'default_buffer_spec', None)
      feat_opaque = alloc.alloc(1 * 25 * 512 * 4, opts)
      des_opaque = alloc.alloc(1 * 25 * 8 * 4, opts)
      self._features_buf = Buffer('CL', 1 * 25 * 512, dtypes.float32, opaque=feat_opaque)
      self._desire_buf = Buffer('CL', 1 * 25 * 8, dtypes.float32, opaque=des_opaque)
      self._features_buf.copyin(memoryview(bytearray(1 * 25 * 512 * 4)))
      self._desire_buf.copyin(memoryview(bytearray(1 * 25 * 8 * 4)))
      # Temp buffers for shift (copy to self is invalid when regions overlap)
      self._features_tmp = Buffer('CL', 24 * 512, dtypes.float32, opaque=alloc.alloc(24 * 512 * 4, opts))
      self._desire_tmp = Buffer('CL', 24 * 8, dtypes.float32, opaque=alloc.alloc(24 * 8 * 4, opts))
      hs = self.vision_output_slices['hidden_state']
      self._vision_hidden_start = int(hs.start) * 4
      self._vision_hidden_size = int(hs.stop - hs.start) * 4
      self._features_shift_src_offset = 1 * 512 * 4
      self._features_shift_size = 24 * 512 * 4
      self._features_append_offset = 24 * 512 * 4
      self._desire_shift_src_offset = 1 * 8 * 4
      self._desire_shift_size = 24 * 8 * 4
      self._desire_append_offset = 24 * 8 * 4
    with open(VISION_PKL_PATH, "rb") as f:
      self.vision_run = pickle.load(f)

    with open(POLICY_PKL_PATH, "rb") as f:
      self.policy_run = pickle.load(f)

  def slice_outputs(self, model_outputs: np.ndarray, output_slices: dict[str, slice]) -> dict[str, np.ndarray]:
    parsed_model_outputs = {k: model_outputs[np.newaxis, v] for k,v in output_slices.items()}
    return parsed_model_outputs

  def _update_cl_rolling_buffers(self, vision_out_buffer, new_desire: np.ndarray) -> None:
    """Update features and desire buffers on GPU: shift left, append hidden_state and new_desire."""
    cl_device = Device['CL'].allocator.dev
    # Shift features via temp (same-buffer copy overlaps); then append vision hidden at [24]
    enqueue_copy_buffer(cl_device, self._features_buf, self._features_tmp,
                        self._features_shift_src_offset, 0, self._features_shift_size)
    enqueue_copy_buffer(cl_device, self._features_tmp, self._features_buf, 0, 0, self._features_shift_size)
    enqueue_copy_buffer(cl_device, vision_out_buffer, self._features_buf,
                        self._vision_hidden_start, self._features_append_offset, self._vision_hidden_size)
    # Shift desire via temp; then append new_desire at [24]
    enqueue_copy_buffer(cl_device, self._desire_buf, self._desire_tmp,
                        self._desire_shift_src_offset, 0, self._desire_shift_size)
    enqueue_copy_buffer(cl_device, self._desire_tmp, self._desire_buf, 0, 0, self._desire_shift_size)
    new_desire_mv = memoryview(np.ascontiguousarray(new_desire.astype(np.float32)))
    enqueue_write_buffer(cl_device, self._desire_buf, self._desire_append_offset, new_desire_mv)

  def run(self, bufs: dict[str, VisionBuf], transforms: dict[str, np.ndarray],
                inputs: dict[str, np.ndarray], prepare_only: bool, frame_id: int | None = None) -> dict[str, np.ndarray] | None:
    # Model decides when action is completed, so desire input is just a pulse triggered on rising edge
    inputs['desire_pulse'][0] = 0
    new_desire = np.where(inputs['desire_pulse'] - self.prev_desire > .99, inputs['desire_pulse'], 0)
    self.prev_desire[:] = inputs['desire_pulse']

    imgs_cl = {name: self.frames[name].prepare(bufs[name], transforms[name].flatten()) for name in self.vision_input_names}
    if TICI and not USBGPU:
      # The imgs tensors are backed by opencl memory, only need init once
      for key in imgs_cl:
        if key not in self.vision_inputs:
          self.vision_inputs[key] = qcom_tensor_from_opencl_address(imgs_cl[key].mem_address, self.vision_input_shapes[key], dtype=dtypes.uint8)
    elif KA2 and not USBGPU:
      # Zero-copy: wrap CL buffer from prepare() (same context via set_external_cl_context)
      for key in imgs_cl:
        if key not in self.vision_inputs:
          self.vision_inputs[key] = Tensor.from_blob(
            imgs_cl[key].mem_handle, self.vision_input_shapes[key], dtype=dtypes.uint8, device='CL'
          ).realize()
    else:
      for key in imgs_cl:
        frame_input = self.frames[key].buffer_from_cl(imgs_cl[key]).reshape(self.vision_input_shapes[key])
        self.vision_inputs[key] = Tensor(frame_input, dtype=dtypes.uint8).realize()
    if prepare_only:
      return None

    # Vision: realize() = kernel time; .numpy() = sync + GPU->CPU copy
    vision_out = self.vision_run(**self.vision_inputs).contiguous().realize()

    cl_path_used = False
    if self._cl_rolling:
      try:
        self._update_cl_rolling_buffers(vision_out.uop.base.buffer, new_desire)
        policy_inputs_cl = {
          'desire_pulse': Tensor.from_blob(get_cl_buffer_ptr(self._desire_buf), (1, 25, 8), dtype=dtypes.float32, device='CL').realize(),
          'traffic_convention': Tensor(np.asarray(inputs['traffic_convention'], dtype=np.float32), device='CL').realize(),
          'features_buffer': Tensor.from_blob(get_cl_buffer_ptr(self._features_buf), (1, 25, 512), dtype=dtypes.float32, device='CL').realize(),
        }
        policy_out = self.policy_run(**policy_inputs_cl).contiguous().realize()
        Device['CL'].synchronize()
        self.vision_output = vision_out.uop.base.buffer.numpy()
        self.policy_output = policy_out.uop.base.buffer.numpy()
        vision_outputs_dict = self.parser.parse_vision_outputs(self.slice_outputs(self.vision_output, self.vision_output_slices))
        policy_outputs_dict = self.parser.parse_policy_outputs(self.slice_outputs(self.policy_output, self.policy_output_slices))
        self.full_input_queues.enqueue({'features_buffer': vision_outputs_dict['hidden_state'], 'desire_pulse': new_desire})
        for k in ['desire_pulse', 'features_buffer']:
          self.numpy_inputs[k][:] = self.full_input_queues.get(k)[k]
        self.numpy_inputs['traffic_convention'][:] = inputs['traffic_convention']
        cl_path_used = True
      except Exception as e:
        if "args mismatch in JIT" in str(e) or "expected_input_info" in str(e):
          cloudlog.warning("modeld: policy JIT expects NPY, falling back to legacy vision→CPU→policy path")
          self._cl_rolling = False
        else:
          raise
    if not cl_path_used:
      self.vision_output = vision_out.uop.base.buffer.numpy()
      vision_outputs_dict = self.parser.parse_vision_outputs(self.slice_outputs(self.vision_output, self.vision_output_slices))
      self.full_input_queues.enqueue({'features_buffer': vision_outputs_dict['hidden_state'], 'desire_pulse': new_desire})
      for k in ['desire_pulse', 'features_buffer']:
        self.numpy_inputs[k][:] = self.full_input_queues.get(k)[k]
      self.numpy_inputs['traffic_convention'][:] = inputs['traffic_convention']
      policy_out = self.policy_run(**self.policy_inputs).contiguous().realize()
      self.policy_output = policy_out.uop.base.buffer.numpy()
      policy_outputs_dict = self.parser.parse_policy_outputs(self.slice_outputs(self.policy_output, self.policy_output_slices))

    combined_outputs_dict = {**vision_outputs_dict, **policy_outputs_dict}
    if SEND_RAW_PRED:
      combined_outputs_dict['raw_pred'] = np.concatenate([self.vision_output.copy(), self.policy_output.copy()])

    return combined_outputs_dict


def _load_rknn_metadata():
  """Load vision/policy metadata for RKNN path (shapes, slices)."""
  with open(VISION_METADATA_PATH, 'rb') as f:
    v = pickle.load(f)
  with open(POLICY_METADATA_PATH, 'rb') as f:
    p = pickle.load(f)
  v_size = int(np.prod(v['output_shapes']['outputs']))
  p_size = int(np.prod(p['output_shapes']['outputs']))
  return {
    'vision_input_shapes': v['input_shapes'],
    'vision_input_names': list(v['input_shapes'].keys()),
    'vision_output_slices': v['output_slices'],
    'policy_input_shapes': p['input_shapes'],
    'policy_output_slices': p['output_slices'],
    'vision_output_size': v_size,
    'policy_output_size': p_size,
  }


class ModelStateRKNN:
  """Driving model state using RKNN (vision + policy). Inputs are cast to float16 before inference. Prefers C++ path when built."""

  def __init__(self, context: CLContext):
    meta = _load_rknn_metadata()
    self.vision_input_shapes = meta['vision_input_shapes']
    self.vision_input_names = meta['vision_input_names']
    self.vision_output_slices = meta['vision_output_slices']
    self.policy_input_shapes = meta['policy_input_shapes']
    self.policy_output_slices = meta['policy_output_slices']
    self._vision_output_size = meta['vision_output_size']
    self._policy_output_size = meta['policy_output_size']
    # Prefer C++ RKNN runner when available (no lite path)
    self._rknn_cpp = None
    force_rknn_python = os.getenv("RKNN_USE_PYTHON", "0") == "1"
    if not force_rknn_python:
      try:
        from openpilot.selfdrive.modeld.runners.driving_rknnmodel_pyx import DrivingRKNNRunnerCpp
        self._rknn_cpp = DrivingRKNNRunnerCpp(str(VISION_RKNN_PATH), str(POLICY_RKNN_PATH))
        cloudlog.warning("modeld RKNN: using C++ runner (driving_rknnmodel_pyx)")
      except Exception as e:
        cloudlog.warning("modeld RKNN: C++ runner unavailable (%s), using Python rknnlite", e)
    else:
      cloudlog.warning("modeld RKNN: RKNN_USE_PYTHON=1, forcing Python rknnlite runner")
    if self._rknn_cpp is None:
      from openpilot.selfdrive.modeld.runners.driving_rknn import DrivingRKNNRunner
      self._rknn = DrivingRKNNRunner(MODEL_DIR)
    self.frames = {
      name: DrivingModelFrame(context, ModelConstants.MODEL_RUN_FREQ // ModelConstants.MODEL_CONTEXT_FREQ)
      for name in self.vision_input_names
    }
    self.prev_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
    self.numpy_inputs = {k: np.zeros(self.policy_input_shapes[k], dtype=np.float32) for k in self.policy_input_shapes}
    self.full_input_queues = InputQueues(
      ModelConstants.MODEL_CONTEXT_FREQ, ModelConstants.MODEL_RUN_FREQ, ModelConstants.N_FRAMES
    )
    for k in ['desire_pulse', 'features_buffer']:
      self.full_input_queues.update_dtypes_and_shapes(
        {k: self.numpy_inputs[k].dtype}, {k: self.numpy_inputs[k].shape}
      )
    self.full_input_queues.reset()
    self.vision_output = np.zeros(self._vision_output_size, dtype=np.float32)
    self.policy_output = np.zeros(self._policy_output_size, dtype=np.float32)
    self.parser = Parser()
    # Temporary mitigation: suppress one-frame "straight blips" in RKNN plan output.
    self._blip_guard_enabled = os.getenv("RKNN_BLIP_GUARD", "1") != "0"
    self._blip_guard_context = 3
    self._blip_guard_curved = 0.8
    self._blip_guard_straight = 0.25
    self._blip_guard_recent_y20 = deque(maxlen=self._blip_guard_context)
    self._blip_guard_prev_plan_position: np.ndarray | None = None
    self._blip_guard_prev_plan_stds_position: np.ndarray | None = None
    self._stage_capture_dir = os.getenv("RKNN_STAGE_CAPTURE_DIR", "").strip()
    self._stage_capture_window = int(os.getenv("RKNN_STAGE_CAPTURE_WINDOW", "0"))
    ids_raw = os.getenv("RKNN_STAGE_CAPTURE_FRAME_IDS", "").strip()
    self._stage_capture_ids = {
      int(v.strip()) for v in ids_raw.split(",") if v.strip()
    } if ids_raw else set()
    self._stage_capture_enabled = bool(self._stage_capture_dir and self._stage_capture_ids)
    if self._stage_capture_enabled:
      Path(self._stage_capture_dir).mkdir(parents=True, exist_ok=True)
      cloudlog.warning("modeld RKNN stage capture enabled: dir=%s frames=%s window=%d",
                       self._stage_capture_dir, sorted(self._stage_capture_ids), self._stage_capture_window)

  def slice_outputs(self, model_outputs: np.ndarray, output_slices: dict[str, slice]) -> dict[str, np.ndarray]:
    return {k: model_outputs[np.newaxis, v] for k, v in output_slices.items()}

  def _y_at_distance_from_plan(self, plan: np.ndarray, distance_m: float = 20.0) -> float | None:
    x = np.asarray(plan[0, :, Plan.POSITION.start], dtype=np.float64)
    y = np.asarray(plan[0, :, Plan.POSITION.start + 1], dtype=np.float64)
    if x.size < 3 or y.size != x.size:
      return None
    if not np.all(np.diff(x) >= 0):
      return None
    if distance_m < x[0] or distance_m > x[-1]:
      return None
    return float(np.interp(distance_m, x, y))

  def _apply_blip_guard(self, policy_outputs_dict: dict[str, np.ndarray]) -> None:
    plan = policy_outputs_dict.get("plan", None)
    plan_stds = policy_outputs_dict.get("plan_stds", None)
    if plan is None or plan_stds is None:
      return

    y20 = self._y_at_distance_from_plan(plan)
    if y20 is None:
      return

    should_guard = False
    if len(self._blip_guard_recent_y20) >= self._blip_guard_context:
      recent = list(self._blip_guard_recent_y20)
      all_curved = all(abs(v) >= self._blip_guard_curved for v in recent)
      same_side_curve = (min(recent) > 0.0) or (max(recent) < 0.0)
      now_straight = abs(y20) < self._blip_guard_straight
      should_guard = all_curved and same_side_curve and now_straight

    if should_guard and self._blip_guard_prev_plan_position is not None and self._blip_guard_prev_plan_stds_position is not None:
      plan[0, :, Plan.POSITION] = self._blip_guard_prev_plan_position
      plan_stds[0, :, Plan.POSITION] = self._blip_guard_prev_plan_stds_position

    self._blip_guard_prev_plan_position = plan[0, :, Plan.POSITION].copy()
    self._blip_guard_prev_plan_stds_position = plan_stds[0, :, Plan.POSITION].copy()
    y20_after = self._y_at_distance_from_plan(plan)
    if y20_after is not None:
      self._blip_guard_recent_y20.append(y20_after)

  def _should_stage_capture(self, frame_id: int | None) -> bool:
    if not self._stage_capture_enabled or frame_id is None:
      return False
    if frame_id in self._stage_capture_ids:
      return True
    w = self._stage_capture_window
    return w > 0 and any(abs(frame_id - fid) <= w for fid in self._stage_capture_ids)

  def _stage_capture(self, frame_id: int, img_np: np.ndarray, big_img_np: np.ndarray,
                     vision_outputs_dict: dict[str, np.ndarray]) -> None:
    out_path = Path(self._stage_capture_dir) / f"frame_{frame_id}.npz"
    np.savez_compressed(
      out_path,
      frame_id=np.asarray(frame_id, dtype=np.int64),
      img_input=img_np.copy(),
      big_img_input=big_img_np.copy(),
      desire_pulse_input=self.numpy_inputs["desire_pulse"].copy(),
      traffic_convention_input=self.numpy_inputs["traffic_convention"].copy(),
      features_buffer_input=self.numpy_inputs["features_buffer"].copy(),
      vision_output_raw=self.vision_output.copy(),
      policy_output_raw=self.policy_output.copy(),
      hidden_state=vision_outputs_dict["hidden_state"].copy(),
    )

  def run(self, bufs: dict[str, VisionBuf], transforms: dict[str, np.ndarray],
          inputs: dict[str, np.ndarray], prepare_only: bool, frame_id: int | None = None) -> dict[str, np.ndarray] | None:
    inputs['desire_pulse'][0] = 0
    new_desire = np.where(inputs['desire_pulse'] - self.prev_desire > .99, inputs['desire_pulse'], 0)
    self.prev_desire[:] = inputs['desire_pulse']

    imgs_cl = {name: self.frames[name].prepare(bufs[name], transforms[name].flatten()) for name in self.vision_input_names}
    # Get numpy from CL (inputs cast to float16 inside runner)
    img_np = self.frames['img'].buffer_from_cl(imgs_cl['img']).reshape(self.vision_input_shapes['img'])
    big_img_np = self.frames['big_img'].buffer_from_cl(imgs_cl['big_img']).reshape(self.vision_input_shapes['big_img'])

    if prepare_only:
      return None

    if self._rknn_cpp is not None:
      self.vision_output = self._rknn_cpp.run_vision(img_np, big_img_np).reshape(-1)
      vision_outputs_dict = self.parser.parse_vision_outputs(
        self.slice_outputs(self.vision_output, self.vision_output_slices)
      )
      self.full_input_queues.enqueue({'features_buffer': vision_outputs_dict['hidden_state'], 'desire_pulse': new_desire})
      for k in ['desire_pulse', 'features_buffer']:
        self.numpy_inputs[k][:] = self.full_input_queues.get(k)[k]
      self.numpy_inputs['traffic_convention'][:] = inputs['traffic_convention']
      self.policy_output = self._rknn_cpp.run_policy(
        self.numpy_inputs['desire_pulse'],
        self.numpy_inputs['traffic_convention'],
        self.numpy_inputs['features_buffer'],
      ).reshape(-1)
    else:
      self.vision_output = self._rknn.run_vision(img_np, big_img_np).reshape(-1)
      vision_outputs_dict = self.parser.parse_vision_outputs(
        self.slice_outputs(self.vision_output, self.vision_output_slices)
      )
      self.full_input_queues.enqueue({'features_buffer': vision_outputs_dict['hidden_state'], 'desire_pulse': new_desire})
      for k in ['desire_pulse', 'features_buffer']:
        self.numpy_inputs[k][:] = self.full_input_queues.get(k)[k]
      self.numpy_inputs['traffic_convention'][:] = inputs['traffic_convention']
      self.policy_output = self._rknn.run_policy(
        self.numpy_inputs['desire_pulse'],
        self.numpy_inputs['traffic_convention'],
        self.numpy_inputs['features_buffer'],
      ).reshape(-1)
    policy_outputs_dict = self.parser.parse_policy_outputs(
      self.slice_outputs(self.policy_output, self.policy_output_slices)
    )
    if self._should_stage_capture(frame_id):
      self._stage_capture(frame_id, img_np, big_img_np, vision_outputs_dict)
    if self._blip_guard_enabled:
      self._apply_blip_guard(policy_outputs_dict)

    combined_outputs_dict = {**vision_outputs_dict, **policy_outputs_dict}
    if SEND_RAW_PRED:
      combined_outputs_dict['raw_pred'] = np.concatenate([self.vision_output.copy(), self.policy_output.copy()])

    return combined_outputs_dict


def main(demo=False):
  cloudlog.warning("modeld init")
  if demo and KA2:
    HARDWARE.set_power_save(False)
    cloudlog.warning("modeld --demo: set GPU/CPU performance mode")
    atexit.register(HARDWARE.set_power_save, True)

  if not USBGPU:
    # USB GPU currently saturates a core so can't do this yet.
    config_realtime_process(7, 54)

  st = time.monotonic()
  cloudlog.warning("setting up CL context")
  cl_context = CLContext()
  if KA2 and not USBGPU:
    set_external_cl_context(cl_context.context_ptr, cl_context.device_id_ptr, cl_context.queue_ptr)
  cloudlog.warning("CL context ready; loading model")
  if _use_rknn_driving():
    cloudlog.warning("using RKNN driving runner (vision=%s policy=%s); inputs cast to float16", VISION_RKNN_PATH.name, POLICY_RKNN_PATH.name)
    model = ModelStateRKNN(cl_context)
  else:
    override = " (USE_RKNN=0)" if (VISION_RKNN_PATH.exists() and POLICY_RKNN_PATH.exists()) else ""
    cloudlog.warning(f"using tinygrad driving runner{override}")
    model = ModelState(cl_context)
  cloudlog.warning(f"models loaded in {time.monotonic() - st:.1f}s, modeld starting")

  # visionipc clients
  while True:
    available_streams = VisionIpcClient.available_streams("camerad", block=False)
    if available_streams:
      use_extra_client = VisionStreamType.VISION_STREAM_WIDE_ROAD in available_streams and VisionStreamType.VISION_STREAM_ROAD in available_streams
      main_wide_camera = VisionStreamType.VISION_STREAM_ROAD not in available_streams
      break
    time.sleep(.1)

  vipc_client_main_stream = VisionStreamType.VISION_STREAM_WIDE_ROAD if main_wide_camera else VisionStreamType.VISION_STREAM_ROAD
  vipc_client_main = VisionIpcClient("camerad", vipc_client_main_stream, True, cl_context)
  vipc_client_extra = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_WIDE_ROAD, False, cl_context)
  cloudlog.warning(f"vision stream set up, main_wide_camera: {main_wide_camera}, use_extra_client: {use_extra_client}")

  while not vipc_client_main.connect(False):
    time.sleep(0.1)
  while use_extra_client and not vipc_client_extra.connect(False):
    time.sleep(0.1)

  cloudlog.warning(f"connected main cam with buffer size: {vipc_client_main.buffer_len} ({vipc_client_main.width} x {vipc_client_main.height})")
  if use_extra_client:
    cloudlog.warning(f"connected extra cam with buffer size: {vipc_client_extra.buffer_len} ({vipc_client_extra.width} x {vipc_client_extra.height})")

  # messaging
  pm = PubMaster(["modelV2", "drivingModelData", "cameraOdometry"])
  sm = SubMaster(["deviceState", "carState", "roadCameraState", "liveCalibration", "driverMonitoringState", "carControl", "liveDelay"])

  publish_state = PublishState()
  params = Params()

  # setup filter to track dropped frames
  frame_dropped_filter = FirstOrderFilter(0., 10., 1. / ModelConstants.MODEL_RUN_FREQ)
  frame_id = 0
  last_vipc_frame_id = 0
  run_count = 0

  model_transform_main = np.zeros((3, 3), dtype=np.float32)
  model_transform_extra = np.zeros((3, 3), dtype=np.float32)
  live_calib_seen = False
  buf_main, buf_extra = None, None
  meta_main = FrameMeta()
  meta_extra = FrameMeta()
  out_of_sync_reported = False

  if demo:
    CP = get_demo_car_params()
  else:
    CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  cloudlog.info("modeld got CarParams: %s", CP.brand)
  drive_path_offset = read_drive_path_offset(params)
  drive_path_pix_main, drive_path_pix_extra = tuple(
    drive_path_offset_pix(drive_path_offset, fl) for fl in (medmodel_fl, sbigmodel_fl))
  cloudlog.info("modeld DrivePathOffset: %.2f m (warp +%.3f/+%.3f px)",
                drive_path_offset, drive_path_pix_main, drive_path_pix_extra)

  # TODO this needs more thought, use .2s extra for now to estimate other delays
  # TODO Move smooth seconds to action function
  long_delay = CP.longitudinalActuatorDelay + LONG_SMOOTH_SECONDS
  prev_action = log.ModelDataV2.Action()

  DH = DesireHelper()

  while True:
    # Keep receiving frames until we are at least 1 frame ahead of previous extra frame
    while meta_main.timestamp_sof < meta_extra.timestamp_sof + 25000000:
      buf_main = vipc_client_main.recv()
      meta_main = FrameMeta(vipc_client_main)
      if buf_main is None:
        break

    if buf_main is None:
      cloudlog.debug("vipc_client_main no frame")
      continue

    if use_extra_client:
      # Keep receiving extra frames until frame id matches main camera
      while True:
        buf_extra = vipc_client_extra.recv()
        meta_extra = FrameMeta(vipc_client_extra)
        if buf_extra is None or meta_main.timestamp_sof < meta_extra.timestamp_sof + 25000000:
          break

      if buf_extra is None:
        cloudlog.debug("vipc_client_extra no frame")
        continue

      if abs(meta_main.timestamp_sof - meta_extra.timestamp_sof) > 10000000 and not out_of_sync_reported:
        cloudlog.error("frames out of sync! main: {} ({:.5f}), extra: {} ({:.5f})".format(
          meta_main.frame_id, meta_main.timestamp_sof / 1e9,
          meta_extra.frame_id, meta_extra.timestamp_sof / 1e9))
        out_of_sync_reported = True

    else:
      # Use single camera
      buf_extra = buf_main
      meta_extra = meta_main

    sm.update(0)
    desire = DH.desire
    is_rhd = sm["driverMonitoringState"].isRHD
    frame_id = sm["roadCameraState"].frameId
    v_ego = max(sm["carState"].vEgo, 0.)
    lat_delay = sm["liveDelay"].lateralDelay + LAT_SMOOTH_SECONDS
    if sm.updated["liveCalibration"] and sm.seen['roadCameraState'] and sm.seen['deviceState']:
      device_from_calib_euler = np.array(sm["liveCalibration"].rpyCalib, dtype=np.float32)
      dc = DEVICE_CAMERAS[(str(sm['deviceState'].deviceType), str(sm['roadCameraState'].sensor))]
      model_transform_main = get_warp_matrix(
        device_from_calib_euler,
        dc.ecam.intrinsics if main_wide_camera else dc.fcam.intrinsics,
        False,
        x_offset_pix=KA2_MODEL_X_OFFSET_PIX - drive_path_pix_main,
      ).astype(np.float32)
      model_transform_extra = get_warp_matrix(
        device_from_calib_euler,
        dc.ecam.intrinsics,
        True,
        x_offset_pix=KA2_MODEL_X_OFFSET_PIX - drive_path_pix_extra,
      ).astype(np.float32)
      live_calib_seen = True

    traffic_convention = np.zeros(2)
    traffic_convention[int(is_rhd)] = 1

    vec_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
    if desire >= 0 and desire < ModelConstants.DESIRE_LEN:
      vec_desire[desire] = 1

    # tracked dropped frames
    vipc_dropped_frames = max(0, meta_main.frame_id - last_vipc_frame_id - 1)
    frames_dropped = frame_dropped_filter.update(min(vipc_dropped_frames, 10))
    if run_count < 10: # let frame drops warm up
      frame_dropped_filter.x = 0.
      frames_dropped = 0.
    run_count = run_count + 1

    frame_drop_ratio = frames_dropped / (1 + frames_dropped)
    prepare_only = vipc_dropped_frames > 0
    if prepare_only:
      cloudlog.error(f"skipping model eval. Dropped {vipc_dropped_frames} frames")

    bufs = {name: buf_extra if 'big' in name else buf_main for name in model.vision_input_names}
    transforms = {name: model_transform_extra if 'big' in name else model_transform_main for name in model.vision_input_names}
    inputs:dict[str, np.ndarray] = {
      'desire_pulse': vec_desire,
      'traffic_convention': traffic_convention,
    }

    mt1 = time.perf_counter()
    model_output = model.run(bufs, transforms, inputs, prepare_only, frame_id)
    mt2 = time.perf_counter()
    model_execution_time = mt2 - mt1

    if model_output is not None:
      modelv2_send = messaging.new_message('modelV2')
      drivingdata_send = messaging.new_message('drivingModelData')
      posenet_send = messaging.new_message('cameraOdometry')

      apply_parallel_output_shift(model_output, drive_path_offset)
      action = get_action_from_model(model_output, prev_action, lat_delay + DT_MDL, long_delay + DT_MDL, v_ego)
      prev_action = action
      fill_model_msg(drivingdata_send, modelv2_send, model_output, action,
                     publish_state, meta_main.frame_id, meta_extra.frame_id, frame_id,
                     frame_drop_ratio, meta_main.timestamp_eof, model_execution_time, live_calib_seen)

      desire_state = modelv2_send.modelV2.meta.desireState
      l_lane_change_prob = desire_state[log.Desire.laneChangeLeft]
      r_lane_change_prob = desire_state[log.Desire.laneChangeRight]
      lane_change_prob = l_lane_change_prob + r_lane_change_prob
      DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob, modelv2_send.modelV2)
      modelv2_send.modelV2.meta.laneChangeState = DH.lane_change_state
      modelv2_send.modelV2.meta.laneChangeDirection = DH.lane_change_direction
      drivingdata_send.drivingModelData.meta.laneChangeState = DH.lane_change_state
      drivingdata_send.drivingModelData.meta.laneChangeDirection = DH.lane_change_direction

      fill_pose_msg(posenet_send, model_output, meta_main.frame_id, vipc_dropped_frames, meta_main.timestamp_eof, live_calib_seen)
      pm.send('modelV2', modelv2_send)
      pm.send('drivingModelData', drivingdata_send)
      pm.send('cameraOdometry', posenet_send)
    last_vipc_frame_id = meta_main.frame_id


if __name__ == "__main__":
  try:
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--demo', action='store_true', help='A boolean for demo mode.')
    args = parser.parse_args()
    main(demo=args.demo)
  except KeyboardInterrupt:
    cloudlog.warning("got SIGINT")
