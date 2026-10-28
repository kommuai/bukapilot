#!/usr/bin/env python3
"""Run the simulator's Tinygrad/OpenCL modeld without desktop CPU fallback."""

import os
from pathlib import Path

import pyopencl as cl


def require_opencl_gpu() -> str:
  platforms = cl.get_platforms()
  if not platforms:
    raise RuntimeError("No OpenCL platform found; refusing to start modeld on CPU")

  # tinygrad's OpenCL backend selects from the first platform. Check that same
  # platform so a GPU on some other platform cannot mask a CPU fallback here.
  devices = platforms[0].get_devices(device_type=cl.device_type.GPU)
  if not devices:
    raise RuntimeError(
      f"OpenCL platform {platforms[0].name!r} has no GPU device; refusing CPU fallback"
    )
  return devices[0].name


gpu_name = require_opencl_gpu()

# On desktop, modeld normally selects CPU. Set this hardware flag before the
# module import so it takes the same CL/OpenCL data path used by KA2.
import openpilot.system.hardware as hardware  # noqa: E402

hardware.KA2 = True
os.environ["DEV"] = "CL"
os.environ["USE_RKNN"] = "0"

import selfdrive.modeld.modeld as modeld  # noqa: E402

models_dir = Path(os.environ.get("METADRIVE_GPU_MODEL_DIR", "/tmp/bukapilot-metadrive-gpu-models"))
vision_model = models_dir / "driving_vision_tinygrad.pkl"
policy_model = models_dir / "driving_policy_tinygrad.pkl"
for path in (vision_model, policy_model):
  if not path.is_file():
    raise FileNotFoundError(f"Missing OpenCL model artifact: {path}. Start Terminal 1 first.")

modeld.VISION_PKL_PATH = vision_model
modeld.POLICY_PKL_PATH = policy_model

print(f"GPU_RUN hardware.KA2=True DEV=CL tinygrad_default=CL OpenCL_GPU={gpu_name}", flush=True)
modeld.main()
