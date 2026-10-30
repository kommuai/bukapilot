#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"

if [[ ! -x "$ROOT_DIR/.venv/bin/python3" ]]; then
  echo "Missing checkout virtualenv at $ROOT_DIR/.venv. Create the project venv first." >&2
  exit 1
fi

export PATH="$ROOT_DIR/.venv/bin:$PATH"
export PARAMS_ROOT="${PARAMS_ROOT:-/tmp/bukapilot-metadrive-params}"
export METADRIVE_GPU_MODEL_DIR="${METADRIVE_GPU_MODEL_DIR:-/tmp/bukapilot-metadrive-gpu-models}"
mkdir -p "$PARAMS_ROOT" "$METADRIVE_GPU_MODEL_DIR"

echo "Building desktop simulator prerequisites (incremental after the first run)..."
scons -u -j"$(nproc)" \
  msgq_repo/msgq/visionipc/visionipc_pyx.so \
  system/loggerd/bootlog \
  selfdrive/controls/lib/longitudinal_mpc_lib/c_generated_code/acados_ocp_solver_pyx.so \
  selfdrive/controls/lib/lateral_mpc_lib/c_generated_code/acados_ocp_solver_pyx.so \
  selfdrive/locationd/models/generated/libpose.so \
  selfdrive/locationd/models/generated/libcar.so \
  selfdrive/modeld/models/commonmodel_pyx.so \
  selfdrive/modeld/models/driving_vision_metadata.pkl \
  selfdrive/modeld/models/driving_policy_metadata.pkl

prepare_gpu_model() {
  local model_name="$1"
  local source="$ROOT_DIR/selfdrive/modeld/models/${model_name}.onnx"
  local output="$METADRIVE_GPU_MODEL_DIR/${model_name}_tinygrad.pkl"
  local cache_meta="$output.meta"
  local opencl_compat="$ROOT_DIR/tools/sim/gpu_compat/sitecustomize.py"
  local tinygrad_revision
  local cache_key

  tinygrad_revision=$(git -C "$ROOT_DIR/tinygrad_repo" rev-parse HEAD)
  cache_key="$({ sha256sum "$source" "$opencl_compat" "$ROOT_DIR/tinygrad_repo/examples/openpilot/compile3.py"; printf '%s\n' "$tinygrad_revision" 'DEV=CL FLOAT16=1 IMAGE=2 JIT_BATCH_SIZE=0 METADRIVE_GENERIC_OPENCL=1 INTEL_MATRIX_EXTENSION=AUTO SUBGROUP_SIZE=16'; } | sha256sum | cut -d ' ' -f1)"

  if [[ -s "$output" && -f "$cache_meta" && "$(<"$cache_meta")" == "$cache_key" ]]; then
    echo "Using cached OpenCL model: $output"
    return
  fi

  echo "Compiling ${model_name} for Tinygrad OpenCL (first run can take a few minutes)..."
  env \
    PYTHONPATH="$ROOT_DIR/tools/sim/gpu_compat:$ROOT_DIR/tinygrad_repo${PYTHONPATH:+:$PYTHONPATH}" \
    METADRIVE_GENERIC_OPENCL=1 \
    DEV=CL FLOAT16=1 IMAGE=2 JIT_BATCH_SIZE=0 \
    "$ROOT_DIR/.venv/bin/python3" \
    "$ROOT_DIR/tinygrad_repo/examples/openpilot/compile3.py" \
    "$source" "$output"

  printf '%s\n' "$cache_key" > "$cache_meta"
}

prepare_gpu_model driving_vision
prepare_gpu_model driving_policy

# The manager must not start its desktop CPU modeld. Terminal 2 launches the
# dedicated Tinygrad/OpenCL modeld after the simulator bridge is started.
export BLOCK="modeld${BLOCK:+,$BLOCK}"

echo
echo "Terminal 1: starting openpilot manager/UI. When it is ready, run this in Terminal 2:"
echo "  ./tools/sim/start_metadrive_bridge.sh"
exec bash "$ROOT_DIR/tools/sim/launch_openpilot.sh"
