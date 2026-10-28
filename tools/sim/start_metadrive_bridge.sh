#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"

if [[ ! -x "$ROOT_DIR/.venv/bin/python3" ]]; then
  echo "Missing checkout virtualenv at $ROOT_DIR/.venv. Start Terminal 1 first." >&2
  exit 1
fi

export PATH="$ROOT_DIR/.venv/bin:$PATH"
export PARAMS_ROOT="${PARAMS_ROOT:-/tmp/bukapilot-metadrive-params}"
export METADRIVE_GPU_MODEL_DIR="${METADRIVE_GPU_MODEL_DIR:-/tmp/bukapilot-metadrive-gpu-models}"
export USE_RKNN=0
mkdir -p "$PARAMS_ROOT"

if (( $# == 0 )); then
  if [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    set -- --render
  else
    set -- --headless
  fi
fi

bridge_pid=""
modeld_pid=""
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if [[ -n "$modeld_pid" ]] && kill -0 "$modeld_pid" 2>/dev/null; then
    kill -TERM "$modeld_pid" 2>/dev/null || true
  fi
  if [[ -n "$bridge_pid" ]]; then
    # The supervisor forwards shutdown through the bridge's quit queue and
    # joins its simulator worker before it exits.
    if kill -0 "$bridge_pid" 2>/dev/null; then
      kill -TERM "$bridge_pid" 2>/dev/null || true
    fi
  fi
  [[ -z "$modeld_pid" ]] || wait "$modeld_pid" 2>/dev/null || true
  [[ -z "$bridge_pid" ]] || wait "$bridge_pid" 2>/dev/null || true
  return "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "Starting MetaDrive bridge ($*) and Tinygrad/OpenCL modeld..."
"$ROOT_DIR/.venv/bin/python3" -u "$ROOT_DIR/tools/sim/run_metadrive_bridge_process.py" \
  "$@" &
bridge_pid=$!

# This runner waits for camerad's VisionIPC stream, so it is safe to start
# alongside the bridge while the simulator window initializes.
env \
  PYTHONPATH="$ROOT_DIR/tools/sim/gpu_compat:$ROOT_DIR/tinygrad_repo${PYTHONPATH:+:$PYTHONPATH}" \
  METADRIVE_GPU_MODEL_DIR="$METADRIVE_GPU_MODEL_DIR" \
  PARAMS_ROOT="$PARAMS_ROOT" USE_RKNN=0 \
  "$ROOT_DIR/.venv/bin/python3" -u "$ROOT_DIR/tools/sim/run_metadrive_gpu_modeld.py" &
modeld_pid=$!

# Stop the sibling process when either the bridge or modeld exits.
wait -n "$bridge_pid" "$modeld_pid"
