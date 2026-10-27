#!/usr/bin/env bash

export PASSIVE="0"
export NOBOARD="1"
export SIMULATION="1"
export SKIP_FW_QUERY="1"
export FINGERPRINT="HONDA_CIVIC_2022"

# Desktop MetaDrive has no ALSA mixer and cannot upload to Kommu, so keep
# those services disabled for simulator runs.
export BLOCK="${BLOCK},soundd,uploader,camerad,loggerd,encoderd,micd,logmessaged"
export USE_RKNN="${USE_RKNN:-0}"
export BIG="${BIG:-1}"
export SCALE="${SCALE:-0.6}"
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  export BLOCK="${BLOCK},ui"
fi
if [[ "$CI" ]]; then
  # TODO: offscreen UI should work
  export BLOCK="${BLOCK},ui"
fi

python3 -c "from openpilot.selfdrive.test.helpers import set_params_enabled; set_params_enabled()"

# This branch's registration helper blocks on Kommu even on PC. When the
# caller supplies an isolated params root, seed the active prefix so manager
# startup remains offline and deterministic (pytest uses a fresh prefix).
if [[ -n "${PARAMS_ROOT:-}" ]]; then
  python3 -c 'from openpilot.common.params import Params; Params().put("DongleId", "SIMULATOR")'
fi

SCRIPT_DIR=$(dirname "$0")
OPENPILOT_DIR=$SCRIPT_DIR/../../

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
cd $OPENPILOT_DIR/system/manager && exec ./manager.py
