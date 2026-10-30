#!/usr/bin/env bash

export PASSIVE="0"
export NOBOARD="1"
export SIMULATION="1"
export SKIP_FW_QUERY="1"
export FINGERPRINT="HONDA_CIVIC_2022"

# Desktop MetaDrive has no device registration or uploader session. Keep
# cloud and device-only services disabled for simulator runs.
export BLOCK="${BLOCK},manage_athenad,uploader,soundd,camerad,loggerd,encoderd,micd,logmessaged"
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

python3 - <<'PY'
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.system.version import terms_version, training_version

params = Params()
params.put("HasAcceptedTerms", terms_version)
params.put("CompletedTrainingVersion", training_version)
params.put_bool("OpenpilotEnabledToggle", True)

calibration = messaging.new_message("liveCalibration")
calibration.liveCalibration.validBlocks = 20
calibration.liveCalibration.rpyCalib = [0.0, 0.0, 0.0]
params.put("CalibrationParams", calibration.to_bytes())
PY

# The manager requires a DongleId before starting processes. Use a local
# simulator identity so startup never registers with Kommu (including when
# Params uses its default root).
python3 -c 'from openpilot.common.params import Params; Params().put("DongleId", "SIMULATOR")'

SCRIPT_DIR=$(dirname "$0")
OPENPILOT_DIR=$SCRIPT_DIR/../../

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
cd $OPENPILOT_DIR/system/manager && exec ./manager.py
