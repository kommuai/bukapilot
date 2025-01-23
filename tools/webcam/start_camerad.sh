#!/bin/bash

# export the block below when call manager.py
export BLOCK="${BLOCK},camerad"
export USE_WEBCAM="1"

# Change camera index according to your setting
export CAMERA_ROAD_ID="42"
export CAMERA_DRIVER_ID="33"
export DUAL_CAMERA="51" # camera index for wide road camera

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

$DIR/camerad.py
