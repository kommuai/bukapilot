#!/usr/bin/bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1
export QT_QPA_EVDEV_TOUCHSCREEN_PARAMETERS=/dev/input/event1:rotate=90


file_path="/system/priv-app/plusneossetup/apk_v1"

# Check if the file exists
if [ -e "$file_path" ]; then
    echo "File 'apk_v1' exists in /system/priv-app/plusneossetup/"
else
    echo "File 'apk_v1' does not exist in /system/priv-app/plusneossetup/"
    bash install_neos_apk.sh
fi

if [ -z "$REQUIRED_NEOS_VERSION" ]; then
  export REQUIRED_NEOS_VERSION="19.1"
fi

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="4"
fi

if [ -z "$PASSIVE" ]; then
  export PASSIVE="1"
fi

export STAGING_ROOT="/data/safe_staging"

export NOSENSOR=1
