#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
scons -u -j8 --minimal .
export DEBUG_FRAMES=1
export DISABLE_ROAD=1 DISABLE_WIDE_ROAD=1
export LOGPRINT=debug
exec ./camerad
