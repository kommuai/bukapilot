#!/usr/bin/env bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
cd "$DIR"

source ../../../setup.sh

rm -f ./libsafety/*.gcda
scons -j"$(nproc)" -D

pytest test_dnga.py -v "$@"
