#!/bin/bash
set -euo pipefail
cd /data/openpilot
g++ -c -fPIC -x assembler-with-cpp -o system/camerad/rk/rk_sdg_entry.o system/camerad/rk/rk_sdg_entry.S
g++ -c -fPIC -x assembler-with-cpp -o system/camerad/rk/rk_sdg_exit.o system/camerad/rk/rk_sdg_exit.S
g++ -shared -fPIC -O2 -std=c++17 -I. -I/usr/include/rkaiq \
  -o system/camerad/rk/librk_sdg_hook.so \
  system/camerad/rk/rk_sdg_hook.cc \
  system/camerad/rk/rk_sdg_entry.o \
  system/camerad/rk/rk_sdg_exit.o \
  -ldl
echo "built system/camerad/rk/librk_sdg_hook.so"
