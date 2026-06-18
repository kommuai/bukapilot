#!/bin/bash
# KA2-only deps: local Mull/LLVM installs for read-only rootfs (no apt to /etc or /usr).
# Sourced from setup.sh when /KA2 is present.

mull_linux_deb_name() {
  local arch ubuntu_ver
  arch="$(uname -m)"
  ubuntu_ver="24.04"
  if [ -r /etc/os-release ]; then
    # shellcheck source=/dev/null
    . /etc/os-release
    if [ "${ID:-}" = "ubuntu" ] && [ -n "${VERSION_ID:-}" ]; then
      ubuntu_ver="${VERSION_ID}"
    fi
  fi
  case "$arch" in
    aarch64|arm64)
      echo "Mull-18-${MULL_VERSION}-LLVM-18.1.3-ubuntu-aarch64-${ubuntu_ver}.deb"
      ;;
    x86_64|amd64)
      echo "Mull-18-${MULL_VERSION}-LLVM-18.1.3-ubuntu-amd64-${ubuntu_ver}.deb"
      ;;
    *)
      echo "unsupported architecture: $arch" >&2
      return 1
      ;;
  esac
}

clang_rt_profile_lib_name() {
  case "$(uname -m)" in
    aarch64|arm64) echo "libclang_rt.profile-aarch64.a" ;;
    x86_64|amd64) echo "libclang_rt.profile-x86_64.a" ;;
    *) echo "unsupported architecture: $(uname -m)" >&2; return 1 ;;
  esac
}

install_clang_rt_linux() {
  local profile_name system_profile local_profile deb_path deb_url
  profile_name="$(clang_rt_profile_lib_name)" || return 0

  system_profile="/usr/lib/llvm-18/lib/clang/18/lib/linux/$profile_name"
  if [ -f "$system_profile" ]; then
    return 0
  fi

  local_profile="$BASEDIR/.llvm/root/usr/lib/llvm-18/lib/clang/18/lib/linux/$profile_name"
  if [ -f "$local_profile" ]; then
    return 0
  fi

  mkdir -p "$BASEDIR/.llvm/cache" "$BASEDIR/.llvm/root"
  case "$(uname -m)" in
    aarch64|arm64)
      deb_url="http://ports.ubuntu.com/ubuntu-ports/pool/universe/l/llvm-toolchain-18/libclang-rt-18-dev_18.1.3-1ubuntu1_arm64.deb"
      ;;
    x86_64|amd64)
      deb_url="http://archive.ubuntu.com/ubuntu/pool/universe/l/llvm-toolchain-18/libclang-rt-18-dev_18.1.3-1ubuntu1_amd64.deb"
      ;;
    *) return 0 ;;
  esac
  deb_path="$BASEDIR/.llvm/cache/libclang-rt-18-dev.deb"

  if [ ! -f "$deb_path" ]; then
    echo "Downloading LLVM compiler-rt to $deb_path ..."
    if ! curl -fL -o "$deb_path" "$deb_url"; then
      rm -f "$deb_path"
      echo "LLVM compiler-rt download failed; coverage/Mull builds may fail."
      return 0
    fi
  fi
  if ! dpkg-deb -x "$deb_path" "$BASEDIR/.llvm/root"; then
    echo "LLVM compiler-rt extract failed; coverage/Mull builds may fail."
    return 0
  fi
}

setup_clang_resource_dir() {
  if ! command -v clang-18 > /dev/null 2>&1; then
    return 0
  fi

  local profile_name system_res merged system_profile local_profile
  profile_name="$(clang_rt_profile_lib_name)" || return 0
  system_res="$(clang-18 -print-resource-dir)"
  merged="$BASEDIR/.llvm/clang-resource"

  system_profile="$system_res/lib/linux/$profile_name"
  local_profile="$BASEDIR/.llvm/root/usr/lib/llvm-18/lib/clang/18/lib/linux/$profile_name"

  if [ -f "$merged/lib/linux/$profile_name" ]; then
    export CLANG_RESOURCE_DIR="$merged"
    return 0
  fi
  if [ -f "$system_profile" ]; then
    return 0
  fi
  if [ ! -f "$local_profile" ]; then
    return 0
  fi

  echo "Preparing local Clang resource dir at $merged ..."
  rm -rf "$merged"
  mkdir -p "$merged/lib/linux"
  for entry in "$system_res"/*; do
    ln -sfn "$entry" "$merged/$(basename "$entry")"
  done
  cp "$local_profile" "$merged/lib/linux/"
  export CLANG_RESOURCE_DIR="$merged"
}

install_mull_linux() {
  if [ -n "${SKIP_MULL:-}" ]; then
    echo "SKIP_MULL set; skipping Mull install (mutation tests unavailable)."
    return 0
  fi

  local mull_bin="$BASEDIR/.mull/root/usr/bin/mull-runner-18"
  if [ -x "$mull_bin" ]; then
    export PATH="$BASEDIR/.mull/root/usr/bin:$PATH"
    return 0
  fi
  if command -v "mull-runner-18" > /dev/null 2>&1; then
    return 0
  fi

  MULL_VERSION="${MULL_VERSION:-0.34.0}"
  local deb_name deb_path
  deb_name="$(mull_linux_deb_name)" || return 0
  deb_path="$BASEDIR/.mull/cache/$deb_name"

  mkdir -p "$BASEDIR/.mull/cache" "$BASEDIR/.mull/root"
  if [ ! -f "$deb_path" ]; then
    echo "Downloading Mull to $deb_path ..."
    if ! curl -fL -o "$deb_path" \
      "https://github.com/mull-project/mull/releases/download/${MULL_VERSION}/${deb_name}"; then
      rm -f "$deb_path"
      echo "Mull download failed; skipping (mutation tests unavailable)."
      return 0
    fi
  fi
  if ! dpkg-deb -x "$deb_path" "$BASEDIR/.mull/root"; then
    echo "Mull extract failed; skipping (mutation tests unavailable)."
    return 0
  fi
  export PATH="$BASEDIR/.mull/root/usr/bin:$PATH"

  if ! command -v clang-18 > /dev/null 2>&1; then
    echo "Note: clang-18 is required for mutation tests but was not found in PATH."
  fi
}

install_opendbc_ka2_deps() {
  install_clang_rt_linux
  setup_clang_resource_dir
  install_mull_linux
}
