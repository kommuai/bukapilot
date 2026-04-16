#!/usr/bin/env bash
set -e
set -x

if [ -z "$RELEASE_BRANCH" ]; then
  echo "RELEASE_BRANCH is not set"
  exit 1
fi

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
cd "$DIR"

# Source repo is expected to be the parent of this release/ directory.
# This script builds into a separate BUILD_DIR to avoid wiping the checkout.
SOURCE_DIR="$(cd "$DIR/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-/data/openpilot-release}"

if [ "$BUILD_DIR" = "$SOURCE_DIR" ]; then
  echo "Refusing to build: BUILD_DIR equals SOURCE_DIR ($BUILD_DIR)"
  echo "Set BUILD_DIR to a different path, e.g. BUILD_DIR=/data/openpilot-release"
  exit 1
fi

# Kommu identity (overrides identity.sh in upstream script)
export GIT_COMMITTER_NAME="${GIT_COMMITTER_NAME:-Penyelidik Kereta}"
export GIT_COMMITTER_EMAIL="${GIT_COMMITTER_EMAIL:-bot@kommu.ai}"
export GIT_AUTHOR_NAME="${GIT_AUTHOR_NAME:-Penyelidik Kereta}"
export GIT_AUTHOR_EMAIL="${GIT_AUTHOR_EMAIL:-bot@kommu.ai}"

REMOTE_URL="${REMOTE_URL:-$(git -C "$SOURCE_DIR" remote get-url origin 2>/dev/null || true)}"
if [ -z "$REMOTE_URL" ]; then
  echo "REMOTE_URL is not set and no origin remote found in SOURCE_DIR"
  exit 1
fi

PROJECT_NAME="${PROJECT_NAME:-bukapilot}"

echo "[-] Setting up repo T=$SECONDS"
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
git init
git remote add origin "$REMOTE_URL"
git checkout --orphan "$RELEASE_BRANCH"

echo "[-] copying files T=$SECONDS"
RELEASE_FILES_SCRIPT="$SOURCE_DIR/release/release_files.py"
if [ ! -f "$RELEASE_FILES_SCRIPT" ]; then
  echo "Missing $RELEASE_FILES_SCRIPT"
  exit 1
fi
cd "$SOURCE_DIR"
cp -pR --parents $("$RELEASE_FILES_SCRIPT") "$BUILD_DIR/"

cd "$BUILD_DIR"

rm -f panda/board/obj/panda.bin.signed
rm -f panda/board/obj/panda_h7.bin.signed

VERSION=$(cat common/version.h | awk -F[\"-]  '{print $2}')
echo "[-] committing version $VERSION T=$SECONDS"
# Kommu convention: mark release builds in version string
echo "#define COMMA_VERSION \"$VERSION-release\"" > common/version.h
git add -f .
git commit -a -m "$PROJECT_NAME v$VERSION release"

# Build
export PYTHONPATH="$BUILD_DIR"

# Kommu builds often run on-device; default to fewer jobs to avoid OOM.
if [ -z "${JOBS:-}" ]; then
  if [ -e /EON ] || [ -e /TICI ] || [ -e /AGNOS ]; then
    JOBS=4
  else
    JOBS="$(nproc)"
  fi
fi

scons -j"$JOBS" --minimal

if [ -z "${PANDA_DEBUG_BUILD:-}" ]; then
  PANDA_CERT="${PANDA_CERT:-/data/pandaextra/certs/release}"
  if [ -d "$PANDA_CERT" ]; then
    CERT="$PANDA_CERT" RELEASE=1 scons -j"$JOBS" panda/
  else
    RELEASE=1 scons -j"$JOBS" panda/
  fi
else
  scons -j"$JOBS" panda/
fi

# Ensure no submodules in release
if test "$(git submodule--helper list | wc -l)" -gt "0"; then
  echo "submodules found:"
  git submodule--helper list
  exit 1
fi
git submodule status

# Cleanup
find . -name '*.a' -delete
find . -name '*.o' -delete
find . -name '*.os' -delete
find . -name '*.pyc' -delete
find . -name 'moc_*' -delete
find . -name '__pycache__' -delete
rm -rf .sconsign.dblite Jenkinsfile release/
rm -rf panda/certs panda/crypto
rm selfdrive/modeld/models/driving_vision.onnx
rm selfdrive/modeld/models/driving_policy.onnx

find third_party/ -name '*x86*' -exec rm -r {} +
find third_party/ -name '*Darwin*' -exec rm -r {} +

# Restore third_party
git checkout third_party/

# Mark as prebuilt release
touch prebuilt

# Add built files to git
git add -f .

# Finalize commit message (same flow as upstream script)
git commit --amend -m "$PROJECT_NAME v$VERSION"

# Run tests (keep consistent with upstream; can be skipped via SKIP_TESTS=1)
if [ -z "${SKIP_TESTS:-}" ]; then
  cd "$BUILD_DIR"
  RELEASE=1 pytest -n0 -s selfdrive/test/test_onroad.py
fi

echo "[-] pushing release T=$SECONDS"
git push -f origin "$RELEASE_BRANCH:$RELEASE_BRANCH"

echo "[-] done T=$SECONDS"
