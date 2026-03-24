#!/usr/bin/env bash
set -e

if [ -z "$RELEASE_BRANCH" ]; then
  echo "RELEASE_BRANCH is not set"
  exit 1
fi

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
cd $DIR

BUILD_DIR=/data/openpilot

export GIT_COMMITTER_NAME="Penyelidik Kereta"
export GIT_COMMITTER_EMAIL="bot@kommu.ai"
export GIT_AUTHOR_NAME="Penyelidik Kereta"
export GIT_AUTHOR_EMAIL="bot@kommu.ai"

# Build stuff
cd $BUILD_DIR

rm -f panda/board/obj/panda.bin.signed
rm -f panda/board/obj/panda_h7.bin.signed

VERSION=$(cat common/version.h | awk -F[\"-]  '{print $2}')
echo "#define COMMA_VERSION \"$VERSION-release\"" > common/version.h

ln -sfn /data/openpilot /data/pythonpath
export PYTHONPATH="$BUILD_DIR"
# Only 4 cores if build directly on device, in case out of ram
scons -j4

# Cleanup
find . -name '*.a' -delete
find . -name '*.o' -delete
find . -name '*.os' -delete
find . -name '*.pyc' -delete
find . -name '__pycache__' -delete
rm -rf panda/certs panda/crypto
rm -rf .sconsign.dblite Jenkinsfile release/
rm selfdrive/modeld/models/*.dlc
rm selfdrive/modeld/models/*.onnx

# Restore third_party
# git checkout third_party/

# Mark as prebuilt release
touch prebuilt

# Add built files to git
git add -f .

# Commit
git commit -m "bukapilot $RELEASE_BRANCH"

echo Done.
