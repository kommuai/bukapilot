#!/usr/bin/env bash
set -e

export GIT_COMMITTER_NAME="Penyelidik Kereta"
export GIT_COMMITTER_EMAIL="bot@blackhole.kommu.ai"
export GIT_AUTHOR_NAME="Penyelidik Kereta"
export GIT_AUTHOR_EMAIL="bot@blackhole.kommu.ai"

VERSION=1-firstbatch

# Build stuff
ln -sfn /data/openpilot /data/pythonpath
export PYTHONPATH="/data/openpilot:/data/openpilot/pyextra"
scons -j3

# Run tests
python selfdrive/manager/test/test_manager.py
selfdrive/car/tests/test_car_interfaces.py

# Cleanup
find . -name '*.a' -delete
find . -name '*.o' -delete
find . -name '*.os' -delete
find . -name '*.pyc' -delete
find . -name '__pycache__' -delete
rm -rf panda/certs panda/crypto
rm -rf .sconsign.dblite Jenkinsfile release/
rm models/supercombo.dlc

# Restore phonelibs
git checkout third_party/

# Mark as prebuilt release
touch prebuilt

# Add built files to git
git add -f .

# Commit
git commit -m "bukapilot v$VERSION"

# Print committed files that are normally gitignored
#git status --ignored

echo Done.
