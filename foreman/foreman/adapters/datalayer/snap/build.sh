#!/bin/bash
set -e
# Build instructions:

# 1. Build flatbuffers
#
# from foreman/datalayer/snap directory, run `make-bfbs.sh`.
# This will create the necessary flatbuffer files.

# Then, use colcon in the root of your workspace, but install files in foreman.
# Then we dump the isntalled files in the snap.
#
# colcon build --packages-up-to foreman --install-base src/foreman/foreman/foreman/adapters/datalayer/snap/foreman_snap_install
#

# Lastly, cd back into snap/ directory and run `./build.sh`

snapcraft clean && snapcraft pack
