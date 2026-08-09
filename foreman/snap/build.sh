#!/bin/bash
set -e
# Build instructions:

# Use colcon in the root of your workspace, but install files in foreman.
# Then we dump the installed files in the snap.
#
# colcon build --packages-up-to foreman --install-base src/foreman/foreman/snap/foreman_snap_install
#

# Then, cd back into snap/ directory and run `./build.sh`

snapcraft clean && snapcraft pack
