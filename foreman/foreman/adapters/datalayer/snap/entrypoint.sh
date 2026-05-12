#!/bin/bash
set -e

# --- Base Snap Approach Environment Setup ---
# Because we removed the ROS 2 snapcraft extension, we must manually 
# set up the paths to point to the mounted Rexroth base snap.
export ROS_BASE=$SNAP/rosruntime

# Export python and lib paths linked to the base snap
export PYTHONPATH=$PYTHONPATH:$ROS_BASE/lib/python3.12/site-packages
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROS_BASE/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROS_BASE/usr/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROS_BASE/usr/include/comm/datalayer/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROS_BASE/usr/lib/x86_64-linux-gnu/
export PATH=${PATH}:${ROS_BASE}/opt/ros/jazzy/bin
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${ROS_BASE}/opt/ros/jazzy/lib

# Source the ROS 2 setup from the base snap
source $ROS_BASE/opt/ros/jazzy/setup.bash
# Source the local setup dumped from the colcon install directory
source $SNAP/local_setup.bash

while ! snapctl is-connected active-solution
do
  sleep 5
done

CONFIG_DIR="$SNAP_COMMON/solutions/activeConfiguration/foreman"
ENV_FILE="$CONFIG_DIR/foreman.env"
DEFAULT_ENV_FILE="$SNAP/etc/foreman.default.env"

if [ ! -f "$ENV_FILE" ]; then
    echo "Creating default configuration at $ENV_FILE"
    mkdir -p "$CONFIG_DIR"
    cp "$DEFAULT_ENV_FILE" "$ENV_FILE"
fi

source "$ENV_FILE"

echo "Loading configuration from $ENV_FILE"

# Config file setup
CONFIG_FILE="$CONFIG_DIR/scenario.yaml"
DEFAULT_CONFIG="$SNAP/share/foreman/config/scenario.yaml"

mkdir -p "$CONFIG_DIR"

if [ ! -f "$CONFIG_FILE" ]; then
    echo "Config file not found at $CONFIG_FILE."
    if [ -f "$DEFAULT_CONFIG" ]; then
        echo "Populating with default scenario config..."
        cp "$DEFAULT_CONFIG" "$CONFIG_FILE"
    else
        echo "WARNING: Default config $DEFAULT_CONFIG not found in snap."
    fi
else
    echo "Using existing config: $CONFIG_FILE"
fi

exec ros2 run foreman foreman_node --ros-args -p config_path:="$CONFIG_FILE"