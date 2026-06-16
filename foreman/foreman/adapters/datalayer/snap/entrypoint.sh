#!/bin/bash
set -e

while ! snapctl is-connected active-solution
do
  sleep 1
done

while ! snapctl is-connected ros-base
do
  sleep 1
done

# Path setup
# We're connection to ros-jazzy base snap for runtime information
export ROS_BASE=$SNAP/rosruntime

export PYTHONPATH=$PYTHONPATH:$ROS_BASE/lib/python3.12/site-packages
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROS_BASE/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROS_BASE/usr/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROS_BASE/usr/include/comm/datalayer/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROS_BASE/usr/lib/x86_64-linux-gnu/
export PATH=${PATH}:${ROS_BASE}/opt/ros/jazzy/bin
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${ROS_BASE}/opt/ros/jazzy/lib

source $ROS_BASE/opt/ros/jazzy/setup.bash
# source install dir we dumped from building the foreman packages
source $SNAP/local_setup.bash

# env file setup
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

# config file setup
CONFIG_FILE="$CONFIG_DIR/scenario.yaml"
DEFAULT_CONFIG="$SNAP/foreman/share/foreman/config/scenario.yaml"

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
