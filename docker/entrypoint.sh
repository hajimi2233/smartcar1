#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/kinetic/setup.bash
source /home/hajimi/smartcar_ws/devel/setup.bash
export GAZEBO_PLUGIN_PATH="/home/hajimi/smartcar_ws/devel/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
export GAZEBO_MODEL_DATABASE_URI=""
export SMARTCAR_PROFILE=sim
export SMARTCAR_ROOT=/home/hajimi/smartcar
exec bash "$SMARTCAR_ROOT/scripts/robot.sh" "$@"
