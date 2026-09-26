#!/usr/bin/env bash
set -e
source /opt/ros/kinetic/setup.bash
source /home/hajimi/smartcar_2026_ws/devel/setup.bash
exec rosrun smartcar_navigation send_single_goal.py "$@"
