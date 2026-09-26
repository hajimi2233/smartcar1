#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source scripts/source_env.sh
if ! timeout 3 rosnode list >/dev/null 2>&1; then
  echo 'ROS master is unavailable. Start the simulation first.'; exit 1
fi
if rosnode list 2>/dev/null | grep -qx '/single_goal_nav'; then
  echo 'Close the navigation terminal with Ctrl+C before starting keyboard control.'
  exit 1
fi
exec rosrun smartcar_navigation terminal_teleop.py
