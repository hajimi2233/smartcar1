#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source scripts/source_env.sh
if ! rosnode list 2>/dev/null | grep -q '/gazebo'; then
  echo 'Start the simulation first.'
  exit 1
fi
if rosnode list 2>/dev/null | grep -q '/waypoint_nav'; then
  echo 'Navigation is already running.'
  exit 0
fi
# AMCL publishes map->sim_world; stop SLAM so the two do not fight.
rosnode kill /slam_gmapping /laser_scan_matcher_node /imu_bridge /laser_noise >/dev/null 2>&1 || true
rosnode kill /sim_reference_map >/dev/null 2>&1 || true
sleep 1
echo 'Starting AMCL + waypoint navigation.'
exec roslaunch smartcar_navigation navigation.launch
