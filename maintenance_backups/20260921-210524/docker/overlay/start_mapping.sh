#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source scripts/source_env.sh
if ! rosnode list 2>/dev/null | grep -q '/gazebo'; then
  echo 'Start the simulation first, then run this again.'
  exit 1
fi
if rosnode list 2>/dev/null | grep -q '/slam_gmapping'; then
  echo 'gmapping is already running.'
  exit 0
fi
rosnode kill /sim_reference_map >/dev/null 2>&1 || true
rosnode kill /amcl /slam_map_server /scan_in_map /laser_scan_matcher_node /imu_bridge /laser_noise >/dev/null 2>&1 || true
sleep 1
echo 'Mapping started with laser + IMU local motion. Drive the car, then save with: bash scripts/save_map.sh'
exec roslaunch smartcar_mapping gmapping.launch
