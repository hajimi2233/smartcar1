#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source scripts/source_env.sh
if ! rosnode list 2>/dev/null | grep -q '/gazebo'; then
  echo 'Start the simulation first.'
  exit 1
fi
rosnode kill /slam_gmapping /sim_reference_map /waypoint_nav \
  /amcl /slam_map_server /map_to_odom /imu_bridge \
  /scan_match_localizer /laser_scan_matcher_node >/dev/null 2>&1 || true
sleep 1
echo 'Localization: laser scan matcher + IMU + AMCL on the prior map; no wheel odometry.'
echo 'In RViz: 2D Pose Estimate, drag on the map to set initial pose and heading.'
exec roslaunch smartcar_navigation localization.launch
