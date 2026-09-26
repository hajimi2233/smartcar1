#!/usr/bin/env bash
# Restart localization without killing the docker exec session.
# Usage (on the host):  bash ~/smartcar/scripts/restart-localization.sh
set -euo pipefail
if docker info >/dev/null 2>&1; then d=(docker); else d=(sudo docker); fi
"${d[@]}" exec smartcar-sim bash -lc \
  'source /opt/ros/kinetic/setup.bash; source ~/smartcar_2026_ws/devel/setup.bash
   for n in /amcl /amcl_tf_relay /wheel_odom /laser_noise /imu_bridge /scan_in_map /scan_match_localizer /laser_scan_matcher_node /slam_map_server; do
     rosnode kill $n >/dev/null 2>&1 || true
   done'
sleep 2
"${d[@]}" exec -d smartcar-sim bash -lc \
  'source /opt/ros/kinetic/setup.bash; source ~/smartcar_2026_ws/devel/setup.bash
   roslaunch smartcar_navigation localization.launch > /tmp/localization.log 2>&1'
echo "OK: localization started in background. RViz should move again after 2D Pose Estimate."
