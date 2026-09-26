#!/usr/bin/env bash
# Write a one-shot health snapshot. Does not start or kill anything.
set +e
source /opt/ros/kinetic/setup.bash
source /home/hajimi/smartcar_2026_ws/devel/setup.bash
LOG=/home/hajimi/smartcar_2026_ws/logs/health_$(date +%Y%m%d_%H%M%S).log
mkdir -p /home/hajimi/smartcar_2026_ws/logs
{
  echo "=== $(date -Is) ==="
  echo "DISPLAY=$DISPLAY LIBGL_ALWAYS_SOFTWARE=$LIBGL_ALWAYS_SOFTWARE"
  echo "--- processes ---"
  ps -ef | grep -E "gzserver|gzclient|rviz|roslaunch" | grep -v grep
  echo "--- nodes ---"
  rosnode list 2>/dev/null
  echo "--- required topics ---"
  for t in /clock /sim/cmd_vel /sim/scan /sim/joint_states /sim/imu /imu/data /map /amcl_pose /scan; do
    if timeout 1 rostopic echo -n 1 "$t" >/dev/null 2>&1; then
      echo "OK  $t"
    else
      echo "FAIL $t"
    fi
  done
  echo "--- steer joints ---"
  timeout 2 rostopic echo -n 1 /sim/joint_states 2>/dev/null | tail -8
} | tee "$LOG"
echo "wrote $LOG"
