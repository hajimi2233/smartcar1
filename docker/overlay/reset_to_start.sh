#!/usr/bin/env bash
# Put the car at the field start. Does not restart Gazebo/RViz:
# reset_world + gzclient restart was killing the Gazebo ROS API.
cd "$(dirname "$0")/.."
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"
if [ -f scripts/source_env.sh ]; then
  source scripts/source_env.sh
else
  source /opt/ros/kinetic/setup.bash
  [ -f devel/setup.bash ] && source devel/setup.bash
fi
export DISPLAY="${DISPLAY:-:0}"

if ! rosservice list 2>/dev/null | grep -q '/gazebo/set_model_state'; then
  echo '[reset] ERROR: Gazebo ROS API is down (/gazebo/set_model_state missing).'
  echo 'On the host run:  bash ~/smartcar/scripts/reset-to-start.sh'
  exit 2
fi

echo '[reset] stopping motion and placing car at start'
python /home/hajimi/smartcar_2026_ws/src/smartcar_navigation/scripts/reset_to_start.py
echo '[reset] done. Car at start (3.86, 2.80).'
