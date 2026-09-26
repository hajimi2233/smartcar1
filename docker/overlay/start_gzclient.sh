#!/usr/bin/env bash
set -e
source /opt/ros/kinetic/setup.bash
export DISPLAY="${DISPLAY:-:0}"
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1
export OGRE_RTT_MODE=Copy
if pgrep -x gzclient >/dev/null; then
  echo "gzclient already running"
  exit 0
fi
exec rosrun gazebo_ros gzclient
