#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/kinetic/setup.bash
if [ -f /home/hajimi/smartcar_2026_ws/devel/setup.bash ]; then
  source /home/hajimi/smartcar_2026_ws/devel/setup.bash
fi
set -u
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-0}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
export QT_AUTO_SCREEN_SCALE_FACTOR="${QT_AUTO_SCREEN_SCALE_FACTOR:-0}"
export OGRE_RTT_MODE="${OGRE_RTT_MODE:-Copy}"
export GAZEBO_PLUGIN_PATH="/home/hajimi/smartcar_2026_ws/devel/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
export GAZEBO_MODEL_DATABASE_URI=""

cmd="${1:-sim}"
if [ $# -gt 0 ]; then
  shift
fi
case "$cmd" in
  sim)
    if [ $# -eq 0 ]; then
      set -- gui:=false rviz:=false
    fi
    exec bash /home/hajimi/smartcar_2026_ws/scripts/run_sim.sh "$@"
    ;;
  sim-gui)
    exec bash /home/hajimi/smartcar_2026_ws/scripts/run_sim.sh "$@"
    ;;
  keyboard)
    exec bash /home/hajimi/smartcar_2026_ws/scripts/keyboard.sh "$@"
    ;;
  shell|bash)
    exec bash "$@"
    ;;
  *)
    exec "$cmd" "$@"
    ;;
esac
