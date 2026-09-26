#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source scripts/source_env.sh
# VirtualBox SVGA3D needed software GL. Real GPU / Docker with /dev/dri should not.
if [ -z "${LIBGL_ALWAYS_SOFTWARE+x}" ]; then
  if [ -e /dev/dri/renderD128 ] || [ -e /dev/dri/card0 ] || [ -e /dev/dri/card1 ]; then
    export LIBGL_ALWAYS_SOFTWARE=0
  else
    export LIBGL_ALWAYS_SOFTWARE=1
  fi
fi
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
export QT_AUTO_SCREEN_SCALE_FACTOR="${QT_AUTO_SCREEN_SCALE_FACTOR:-0}"
export OGRE_RTT_MODE="${OGRE_RTT_MODE:-Copy}"
sim_gui=true
sim_rviz=true
for sim_arg in "$@"; do
  case "$sim_arg" in
    gui:=*) sim_gui="${sim_arg#gui:=}" ;;
    rviz:=*) sim_rviz="${sim_arg#rviz:=}" ;;
  esac
done
if [ "$sim_gui" != false ] || [ "$sim_rviz" != false ]; then
  export DISPLAY="${DISPLAY:-:0}"
  if ! xrandr --query >/dev/null 2>&1; then
    echo "Cannot access desktop DISPLAY=$DISPLAY. Log in to the graphical desktop first."
    echo 'For no windows use: bash ~/smartcar_2026_ws/scripts/run_sim.sh gui:=false rviz:=false'
    exit 1
  fi
  echo "Windows will open on DISPLAY=$DISPLAY (software_gl=$LIBGL_ALWAYS_SOFTWARE)"
fi
exec roslaunch smartcar_bringup simulation.launch "$@"
