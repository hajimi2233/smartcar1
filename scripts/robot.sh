#!/usr/bin/env bash
# Native ROS entry point. Every long-running stage stays in its own terminal.
set -euo pipefail
project_root="${SMARTCAR_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
profile="${SMARTCAR_PROFILE:-real}"
case "$profile" in sim) nav_config="$project_root/config/navigation.yaml" ;; real) nav_config="$project_root/config/navigation-real.yaml" ;; *) echo 'SMARTCAR_PROFILE must be sim or real'; exit 2 ;; esac
nav_config="${SMARTCAR_NAV_CONFIG:-$nav_config}"
stage="${1:-help}"
if [ "$#" -gt 0 ]; then shift; fi
require_ros() { command -v roslaunch >/dev/null || { echo 'Source /opt/ros/kinetic/setup.bash and your workspace devel/setup.bash first.'; exit 2; }; }
check_abs_file() { case "$1" in /*) ;; *) echo "Expected absolute file path: $1"; exit 2 ;; esac; test -f "$1" || { echo "Missing file: $1"; exit 2; }; }
reject_nodes() {
  local nodes n
  nodes="$(rosnode list 2>/dev/null || true)"
  for n in "$@"; do
    if printf '%s\n' "$nodes" | grep -Fxq "$n"; then
      echo "Stage already active or conflicting: $n. Stop its launch terminal with Ctrl+C first."; exit 2
    fi
  done
}
case "$stage" in
  drivers-sim)
    require_ros; reject_nodes /gazebo /laser_noise
    exec roslaunch smartcar_bringup drivers_sim.launch data_dir:="$project_root/data/maps/sim_field_v1" sensor_config:="$project_root/config/simulation.yaml" "$@" ;;
  drivers-real)
    require_ros
    if [ "$#" -ne 2 ]; then echo 'Usage: robot.sh drivers-real /absolute/chassis_adapter.launch /absolute/lidar.launch'; exit 2; fi
    check_abs_file "$1"; check_abs_file "$2"
    reject_nodes /gazebo /laser_noise
    exec roslaunch smartcar_bringup drivers_real.launch chassis_launch:="$1" lidar_launch:="$2" ;;
  mapping)
    require_ros; reject_nodes /slam_gmapping /amcl /slam_map_server /single_goal_nav /multi_goal_nav
    exec roslaunch smartcar_mapping gmapping.launch "$@" ;;
  save-map)
    require_ros
    if [ "$#" -ne 1 ]; then echo 'Usage: robot.sh save-map /absolute/output/map_basename'; exit 2; fi
    case "$1" in /*) ;; *) echo 'Use an absolute output path'; exit 2 ;; esac
    if [ -e "$1.yaml" ] || [ -e "$1.pgm" ]; then echo 'Map already exists; choose a new basename.'; exit 2; fi
    mkdir -p "$(dirname "$1")"
    exec rosrun map_server map_saver -f "$1" ;;
  localization)
    require_ros; reject_nodes /slam_gmapping /amcl /slam_map_server /single_goal_nav /multi_goal_nav
    if [ "$#" -lt 1 ]; then echo 'Usage: robot.sh localization /absolute/map.yaml [scan_odometry:=false ...]'; exit 2; fi
    check_abs_file "$1"; map_file="$1"; shift
    exec roslaunch smartcar_localization localization.launch map_file:="$map_file" params:="$project_root/config/localization.yaml" "$@" ;;
  single|multi|planning-test)
    require_ros; reject_nodes /single_goal_nav /multi_goal_nav /slam_gmapping
    if [ "$stage" = planning-test ]; then
      if [ "$profile" != sim ]; then echo 'planning-test requires SMARTCAR_PROFILE=sim and Gazebo.'; exit 2; fi
      launch=planning_test.launch
    else launch="$stage.launch"; fi
    exec roslaunch smartcar_bringup "$launch" config_file:="$nav_config" "$@" ;;
  rviz|rviz-multi)
    require_ros
    view=single; if [ "$stage" = rviz-multi ]; then view=multi; fi
    exec rosrun rviz rviz -d "$(rospack find smartcar_bringup)/rviz/$view.rviz" "$@" ;;
  keyboard-sim)
    require_ros
    if [ "$profile" != sim ]; then echo 'Use the real driver vendor teleop for initial chassis commissioning.'; exit 2; fi
    reject_nodes /single_goal_nav
    exec rosrun smartcar_sim terminal_teleop.py ;;
  goal)
    require_ros
    if [ "$#" -ne 3 ]; then echo 'Usage: robot.sh goal X Y YAW_DEGREES'; exit 2; fi
    reject_nodes /multi_goal_nav
    exec rosrun smartcar_navigation send_single_goal.py "$@" ;;
  multi-execute|multi-clear|multi-undo|multi-cancel)
    require_ros; exec rosservice call "/multi_goal_nav/${stage#multi-}" ;;
  cancel)
    require_ros; exec rostopic pub -1 /single_nav/cancel std_msgs/String 'data: cancel' ;;
  check)
    require_ros
    if [ "$#" -ne 1 ]; then echo 'Usage: robot.sh check drivers|mapping|localization|single|multi'; exit 2; fi
    exec rosrun smartcar_drivers check_stage.py _stage:="$1" _profile:="$profile" _navigation_config:="$nav_config" ;;
  *)
    echo 'Stages: drivers-real, drivers-sim, mapping, save-map, localization, single, multi, planning-test'
    echo 'Tools: check STAGE, rviz, rviz-multi, keyboard-sim, goal X Y YAW, cancel, multi-execute/clear/undo/cancel'
    echo 'See docs/REPRODUCE.md for the stage-by-stage workflow.' ;;
esac
