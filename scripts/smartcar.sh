#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if docker info >/dev/null 2>&1; then d=(docker); else d=(sudo docker); fi
c=("${d[@]}" compose -p smartcar-baseline)
ws=/home/hajimi/smartcar_2026_ws
case "${1:-help}" in
  build) COMPOSE_BAKE=false DOCKER_BUILDKIT=0 "${c[@]}" build sim ;;
  start)
    for name in smartcar-sim smartcar-sim-gui; do
      if [ "$("${d[@]}" inspect -f '{{.State.Running}}' "$name" 2>/dev/null || true)" = true ]; then
        echo "Stop the old container first: $name (shared ROS ports)"; exit 1
      fi
    done
    "${c[@]}" up -d sim
    for attempt in $(seq 1 30); do
      if "${c[@]}" exec -T sim bash -lc 'source /opt/ros/kinetic/setup.bash; timeout 2 rostopic echo -n 1 /scan/header' >/dev/null 2>&1; then
        echo 'Scan received. Run check, then RViz and set initial pose.'; exit 0
      fi
      sleep 1
    done
    echo 'No scan received; run logs. Startup NOT verified.'; exit 1 ;;
  stop) "${c[@]}" stop sim ;;
  reset) "${c[@]}" exec -T sim bash /home/hajimi/smartcar_2026_ws/src/smartcar_navigation/scripts/reset_to_start.sh ;;
  logs) "${c[@]}" logs --tail=150 sim ;;
  shell) "${c[@]}" exec sim bash ;;
  region)
    action="${2:-status}"
    case "$action" in
      begin|undo|save|cancel|clear)
        "${c[@]}" exec -T sim bash -lc 'source /opt/ros/kinetic/setup.bash; rosservice call "/region_editor/$1"' -- "$action" ;;
      status) "${c[@]}" exec -T sim bash -lc 'source /opt/ros/kinetic/setup.bash; timeout 3 rostopic echo -n 1 /external_region/status' ;;
      *) echo 'Usage: region {begin|undo|save|cancel|clear|status}'; exit 1 ;;
    esac ;;
  nav) "${c[@]}" exec sim bash -lc 'source /opt/ros/kinetic/setup.bash; source ~/smartcar_2026_ws/devel/setup.bash; exec roslaunch smartcar_navigation single_goal.launch' ;;
  nav-test) "${c[@]}" exec sim bash -lc 'source /opt/ros/kinetic/setup.bash; source ~/smartcar_2026_ws/devel/setup.bash; exec roslaunch smartcar_navigation single_goal.launch ground_truth_test:=true test_auto_arrive:=true test_auto_arrive_delay:=1.0' ;;
  multi-nav) "${c[@]}" exec sim bash -lc 'source /opt/ros/kinetic/setup.bash; source ~/smartcar_2026_ws/devel/setup.bash; exec roslaunch smartcar_navigation multi_goal_nav.launch' ;;
  multi-clear) "${c[@]}" exec -T sim bash -lc 'source /opt/ros/kinetic/setup.bash; rosservice call /multi_goal_nav/clear' ;;
  multi-undo) "${c[@]}" exec -T sim bash -lc 'source /opt/ros/kinetic/setup.bash; rosservice call /multi_goal_nav/undo' ;;
  multi-execute) "${c[@]}" exec -T sim bash -lc 'source /opt/ros/kinetic/setup.bash; rosservice call /multi_goal_nav/execute' ;;
  multi-cancel) "${c[@]}" exec -T sim bash -lc 'source /opt/ros/kinetic/setup.bash; rosservice call /multi_goal_nav/cancel' ;;
  goal)
    shift
    if [ "$#" -ne 3 ]; then echo 'Usage: bash scripts/smartcar.sh goal X Y YAW_DEGREES'; exit 1; fi
    "${c[@]}" exec -T sim bash "$ws/scripts/send_single_goal.sh" "$@" ;;
  clear-line) "${c[@]}" exec -T sim bash -lc 'source /opt/ros/kinetic/setup.bash; rosservice call /single_nav/clear_zero_cost_line' ;;
  cancel) "${c[@]}" exec -T sim bash -lc 'source /opt/ros/kinetic/setup.bash; timeout 4 rostopic pub -1 /single_nav/cancel std_msgs/String "data: cancel"' ;;
  nav-status) "${c[@]}" exec sim bash -lc 'source /opt/ros/kinetic/setup.bash; rostopic echo /single_nav/status' ;;
  mapping) "${c[@]}" exec sim bash "$ws/scripts/start_mapping.sh" ;;
  localization) "${c[@]}" exec sim bash "$ws/scripts/start_localization.sh" ;;
  save-map) "${c[@]}" exec sim bash "$ws/scripts/save_map.sh" ;;
  keyboard) "${c[@]}" exec sim bash "$ws/scripts/keyboard.sh" ;;
  rviz) "${c[@]}" exec sim bash -lc 'source /opt/ros/kinetic/setup.bash; source ~/smartcar_2026_ws/devel/setup.bash; exec rosrun rviz rviz -d ~/smartcar_2026_ws/src/smartcar_sim/config/inspection.rviz' ;;
  gazebo) "${c[@]}" exec sim gzclient ;;
  check)
    "${c[@]}" exec -T sim bash -lc 'source /opt/ros/kinetic/setup.bash; source ~/smartcar_2026_ws/devel/setup.bash
      set -e
      rosnode ping -c 1 /gazebo
      rosnode ping -c 1 /amcl
      rosnode ping -c 1 /laser_scan_matcher_node
      timeout 5 rostopic echo -n 1 /scan/header
      timeout 5 rostopic echo -n 1 /map/info'
    echo 'Node/topic checks passed; inspect localization and motion in RViz separately.' ;;
  *) echo 'Usage: bash scripts/smartcar.sh {build|start|rviz|gazebo|keyboard|nav|nav-test|multi-nav|multi-clear|multi-undo|multi-execute|multi-cancel|goal X Y YAW_DEGREES|cancel|nav-status|mapping|save-map|localization|check|logs|shell|stop}' ;;
esac
