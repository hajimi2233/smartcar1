#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if docker info >/dev/null 2>&1; then d=(docker); else d=(sudo docker); fi
c=("${d[@]}" compose -p smartcar-baseline)
ws=/home/hajimi/smartcar_2026_ws
case "${1:-help}" in
  build) "${c[@]}" build sim ;;
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
  logs) "${c[@]}" logs --tail=150 sim ;;
  shell) "${c[@]}" exec sim bash ;;
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
  *) echo 'Usage: bash scripts/smartcar.sh {build|start|rviz|gazebo|keyboard|mapping|save-map|localization|check|logs|shell|stop}' ;;
esac
