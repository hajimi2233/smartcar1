#!/usr/bin/env bash
# Docker management and transport only; stage semantics live in robot.sh.
set -euo pipefail
cd "$(dirname "$0")/.."
if docker info >/dev/null 2>&1; then d=(docker); else d=(sudo docker); fi
c=("${d[@]}" compose -p smartcar-layered)
action="${1:-help}"; if [ "$#" -gt 0 ]; then shift; fi
case "$action" in
  build) COMPOSE_BAKE=false DOCKER_BUILDKIT=0 "${c[@]}" build sim ;;
  start)
    for name in smartcar-baseline smartcar-sim smartcar-sim-gui; do
      if [ "$("${d[@]}" inspect -f '{{.State.Running}}' "$name" 2>/dev/null || true)" = true ]; then
        echo "Stop old container $name before starting this project (shared ROS ports)."; exit 2
      fi
    done
    "${c[@]}" up -d sim ;;
  stop) "${c[@]}" stop sim ;;
  logs) "${c[@]}" logs --tail=120 sim ;;
  shell) "${c[@]}" exec sim bash ;;
  help) echo 'sim.sh build|start|stop|logs|shell; sim.sh <robot.sh stage/tool> [arguments]' ;;
  *) "${c[@]}" exec -e SMARTCAR_PROFILE=sim -e SMARTCAR_ROOT=/home/hajimi/smartcar sim bash -lc 'source /opt/ros/kinetic/setup.bash; source /home/hajimi/smartcar_ws/devel/setup.bash; exec bash /home/hajimi/smartcar/scripts/robot.sh "$@"' -- "$action" "$@" ;;
esac
