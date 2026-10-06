#!/usr/bin/env bash
# Docker management and transport only; stage semantics live in robot.sh.
set -euo pipefail
cd "$(dirname "$0")/.."
if docker info >/dev/null 2>&1; then d=(docker); else d=(sudo docker); fi
c=("${d[@]}" compose -p smartcar-layered)
action="${1:-help}"; if [ "$#" -gt 0 ]; then shift; fi
# Copy a display-specific X11 cookie into the running container. FamilyWild
# permits the container hostname to differ without disabling X server access control.
prepare_display() {
  command -v xauth >/dev/null || { echo 'Install xauth on the host to start RViz.'; return 2; }
  [ -n "${DISPLAY:-}" ] || { echo 'Run RViz from a terminal in the graphical desktop session.'; return 2; }
  local auth_dir
  auth_dir="$(mktemp -d)"
  chmod 700 "$auth_dir"
  touch "$auth_dir/authority"
  chmod 600 "$auth_dir/authority"
  if ! xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | xauth -f "$auth_dir/authority" nmerge -; then
    rm -rf "$auth_dir"; echo 'Could not read desktop X11 authorization.'; return 2
  fi
  if [ ! -s "$auth_dir/authority" ]; then
    rm -rf "$auth_dir"; echo 'No X11 cookie found; check DISPLAY and XAUTHORITY in this desktop terminal.'; return 2
  fi
  if ! "${c[@]}" cp "$auth_dir/authority" sim:/tmp/smartcar-display.xauth; then
    rm -rf "$auth_dir"; return 2
  fi
  rm -rf "$auth_dir"
}
case "$action" in
  start|nav)
    if [ "$#" -gt 1 ]; then echo 'Usage: sim.sh start [normal|narrow]; sim.sh nav'; exit 2; fi
    scene="${1:-narrow}"
    if [ "$action" = nav ]; then
      if [ "$("${d[@]}" inspect -f '{{.State.Running}}' smartcar-layered 2>/dev/null || true)" != true ]; then
        echo 'Run sim.sh start first.'; exit 2
      fi
      current_command="$("${d[@]}" inspect -f '{{json .Config.Cmd}}' smartcar-layered)"
      case "$current_command" in *sim_field_narrow*) active_scene=narrow ;; *) active_scene=normal ;; esac
      scene="${1:-$active_scene}"
      if [ "$scene" != "$active_scene" ]; then
        echo 'Use sim.sh start normal|narrow to change scenes; nav never resets the simulation.'; exit 2
      fi
    fi
    case "$scene" in
      normal) map_dir=sim_field_v1; lines=data/navigation/low_cost_lines.json ;;
      narrow) map_dir=sim_field_narrow; lines=data/maps/sim_field_narrow/low_cost_lines.json ;;
      *) echo 'Usage: sim.sh nav [normal|narrow]'; exit 2 ;;
    esac
    for name in smartcar-baseline smartcar-sim smartcar-sim-gui; do
      if [ "$("${d[@]}" inspect -f '{{.State.Running}}' "$name" 2>/dev/null || true)" = true ]; then
        echo "Stop old container $name before starting this project (shared ROS ports)."; exit 2
      fi
    done
    scene_compose=("${c[@]}" -f docker-compose.yml)
    if [ "$scene" = narrow ]; then scene_compose+=(-f docker-compose.narrow.yml); fi
    if [ "$action" = start ]; then "${scene_compose[@]}" up -d sim; fi
    "${d[@]}" cp scripts/sim_navigation.py smartcar-layered:/tmp/smartcar-quick-nav.py
    "${d[@]}" cp scripts/sim_navigation.launch smartcar-layered:/tmp/smartcar-quick-nav.launch
    "${d[@]}" cp scripts/sim_localization.launch smartcar-layered:/tmp/smartcar-quick-localization.launch
    echo "$action scene: $scene. Open RViz with: bash ~/smartcar1/scripts/sim.sh rviz"
    nav_terminal=(-i)
    if [ -t 0 ] && [ -t 1 ]; then nav_terminal+=(-t); fi
    "${d[@]}" exec "${nav_terminal[@]}" -e SMARTCAR_ROOT=/home/hajimi/smartcar -e SMARTCAR_PROFILE=sim smartcar-layered bash -c \
      'source /opt/ros/kinetic/setup.bash; source /home/hajimi/smartcar_ws/devel/setup.bash; exec python /tmp/smartcar-quick-nav.py "$@"' \
      -- "$action" "/home/hajimi/smartcar/data/maps/$map_dir/map.yaml" "/home/hajimi/smartcar/$lines"
    ;;
  build) COMPOSE_BAKE=false DOCKER_BUILDKIT=0 "${c[@]}" build sim ;;
  drivers)
    for name in smartcar-baseline smartcar-sim smartcar-sim-gui; do
      if [ "$("${d[@]}" inspect -f '{{.State.Running}}' "$name" 2>/dev/null || true)" = true ]; then
        echo "Stop old container $name before starting this project (shared ROS ports)."; exit 2
      fi
    done
    "${c[@]}" up -d sim ;;
  stop) "${c[@]}" stop sim ;;
  logs) "${c[@]}" logs --tail=120 sim ;;
  shell) "${c[@]}" exec sim bash ;;
  help) echo 'Layers: sim.sh start [normal|narrow]; sim.sh nav; sim.sh rviz; sim.sh keyboard-sim; sim.sh stop'
    echo 'Advanced: sim.sh build|drivers|logs|shell; sim.sh <robot.sh stage/tool> [arguments]' ;;
  *)
    display_args=()
    if [ "$action" = rviz ] || [ "$action" = rviz-multi ]; then
      prepare_display
      display_args=(-e "DISPLAY=$DISPLAY" -e XAUTHORITY=/tmp/smartcar-display.xauth)
    fi
    "${c[@]}" exec "${display_args[@]}" -e SMARTCAR_PROFILE=sim -e SMARTCAR_ROOT=/home/hajimi/smartcar sim bash -lc 'source /opt/ros/kinetic/setup.bash; source /home/hajimi/smartcar_ws/devel/setup.bash; exec bash /home/hajimi/smartcar/scripts/robot.sh "$@"' -- "$action" "$@" ;;
esac
