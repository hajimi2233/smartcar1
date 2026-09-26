#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source scripts/source_env.sh
if ! rosnode list 2>/dev/null | grep -q '/slam_gmapping'; then
  echo 'gmapping is not running. Start mapping first.'
  exit 1
fi
out="${1:-$PWD/data/maps/slam_current/map}"
mkdir -p "$(dirname "$out")"
rosrun map_server map_saver -f "$out"
echo "Saved:"
echo "  $out.pgm"
echo "  $out.yaml"
