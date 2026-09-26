#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source scripts/source_env.sh
if rosnode list 2>/dev/null | grep -q '/waypoint_marker'; then
  echo 'waypoint_marker is already running.'
  exit 0
fi
exec rosrun smartcar_mapping waypoint_marker.py
