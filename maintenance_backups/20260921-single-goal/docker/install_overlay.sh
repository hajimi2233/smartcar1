#!/usr/bin/env bash
set -euo pipefail
ws=/home/hajimi/smartcar_2026_ws
overlay=/tmp/project-overlay
nav="$ws/src/smartcar_navigation"
mkdir -p "$nav/scripts" "$nav/config" "$nav/launch"
cp "$overlay"/*.py "$nav/scripts/"
for file in localization navigation amcl; do cp "$overlay/$file.launch" "$nav/launch/"; done
cp "$overlay/localization.yaml" "$nav/config/localization.yaml"
cp "$overlay/smartcar_navigation.CMakeLists.txt" "$nav/CMakeLists.txt"
cp "$overlay/inspection_car.urdf" "$ws/src/smartcar_description/urdf/inspection_car.urdf"
cp "$overlay/inspection.rviz" "$ws/src/smartcar_sim/config/inspection.rviz"
cp "$overlay"/*.sh "$ws/scripts/"
chmod +x "$ws/scripts/"*.sh "$nav/scripts/"*.py
