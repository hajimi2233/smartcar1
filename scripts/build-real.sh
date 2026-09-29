#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/kinetic/setup.bash
# Simulation physics and old inspection preview are not part of real navigation.
find src -path '*/scripts/*.py' -exec chmod +x {} +
exec catkin_make -DCATKIN_BLACKLIST_PACKAGES='smartcar_sim' "$@"
