#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source scripts/source_env.sh
export DISPLAY="${DISPLAY:-:0}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
exec rosrun smartcar_sim teleop.py
