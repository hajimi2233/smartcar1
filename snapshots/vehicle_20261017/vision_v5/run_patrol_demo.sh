#!/usr/bin/env bash
set -e
camera_environment() { source /home/smartcar/smartcar_2026_ws/scripts/source_env.sh; }
camera_environment
VISION_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VISION_PYTHON="${VISION_PYTHON:-/usr/bin/python2}"
exec "$VISION_PYTHON" -u "$VISION_DIR/patrol_vision_demo.py" "$@"
