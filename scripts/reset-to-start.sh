#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
docker compose -p smartcar-baseline exec -T sim bash /home/hajimi/smartcar_2026_ws/src/smartcar_navigation/scripts/reset_to_start.sh
