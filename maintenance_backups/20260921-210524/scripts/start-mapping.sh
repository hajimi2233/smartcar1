#!/usr/bin/env bash
set -euo pipefail
if docker info >/dev/null 2>&1; then docker_cmd=(docker); else docker_cmd=(sudo docker); fi
exec "${docker_cmd[@]}" exec -d smartcar-sim bash /home/hajimi/smartcar_2026_ws/scripts/start_mapping.sh
