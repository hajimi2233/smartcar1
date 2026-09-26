#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export COMPOSE_BAKE=false
export DOCKER_BUILDKIT=0
exec docker compose build sim
