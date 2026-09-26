#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
xhost +local: >/dev/null 2>&1 || true
if docker info >/dev/null 2>&1; then
  docker_cmd=(docker)
else
  docker_cmd=(sudo docker)
fi
"${docker_cmd[@]}" rm -f smartcar-sim >/dev/null 2>&1 || true
render_gid="$(getent group render | cut -d: -f3 || true)"
video_gid="$(getent group video | cut -d: -f3 || true)"
extra=()
if [ -n "$video_gid" ]; then extra+=(--group-add "$video_gid"); fi
if [ -n "$render_gid" ]; then extra+=(--group-add "$render_gid"); fi
if [ -e /dev/dri ]; then extra+=(--device /dev/dri); fi
image=smartcar:kinetic-run
if ! "${docker_cmd[@]}" image inspect "$image" >/dev/null 2>&1; then
  image=smartcar:kinetic
fi
exec "${docker_cmd[@]}" run -d --name smartcar-sim --network host \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_X11_NO_MITSHM=1 \
  -e QT_AUTO_SCREEN_SCALE_FACTOR=0 \
  -e LIBGL_ALWAYS_SOFTWARE=0 \
  -e OGRE_RTT_MODE=Copy \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /etc/machine-id:/etc/machine-id:ro \
  "${extra[@]}" \
  "$image" sim-gui
