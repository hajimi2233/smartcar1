#!/usr/bin/env bash
# Host helper: revive Gazebo ROS API if dead, then reset the car to start.
set -euo pipefail
if docker info >/dev/null 2>&1; then docker_cmd=(docker); else docker_cmd=(sudo docker); fi
cname=smartcar-sim

api_ok() {
  "${docker_cmd[@]}" exec "$cname" bash -lc \
    'export ROS_MASTER_URI=http://localhost:11311
     source /opt/ros/kinetic/setup.bash
     source ~/smartcar_2026_ws/devel/setup.bash
     rosservice list 2>/dev/null | grep -q /gazebo/set_model_state'
}

if ! "${docker_cmd[@]}" ps --format '{{.Names}}' | grep -qx "$cname"; then
  echo "[host] starting $cname"
  "${docker_cmd[@]}" start "$cname"
  sleep 10
fi

if ! api_ok; then
  echo "[host] Gazebo ROS API is down. Restarting $cname ..."
  "${docker_cmd[@]}" restart "$cname"
  echo "[host] waiting for /gazebo/set_model_state"
  ok=0
  for i in $(seq 1 40); do
    if api_ok; then ok=1; break; fi
    sleep 1
  done
  if [ "$ok" != 1 ]; then
    echo "[host] ERROR: Gazebo API did not come up. sudo docker logs $cname"
    exit 1
  fi
  "${docker_cmd[@]}" exec -e DISPLAY="${DISPLAY:-:0}" -e LIBGL_ALWAYS_SOFTWARE=1 "$cname" bash -lc \
    'source /opt/ros/kinetic/setup.bash; source ~/smartcar_2026_ws/devel/setup.bash
     pgrep -f localization.launch >/dev/null || nohup roslaunch smartcar_navigation localization.launch >/tmp/localization.log 2>&1 &
     pgrep -x gzclient >/dev/null || (export LIBGL_ALWAYS_SOFTWARE=1; nohup rosrun gazebo_ros gzclient >/tmp/gzclient.log 2>&1 &)'
  sleep 4
fi

exec "${docker_cmd[@]}" exec -e DISPLAY="${DISPLAY:-:0}" "$cname" \
  bash /home/hajimi/smartcar_2026_ws/scripts/reset_to_start.sh
