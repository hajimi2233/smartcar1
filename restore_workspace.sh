#!/usr/bin/env bash
set -euo pipefail
backup_dir="$(cd "$(dirname "$0")" && pwd)"
target_dir="${1:-$HOME/smartcar_2026_ws}"
mode="${2:-build}"
if [ "$mode" != build ] && [ "$mode" != --extract-only ]; then
  echo 'Usage: bash restore_workspace.sh [absolute-destination] [--extract-only]'; exit 1
fi
case "$target_dir" in /*) ;; *) echo 'Destination must be an absolute Linux path.'; exit 1;; esac
if [ -e "$target_dir" ]; then echo "Destination exists; choose a new directory: $target_dir"; exit 1; fi
if [ "$mode" = build ] && [ ! -f /opt/ros/kinetic/setup.bash ]; then
  echo 'Build inside Ubuntu 16.04 / ROS Kinetic. Use --extract-only to restore sources on another host.'; exit 1
fi
(cd "$backup_dir/vm_backup" && sha256sum -c SHA256SUMS)
mkdir -p "$(dirname "$target_dir")"
stage_dir="$(mktemp -d "$(dirname "$target_dir")/.smartcar-restore.XXXXXX")"
tar -xzf "$backup_dir/vm_backup/smartcar_2026_ws_source.tar.gz" -C "$stage_dir"
mv "$stage_dir/smartcar_2026_ws" "$target_dir"
rmdir "$stage_dir"
python_bin=python3
if ! command -v "$python_bin" >/dev/null; then python_bin=python; fi
"$python_bin" - "$target_dir" <<'PY'
from __future__ import print_function
import os,sys
target=os.path.realpath(sys.argv[1])
for root,dirs,files in os.walk(os.path.join(target,'src')):
    for name in files:
        if not name.endswith('.launch'): continue
        path=os.path.join(root,name)
        with open(path) as f: content=f.read()
        content=content.replace('/home/hajimi/smartcar_2026_ws',target)
        with open(path,'w') as f:f.write(content)
print('Restored and adjusted workspace paths: '+target)
PY
if [ "$mode" = --extract-only ]; then
  echo 'Source restoration complete. ROS dependencies and compilation are still required.'; exit 0
fi
bash "$target_dir/scripts/build.sh"
echo "Ready: bash $target_dir/scripts/run_sim.sh"
