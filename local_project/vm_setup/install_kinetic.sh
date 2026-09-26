#!/usr/bin/env bash
# Run as the normal Ubuntu user: bash ~/inspection_setup/install_kinetic.sh
set -euo pipefail
if [ "$(id -u)" -eq 0 ]; then
  echo 'Run this script as your normal user, without sudo before bash.'
  exit 1
fi
. /etc/os-release
if [ "$ID" != ubuntu ] || [ "$VERSION_ID" != 16.04 ] || [ "$(dpkg --print-architecture)" != amd64 ]; then
  echo 'This installer requires Ubuntu 16.04 amd64.'
  exit 1
fi
setup_dir="$HOME/inspection_setup"
mkdir -p "$setup_dir"
exec > >(tee -a "$setup_dir/install.log") 2>&1
trap 'echo "Installation stopped at line $LINENO. Send the end of ~/inspection_setup/install.log for diagnosis."' ERR
sudo -v
echo 'Installing small repository prerequisites...'
sudo apt-get install -y --no-install-recommends ca-certificates wget gnupg apt-transport-https
cached_key_sha=4a91c49af0d6f0016108b93698782b596c27ccd836937e18e0e36c3347dc602f
if [ -f "$setup_dir/ros.key" ] && printf '%s  %s\n' "$cached_key_sha" "$setup_dir/ros.key" | sha256sum -c -; then
  cp "$setup_dir/ros.key" "$setup_dir/ros.key.new"
else
  wget --timeout=30 --tries=3 -O "$setup_dir/ros.key.new" https://raw.githubusercontent.com/ros/rosdistro/master/ros.key
fi
wget --timeout=30 --tries=3 -O "$setup_dir/Release" https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/dists/xenial/Release
wget --timeout=30 --tries=3 -O "$setup_dir/Release.gpg" https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/dists/xenial/Release.gpg
gpgv --keyring "$setup_dir/ros.key.new" "$setup_dir/Release.gpg" "$setup_dir/Release"
sudo install -m 644 "$setup_dir/ros.key.new" /usr/share/keyrings/inspection-ros.gpg
repo='deb [arch=amd64 signed-by=/usr/share/keyrings/inspection-ros.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ xenial main'
repo_file=/etc/apt/sources.list.d/inspection-ros.list
if [ -f "$repo_file" ] && ! grep -Fxq "$repo" "$repo_file"; then
  echo "Existing $repo_file differs; stopping without overwriting it."
  exit 1
fi
printf '%s\n' "$repo" | sudo tee "$repo_file" >/dev/null
sudo apt-get update
packages=(build-essential cmake git mesa-utils
  ros-kinetic-ros-base ros-kinetic-rviz
  ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
  ros-kinetic-ros-controllers ros-kinetic-xacro
  ros-kinetic-robot-state-publisher ros-kinetic-joint-state-publisher
  ros-kinetic-navigation ros-kinetic-teb-local-planner)
echo 'Checking dependency resolution (no packages installed by this check)...'
sudo apt-get --simulate --no-install-recommends --no-remove install "${packages[@]}" > "$setup_dir/package-plan.txt"
df -h /
echo 'Installing the requested base simulation and navigation packages...'
sudo apt-get -y --no-install-recommends --no-remove install "${packages[@]}"
source_line='source /opt/ros/kinetic/setup.bash'
if ! grep -Fxq "$source_line" "$HOME/.bashrc"; then
  printf '\n%s\n' "$source_line" >> "$HOME/.bashrc"
fi
set +u
source /opt/ros/kinetic/setup.bash
set -u
rosversion -d
gazebo_status=0
gazebo --version > "$setup_dir/gazebo-version.txt" 2>&1 || gazebo_status=$?
cat "$setup_dir/gazebo-version.txt"
# Gazebo 7 can print its version successfully and still return 255.
if ! grep -q 'version 7\.' "$setup_dir/gazebo-version.txt" || { [ "$gazebo_status" -ne 0 ] && [ "$gazebo_status" -ne 255 ]; }; then
  echo 'Unexpected Gazebo version result.'
  exit 1
fi
df -h /
echo 'Base packages installed. GUI acceleration, simulation and navigation still need runtime testing.'
echo 'Open a new terminal to load the ROS environment.'
