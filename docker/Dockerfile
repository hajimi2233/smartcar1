FROM ubuntu:16.04

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    ROS_DISTRO=kinetic \
    QT_X11_NO_MITSHM=1 \
    QT_AUTO_SCREEN_SCALE_FACTOR=0 \
    OGRE_RTT_MODE=Copy

# Ubuntu 16.04 is EOL; Tsinghua still carries xenial plus the ROS Kinetic archive.
RUN printf '%s\n' \
      'deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ xenial main restricted universe multiverse' \
      'deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ xenial-updates main restricted universe multiverse' \
      'deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ xenial-security main restricted universe multiverse' \
      > /etc/apt/sources.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
      ca-certificates wget gnupg apt-transport-https sudo python3 \
 && rm -rf /var/lib/apt/lists/*

COPY docker/bootstrap/vm_setup_and_logs.tar.gz /tmp/vm_setup_and_logs.tar.gz
RUN tar -xzf /tmp/vm_setup_and_logs.tar.gz -C /tmp \
 && install -m 644 /tmp/inspection_setup/ros.key /usr/share/keyrings/inspection-ros.gpg \
 && echo 'deb [arch=amd64 signed-by=/usr/share/keyrings/inspection-ros.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ xenial main' \
      > /etc/apt/sources.list.d/ros-kinetic.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
      build-essential cmake git mesa-utils x11-xserver-utils \
      python python-yaml python-numpy \
      ros-kinetic-ros-base \
      ros-kinetic-rviz \
      ros-kinetic-gazebo-ros-pkgs \
      ros-kinetic-gazebo-ros-control \
      ros-kinetic-ros-controllers \
      ros-kinetic-xacro \
      ros-kinetic-robot-state-publisher \
      ros-kinetic-joint-state-publisher \
      ros-kinetic-map-server \
      ros-kinetic-tf \
      ros-kinetic-laser-scan-matcher \
      ros-kinetic-navigation \
      ros-kinetic-gmapping \
      ros-kinetic-amcl \
      ros-kinetic-move-base \
 && rm -rf /var/lib/apt/lists/* /tmp/vm_setup_and_logs.tar.gz /tmp/inspection_setup \
 && echo 'source /opt/ros/kinetic/setup.bash' >> /etc/bash.bashrc \
 && python3 -c "import uuid; open('/etc/machine-id','w').write(uuid.uuid4().hex+chr(10))"

RUN useradd -m -u 1000 -s /bin/bash hajimi \
 && echo 'hajimi ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

COPY --chown=hajimi:hajimi src /home/hajimi/smartcar_ws/src
WORKDIR /home/hajimi/smartcar_ws
RUN find src -path '*/scripts/*.py' -exec chmod +x {} + \
 && bash -lc 'source /opt/ros/kinetic/setup.bash && catkin_make -j2'
COPY --chown=hajimi:hajimi config /home/hajimi/smartcar/config
COPY --chown=hajimi:hajimi data /home/hajimi/smartcar/data
COPY --chown=hajimi:hajimi scripts /home/hajimi/smartcar/scripts
COPY --chown=hajimi:hajimi docker/entrypoint.sh /home/hajimi/smartcar/entrypoint.sh
ENTRYPOINT ["bash", "/home/hajimi/smartcar/entrypoint.sh"]
CMD ["drivers-sim", "gui:=false", "rviz:=false"]
