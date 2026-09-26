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

COPY vm_backup/vm_setup_and_logs.tar.gz /tmp/vm_setup_and_logs.tar.gz
RUN tar -xzf /tmp/vm_setup_and_logs.tar.gz -C /tmp \
 && install -m 644 /tmp/inspection_setup/ros.key /usr/share/keyrings/inspection-ros.gpg \
 && echo 'deb [arch=amd64 signed-by=/usr/share/keyrings/inspection-ros.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ xenial main' \
      > /etc/apt/sources.list.d/ros-kinetic.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
      build-essential cmake git mesa-utils x11-xserver-utils \
      python python-yaml \
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
      ros-kinetic-navigation \
      ros-kinetic-gmapping \
      ros-kinetic-amcl \
      ros-kinetic-move-base \
 && rm -rf /var/lib/apt/lists/* /tmp/vm_setup_and_logs.tar.gz /tmp/inspection_setup \
 && echo 'source /opt/ros/kinetic/setup.bash' >> /etc/bash.bashrc \
 && python3 -c "import uuid; open('/etc/machine-id','w').write(uuid.uuid4().hex+chr(10))"

RUN useradd -m -u 1000 -s /bin/bash hajimi \
 && echo 'hajimi ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

COPY --chown=hajimi:hajimi restore_workspace.sh /home/hajimi/smartcar/restore_workspace.sh
COPY --chown=hajimi:hajimi vm_backup /home/hajimi/smartcar/vm_backup
USER hajimi
WORKDIR /home/hajimi
COPY --chown=hajimi:hajimi local_project/sim_workspace/templates/teleop.py /tmp/overlay/teleop.py
COPY --chown=hajimi:hajimi local_project/sim_workspace/templates/ackermann_plugin.cpp /tmp/overlay/ackermann_plugin.cpp
COPY --chown=hajimi:hajimi local_project/sim_workspace/templates/run_sim.sh /tmp/overlay/run_sim.sh
COPY --chown=hajimi:hajimi docker/overlay/keyboard.sh /tmp/overlay/keyboard.sh
COPY --chown=hajimi:hajimi docker/overlay/simulation.launch /tmp/overlay/simulation.launch
COPY --chown=hajimi:hajimi docker/overlay/gmapping.launch /tmp/overlay/gmapping.launch
COPY --chown=hajimi:hajimi docker/overlay/start_mapping.sh /tmp/overlay/start_mapping.sh
COPY --chown=hajimi:hajimi docker/overlay/save_map.sh /tmp/overlay/save_map.sh
COPY --chown=hajimi:hajimi docker/overlay/amcl.launch /tmp/overlay/amcl.launch
COPY --chown=hajimi:hajimi docker/overlay/navigation.launch /tmp/overlay/navigation.launch
COPY --chown=hajimi:hajimi docker/overlay/waypoint_nav.py /tmp/overlay/waypoint_nav.py
COPY --chown=hajimi:hajimi docker/overlay/start_navigation.sh /tmp/overlay/start_navigation.sh
COPY --chown=hajimi:hajimi docker/overlay/smartcar_navigation.CMakeLists.txt /tmp/overlay/smartcar_navigation.CMakeLists.txt
RUN bash /home/hajimi/smartcar/restore_workspace.sh /home/hajimi/smartcar_2026_ws \
 && cp /tmp/overlay/teleop.py /home/hajimi/smartcar_2026_ws/src/smartcar_sim/scripts/teleop.py \
 && cp /tmp/overlay/ackermann_plugin.cpp /home/hajimi/smartcar_2026_ws/src/smartcar_sim/src/ackermann_plugin.cpp \
 && cp /tmp/overlay/run_sim.sh /home/hajimi/smartcar_2026_ws/scripts/run_sim.sh \
 && cp /tmp/overlay/keyboard.sh /home/hajimi/smartcar_2026_ws/scripts/keyboard.sh \
 && cp /tmp/overlay/simulation.launch /home/hajimi/smartcar_2026_ws/src/smartcar_bringup/launch/simulation.launch \
 && mkdir -p /home/hajimi/smartcar_2026_ws/src/smartcar_mapping/launch \
 && cp /tmp/overlay/gmapping.launch /home/hajimi/smartcar_2026_ws/src/smartcar_mapping/launch/gmapping.launch \
 && cp /tmp/overlay/start_mapping.sh /home/hajimi/smartcar_2026_ws/scripts/start_mapping.sh \
 && cp /tmp/overlay/save_map.sh /home/hajimi/smartcar_2026_ws/scripts/save_map.sh \
 && mkdir -p /home/hajimi/smartcar_2026_ws/src/smartcar_navigation/launch \
            /home/hajimi/smartcar_2026_ws/src/smartcar_navigation/scripts \
 && cp /tmp/overlay/amcl.launch /home/hajimi/smartcar_2026_ws/src/smartcar_navigation/launch/amcl.launch \
 && cp /tmp/overlay/navigation.launch /home/hajimi/smartcar_2026_ws/src/smartcar_navigation/launch/navigation.launch \
 && cp /tmp/overlay/waypoint_nav.py /home/hajimi/smartcar_2026_ws/src/smartcar_navigation/scripts/waypoint_nav.py \
 && cp /tmp/overlay/start_navigation.sh /home/hajimi/smartcar_2026_ws/scripts/start_navigation.sh \
 && cp /tmp/overlay/smartcar_navigation.CMakeLists.txt /home/hajimi/smartcar_2026_ws/src/smartcar_navigation/CMakeLists.txt \
 && chmod +x /home/hajimi/smartcar_2026_ws/scripts/*.sh \
 && chmod +x /home/hajimi/smartcar_2026_ws/src/*/scripts/*.py \
 && bash -lc 'source /opt/ros/kinetic/setup.bash && cd /home/hajimi/smartcar_2026_ws && catkin_make -j2'

WORKDIR /home/hajimi/smartcar_2026_ws
COPY docker/entrypoint.sh /home/hajimi/entrypoint.sh
USER root
RUN chmod +x /home/hajimi/entrypoint.sh /home/hajimi/smartcar/restore_workspace.sh
USER hajimi
ENTRYPOINT ["/home/hajimi/entrypoint.sh"]
CMD ["sim"]
