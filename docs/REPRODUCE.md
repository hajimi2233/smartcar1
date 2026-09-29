# 按层复现：驱动 → 建图 → 定位 → 单点 → 多点

每一层先单独验收，再进入下一层。启动命令占用终端是正常行为；使用新的终端做检查。Ctrl+C 停止该层。切换建图/定位、单点/多点时，先停止旧层，不要同时启动两个 TF 或指令发布者。

## 0. 准备代码和环境

解压到 Linux `~/smartcar1`。本版采用 ROS1 Kinetic，不把 Kinetic 项目直接视为 ROS2 项目。

实车本机安装 ROS 和对应依赖。下面依赖安装只选择实际需要的包，避免为实车安装 Gazebo：

```bash
cd ~/smartcar1
source /opt/ros/kinetic/setup.bash
# rosdep 首次使用前，按本机 ROS 安装流程完成 rosdep init / rosdep update。
rosdep install --from-paths src/smartcar_drivers src/smartcar_mapping src/smartcar_localization src/smartcar_navigation src/smartcar_mission src/smartcar_bringup src/smartcar_region_tool src/smartcar_description --ignore-src -r -y
bash scripts/build-real.sh
source devel/setup.bash
export SMARTCAR_PROFILE=real
```

新终端都需要 source ROS 和本工作空间。实际雷达、底盘厂商驱动另外安装，先按 [驱动接口](DRIVER_CONTRACT.md) 对接。导航调参使用 `config/navigation-real.yaml`。

若先在仿真演练：

```bash
cd ~/smartcar1
# 停止旧版 smartcar-baseline/smartcar-sim 容器后再启动本版。
bash scripts/sim.sh build
bash scripts/sim.sh start
```

图形窗口需要 Linux 图形桌面和 X11 授权，可按本机环境执行 `xhost +si:localuser:$(id -un)`；容器默认 UID=1000。DISPLAY/显卡配置属于运行环境，不影响模块边界。

## 1. 驱动层

实车：在厂商驱动基础上准备两个实际 launch 文件，然后运行：

```bash
bash scripts/robot.sh drivers-real /绝对路径/chassis_adapter.launch /绝对路径/lidar.launch
# 另开终端：
bash scripts/robot.sh check drivers
```

仿真：`sim.sh start` 已启动这一层；检查用 `bash scripts/sim.sh check drivers`。手动驾驶可另开终端运行 `bash scripts/sim.sh keyboard-sim`。

**通过标准**：底盘手动控制和停车正确；scan/转向/轮速反馈持续更新；雷达外参正确；驱动接受规定的指令话题。检查程序只做约 5 秒的只读连通性快照，仍需人工验收运动方向、比例和停车能力。

**此时没有**地图定位和导航目标执行。驱动不应该依赖地图文件才能工作。

## 2. 建图层

保持驱动运行，另开终端：

```bash
bash scripts/robot.sh mapping
# 另开终端手动移动、观察地图；保存到新文件名：
bash scripts/robot.sh check mapping
bash scripts/robot.sh save-map /home/你的用户名/smartcar1/data/maps/real_track/map
```

如果底盘已经提供可信且唯一的 `odom → base`，用 `mapping scan_odometry:=false`。否则默认启动激光里程计。实车手动驾驶使用已经验收的厂商工具。

仿真对应：`sim.sh mapping`、`sim.sh keyboard-sim`、`sim.sh save-map /home/hajimi/smartcar/data/maps/my_track/map`。Docker 内的地图输出通过 data 挂载回宿主机。

**通过标准**：地图比例正确、墙体没有明显重影，闭环回到原处合理，保存得到 `.yaml` 与 `.pgm`。保存脚本拒绝覆盖已有同名地图。完成后 Ctrl+C 停止 mapping，驱动继续运行。

## 3. 定位层

```bash
bash scripts/robot.sh localization /绝对路径/已保存地图/map.yaml
# 另开终端：
bash scripts/robot.sh rviz
bash scripts/robot.sh check localization
```

在 RViz 用 2D Pose Estimate 设置初始位置和方向，观察 LaserScan 和地图墙体是否重合。若用了底盘里程计，仍加 `scan_odometry:=false`。自定义 frame 时需同步 launch 的 base_frame/odom_frame/map_frame、导航配置和 RViz Fixed Frame。

仿真可先用现有地图：`bash scripts/sim.sh localization /home/hajimi/smartcar/data/maps/slam_current/map.yaml`。

**通过标准**：车辆静止时定位稳定；手动直行、倒退、转弯后 scan 仍与地图吻合；没有重复 TF 发布者。检查能收到 TF 不等于定位精度合格。此时不需要启动导航。

## 4. 单点导航层

退出手动驾驶命令源，保留驱动和定位：

```bash
bash scripts/robot.sh single
# 另开终端：
bash scripts/robot.sh check single
bash scripts/robot.sh rviz
```

在单点 RViz 的 2D Nav Goal 选择目标。也可使用 `robot.sh goal X Y YAW_DEGREES`。目标代表前轴/后轴/自定义点，由导航配置决定，不能只根据箭头猜测。

**验收顺序**：短距离直行 → 倒退 → 单个 90° 转弯 → 其他动作 → 不同起终点。记录规划路径、实际轨迹、阶段状态和最终位姿。实车低速标定和碰撞阈值调节仍按实测进行，本结构版不代替这些验收。

仿真对应 `sim.sh single`。只检查全局规划和流程时：停止 single，运行 `sim.sh planning-test`。这仍使用虚拟到达，不代表车已经实际执行。

## 5. 多点导航层

先 Ctrl+C 停止 single，保留驱动和定位：

```bash
bash scripts/robot.sh multi
# 另开终端：
bash scripts/robot.sh rviz-multi
bash scripts/robot.sh check multi
```

使用这个多点 RViz 的 2D Nav Goal 按顺序选点，再执行：

```bash
bash scripts/robot.sh multi-execute
# 需要时：
bash scripts/robot.sh multi-cancel
bash scripts/robot.sh multi-clear
bash scripts/robot.sh multi-undo
```

`multi` 已包含单点执行器，不要再启动 single。选点使用 `/multi_nav/goal`，执行使用 `/move_base_simple/goal`；选点本身不会启动车辆。先验收两个点的衔接，再增加到全程。队列完成或失败后仍按原版清空，点位还未增加文件持久化。

仿真命令把 robot.sh 换成 sim.sh。测试多点全局规划用 `sim.sh planning-test multi:=true`，搭配 `sim.sh rviz-multi` 和 `sim.sh multi-execute`。

## 旧入口对照

| 原命令 | 本版 |
|---|---|
| smartcar.sh start（带定位） | sim.sh start，然后显式 localization |
| smartcar.sh mapping | sim.sh mapping |
| smartcar.sh nav | sim.sh single |
| smartcar.sh nav-test | sim.sh planning-test |
| smartcar.sh multi-nav | sim.sh multi（包含单点执行器） |
| smartcar.sh rviz | 单点用 sim.sh rviz，多点用 sim.sh rviz-multi |

修改 config 后重启对应层；修改源码后重新构建。仿真和实车配置分开，避免将仿真话题直接带到车上。实车模板的几何默认值仍来自当前车型，必须按车校验。
