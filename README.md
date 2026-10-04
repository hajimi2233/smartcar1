# 智能车：分层复现版

按照 **驱动 → 建图 → 定位 → 单点导航 → 多点导航** 逐层运行与验收。导航算法和参数默认值沿用参数化版本；本次主要整理源码、依赖、启动入口和复现流程。

先读 [逐步复现手册](docs/REPRODUCE.md)，接实车时对照 [驱动接口](docs/DRIVER_CONTRACT.md)，调整参数看 [调参说明](docs/PARAMETERS.md)。目录与依赖见 [架构说明](docs/ARCHITECTURE.md)。

```text
src/
  smartcar_drivers/       实车驱动接入入口、分层检查
  smartcar_description/   仿真车体模型
  smartcar_sim/           Gazebo 底盘插件、模拟雷达与仿真工具
  smartcar_mapping/       建图
  smartcar_localization/  激光里程计与全地图/选墙匹配，保留 AMCL
  smartcar_navigation/    单点全局规划、局部跟踪、导航参数
  smartcar_mission/       多点任务顺序管理
  smartcar_bringup/       各层组合入口、RViz 配置
  smartcar_region_tool/   RViz 低代价线工具
config/                  可编辑配置，仿真与实车导航配置分开
scripts/                 robot.sh 本机入口、sim.sh Docker 入口
data/                    地图和运行数据
tests/                   离线测试及需 ROS 的测试
docs/                    当前架构与复现说明
archive/                 停用代码，不参与构建
legacy-reference.zip     旧版备份与参考材料，不参与构建
```

正式源码直接位于 `src/`，不再通过“解压工作空间 + docker/overlay 覆盖”构建。实车编译可以排除 Gazebo 包。厂商底盘与雷达协议未知，因此提供接入规范和启动挂接点，未宣称已经完成实车驱动。

## 仿真快速启动

默认使用当前的窄场地方案（上方缩短 0.8 m、下方缩短 1 m）：雷达帧间里程计＋全地图墙面匹配定位、Hybrid A* 全局规划＋局部路径跟踪与安全监督，并自动加载窄场地低代价线。一个命令启动仿真、定位和单点导航：

```bash
bash ~/smartcar1/scripts/sim.sh nav
```

另开一个终端打开 RViz；已经打开则跳过：

```bash
bash ~/smartcar1/scripts/sim.sh rviz
```

在 RViz 用 **2D Pose Estimate** 设置初始位姿，定位稳定后用 **2D Nav Goal** 下发目标。低代价线自动加载，点击 **Draw Low-Cost Lines**（快捷键 **L**）即可继续画线，每条完成后自动保存。

切换回原场地：

```bash
bash ~/smartcar1/scripts/sim.sh nav normal
```

显式指定窄场地也可以，效果与不带参数的 `nav` 相同（不带参数始终选择窄场地）：

```bash
bash ~/smartcar1/scripts/sim.sh nav narrow
```

`nav` 自动配套 Gazebo 场景、定位地图和低代价线文件，不需要再单独执行 `start`、`localization`、`single` 或填写容器路径。相同场地不会重建正在运行的仿真，匹配的定位、导航节点会复用；若手动启动的节点使用了另一张地图或另一套配置，会提示先关闭对应终端。切换场地会重建仿真容器，重置车辆及定位，需重新设置初始位姿。

| 场地 | 定位地图 | 低代价线（宿主机路径，相对项目目录） |
| --- | --- | --- |
| `normal` | `data/maps/sim_field_v1/map.yaml` | `data/navigation/low_cost_lines.json`（保留原有记录） |
| `narrow` | `data/maps/sim_field_narrow/map.yaml` | `data/maps/sim_field_narrow/low_cost_lines.json` |

导航终端保持打开，Ctrl+C 停止本次启动的导航与定位；复用的定位仍由原终端管理。停止整个仿真：

```bash
bash ~/smartcar1/scripts/sim.sh stop
```

首次运行或修改 ROS 源码后先构建镜像；本次快捷命令和 README 的修改不需要重新 build：

```bash
bash ~/smartcar1/scripts/sim.sh build
```

默认定位使用雷达帧间里程计＋全地图墙面局部匹配，不需要画框、选墙或轮脉冲。用 `bash ~/smartcar1/scripts/sim.sh wall-status` 查看 `target_mode: full_map` 与 `state: MATCHED`。当前不会自动按通道内外切换算法。

分层调试仍保留 `start`、`localization`、`single` 等入口，其中 `start` 只启动驱动层。手动选墙、AMCL、建图和实车流程见 [逐步复现手册](docs/REPRODUCE.md)。首次使用分层版应先停止旧版容器，不能让两套 ROS 共用端口同时运行。

实车当前仍针对 ROS1 Kinetic 接口；没有执行 ROS 版本升级。Linux/ROS/Gazebo 的构建和实际运行需要在目标环境验收，离线检查结果见 `docs/VALIDATION.md`。

通道内外判断：复用四角画框工具，按车辆定位参考点是否进框发布状态，操作见 [通道判断说明](docs/CORRIDOR.md)。

手动墙体特征定位：分别画线选择通道内/外优先墙，将线膨胀成区域，匹配区域覆盖的地图墙体，支持编号删除及保存加载，见 [墙体定位说明](docs/WALL_FEATURES.md)。

轮距融合仿真：模拟有符号累计脉冲，与墙体定位融合，并同步对比纯雷达估计，见 [轮距融合验证](docs/WHEEL_FUSION.md)。

低代价线随导航启动自动加载：在 RViz 点击 **Draw Low-Cost Lines**（快捷键 **L**），依次点击起点和终点；每条完成后自动保存，下次使用同一张地图自动恢复。`nav` 使用上表中对应场地的文件；单独启动 `single` 或 `multi` 默认使用 `data/navigation/low_cost_lines.json`，重启容器也保留。新画的线用于下一次规划，不会突然修改正在执行的路径。清空并保存用：

```bash
bash ~/smartcar1/scripts/sim.sh lines-clear
```

墙体居中修正仅在 `STRAIGHT` 模式且当前路径段为直线时参与，修正后的转向仍经过转向速率限制、轨迹预测及碰撞检查。普通模式和弯道不再被墙体居中修改。
