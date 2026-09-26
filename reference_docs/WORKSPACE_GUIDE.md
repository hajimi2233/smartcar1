# RoboCup 2026 工作空间速查

状态：板载工作空间已创建，catkin_make 编译成功（2026-09-14）。本文件保存在 Windows 项目的 docs 目录。

四个新包均已通过 rospack find 验证。旧 catkin_ws 的环境链还包含 /home/smartcar/ros_ws/devel 和 /home/smartcar/ROS_WS/devel；新工作空间继承该链，新工作空间优先级最高，旧环境未修改。

依赖状态：tf、tf2_ros、robot_state_publisher 已安装；move_base、amcl、map_server、navigation 尚未安装。现有 apt 索引有 Candidate，但安装因 ROS 软件包无法通过认证而失败；未更改软件源或绕过认证。

板载路径：`/home/smartcar/smartcar_2026_ws`。使用 Ubuntu 16.04 / ROS Kinetic / catkin_make，叠加旧工作空间 `/home/smartcar/catkin_ws`。不要修改旧工作空间代码或 `~/.bashrc`。

## 目录与文件位置

| 目录 | 用途 |
| --- | --- |
| `src/` | ROS package 与源代码 |
| `build/` | catkin_make 生成的编译中间文件，不手动存放源码 |
| `devel/` | 编译生成的开发环境、可执行文件和 setup.bash |
| `data/` | 实际运行产生的数据，必须与 src 分离 |
| `docs/` | 项目说明与速查文档 |
| `scripts/` | 工作空间级环境与编译辅助脚本 |

| 内容 | 应放位置 |
| --- | --- |
| Python 节点 | `src/<包名>/scripts/`；可复用模块可放包内 `src/<模块名>/` 并配置 setup.py |
| C++ 源码 / 头文件 | `src/<包名>/src/` / `src/<包名>/include/<包名>/` |
| launch 文件 | `src/<包名>/launch/` |
| YAML 配置 | `src/<包名>/config/` |
| 地图成果 | `data/maps/<地图批次>/` |
| rosbag | `data/bags/` |
| 标定结果 | `data/calibration/` |
| 日志 | `data/logs/` |

基础包：`smartcar_bringup`、`smartcar_description`、`smartcar_mapping`、`smartcar_navigation`。本阶段只有包骨架，不编写节点或真实导航参数。

地图批次示例：`data/maps/map_20260914_01/`，未来按实际产物保存 `map.pgm`、`map.yaml`、`map.pbstream` 与 `README.md`。当前不生成地图，不引入 Cartographer 编译依赖。

`/odo_vel` 为 `geometry_msgs/Twist`，不是 `nav_msgs/Odometry`，不能直接作为标准 odom 配置。footprint、轴距、最小转弯半径、雷达外参与标准 odometry 确认前，导航配置保持空骨架。

## 环境加载顺序

```bash
source /opt/ros/kinetic/setup.bash
source /home/smartcar/catkin_ws/devel/setup.bash
# 新工作空间完成编译后再加载：
source /home/smartcar/smartcar_2026_ws/devel/setup.bash
```

`scripts/source_env.sh` 已按上述顺序加载，仅在新工作空间 setup.bash 存在时加载它。使用 `source scripts/source_env.sh`，使环境作用于当前终端。

`scripts/build.sh` 进入新工作空间，加载 ROS 与旧工作空间环境，再执行 `catkin_make`；不自动删除 build 或 devel。

package.xml 的维护者邮箱为占位值 smartcar@example.com，license 为 TODO，待项目维护者确认后填写真实信息。
