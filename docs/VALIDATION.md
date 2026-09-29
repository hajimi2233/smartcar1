# 本次验证与待验收项

基于 GitHub `hajimi2233/smartcar1` 的 `7f9af9a`，先完成参数化，再进行本次目录与启动结构整理。

已执行：

- Windows / Python 3 离线单元测试 85 项：84 项通过。唯一失败仍为原版本的 `test_planner_and_closed_loop_reach_front_axle_pose`，报控制器未到达方向分段终点。
- 新增分层结构测试：项目内 launch include 的目标和必需参数、节点脚本路径、CMake Python 模块路径、驱动与定位隔离、Docker 直接使用源码。
- 新增多点行为测试：选点只记录目标、不发布执行/取消命令；完成反馈逐点推进队列。
- 延续参数专项测试：非默认轴距、参考点偏移、预测一致性、阶段距离、速度、错误配置检查。
- 仿真/实车两套 YAML、package/launch XML、Python 语法及 Bash 语法检查。
- 核心导航 Python 与参数化版逐文件比较，未为本次目录重构修改算法。

尚未执行：

- Linux ROS Kinetic 的 catkin 编译、Docker 镜像重建及图形窗口启动。
- Gazebo 全程实际运行与多点 GUI 操作。
- 底盘/雷达真实驱动集成、TF 外参标定、实车跟踪与碰撞阈值验收。

因此本版是具备源码、启动入口和验收步骤的分层交付，不代表已经在未知型号的实车上完成部署。驱动接入缺少的具体资料列在 DRIVER_CONTRACT.md。
