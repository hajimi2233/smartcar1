# RoboCup 2026 日常命令速查

状态：板载工作空间、四个基础包及辅助脚本已创建，catkin_make 编译成功（2026-09-14）。导航依赖安装因软件包认证失败而未完成，当前没有导航业务节点或 launch 文件。

## 登录、加载与编译

Windows 终端登录（密码在提示时输入）：

```powershell
ssh -t smartcar@10.222.250.172
```

以下命令在板载 Bash 终端执行：

```bash
cd /home/smartcar/smartcar_2026_ws
source /opt/ros/kinetic/setup.bash
source /home/smartcar/catkin_ws/devel/setup.bash
if [ -f devel/setup.bash ]; then source devel/setup.bash; fi

# 编译整个工作空间
catkin_make
source devel/setup.bash

# 使用已部署的辅助脚本
source scripts/source_env.sh
bash scripts/build.sh

# 工作空间已初始化且所需依赖已编译时，编译单个包
catkin_make --pkg smartcar_navigation

# 查找包（其余三个包同理）
rospack find smartcar_navigation
```

每个新终端均需重新加载环境；不修改 `~/.bashrc`。首次建立工作空间或新增 package 后，先完整运行 `catkin_make` 并重新 `source devel/setup.bash`。

## ROS 日常查询与启动语法

```bash
rosnode list
rostopic list
rostopic info /odo_vel
rostopic echo -n 1 /odo_vel
```

上述查询需要可访问的 ROS master；`echo` 在话题无消息时可能等待。`/odo_vel` 的消息类型应为 `geometry_msgs/Twist`，不能当作标准 Odometry 使用。

以下仅为将来有节点或 launch 文件后的语法示例，尖括号内容需替换；当前骨架包无节点或可启动业务 launch：

```text
rosrun <包名> <可执行节点名>
roslaunch <包名> <文件名.launch>
```

## 修改后需要什么操作

| 修改内容 | 操作 |
| --- | --- |
| 已有 Python 文件逻辑 | 一般不用编译，重启相关节点；文件需有正确 shebang 和执行权限 |
| 新增 Python 节点或修改安装规则 | 配置相应 CMake/setup.py 后运行 catkin_make，再加载环境 |
| launch / YAML 内容 | 通常不用编译，重新启动相关 launch 或重新加载参数；运行中的节点不一定自动读取修改 |
| C++ 源码 / 头文件 | 运行 catkin_make（或适当使用 --pkg），成功后重启节点 |
| 新增 package / 修改依赖或 CMake | 在工作空间根目录运行 catkin_make，再 source devel/setup.bash |

地图统一写入 `~/smartcar_2026_ws/data/maps/`；rosbag 写入 `data/bags/`；标定与日志分别写入 `data/calibration/`、`data/logs/`，不写进 `src/`。
