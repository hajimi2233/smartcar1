# 智能车仿真：使用入口

环境是 **Linux + Docker / ROS Kinetic / Gazebo**，不使用虚拟机。所有以下命令在 Linux 宿主机执行，先进入项目目录：`cd ~/smartcar1`。

### 先分清宿主机和容器

本文中的“宿主机”是你实际登录的 Linux 桌面系统；“容器”是 Docker 启动的隔离环境。`bash scripts/smartcar.sh ...` 应在宿主机运行，脚本会自动调用 Docker Compose；只有进入容器后，才直接运行 `rosnode`、`rostopic` 等 ROS 命令。

当前 Compose 服务、镜像和容器名称分别是：

| 项目 | 名称 |
|---|---|
| Compose 服务 | `sim` |
| Docker 镜像 | `smartcar:baseline-v1` |
| Docker 容器 | `smartcar-baseline` |
| ROS 工作空间 | `/home/hajimi/smartcar_2026_ws`（容器内路径） |

如果系统里还存在旧的 `smartcar-sim` 或 `smartcar-sim-gui`，不要把它们与本版容器混用。用 `sudo docker ps -a` 可查看所有容器。

当前可用代码包括仿真、雷达、纯雷达定位、键盘、建图和公共区域单点导航。**已通过离线检查，尚未在你的 Linux、Gazebo 或实车上验收。完整巡检导航尚未接通。**

## 1. 首次安装或更新代码

宿主机需要 Docker Engine、Compose v2、图形桌面及 xhost；无需在宿主机安装 ROS。复制项目时先备份 Linux 上有效的地图和定位参数，避免被演示数据覆盖。

```bash
cd ~/smartcar1
docker compose version
bash scripts/smartcar.sh stop
bash scripts/smartcar.sh build
```

`docker compose version` 只检查 Compose 是否可用；`stop` 停止本项目容器，不删除容器和数据；`build` 根据 `docker/Dockerfile` 重建镜像。

首次构建需要联网下载历史软件依赖，失败就查看构建错误，不要继续假定安装成功。源码改动后必须重新 build；仅修改 `config/localization.yaml` 时 stop 后 start 即可。

本版镜像为 `smartcar:baseline-v1`，容器为 `smartcar-baseline`。先停止旧的 smartcar-sim / smartcar-sim-gui 或其他占用 ROS 端口的进程。不会自动删除旧容器。

## 2. 启动并确认定位

```bash
mkdir -p data/logs
xhost +si:localuser:$(id -un)
gui_user=$(getent passwd 1000 | cut -d: -f1)
[ -n "$gui_user" ] && xhost +si:localuser:"$gui_user"
bash smartcar1/scripts/smartcar.sh start
bash smartcar1/scripts/smartcar.sh check
```

两条 `xhost` 命令只是在当前 X11 会话中允许本地用户连接图形显示器。它们不负责安装显卡驱动，也不会把显示器转发到远程机器。`start` 在后台启动仿真并等待 `/scan` 雷达消息；`check` 再检查 Gazebo、定位节点、雷达和地图话题。

容器用户 UID 为 1000。如果宿主机没有对应用户、显示授权失败或地图保存提示权限不足，先解决用户映射和数据目录权限，不要直接递归赋予 777 权限。图形界面建议用 Xorg 会话。当前 Compose 映射了 `/dev/dri`，使用宿主机的 Mesa/核显图形设备；容器内通过 `llvmpipe` 保证旧版 Ubuntu 16.04/ROS Kinetic 的兼容性。

另开宿主机终端：

```bash
bash smartcar1/scripts/smartcar.sh rviz
```

该命令是在容器内启动 RViz，但窗口显示到宿主机的 `$DISPLAY`。Compose 已映射 `/tmp/.X11-unix` 和 `/dev/dri`，并设置 `QT_X11_NO_MITSHM=1`。当前配置使用 Mesa 的 `llvmpipe` 渲染，避免旧版 ROS Kinetic 与 NVIDIA GLX 库不兼容。若出现 `No matching fbConfigs or visuals found` 或 `failed to load driver`，先确认宿主机是 Xorg 会话、`echo "$DISPLAY"` 有值、当前用户有 `render`/`video` 组权限，并检查 `/dev/dri` 是否已映射。

需要三维场景时再开一个终端执行 `bash smartcar1/scripts/smartcar.sh gazebo`，每种窗口只开一个。

`rviz` 和 `gazebo` 都是图形客户端，必须在执行 `start` 的同一台宿主机上运行。关闭窗口不会停止 ROS 主进程；结束整套仿真仍要执行 `bash scripts/smartcar.sh stop`。

在 RViz 中固定坐标系选 `map`，用 **2D Pose Estimate** 设置车辆实际位置和方向，确认扫描与墙体重合。初始定位标记对应 `base_footprint`。check 只检查节点、雷达和地图消息，不代表定位精度合格。

## 3. 手动驾驶

```bash
bash smartcar1/scripts/smartcar.sh keyboard
```

焦点放在这个终端，不在 RViz 按控制键。

| 按键 | 动作 |
|---|---|
| W | 前进加一档，共四档：0.10 / 0.18 / 0.26 / 0.35 m/s |
| S | 后退加一档，共四档：-0.10 / -0.18 / -0.26 / -0.35 m/s |
| A / D | 每次调整左 / 右 20°，转角限制为 ±40° |
| X | 转角归零 |
| 空格 | 停车 |
| Q / Ctrl+C | 停车并退出 |

速度档位和转角会保持，直到再次按键修改。W 每次前进加一档，S 每次后退加一档；从任一方向切换时先经过 0 档。空格立即停车但保留当前转角，X 将转角回正。关闭键盘后再切换导航。

## 4. 到指定位置和朝向

定位稳定、键盘退出后，另开终端运行并保持打开：

```bash
bash smartcar1/scripts/smartcar.sh nav
```

导航测试：

bash smartcar1/scripts/smartcar.sh nav-test
bash smartcar1/scripts/smartcar.sh multi-nav
bash smartcar1/scripts/smartcar.sh multi-execute
bash smartcar1/scripts/smartcar.sh multi-clear
、bash smartcar1/scripts/smartcar.sh multi-cancel


在 RViz 选择 **2D Nav Goal**，在目标位置按住鼠标并拖出箭头。**箭头起点是最终前轴中点，箭头方向是最终车头方向。** 小车可以前进和倒退就位，换向前先停车。

也可以在另一终端精确输入：

```bash
bash smartcar1/scripts/smartcar.sh goal X Y YAW_DEGREES
```

替换为三个数字：X/Y 是 map 坐标系下的米，角度单位度；0° 朝 +X，90° 朝 +Y，180° 朝 -X，-90° 朝 -Y。不要直接套用其他地图的示例坐标。

绿色路径话题是 `/single_nav/path`，表示后轴参考线，其终点比前轴目标后退 0.62 m。看不到时在 RViz 添加 Path 并选择此话题。

取消当前任务：

```bash
bash smartcar1/scripts/smartcar.sh cancel
```

查看状态：`bash smartcar1/scripts/smartcar.sh nav-status`。需要恢复键盘时，必须先在导航终端 Ctrl+C 退出节点；仅 cancel 不会退出节点。

| 状态 | 意义 |
|---|---|
| IDLE / PLANNING / DRIVING | 等待 / 停车规划 / 行驶 |
| SUCCEEDED | 前轴误差 ≤7 cm、方向误差 ≤4°且静止检查通过 |
| REJECTED / PLAN_FAILED | 目标不合法、车身碰撞、控制源冲突或预算内找不到路径 |
| STOPPED / MAP_CHANGED | 传感器、定位、障碍、路径进度或地图发生问题，已取消任务 |
| FINAL_TOLERANCE_FAILED | 已停车但姿态超差，没有宣布到达 |

默认半径 1.30 m，只是未标定的仿真初值。先在宽敞公共区域测试直行、倒车、90° 转向、换向、取消停车，最后再选通道外准备点。新障碍出现后停车，不自动绕行；查清原因后重新发目标。

本版没有巡检区语义禁入范围，也没有进通道只能前进的任务约束。**目前只在公共区域测试，不能用于完整比赛路线。** 不要启动旧 waypoint_nav 或 start_navigation.sh；新导航入口是 nav。

## 5. 建图与保存（已有可用地图可跳过）

先退出导航。保存会覆盖当前地图，先备份：

```bash
cp -a data/maps/slam_current "data/maps/slam_backup_$(date +%Y%m%d-%H%M%S)"
bash scripts/smartcar.sh mapping
```

保持建图终端运行，另开键盘终端缓慢行驶。停车后在另一终端保存：

```bash
bash scripts/smartcar.sh save-map
```

建图终端 Ctrl+C 后，执行 `bash scripts/smartcar.sh localization`，保持该终端运行，并重新设置初始姿态。建图和定位不能同时运行。切换异常时 stop、start 恢复默认定位模式。

## 6. 停止、排错和数据位置

先关闭键盘或导航，再执行：

```bash
bash scripts/smartcar.sh stop
```

查看仿真日志：`bash scripts/smartcar.sh logs`。导航日志看 nav 所在终端。进入容器：`bash scripts/smartcar.sh shell`，退出用 exit；可在容器里用 `rosnode list`、`rostopic hz /scan`、`rosrun tf tf_echo map base_footprint` 排查，持续命令用 Ctrl+C 结束。

常用的只读检查命令：

```bash
# 宿主机：查看本项目容器是否运行
sudo docker ps -a --filter name=smartcar-baseline

# 宿主机：查看最近的仿真日志
bash scripts/smartcar.sh logs

# 容器内：查看节点和雷达频率
bash scripts/smartcar.sh shell
rosnode list
rostopic hz /scan
```

如果必须直接使用 Docker 命令，当前容器名是 `smartcar-baseline`，进入命令为 `sudo docker exec -it smartcar-baseline bash`。`smartcar-sim` 是旧版本或其他 Compose 项目的名称，除非 `docker ps -a` 明确显示它正在运行，否则不要对它执行 RViz 排错。

### RViz/OpenGL 报错

`xhost` 只解决 X11 访问控制。若日志包含 `No matching fbConfigs or visuals found`、`failed to load driver: swrast`，还要检查显示会话和 Mesa/NVIDIA 驱动：

```bash
echo "$DISPLAY"
glxinfo -B                 # 宿主机安装 mesa-utils 后可用
sudo docker inspect smartcar-baseline
```

本项目 Compose 已挂载 `/tmp/.X11-unix` 和 `/dev/dri`，当前图形环境使用 Mesa/llvmpipe，避免依赖 NVIDIA Container Toolkit。检查设备和用户权限：

```bash
ls -l /dev/dri
groups
docker exec smartcar-baseline ls -l /dev/dri
```

如果出现 `failed to open drm device: Permission denied`，将当前用户加入图形设备组后注销并重新登录：

```bash
sudo usermod -aG render,video "$USER"
```

当前 Compose 中的图形相关配置如下：

```yaml
devices:
  - /dev/dri:/dev/dri
environment:
  LIBGL_ALWAYS_SOFTWARE: "0"
  MESA_LOADER_DRIVER_OVERRIDE: llvmpipe
  GALLIUM_DRIVER: llvmpipe
```

`MESA_LOADER_DRIVER_OVERRIDE=llvmpipe` 指定兼容性最好的 Mesa 软件渲染器；这会比真正的 GPU 硬件渲染慢，但适合当前旧版 ROS/RViz 镜像。

| 路径 | 内容 |
|---|---|
| config/localization.yaml | 当前定位参数 |
| data/maps/slam_current | 当前地图，重建容器后保留 |
| data/logs | 程序输出，不包含全部 ROS 控制台日志 |
| docker/overlay | 构建安装的仿真、定位、导航代码 |
| local_project/inspection_c | 独立 C 巡检算法和输入数据 |
| maintenance_backups | 修改前文件与历史文档压缩备份 |

容器中随手修改的源码不会自动回写宿主机。更新代码使用 stop → build → start。代码备份不等于地图备份，回退前分别确认。

## 7. 需要开发时再看

- [开发交接](docs/开发交接.md)：实现范围、测试结果、未完成事项。
- [后续导航设计](docs/后续导航设计.md)：完整巡检导航设计，未实现部分不作为当前操作说明。
- [巡检核心与点位数据](docs/巡检核心与点位数据.md)：C 程序、26 点格式和动作接口。

队友原始资料保留在 reference_docs，作为实车参考。旧主目录说明和试验日志已归档到 `maintenance_backups/历史说明_整理前_20260921.zip`，不再作为日常操作入口。

### 跟踪偏差恢复

后轴跟踪偏差超过 20 cm 时进入 `WAIT_TRACKING`，立即停车并保留目标。位姿窗口确认停稳后，至少 5 帧新扫描且持续 0.5 秒确认当前车身无碰撞，进入 `REPLANNING`，从当前位置重规划，不直接恢复偏离的旧路径。与遇障恢复共用连续原因计数：跟踪偏差算一类，障碍按来源组合分类，坐标变化不算新原因。成功恢复行驶或重新规划不清零，原因改变或新目标才重计；等待中的每帧复查不累计。持续漂移、定位跳变、扫描/TF 失效、当前位置持续阻挡、规划失败均停止；取消目标不会自动重启。该恢复不能替代跟踪精度修复。

### 外部导航局部轨迹（仿真初版）

全局 Hybrid A* 使用统一硬碰撞包络，并对额外 8 cm 范围内的障碍增加路径代价（软余量，不直接禁行）。执行时在当前前进/倒车段内采样转向和三档低速候选，预测约 3 秒轨迹，按跟踪误差、朝向、净空和速度评分；转向变化受限，不跨换向点，沿途与当前位置的停车范围都使用相同 Grid 硬碰撞检查。无候选时停车并进入原有复查/重规划流程。

RViz 新增橙色 Local trajectory，话题 `/single_nav/local_path`；绿色仍是全局路径。已有 RViz 窗口需重新打开配置，或手动添加该 Path。速度上限不变。局部候选只作小范围修正，大幅绕行交给全局重规划。这是自定义采样控制器，并非 TEB；未接入实测速度和标定制动模型，仍保留最小 18 cm 的保守停车检查。离线与容器计算验证通过，不代表完整 Gazebo 路线或实车验收通过。

### 局部代价与失败诊断更新

局部评分采用整段平均横向误差、终点横向/朝向误差、3/8/15 cm 分层净空惩罚、首个执行舵角变化和到段终点的进度奖励，不再直接奖励速度。参考误差用线段投影计算。预测轨迹最大偏差仍限制为 20 cm；初始偏差 18～20 cm 时允许明确改善偏差的候选。

停车检查从当前位置计算，保留至少 18 cm 的保守检查，不再在 1 秒预测终点后重复追加。预测轨迹本身仍逐段检查碰撞。失败统计 `STOPPING_COLLISION`（当前位置停车范围碰撞）、`ROLLOUT_COLLISION`（预测轨迹碰撞）、`TRACKING_ENVELOPE`（偏差约束），并附各类数量及一个具体样例。净空权重仍是仿真经验参数，实际入口路线待验证。

### 弯道预测优化

局部预测扩展至约 3 秒：先模拟候选转向，再模拟沿后续路径跟踪，保持转向速率限制。前方约 0.4 m 内有明显弯道时将前进候选上限降至 0.10 m/s，倒车原上限不变。进度按参考路径线段投影的累计距离计算，不再用到终点的直线距离奖励，避免绕弯时误判进展。全局搜索本次未改；并未实现全局转向速率硬约束。新增前进、倒车理想运动模型弯道闭环测试；Gazebo 完整入口路线仍待验收。


### 观察模式参数（2026-09-23）

已移除单目标 180 秒执行上限，便于观察规划和跟踪现象。最终成功判定暂放宽为前轴位置误差 ≤0.15 m、车头方向误差 ≤8°；这是调试观察参数，不代表最终验收精度。


### 有限零额外代价线段

RViz 选择 `Set Zero-Cost Line Point` 工具（快捷键 `L`），依次点击两个端点。两点之间的有限线段会显示为青色，默认宽度 1 cm；第三次点击会重新开始定义。导航全局规划在该线段带内不增加软净空代价并给予轻微路径偏好，但硬碰撞、地图边界和最小转弯半径仍有效。
