# smartcar1 使用指南：Linux + Docker

本版用于恢复可检查的仿真基础：车辆、雷达、纯雷达定位、键盘和建图。不是已经验收的自主导航交付版。Windows 上完成的检查不代表 Linux 或实车验证通过。

## 1. 本版如何使用

把整个 smartcar1 文件夹复制到 Linux 的家目录，例如 `~/smartcar1`。以下命令都在 **Linux 宿主机终端**执行，除非特别标注“容器内”。不需要虚拟机，也不需要宿主机安装 ROS。

需要已安装 Docker Engine、Docker Compose v2（`docker compose`）、图形桌面和 xhost。建议在 Ubuntu 的 Xorg 桌面会话运行图形窗口。本指南不升级你的 Linux 或现有 Docker。

```bash
cd ~/smartcar1
docker compose version
bash scripts/smartcar.sh build
```

首次构建需要联网下载 Ubuntu 16.04 / ROS Kinetic 历史依赖，可能受软件源影响；构建失败就停在这里，不要继续假定成功。源码改动后也要重新 build，并重新 start。仅修改 `config/localization.yaml` 时不用重建镜像，但要 stop 后 start。

新镜像名 `smartcar:baseline-v1`，新容器名 `smartcar-baseline`。不会删除旧版容器或覆盖旧版镜像。两个版本不能同时运行，它们共用宿主机 ROS 端口。先手动停止旧的 smartcar-sim / smartcar-sim-gui；也要关闭宿主机其他 ROS 仿真。

## 2. 每次启动

```bash
cd ~/smartcar1
mkdir -p data/logs
# 容器用户 UID 为 1000；只赋予数据目录所需权限，避免 sudo 启动后文件无法保存。
id -u
xhost +si:localuser:$(id -un)
gui_user=$(getent passwd 1000 | cut -d: -f1)
# gui_user 必须非空；为空时先解决容器用户与宿主机的映射。
[ -n "$gui_user" ] && xhost +si:localuser:"$gui_user"
bash scripts/smartcar.sh start
bash scripts/smartcar.sh check
```

如果显示授权被本机拒绝，或宿主机 UID 不是 1000，请先解决显示授权与数据目录写权限，勿用 `chmod -R 777`。图形授权是 X11 的访问授权，不是 ROS 参数。

start 收到一帧 `/scan` 才提示成功；check 检查 Gazebo、AMCL、扫描匹配节点和地图/雷达消息。它们**不验证定位精度**。

另开终端，仍然先 `cd ~/smartcar1`：

```bash
bash scripts/smartcar.sh rviz
```

需要三维场景时，再开一个终端：

```bash
bash scripts/smartcar.sh gazebo
```

每种窗口只开一个。RViz 固定坐标系应为 `map`。使用 **2D Pose Estimate** 指定车辆实际位置和朝向，确认雷达与地图墙体重合。二维雷达用 LaserScan 显示；定位之前 map 坐标系下暂时看不到车不一定是雷达故障。

本版统一默认软件渲染，以减少旧容器图形驱动不匹配的问题，Gazebo 可能较慢。需要时只开 RViz；硬件加速留待确认显卡和容器驱动兼容后单独配置。

## 3. 键盘操作

```bash
bash scripts/smartcar.sh keyboard
```

**焦点必须放在这个终端里，不要在 RViz 内按控制键。**

| 按键 | 功能 |
|---|---|
| W / S | 前进 0.15 m/s / 倒车 0.08 m/s |
| A / D | 增加左 / 右转角，单独按不会驱动车辆 |
| X | 转角归零 |
| 空格 | 停车 |
| Q 或 Ctrl+C | 停车并退出 |

这是终端按键事件控制：最后一次 W/S 事件后 0.25 秒停止，不是精确的物理按下/松开检测；键盘首次长按重复延迟可能造成起步顿一下。可以先 A/D 选转角，再长按 W/S。关闭键盘后再在其他窗口输入文字。本工具只发 `/sim/cmd_vel`，不直接控制实车。

## 4. 重新建图（可选）

已有可用地图时跳过。先备份 `data/maps/slam_current`，保存地图会覆盖其中 map.yaml 和 map.pgm。

```bash
cp -a data/maps/slam_current "data/maps/slam_backup_$(date +%Y%m%d-%H%M%S)"
bash scripts/smartcar.sh mapping
```

保持该终端运行，另外启动键盘，缓慢行驶覆盖场地。然后停车，在另一终端保存：

```bash
bash scripts/smartcar.sh save-map
```

保存后在建图终端 Ctrl+C，再启动定位：

```bash
bash scripts/smartcar.sh localization
```

保持定位终端运行，重新在 RViz 设置初始姿态。建图和定位不可同时运行。切换异常时执行 stop、start 恢复默认定位模式。

## 5. 停止、日志和更新

先退出键盘，再停止仿真：

```bash
bash scripts/smartcar.sh stop
```

排错：

```bash
bash scripts/smartcar.sh logs
bash scripts/smartcar.sh shell
```

shell 打开的是容器终端，退出用 `exit`。在里面可运行 `rosnode list`、`rostopic hz /scan`、`rosrun tf tf_echo map base_footprint`。后两条用 Ctrl+C 结束。请保存具体错误文本，不要只记住最后一句 process died。

源码或启动脚本更新：stop → build → start。地图与定位配置通过目录挂载保存；容器内随手修改的代码不会自动回写到宿主机。不要以 docker commit 作为正式交付途径。

## 6. 数据在哪里

| 宿主机路径 | 用途 |
|---|---|
| config/localization.yaml | 当前 AMCL 参数，唯一日常编辑入口 |
| data/maps/slam_current/map.yaml 和 map.pgm | 定位地图，容器重建后保留 |
| data/logs | 预览等程序输出；ROS 控制台日志另用 logs 查看 |
| local_project/inspection_c | 原有 C 巡检规划代码，本轮未修改 |
| docker/overlay | 仿真及定位文件，构建时统一安装 |
| maintenance_backups | 本轮修改前的文件备份，不进入 Docker 镜像 |

仅把 slam_current 与 logs 挂入容器，避免宿主机不完整的 data 目录遮住镜像里的仿真参考场地图。地图坐标应保持与现场地图一致。

## 7. 验收顺序与边界

1. 从源码成功构建，并且 start/check 通过。
2. RViz 中静止时地图、扫描、车体稳定，没有持续 TF 错误。
3. 空旷位置直行和倒车，雷达定位随车连续移动，无明显跳位；Q、空格和退出键盘能够停车。
4. 左右转向与实际车模一致，绝不能靠原地旋转完成掉头。
5. 通过前面各项后，再开发外部前后换向导航、通道跟踪、任务执行。

不要启动旧的 waypoint_nav/start_navigation.sh 当作正式自主导航；本轮未把它升级成满足最小转弯半径的完整导航。实车没有 IMU 和编码器，本版定位默认关闭两者，但激光匹配是否在重复通道中稳定，仍必须运行验证。

## 8. 回退与旧文档

原文件保存在 maintenance_backups 下带时间的目录内。停止本版容器后，可把备份的同名文件覆盖回来；new_files.txt 列出本轮新建文件，完整回退时需要移走这些新增文件。旧容器和镜像仍保留，可以单独恢复旧环境。地图备份独立于代码备份，不要混淆。

本指南是本轮维护后的操作入口。交接说明、定位试验记录、旧使用方法和导航方案用于历史参考；出现容器名、镜像名、IMU、启动命令冲突时，以本指南及当前代码为准。导航方案是后续设计，不表示已实现。
